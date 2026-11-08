//! Copyright (c) M5Stack. All rights reserved.
//! Licensed under the MIT license.
//! See LICENSE file in the project root for full license information.

#include <WiFi.h>
#include <WiFiServer.h>
#include <ESPmDNS.h>
#include <esp_heap_caps.h>
#include <esp_wifi.h>
#include <math.h>
#include <stdlib.h>
#include <string>

#include "screenshot_streamer.hpp"

#include "common_header.h"
#include "wifi_scan.hpp"

static constexpr const char HTTP_200_html[] =
    "HTTP/1.1 200 OK\r\nContent-Type: text/html; "
    "charset=UTF-8\r\nX-Content-Type-Options: nosniff\r\nConnection: "
    "close\r\nCache-Control: no-cache\r\n\r\n";
static constexpr const char HTTP_200_json[] =
    "HTTP/1.1 200 OK\r\nContent-Type: application/json; "
    "charset=UTF-8\r\nX-Content-Type-Options: nosniff\r\nConnection: "
    "close\r\nCache-Control: no-cache\r\n\r\n";
static constexpr const char HTML_footer[] =
    "<div class='ft'>Copyright &copy;2022 "
    "M5Stack</div></div>\n</body></html>\n\n";

static constexpr const char HTML_style[] =
    "<style>"
    "html,body{margin:0;padding:0;font-family:sans-serif;background-color:#"
    "f5f5f5}"
    ".ct{min-height:100%;width:85%;margin:0 auto;display:flex;flex-direction: "
    "column;font-size:5vw}"
    "h1{display:block;margin:0;padding:3vw 0;font-size:8vw}"
    "h2{margin:0;padding:2vw 3vw;border-radius:2vw 2vw 0 "
    "0;font-size:6vw;background-color:#909ba1}"
    "h1,.ft{text-align:center}"
    ".ft{padding:10px 0;font-size:4vw}"
    ".main{flex-grow:1}"
    ".ls{border-radius:2vw;background-color:#bfced6}"
    "a{padding:3vw;display:block;color:#000;border-bottom:1px solid "
    "#eee;text-decoration:none}"
    "a.active,a:hover{color:#fff;background-color:#8b2de2}"
    "a:last-child:hover{border-radius:0 0 2vw 2vw}"
    "form {margin:0}"
    ".fg{margin:10px 0;padding:5px}"
    ".fg input{margin-top:5px;padding:5px 10px;width:100%;border:1px solid "
    "#000;outline:none;border-radius:2vw;font-size:6vw}"
    ".fc{padding-left:2vw}"
    ".fc input[type=\"checkbox\"]{width:5vw;height:5vw;vertical-align:middle}"
    ".fc button{margin:10px 0 0 "
    "0;padding:10px;width:100%;font-size:8vw;border:none;border-radius:2vw;"
    "background-color:#3aee70;outline:none;cursor:pointer}"
    "@media screen and (min-width:720px){"
    ".ct{width:50%;max-width:720px}"
    "h1{padding:20px 0;font-size: 38px;}"
    ".ft{font-size:18px;}"
    ".ct,.ls a,.fg input,.fc button{font-size:24px;}"
    ".fc{padding-left:5px;}"
    ".fc input[type=\"checkbox\"]{left:10px;top:0px;width:20px;height:20px;}"
    ".ls,.fc button,.fg input{border-radius:10px;}"
    "h2{font-size:32px;padding:10px;border-radius:10px 10px 0 0;}"
    "a{padding:10px;}"
    "a:last-child:hover{border-radius:0 0 10px 10px;}}}"
    "</style>";

struct connection_t {
    uint32_t connect_millis = 0;
    WiFiClient client;
    std::string line_buf;
    std::string request_path;
    std::string request_post;
    std::string request_get;
    bool keep_connection = false;
    bool connected       = false;
    bool is_post         = false;
    char boundary[8]     = {0};

    void clear_request(void) {
        is_post = false;
        // clear() keeps the allocated capacity, so each of the long-lived
        // connection slots would otherwise hold on to the largest request it
        // ever saw (up to the line limit). Release buffers that grew beyond
        // a typical request to avoid tying up and fragmenting the heap.
        release_if_large(line_buf);
        release_if_large(request_path);
        release_if_large(request_post);
        release_if_large(request_get);
    }
    static void release_if_large(std::string& s) {
        if (s.capacity() > 128) {
            std::string().swap(s);
        } else {
            s.clear();
        }
    }
    void stop(void) {
        client.stop();
        keep_connection = false;
        connected       = false;
        clear_request();
    }
};

// Parses a Celsius value from a request parameter into the raw sensor scale.
// Rejects empty/non-numeric/NaN/inf input and clamps to the range the raw
// representation can hold, so the float->int conversion is always defined
// (atof("nan") / "1e30" previously caused undefined behaviour).
static bool parse_celsius(const std::string& s, int32_t& raw) {
    char* end = nullptr;
    float v   = strtof(s.c_str(), &end);
    if (end == s.c_str() || !isfinite(v)) return false;
    if (v < -64.0f) v = -64.0f;
    if (v > 448.0f) v = 448.0f;
    raw = convertCelsiusToRaw(v);
    return true;
}

static int hex_value(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

// Decodes application/x-www-form-urlencoded text ("%xx" escapes, '+' as
// space). The former version decoded "%xx" with sscanf("%2X") into a char,
// which stores a 4-byte unsigned int and overran the caller's stack buffer.
// Invalid escapes are kept literally; decoded NUL bytes are dropped because
// they would silently truncate the value when passed on as a C string.
static std::string decode_uri(const std::string& src) {
    std::string out;
    out.reserve(src.size());
    for (size_t i = 0; i < src.size(); ++i) {
        char c = src[i];
        if (c == '%' && i + 2 < src.size()) {
            int hi = hex_value(src[i + 1]);
            int lo = hex_value(src[i + 2]);
            if (hi >= 0 && lo >= 0) {
                char d = (char)((hi << 4) | lo);
                if (d != 0) out += d;
                i += 2;
                continue;
            }
        }
        out += (c == '+') ? ' ' : c;
    }
    return out;
}

static std::string encode_html(const char* src) {
    std::string result;
    if (!src) return result;
    while (*src) {
        switch (*src++) {
            case '&':
                result += "&amp;";
                break;
            case '<':
                result += "&lt;";
                break;
            case '>':
                result += "&gt;";
                break;
            case '"':
                result += "&quot;";
                break;
            case '\'':
                result += "&#39;";
                break;
            default:
                result += src[-1];
                break;
        }
    }
    return result;
}

static void redirect_header(WiFiClient* client, const char* path) {
    client->printf(
        "HTTP/1.1 302 Found\r\nContent-Type: text/html\r\nContent-Length: "
        "0\r\nConnection: close\r\nLocation: %s\r\n\r\n",
        path);
}

// return: true=keep connect
static bool response_404(draw_param_t* draw_param, connection_t* conn) {
    auto client = &conn->client;
    if (WiFi.getMode() & WIFI_AP) {
        redirect_header(client, "/wifi");
    } else {
        client->print(
            "HTTP/1.1 404 Not Found\r\nContent-type: text/html\r\n"
            "Connection: close\r\n\r\n"
            "404 Page not found.<br>\n\n");
    }
    return false;
}

static bool response_main(draw_param_t* draw_param, connection_t* conn) {
    auto client = &conn->client;

    static constexpr const char html_1[] =
        "<html><head><title>T-Lite</title>\n<script>\n"
        "function f(d) { "
        "fetch(\"/"
        "param?\"+d).then((response)=>response.json()).then((o)=>{for(const k "
        "in o){document.getElementById(k).value=o[k];}});\n return false;};\n"
        "window.addEventListener('DOMContentLoaded', function() {f(\"\");})\n"
        "</script>\n<style>\n"
        "body,select,button{font-size:4.5vw !important; font-size:16px}\n"
        ".ctn{margin:0 "
        "auto;width:90%;height:100%;display:flex;flex-direction:column}"
        "input{font-size: 4.5vw;width:100%;height:5vw}\n"
        "input[type=\"text\"]{height:7vw}\n"
        "ul{list-style:none;padding-left:0}\n"
        ".ft{flex:0 0 auto;padding:10px 0;text-align:center}\n"
        ".tgl "
        "label{text-align:center;display:block;border-radius:10px;color:#FFF;"
        "background:#3DA7C7;padding:8px;margin-bottom:3px;cursor:pointer}"
        ".tgl input[type=\"checkbox\"],"
        ".tgl input[type=\"checkbox\"] +ul{display:none}"
        ".tgl input[type=\"checkbox\"]:checked +ul{display:block}"
        ".imgbx{margin:2vw 0;padding:2vw;border-radius:0 0 2vw "
        "2vw;text-align:center;background-color:#0f0f0f}\n"
        ".imgbx h2{margin:0;font-size:8vw;font-weight:300;color:#2eb840}\n"
        ".imgbx img{width:100%}\n"
        "</style></head><body>"
        "<div class='ctn'>\n"
        "<div class='imgbx'><img src='/stream'><h2>T-Lite</h2></div>\n"
        "<div class='tgl'>";

    std::string strbuf;
    strbuf.reserve(8192);

    strbuf = html_1;
    strbuf +=
        "<label for='tgl_alarm'>Alarm</label><input type='checkbox' "
        "id='tgl_alarm'>\n<ul>\n"
        " <li> Alarm Mode: <select id='alarm_mode' onchange='f(\"alarm_mode=\" "
        "+ this.options[this.selectedIndex].value)'>";
    for (int i = 0; i < draw_param->alarm_mode_max; ++i) {
        appendf(strbuf, "<option value=\"%d\">%s</option>\n", i,
                draw_param->alarm_mode.getText(i));
    }
    strbuf += "</select></li>\n";

    strbuf += " <li> Temperature: <span id='at'>";
    appendf(strbuf, "%3.1f",
            convertRawToCelsius(draw_param->alarm_temperature));
    strbuf +=
        "</span><br>\n<input width='400em' type='range' min='-50' max='350' "
        "step='0.5' id='alarm_temperature' onchange='f(\"alarm_temperature=\" "
        "+ this.value)' "
        "oninput='document.getElementById(\"at\").innerText=this.value'></li>";
    strbuf +=
        " <li> Reference: <select id='alarm_reference' "
        "onchange='f(\"alarm_reference=\" + "
        "this.options[this.selectedIndex].value)'>";
    for (int i = 0; i < draw_param->alarm_reference_max; ++i) {
        appendf(strbuf, "<option value=\"%d\">%s</option>\n", i,
                draw_param->alarm_reference.getText(i));
    }
    strbuf += "</select></li>\n</ul>\n";
    strbuf +=
        "<label for='tgl_sensor'>Sensor</label><input type='checkbox' "
        "id='tgl_sensor'>\n<ul>\n"
        " <li> Refresh Rate: <select id='sens_refreshrate' "
        "onchange='f(\"sens_refreshrate=\" + "
        "this.options[this.selectedIndex].value)'>";
    for (int i = 0; i < draw_param->sens_refreshrate_max; ++i) {
        appendf(strbuf, "<option value=\"%d\">%s</option>\n", i,
                draw_param->sens_refreshrate.getText(i));
    }
    strbuf += "</select></li>\n";

    strbuf +=
        " <li>Noise Filter: <select id='sens_noisefilter' "
        "onchange='f(\"sens_noisefilter=\" + "
        "this.options[this.selectedIndex].value)'>";
    for (int i = 0; i < draw_param->sens_noisefilter_max; ++i) {
        appendf(strbuf, "<option value=\"%d\">%s</option>\n", i,
                draw_param->sens_noisefilter.getText(i));
    }
    strbuf += "</select></li>\n";

    strbuf +=
        " <li>Monitor Area: <select id='sens_monitorarea' "
        "onchange='f(\"sens_monitorarea=\" + "
        "this.options[this.selectedIndex].value)'>";
    for (int i = 0; i < draw_param->sens_monitorarea_max; ++i) {
        appendf(strbuf, "<option value=\"%d\">%s</option>\n", i,
                draw_param->sens_monitorarea.getText(i));
    }
    strbuf += "</select></li>\n";

    strbuf += " <li> Emissivity: <span id='em'>";
    appendf(strbuf, "%d",
            draw_param->sens_emissivity.get());
    // strbuf += draw_param->sens_emissivity.getText();
    strbuf +=
        "</span><br>\n<input type='range' min='20' max='100' "
        "id='sens_emissivity' onchange='f(\"sens_emissivity=\" + this.value)' "
        "oninput='document.getElementById(\"em\").innerText=this.value'></"
        "li>\n</ul>\n";

    strbuf +=
        "<label for='tgl_range'>Range</label><input type='checkbox' "
        "id='tgl_range'>\n<ul>\n"
        "<li>Auto Range: <select id='range_autoswitch' "
        "onchange='f(\"range_autoswitch=\" + "
        "this.options[this.selectedIndex].value)'>";
    for (int i = 0; i < draw_param->range_autoswitch_max; ++i) {
        appendf(strbuf, "<option value=\"%d\">%s</option>\n", i,
                draw_param->range_autoswitch.getText(i));
    }
    strbuf += "</select></li>\n";
    strbuf += " <li> Upper Temperature: <span id='rh'>";
    appendf(strbuf, "%3.1f",
            convertRawToCelsius(draw_param->range_temp_upper));
    strbuf +=
        "</span><br>\n<input width='400em' type='range' min='-50' max='350' "
        "step='0.5' id='range_temp_upper' onchange='f(\"range_temp_upper=\" "
        "+ this.value)' "
        "oninput='document.getElementById(\"rh\").innerText=this.value'></li>";
    strbuf += " <li> Lower Temperature: <span id='rl'>";
    appendf(strbuf, "%3.1f",
            convertRawToCelsius(draw_param->range_temp_lower));
    strbuf +=
        "</span><br>\n<input width='400em' type='range' min='-50' max='350' "
        "step='0.5' id='range_temp_lower' onchange='f(\"range_temp_lower=\" "
        "+ this.value)' "
        "oninput='document.getElementById(\"rl\").innerText=this.value'></"
        "li>\n</ul>\n";

    strbuf +=
        "<label for='tgl_misc'>Others</label><input type='checkbox' "
        "id='tgl_misc'>\n<ul>\n"
        "<li>CPU Speed: <select id='misc_cpuspeed' "
        "onchange='f(\"misc_cpuspeed=\" + "
        "this.options[this.selectedIndex].value)'>";
    for (int i = 0; i < draw_param->misc_cpuspeed_max; ++i) {
        appendf(strbuf, "<option value=\"%d\">%s</option>\n", i,
                draw_param->misc_cpuspeed.getText(i));
    }
    strbuf += "</select></li>\n";

    strbuf +=
        "<li>Sound Volume: <select id='misc_volume' "
        "onchange='f(\"misc_volume=\" + "
        "this.options[this.selectedIndex].value)'>";
    for (int i = 0; i < draw_param->misc_volume_max; ++i) {
        appendf(strbuf, "<option value=\"%d\">%s</option>\n", i,
                draw_param->misc_volume.getText(i));
    }
    strbuf += "</select></li>\n";

    strbuf +=
        "<li>LCD Brightness: <select id='misc_brightness' "
        "onchange='f(\"misc_brightness=\" + "
        "this.options[this.selectedIndex].value)'>";
    for (int i = 0; i < draw_param->misc_brightness_max; ++i) {
        appendf(strbuf, "<option value=\"%d\">%s</option>\n", i,
                draw_param->misc_brightness.getText(i));
    }
    strbuf += "</select></li>\n";

    strbuf +=
        "<li>Language: <select id='misc_language' "
        "onchange='f(\"misc_language=\" + "
        "this.options[this.selectedIndex].value)'>";
    for (int i = 0; i < draw_param->misc_language_max; ++i) {
        appendf(strbuf, "<option value=\"%d\">%s</option>\n", i,
                draw_param->misc_language_text[i]);
    }
    strbuf += "</select></li>\n";

    strbuf += "<li> LAN Stream Quality: <span id='jq'>";
    strbuf += draw_param->net_jpg_quality.getText();
    strbuf +=
        "</span><br>\n<input type='range' min='1' max='100' "
        "id='net_jpg_quality' onchange='f(\"net_jpg_quality=\" + this.value)' "
        "oninput='document.getElementById(\"jq\").innerText=this.value;'></"
        "li>\n";

    strbuf +=
        "<li> Pointer:<select id='misc_pointer' onchange='f(\"misc_pointer=\" "
        "+ this.options[this.selectedIndex].value)'>";
    for (int i = 0; i < draw_param->misc_pointer_max; ++i) {
        appendf(strbuf, "<option value=\"%d\">%s</option>\n", i,
                draw_param->misc_pointer.getText(i));
    }
    strbuf += "</select></li>\n";

    strbuf +=
        "<li> Color:<select id='misc_color' onchange='f(\"misc_color=\" + "
        "this.options[this.selectedIndex].value)'>";
    for (int i = 0; i < color_map_table_len; ++i) {
        appendf(strbuf, "<option value=\"%d\">%s</option>\n", i,
                draw_param->misc_color.getText(i));
    }
    strbuf += "</select></li>\n";

    strbuf +=
        "<li>Layout:<select id='misc_layout' onchange='f(\"misc_layout=\" + "
        "this.options[this.selectedIndex].value)'>";
    for (int i = 0; i < 6; ++i) {
        appendf(strbuf, "<option value=\"%d\">layout %d</option>\n", i, i);
    }
    strbuf += "</select></li>\n</ul>\n";
    strbuf += HTML_footer;

    // client->print(HTTP_200_html);
    client->print(
        "HTTP/1.1 200 OK\r\nContent-Type: text/html; "
        "charset=UTF-8\r\nX-Content-Type-Options: nosniff\r\nConnection: "
        "close\r\nCache-Control: no-cache\r\n");
    client->printf("Content-Length: %d\r\n\r\n", strbuf.size());
    client->write(strbuf.c_str(), strbuf.size());
    return false;
}

static bool response_param(draw_param_t* draw_param, connection_t* conn) {
    auto client = &conn->client;
    int pos     = conn->request_get.find('=');
    if (pos >= 0) {
        auto key = conn->request_get.substr(0, pos);
        ++pos;
        auto val = conn->request_get.substr(pos);

        int32_t raw;
        if (key == "alarm_temperature") {
            if (parse_celsius(val, raw)) draw_param->alarm_temperature = raw;
        } else if (key == "range_temp_upper") {
            if (parse_celsius(val, raw)) draw_param->range_temp_upper = raw;
        } else if (key == "range_temp_lower") {
            if (parse_celsius(val, raw)) draw_param->range_temp_lower = raw;
        } else {
            int v = atoi(val.c_str());
            if (key == "alarm_mode") {
                draw_param->alarm_mode.set(v);
            } else if (key == "alarm_reference") {
                draw_param->alarm_reference.set(v);
            }
            // else if (key == "alarm_behavior"    ) {
            // draw_param->alarm_behavior  .set(v); }
            else if (key == "sens_refreshrate") {
                draw_param->sens_refreshrate.set(v);
            } else if (key == "sens_noisefilter") {
                draw_param->sens_noisefilter.set(v);
            } else if (key == "sens_monitorarea") {
                draw_param->sens_monitorarea.set(v);
            } else if (key == "sens_emissivity") {
                draw_param->sens_emissivity.set(v);
            } else if (key == "range_autoswitch") {
                draw_param->range_autoswitch.set(v);
            } else if (key == "net_jpg_quality") {
                draw_param->net_jpg_quality.set(v);
            } else if (key == "misc_cpuspeed") {
                draw_param->misc_cpuspeed.set(v);
            } else if (key == "misc_volume") {
                draw_param->misc_volume.set(v);
            } else if (key == "misc_brightness") {
                draw_param->misc_brightness.set(v);
            } else if (key == "misc_language") {
                draw_param->misc_language.set(v);
            } else if (key == "misc_pointer") {
                draw_param->misc_pointer.set(v);
            } else if (key == "misc_layout") {
                draw_param->misc_layout.set(v);
                draw_param->in_config_mode = false;
            } else if (key == "misc_color") {
                draw_param->misc_color.set(v);
            }
        }
        // draw_param->saveNvs();
    }

    std::string strbuf;
    appendf(strbuf, "{\n \"alarm_temperature\": \"%3.1f\"",
            convertRawToCelsius(draw_param->alarm_temperature));
    appendf(strbuf, ",\n \"alarm_mode\": \"%d\"",
            draw_param->alarm_mode.get());
    appendf(strbuf, ",\n \"alarm_reference\": \"%d\"",
            draw_param->alarm_reference.get());
    appendf(strbuf, ",\n \"sens_refreshrate\": \"%d\"",
            draw_param->sens_refreshrate.get());
    appendf(strbuf, ",\n \"sens_noisefilter\": \"%d\"",
            draw_param->sens_noisefilter.get());
    appendf(strbuf, ",\n \"sens_monitorarea\": \"%d\"",
            draw_param->sens_monitorarea.get());
    appendf(strbuf, ",\n \"sens_emissivity\": \"%d\"",
            draw_param->sens_emissivity.get());
    appendf(strbuf, ",\n \"range_autoswitch\": \"%d\"",
            draw_param->range_autoswitch.get());
    appendf(strbuf, ",\n \"range_temp_upper\": \"%3.1f\"",
            convertRawToCelsius(draw_param->range_temp_upper));
    appendf(strbuf, ",\n \"range_temp_lower\": \"%3.1f\"",
            convertRawToCelsius(draw_param->range_temp_lower));
    appendf(strbuf, ",\n \"net_jpg_quality\": \"%d\"",
            draw_param->net_jpg_quality.get());
    appendf(strbuf, ",\n \"misc_cpuspeed\": \"%d\"",
            draw_param->misc_cpuspeed.get());
    appendf(strbuf, ",\n \"misc_volume\": \"%d\"",
            draw_param->misc_volume.get());
    appendf(strbuf, ",\n \"misc_brightness\": \"%d\"",
            draw_param->misc_brightness.get());
    appendf(strbuf, ",\n \"misc_language\": \"%d\"",
            draw_param->misc_language.get());
    appendf(strbuf, ",\n \"misc_pointer\": \"%d\"",
            draw_param->misc_pointer.get());
    appendf(strbuf, ",\n \"misc_layout\": \"%d\"",
            draw_param->misc_layout.get());
    appendf(strbuf, ",\n \"misc_color\": \"%d\"",
            draw_param->misc_color.get());
    strbuf += "\n}\n\n";

    client->print(
        "HTTP/1.1 200 OK\r\nContent-Type: application/json; "
        "charset=UTF-8\r\nX-Content-Type-Options: nosniff\r\nConnection: "
        "close\r\nCache-Control: no-cache\r\n");
    client->printf("Content-Length: %d\r\n\r\n", strbuf.size());
    client->write(strbuf.c_str(), strbuf.size());
    return false;
}

static bool response_json(draw_param_t* draw_param, connection_t* conn) {
    auto client = &conn->client;
    /*
        auto t = time(nullptr);
        auto gmt = gmtime(&t);
        client->printf("{\"center\":\"%6.1f\",",
       convertRawToCelsius(draw_param->frame->temp[framedata_t::center]));
        client->printf("\"highest\":\"%6.1f\",",
       convertRawToCelsius(draw_param->frame->temp[framedata_t::highest]));
        client->printf("\"average\":\"%6.1f\",",
       convertRawToCelsius(draw_param->frame->temp[framedata_t::average]));
        client->printf("\"lowest\":\"%6.1f\",",
       convertRawToCelsius(draw_param->frame->temp[framedata_t::lowest]));
        client->printf("\"date\":\"%s, %d %s %04d %02d:%02d:%02d GMT\"}\n",
       wday_tbl[gmt->tm_wday], gmt->tm_mday, mon_tbl[gmt->tm_mon], gmt->tm_year
       + 1900, gmt->tm_hour, gmt->tm_min, gmt->tm_sec);
    */
    std::string strbuf;
    {
        auto frame = *draw_param->frame;
        strbuf     = frame.getJsonData();
    }

    // client->print(HTTP_200_json);
    client->print(
        "HTTP/1.1 200 OK\r\nContent-Type: application/json; "
        "charset=UTF-8\r\nX-Content-Type-Options: nosniff\r\nConnection: "
        "close\r\nCache-Control: no-cache\r\n");
    client->printf("Content-Length: %d\r\n\r\n", strbuf.size());
    client->write(strbuf.c_str(), strbuf.size());
    return false;
}

static bool response_text(draw_param_t* draw_param, connection_t* conn) {
    auto client = &conn->client;
    // The device has no time source (no NTP), so show uptime instead of a
    // wall-clock date/time.
    const uint32_t uptime_s = millis() / 1000;

    static constexpr const char html_1[] =
        "<!DOCTYPE html><html lang=\"en\"><head><meta charset=\"utf-8\">\n"
        "<meta name=\"viewport\" content=\"width=device-width, "
        "initial-scale=1.0\">\n"
        "<meta http-equiv=\"refresh\" content=\"1; URL=\">\n"
        "<title>T-Lite Text Info</title>\n</head>\n<body><table>\n";

    std::string strbuf = html_1;
    appendf(strbuf, "<tr><th>uptime </th><td>%u:%02u:%02u</td></tr>\n",
            unsigned(uptime_s / 3600),
            unsigned((uptime_s / 60) % 60),
            unsigned(uptime_s % 60));
    appendf(strbuf, "<tr><th>center </th><td>%3.1f</td></tr>\n",
            convertRawToCelsius(draw_param->frame->temp[framedata_t::center]));
    appendf(strbuf, "<tr><th>highest</th><td>%3.1f</td></tr>\n",
            convertRawToCelsius(
            draw_param->frame->temp[framedata_t::highest]));
    appendf(strbuf, "<tr><th>average</th><td>%3.1f</td></tr>\n",
            convertRawToCelsius(
            draw_param->frame->temp[framedata_t::average]));
    appendf(strbuf, "<tr><th>lowest </th><td>%3.1f</td></tr>\n",
            convertRawToCelsius(draw_param->frame->temp[framedata_t::lowest]));
    strbuf += "</table></body></html>\n\n";

    client->print(
        "HTTP/1.1 200 OK\r\nContent-Type: text/html; "
        "charset=UTF-8\r\nX-Content-Type-Options: nosniff\r\nConnection: "
        "close\r\nCache-Control: no-cache\r\n");
    client->printf("Content-Length: %d\r\n\r\n", strbuf.size());
    client->write(strbuf.c_str(), strbuf.size());

    return false;
}

static bool response_stream(draw_param_t* draw_param, connection_t* conn) {
    auto client = &conn->client;
    client->print("HTTP/1.1 200 OK\r\nAccess-Control-Allow-Origin: *\r\n");
    client->print("Content-type: multipart/x-mixed-replace;boundary=");
    client->print(conn->boundary);
    client->print("\r\n");
    screenshot_holder.requestScreenShot(client);
    // JPEGストリームを受け取るため戻り値をtrue (keep connection)にする
    return true;
}

static bool response_wifi(draw_param_t* draw_param, connection_t* conn) {
    auto client = &conn->client;
    // APモードでなければ wifi設定を使用できないようにする
    // if (draw_param->net_setup_mode.get() == draw_param->net_setup_mode_off) {
    if (!(WiFi.getMode() & WIFI_AP)) {
        redirect_header(client, "/");
        return false;
    }
    //*/
    if (conn->request_post.length()) {
        std::string ssid, password;
        int pos1 = 0;
        int pos2 = 0;
        bool end = false;
        do {
            pos1 = conn->request_post.find('=', pos2);
            if (pos1 < 0) break;
            auto key = conn->request_post.substr(pos2, pos1 - pos2);
            pos2     = conn->request_post.find('&', ++pos1);
            end      = (pos2 < 0);
            if (end) pos2 = conn->request_post.length();
            auto val = conn->request_post.substr(pos1, pos2++ - pos1++);
            // ESP_LOGE("DEBUG","key : %s  val : %s", key.c_str(), val.c_str());

            if (key == "s") {
                ssid = decode_uri(val);
            } else if (key == "p") {
                password = decode_uri(val);
            }
        } while (!end);
        // 802.11 limits: SSID up to 32 bytes; WPA2 passphrase 8..63
        // characters or a 64-digit hex PSK. Never save a truncated value.
        static constexpr size_t ssid_max = 32;
        static constexpr size_t pass_max = 64;
        if (ssid.length() > ssid_max || password.length() > pass_max) {
            ssid.clear();
        }
        if (ssid.length()) {
            // A STA connection cannot reliably start while an asynchronous AP
            // scan is active. Release its result buffer as well, since the list
            // is no longer needed after credentials have been submitted.
            wifi_scan::cancel_and_clear();
            client->print(HTTP_200_html);
            client->print(
                "<!DOCTYPE html><html><head><meta charset='UTF-8'>"
                "<meta name='viewport' "
                "content='width=device-width,initial-scale=1'>"
                "<title>T-Lite WiFi setup</title></head><body>"
                "<h1>WiFi settings saved</h1>"
                "<p>T-Lite is connecting to the selected network.</p>"
                "</body></html>");
            // Let the response reach the phone before shutting down the AP.
            delay(256);
            draw_param->sys_ssid         = ssid;
            draw_param->net_tmp_ssid     = ssid;
            draw_param->net_tmp_pwd      = password;
            draw_param->net_running_mode = draw_param->net_running_mode_offline;
            delay(64);
            draw_param->net_running_mode = draw_param->net_running_mode_lan;
            draw_param->net_setup_mode = draw_param->net_setup_mode_off;
            /*
                        WiFi.begin(ssid.c_str(), password.c_str());
                        client->setTimeout(10);
                        int retry = 2048;
                        do {
                            delay(1);
                        } while (!WiFi.isConnected() && --retry);
                        if (retry) {
                            static constexpr const char html_success_1[] =
                            "<html><head><meta http-equiv=\"Content-Type\"
            content=\"text/html; charset=UTF-8\"><title>T-Lite</title>\n"
                            "<style>\n body,select,button{font-size:12vw
            !important;font-size:24px;}\n"
                            ".ft{flex:0 0 auto;padding:10px
            0;text-align:center}\n" "ul,li{list-style:none;padding-left:0;}\n"
                            "a{font-size:8vw;text-align:center;display:block;background:#abc;padding:8px;margin-bottom:3px;cursor:pointer;}\n"
                            "</style></head><body><div><h4>T-Lite</h4>\n<ul>"
                            "<li>WiFi connected !</li>\n<li>";

                            static constexpr const char html_success_2[] =
            "</li>\n</ul>";

                            client->print(HTTP_200_html);
                            client->print(html_success_1);
                            client->printf("<a href='%s'>%s</a>",
            draw_param->net_url.c_str(), draw_param->net_url.c_str());
                            client->print(html_success_2);
                            client->print(HTML_footer);
                            client->stop();

                            delay(256);

                            draw_param->net_setup_mode =
            draw_param->net_setup_mode_off; return false;
                        }
            //*/
        } else {
            redirect_header(client, "/wifi");
        }
        return false;
    }

    static constexpr const char html_1[] =
        "<html><head><meta http-equiv=\"Content-Type\" content=\"text/html; "
        "charset=UTF-8\">\n"
        "<meta name=\"viewport\" content=\"width=device-width, "
        "initial-scale=1.0\">\n"
        "<title>T-Lite WiFi setup</title>\n"
        "<script>function s(a){var l=document.querySelectorAll('.ls "
        "a');for(let i=0;i<l.length;i++){"
        "if(a===l[i]){a.classList.add('active')}else{l[i].classList.remove('"
        "active')}}"
        "document.getElementById('s').value=a.innerText||a.textContent;"
        "document.getElementById('p').focus();};"
        "function r(){location.replace('/wifi?scan=1');}"
        "function h() {var p = "
        "document.getElementById('p');p.type==='text'?p.type='password':p.type="
        "'text';}"
        "</script>\n";

    static constexpr const char html_2[] =
        "</head><body><div class='ct'><h1>T-Lite WiFi setup</h1>"
        "<div class='main'>";
    static constexpr const char html_3[] = "<div class='ls'><h2>SSID List</h2>";

    static constexpr const char html_4[] =
        "</div><form method='POST' action='wifi'>"
        "<div class='fg'><label for='s'>SSID: </label><input name='s' id='s' "
        "list='ssid-list' "
        "maxlength='32' autocapitalize='none' autocorrect='off' "
        "placeholder='SSID'></div>"
        "<div class='fg'><label for='p'>Password: </label><input name='p' "
        "id='p' maxlength='64' type='password' placeholder='Password'></div>"
        "<div class='fc'><input id='show_pwd' type='checkbox' "
        "onclick='h()'><label for='show_pwd'>Show Password</label><button "
        "type='submit'>Save</button></div>"
        "</form></div>";

    client->print(HTTP_200_html);
    client->print(html_1);
    // "Refresh networks" asks for a new scan; the first visit (or one after
    // a scan that failed) starts one too. wifi_scan ignores the request
    // while a scan is already running, so reloads cannot stack scans.
    if (conn->request_get == "scan=1" || !wifi_scan::has_results()) {
        wifi_scan::request();
    }
    const bool scan_running = wifi_scan::running();
    // The last complete list stays visible while a new scan runs.
    const std::vector<std::string> ssids = wifi_scan::results();
    if (scan_running) {
        client->print(
            "<script>setTimeout(function(){location.replace('/wifi')},"
            "1500);</script>");
    }
    client->print(HTML_style);
    client->print(html_2);
    if (!draw_param->sys_ssid.empty()) {
        const std::string current_ssid =
            encode_html(draw_param->sys_ssid.c_str());
        client->print("<div class='ls'><h2>Current SSID</h2>");
        client->printf(
            "<a href='javascript:void(0);' onclick='s(this)'> %s </a>",
            current_ssid.c_str());
        client->print("</div><hr>");
    }

    client->print(html_3);
    if (scan_running) {
        client->print("<p>Scanning...</p>");
    } else if (ssids.empty()) {
        client->print("<p>No networks found.</p>");
    }
    // wifi_scan already removed hidden and duplicate entries.
    for (const auto& ssid : ssids) {
        const std::string escaped = encode_html(ssid.c_str());
        client->printf(
            "<a href='javascript:void(0);' onclick='s(this)'>%s</a>",
            escaped.c_str());
    }
    client->print(
        "<a href='javascript:void(0);' onclick='r()'>Refresh networks</a>");
    client->print("<datalist id='ssid-list'>");
    for (const auto& ssid : ssids) {
        const std::string escaped = encode_html(ssid.c_str());
        client->printf("<option value='%s'></option>", escaped.c_str());
    }
    client->print("</datalist>");
    client->print(html_4);
    client->print(HTML_footer);

    return false;
}

static bool response_health(draw_param_t* draw_param, connection_t* conn) {
    (void)draw_param;
    auto client = &conn->client;

    const size_t heap_free_8bit = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    const size_t heap_min_8bit =
        heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
    const size_t heap_largest_8bit =
        heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
    const size_t heap_free_dma = heap_caps_get_free_size(MALLOC_CAP_DMA);
    const size_t heap_largest_dma =
        heap_caps_get_largest_free_block(MALLOC_CAP_DMA);

    const TaskHandle_t wifi_task   = xTaskGetHandle("wifiTask");
    const TaskHandle_t stream_task = xTaskGetHandle("stream");
    const UBaseType_t web_stack    = uxTaskGetStackHighWaterMark(nullptr);
    const UBaseType_t wifi_stack =
        wifi_task ? uxTaskGetStackHighWaterMark(wifi_task) : 0;
    const UBaseType_t stream_stack =
        stream_task ? uxTaskGetStackHighWaterMark(stream_task) : 0;

    const wifi_mode_t wifi_mode   = WiFi.getMode();
    const wl_status_t wifi_status = WiFi.status();
    const bool sta_connected      = wifi_status == WL_CONNECTED;
    const IPAddress sta_ip = sta_connected ? WiFi.localIP() : IPAddress();
    const IPAddress ap_ip =
        (wifi_mode & WIFI_AP) ? WiFi.softAPIP() : IPAddress();

    char body[768];
    const int formatted_len = snprintf(
        body, sizeof(body),
        "{\n"
        "  \"uptime_ms\": %lu,\n"
        "  \"heap\": {\n"
        "    \"free_8bit\": %u,\n"
        "    \"minimum_free_8bit\": %u,\n"
        "    \"largest_8bit\": %u,\n"
        "    \"free_dma\": %u,\n"
        "    \"largest_dma\": %u\n"
        "  },\n"
        "  \"stack_high_water\": {\n"
        "    \"web\": %u,\n"
        "    \"wifi\": %u,\n"
        "    \"stream\": %u\n"
        "  },\n"
        "  \"wifi\": {\n"
        "    \"mode\": %u,\n"
        "    \"status\": %u,\n"
        "    \"connected\": %s,\n"
        "    \"rssi\": %d,\n"
        "    \"sta_ip\": \"%u.%u.%u.%u\",\n"
        "    \"ap_ip\": \"%u.%u.%u.%u\",\n"
        "    \"ap_clients\": %u\n"
        "  }\n"
        "}\n",
        static_cast<unsigned long>(millis()),
        static_cast<unsigned>(heap_free_8bit),
        static_cast<unsigned>(heap_min_8bit),
        static_cast<unsigned>(heap_largest_8bit),
        static_cast<unsigned>(heap_free_dma),
        static_cast<unsigned>(heap_largest_dma),
        static_cast<unsigned>(web_stack), static_cast<unsigned>(wifi_stack),
        static_cast<unsigned>(stream_stack), static_cast<unsigned>(wifi_mode),
        static_cast<unsigned>(wifi_status), sta_connected ? "true" : "false",
        sta_connected ? WiFi.RSSI() : 0, sta_ip[0], sta_ip[1], sta_ip[2],
        sta_ip[3], ap_ip[0], ap_ip[1], ap_ip[2], ap_ip[3],
        static_cast<unsigned>(WiFi.softAPgetStationNum()));
    const size_t body_len =
        formatted_len < 0
            ? 0
            : std::min(static_cast<size_t>(formatted_len), sizeof(body) - 1);

    client->print(
        "HTTP/1.1 200 OK\r\nContent-Type: application/json; "
        "charset=UTF-8\r\nX-Content-Type-Options: nosniff\r\nConnection: "
        "close\r\nCache-Control: no-cache\r\n");
    client->printf("Content-Length: %u\r\n\r\n",
                   static_cast<unsigned>(body_len));
    client->write(reinterpret_cast<const uint8_t*>(body), body_len);
    return false;
}

static bool response_top(draw_param_t* draw_param, connection_t* conn) {
    auto client = &conn->client;
    if ((WiFi.getMode() & WIFI_AP) && !WiFi.isConnected()) {
        // redirect_header(client, "/wifi");
        // return false;
        return response_wifi(draw_param, conn);
    }
    // if (!WiFi.isConnected()) {
    //     redirect_header(client, "/main");
    // } else {
    //     redirect_header(client, "/wifi");
    // }

    static constexpr const char html_1[] =
        "<!DOCTYPE html><html lang=\"en\"><head><meta charset=\"utf-8\">\n"
        "<meta name=\"viewport\" content=\"width=device-width, "
        "initial-scale=1.0\">\n"
        "<title>T-Lite Top Menu</title>\n";

    static constexpr const char html_2[] =
        "</head><body><div class='ct'><h1>T-Lite Top menu</h1>"
        "<div class='main'><div class='ls'><h2>LAN</h2>";

    static constexpr const char html_4[] =
        "<a href=\"/main\">Browser control</a>\n"
        "<a href=\"/text\">Text infomation</a>\n"
        "<a href=\"/json\">JSON data</a>\n"
        "<a href=\"/health\">System health</a>\n"
        "<a href=\"/stream\">Stream Image</a>\n"
        "</div></div>\n";

    client->print(HTTP_200_html);
    client->print(html_1);
    client->print(HTML_style);
    client->print(html_2);

    if (WiFi.getMode() & WIFI_AP) {
        client->print("<a href=\"/wifi\">WiFi setting</a>\n");
    }

    client->print(html_4);

    client->print(HTML_footer);
    return false;
}

struct response_table_t {
    const char* path;
    bool (*response_func)(draw_param_t*, connection_t*);
};

static constexpr const response_table_t response_table[] = {
    {"/", response_top},        {"/main", response_main},
    {"/json", response_json},   {"/text", response_text},
    {"/wifi", response_wifi},   {"/stream", response_stream},
    {"/param", response_param}, {"/health", response_health},
};

void webserverTask(void* arg) {
    auto draw_param = (draw_param_t*)arg;

    WiFiServer httpServer(80, 4);

    static constexpr const size_t connection_size = 8;
    connection_t connection[connection_size];
    uint8_t connection_index  = 0;
    uint32_t conn_idx         = 0;
    bool prev_connected       = false;
    bool mdns_started         = false;
    uint32_t mdns_retry_at    = 0;
    uint32_t prev_sta_ip      = 0;
    uint32_t prev_ap_ip       = 0;
    uint8_t restart_countdown = 0;
    uint8_t prev_active_count = 0;
    uint8_t active_count      = 0;
    uint8_t loop_counter      = 0;

    for (;;) {
        // switch (screenshot_holder.processCapture()) {
        //     screenshot_streamer_t::pr_nothing:
        //     break;
        // }
        if (++loop_counter == 0) delay(1);
        if (screenshot_holder.processCapture() ==
            screenshot_streamer_t::process_result_t::pr_nothing) {
            if (!active_count) {
                delay(1);
            }
        }
        // if (prev_active_mask != active_mask) {
        //     prev_active_mask = active_mask;
        // ESP_EARLY_LOGD("DEBUG","httpServer active_mask = %02x", active_mask);
        // }
        bool connected = (WiFi.status() == WL_CONNECTED) ||
                         (WiFi.getMode() & wifi_mode_t::WIFI_MODE_AP);

        if (prev_connected != connected) {
            prev_connected = connected;
            if (connected) {
                httpServer.begin();
                // httpServer.setTimeout(3);
                // httpServer.setNoDelay(true);
                // ESP_EARLY_LOGD("DEBUG","httpServer begin");
            } else {
                for (auto& conn : connection) {
                    conn.stop();
                }
                httpServer.end();
                // ESP_EARLY_LOGD("DEBUG","httpServer end");
            }
        }
        // AP and STA may change without the HTTP server becoming disconnected.
        const uint32_t sta_ip =
            WiFi.isConnected() ? uint32_t(WiFi.localIP()) : 0;
        const uint32_t ap_ip =
            (WiFi.getMode() & WIFI_MODE_AP) ? uint32_t(WiFi.softAPIP()) : 0;
        if (!sta_ip || sta_ip != prev_sta_ip || ap_ip != prev_ap_ip) {
            if (mdns_started) MDNS.end();
            mdns_started  = false;
            mdns_retry_at = millis();
            prev_sta_ip   = sta_ip;
            prev_ap_ip    = ap_ip;
        }
        if (sta_ip && !mdns_started && int32_t(millis() - mdns_retry_at) >= 0) {
            mdns_started = MDNS.begin(draw_param->net_hostname.c_str());
            if (mdns_started) {
                MDNS.addService("http", "tcp", 80);
            } else {
                // begin() may fail after allocating the responder.
                MDNS.end();
                ESP_LOGW("mDNS",
                         "Responder startup failed; retrying in 5 seconds");
                mdns_retry_at = millis() + 5000;
            }
        }
        if (!connected) {
            delay(32);
            continue;
        }
        /*
                if (active_count == 0) {
                    if (prev_active_count) {
                        restart_countdown = 255;
                    }
                    if (restart_countdown) {
                        if (0 == --restart_countdown) {
        ESP_EARLY_LOGD("DEBUG","httpServer restart");
                                httpServer.end();
                                httpServer.begin();
                        }
                    }
                }
        //*/

        uint32_t current_millis = millis();

        if (httpServer.hasClient()) {
            for (int i = 0; i < connection_size; ++i) {
                connection_index = connection_index != connection_size - 1
                                       ? connection_index + 1
                                       : 0;
                if (!connection[connection_index].connected) {
                    break;
                }
                if (!(connection[connection_index].client.connected())) {
                    // ESP_EARLY_LOGD("DEBUG", "connected -> close");
                    connection[connection_index].stop();
                    break;
                }
            }
            // ESP_EARLY_LOGD("DEBUG","connection_index: %d", connection_index);
            auto& conn = connection[connection_index];
            if (!conn.connected) {
                conn.connected = true;
                conn.client    = httpServer.available();
                snprintf(conn.boundary, sizeof(conn.boundary), "tlite");
                conn.connect_millis = current_millis;
            }
        }

        prev_active_count = active_count;
        active_count      = 0;

        for (auto& conn : connection) {
            if (!conn.connected) {
                continue;
            }
            WiFiClient* client = &(conn.client);
            ++active_count;

            if (!client->available()) {
                if (!client->connected()) {
                    // ESP_EARLY_LOGD("DEBUG", "no connected stop");
                    conn.stop();
                    // client->stop();
                    continue;
                }
                if (!conn.keep_connection &&
                    2048 < current_millis - conn.connect_millis) {
                    // ESP_EARLY_LOGD("DEBUG", "keep_connection timeout stop");
                    conn.stop();
                    // client->stop();
                    continue;
                }
            } else {
                conn.connect_millis = current_millis;
                int available_len;
                while (available_len = client->available()) {
                    char c = client->read();
                    if (c == '\r') {
                        continue;
                    }
                    if (c != '\n') {
                        // Bound each request/header line. Without a limit a
                        // client sending bytes without a newline grew this
                        // string until the heap was exhausted (abort).
                        static constexpr size_t line_max = 1024;
                        if (conn.line_buf.size() >= line_max) {
                            client->print(
                                "HTTP/1.1 431 Request Header Fields Too "
                                "Large\r\nConnection: close\r\n"
                                "Content-Length: 0\r\n\r\n");
                            conn.stop();
                            break;
                        }
                        conn.line_buf.append(1, c);
                    } else {
                        // ESP_EARLY_LOGD("DEBUG", "line_buf : %s",
                        // conn.line_buf.c_str());
                        if (conn.line_buf.empty()) {
                            if (conn.is_post) {
                                char buffer[256] = {
                                    0,
                                };
                                // memset(conn.post_data, 0,
                                // sizeof(conn.post_data));
                                int len;
                                int retry = 256;
                                do {
                                    delay(1);
                                } while (0 == (len = client->available()) &&
                                         --retry);
                                if (len > 255) len = 255;
                                if (len) {
                                    client->readBytes(buffer, len);
                                    conn.request_post = buffer;
                                }
                                // conn.post_data_len = len;
                                // ESP_EARLY_LOGD("DEBUG","POST_DATA : %s",
                                // buffer);
                            }
                            if (conn.request_path.length()) {
                                bool hit = false;
                                for (auto& res : response_table) {
                                    // ESP_EARLY_LOGD("DEBUG","cmp:%s : %s",
                                    // conn.request_path.c_str(), res.path);
                                    hit = (strcmp(conn.request_path.c_str(),
                                                  res.path) == 0);
                                    if (hit) {
                                        conn.keep_connection =
                                            res.response_func(draw_param,
                                                              &conn);
                                        break;
                                    }
                                }
                                if (!hit) {
                                    response_404(draw_param, &conn);
                                }
                                if (conn.keep_connection == false) {
                                    // ESP_EARLY_LOGD("DEBUG", "keep_connection
                                    // false stop");
                                    conn.stop();
                                    // client->stop();
                                }
                                conn.clear_request();
                                // ESP_EARLY_LOGD("DEBUG", "clear_request");
                            }
                        } else {
                            bool is_post =
                                conn.line_buf.compare(0, 6, "POST /") == 0;
                            if (is_post ||
                                conn.line_buf.compare(0, 5, "GET /") == 0) {
                                conn.is_post = is_post;
                                int pos1     = conn.line_buf.find('/');
                                int pos2     = conn.line_buf.find('?', pos1);
                                int pos3     = conn.line_buf.find(' ', pos1);
                                if (pos2 < 0) {
                                    conn.request_path =
                                        conn.line_buf.substr(pos1, pos3 - pos1)
                                            .c_str();
                                    conn.request_get = "";
                                } else {
                                    conn.request_path =
                                        conn.line_buf.substr(pos1, pos2 - pos1)
                                            .c_str();
                                    ++pos2;
                                    conn.request_get =
                                        conn.line_buf.substr(pos2, pos3 - pos2)
                                            .c_str();
                                }
                            }
                            conn.line_buf = "";
                        }
                    }
                }
            }
        }
    }
}
