//! Copyright (c) M5Stack. All rights reserved.
//! Licensed under the MIT license.
//! See LICENSE file in the project root for full license information.

#include <WiFi.h>
#include <WiFiServer.h>
#include <ESPmDNS.h>
#include <string>

#include "screenshot_streamer.hpp"

#include "common_header.h"

static constexpr const char HTTP_200_html[] =
    "HTTP/1.1 200 OK\nContent-Type: text/html; "
    "charset=UTF-8\nX-Content-Type-Options: nosniff\nConnection: "
    "keep-alive\nCache-Control: no-cache\n\n";
static constexpr const char HTTP_200_json[] =
    "HTTP/1.1 200 OK\nContent-Type: application/json; "
    "charset=UTF-8\nX-Content-Type-Options: nosniff\nConnection: "
    "keep-alive\nCache-Control: no-cache\n\n";
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
        line_buf.clear();
        request_path.clear();
        request_post.clear();
        request_get.clear();
    }
    void stop(void) {
        client.stop();
        keep_connection = false;
        connected       = false;
        clear_request();
    }
};

static size_t decode_uri(char* dest, const char* src, size_t bufsiz) {
    int status     = 0;
    size_t current = 0;
    const char* p  = src;
    if (bufsiz == 0) return 0;
    bufsiz--;

    while (*p != 0 && current < bufsiz) {
        if (p[0] == '%' && p[1] != 0 && p[2] != 0) {
            sscanf(&p[1], "%2X", &dest[current]);
            p += 3;
        } else if (p[0] == '+') {
            dest[current] = ' ';
            p += 1;
        } else {
            dest[current] = *p++;
        }
        ++current;
    }
    dest[current] = 0;
    return current;
}

static void redirect_header(WiFiClient* client, const char* path) {
    client->printf(
        "HTTP/1.1 302 Found\nContent-Type: text/html\nContent-Length: "
        "0\nLocation: %s\n\n",
        path);
}

// return: true=keep connect
static bool response_404(draw_param_t* draw_param, connection_t* conn) {
    auto client = &conn->client;
    if (WiFi.getMode() & WIFI_AP) {
        redirect_header(client, "/wifi");
    } else {
        client->print(
            "HTTP/1.1 404 Not Found\nContent-type: text/html\n\n"
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
    char cbuf[64];

    strbuf = html_1;
    strbuf +=
        "<label for='tgl_cloud'>Cloud</label><input type='checkbox' "
        "id='tgl_cloud'>\n<ul>\n"
        "<li>Upload Interval:<select id='cloud_interval' "
        "onchange='f(\"cloud_interval=\" + "
        "this.options[this.selectedIndex].value)'>";
    for (int i = 0; i < draw_param->cloud_interval_max; ++i) {
        strbuf.append(cbuf, snprintf(cbuf, sizeof(cbuf),
                                     "<option value=\"%d\">%s</option>\n", i,
                                     draw_param->cloud_interval.getText(i)));
    }
    strbuf +=
        "</select></li>\n"
        "<li><form "
        "onsubmit='f(\"cloud_token=\"+document.getElementById(\"cloud_token\")."
        "value); return false;'>Confirm Code:<br>"
        "<input type='text' name='cloud_token' id='cloud_token' "
        "placeholder='User defined code'>\n"
        "<button type='submit'>Save</button></form></li>";
    strbuf +=
        "<li> Cloud Online URL:<br>\n<a target='_blank' rel='noreferrer' "
        "href=\"";
    strbuf += draw_param->cloud_url;
    strbuf += "\">";
    strbuf += draw_param->cloud_url;
    strbuf += "</a>\n</li></ul>\n";

    strbuf +=
        "<label for='tgl_alarm'>Alarm</label><input type='checkbox' "
        "id='tgl_alarm'>\n<ul>\n"
        " <li> Alarm Mode: <select id='alarm_mode' onchange='f(\"alarm_mode=\" "
        "+ this.options[this.selectedIndex].value)'>";
    for (int i = 0; i < draw_param->alarm_mode_max; ++i) {
        strbuf.append(cbuf, snprintf(cbuf, sizeof(cbuf),
                                     "<option value=\"%d\">%s</option>\n", i,
                                     draw_param->alarm_mode.getText(i)));
    }
    strbuf += "</select></li>\n";

    strbuf += " <li> Temperature: <span id='at'>";
    strbuf.append(cbuf,
                  snprintf(cbuf, sizeof(cbuf), "%3.1f",
                           convertRawToCelsius(draw_param->alarm_temperature)));
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
        strbuf.append(cbuf, snprintf(cbuf, sizeof(cbuf),
                                     "<option value=\"%d\">%s</option>\n", i,
                                     draw_param->alarm_reference.getText(i)));
    }
    strbuf += "</select></li>\n</ul>\n";
    strbuf +=
        "<label for='tgl_sensor'>Sensor</label><input type='checkbox' "
        "id='tgl_sensor'>\n<ul>\n"
        " <li> Refresh Rate: <select id='sens_refreshrate' "
        "onchange='f(\"sens_refreshrate=\" + "
        "this.options[this.selectedIndex].value)'>";
    for (int i = 0; i < draw_param->sens_refreshrate_max; ++i) {
        strbuf.append(cbuf, snprintf(cbuf, sizeof(cbuf),
                                     "<option value=\"%d\">%s</option>\n", i,
                                     draw_param->sens_refreshrate.getText(i)));
    }
    strbuf += "</select></li>\n";

    strbuf +=
        " <li>Noise Filter: <select id='sens_noisefilter' "
        "onchange='f(\"sens_noisefilter=\" + "
        "this.options[this.selectedIndex].value)'>";
    for (int i = 0; i < draw_param->sens_noisefilter_max; ++i) {
        strbuf.append(cbuf, snprintf(cbuf, sizeof(cbuf),
                                     "<option value=\"%d\">%s</option>\n", i,
                                     draw_param->sens_noisefilter.getText(i)));
    }
    strbuf += "</select></li>\n";

    strbuf +=
        " <li>Monitor Area: <select id='sens_monitorarea' "
        "onchange='f(\"sens_monitorarea=\" + "
        "this.options[this.selectedIndex].value)'>";
    for (int i = 0; i < draw_param->sens_monitorarea_max; ++i) {
        strbuf.append(cbuf, snprintf(cbuf, sizeof(cbuf),
                                     "<option value=\"%d\">%s</option>\n", i,
                                     draw_param->sens_monitorarea.getText(i)));
    }
    strbuf += "</select></li>\n";

    strbuf += " <li> Emissivity: <span id='em'>";
    strbuf.append(cbuf, snprintf(cbuf, sizeof(cbuf), "%d",
                                 draw_param->sens_emissivity.get()));
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
        strbuf.append(cbuf, snprintf(cbuf, sizeof(cbuf),
                                     "<option value=\"%d\">%s</option>\n", i,
                                     draw_param->range_autoswitch.getText(i)));
    }
    strbuf += "</select></li>\n";
    strbuf += " <li> Upper Temperature: <span id='rh'>";
    strbuf.append(cbuf,
                  snprintf(cbuf, sizeof(cbuf), "%3.1f",
                           convertRawToCelsius(draw_param->range_temp_upper)));
    strbuf +=
        "</span><br>\n<input width='400em' type='range' min='-50' max='350' "
        "step='0.5' id='range_temp_upper' onchange='f(\"range_temp_upper=\" "
        "+ this.value)' "
        "oninput='document.getElementById(\"rh\").innerText=this.value'></li>";
    strbuf += " <li> Lower Temperature: <span id='rl'>";
    strbuf.append(cbuf,
                  snprintf(cbuf, sizeof(cbuf), "%3.1f",
                           convertRawToCelsius(draw_param->range_temp_lower)));
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
        strbuf.append(cbuf, snprintf(cbuf, sizeof(cbuf),
                                     "<option value=\"%d\">%s</option>\n", i,
                                     draw_param->misc_cpuspeed.getText(i)));
    }
    strbuf += "</select></li>\n";

    strbuf +=
        "<li>Sound Volume: <select id='misc_volume' "
        "onchange='f(\"misc_volume=\" + "
        "this.options[this.selectedIndex].value)'>";
    for (int i = 0; i < draw_param->misc_volume_max; ++i) {
        strbuf.append(cbuf, snprintf(cbuf, sizeof(cbuf),
                                     "<option value=\"%d\">%s</option>\n", i,
                                     draw_param->misc_volume.getText(i)));
    }
    strbuf += "</select></li>\n";

    strbuf +=
        "<li>LCD Brightness: <select id='misc_brightness' "
        "onchange='f(\"misc_brightness=\" + "
        "this.options[this.selectedIndex].value)'>";
    for (int i = 0; i < draw_param->misc_brightness_max; ++i) {
        strbuf.append(cbuf, snprintf(cbuf, sizeof(cbuf),
                                     "<option value=\"%d\">%s</option>\n", i,
                                     draw_param->misc_brightness.getText(i)));
    }
    strbuf += "</select></li>\n";

    strbuf +=
        "<li>Language: <select id='misc_language' "
        "onchange='f(\"misc_language=\" + "
        "this.options[this.selectedIndex].value)'>";
    for (int i = 0; i < draw_param->misc_language_max; ++i) {
        strbuf.append(cbuf, snprintf(cbuf, sizeof(cbuf),
                                     "<option value=\"%d\">%s</option>\n", i,
                                     draw_param->misc_language_text[i]));
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
        strbuf.append(cbuf, snprintf(cbuf, sizeof(cbuf),
                                     "<option value=\"%d\">%s</option>\n", i,
                                     draw_param->misc_pointer.getText(i)));
    }
    strbuf += "</select></li>\n";

    strbuf +=
        "<li> Color:<select id='misc_color' onchange='f(\"misc_color=\" + "
        "this.options[this.selectedIndex].value)'>";
    for (int i = 0; i < color_map_table_len; ++i) {
        strbuf.append(cbuf, snprintf(cbuf, sizeof(cbuf),
                                     "<option value=\"%d\">%s</option>\n", i,
                                     draw_param->misc_color.getText(i)));
    }
    strbuf += "</select></li>\n";

    strbuf +=
        "<li>Layout:<select id='misc_layout' onchange='f(\"misc_layout=\" + "
        "this.options[this.selectedIndex].value)'>";
    for (int i = 0; i < 6; ++i) {
        strbuf.append(
            cbuf, snprintf(cbuf, sizeof(cbuf),
                           "<option value=\"%d\">layout %d</option>\n", i, i));
    }
    strbuf += "</select></li>\n</ul>\n";
    strbuf += HTML_footer;

    // client->print(HTTP_200_html);
    client->print(
        "HTTP/1.1 200 OK\nContent-Type: text/html; "
        "charset=UTF-8\nX-Content-Type-Options: nosniff\nConnection: "
        "keep-alive\nCache-Control: no-cache\n");
    client->printf("Content-Length: %d\n\n", strbuf.size());
    client->write(strbuf.c_str(), strbuf.size());
    client->print("\n");
    return true;
}

static bool response_param(draw_param_t* draw_param, connection_t* conn) {
    auto client = &conn->client;
    int pos     = conn->request_get.find('=');
    if (pos >= 0) {
        auto key = conn->request_get.substr(0, pos);
        ++pos;
        auto val = conn->request_get.substr(pos);

        if (key == "alarm_temperature") {
            draw_param->alarm_temperature =
                convertCelsiusToRaw(atof(val.c_str()));
        } else if (key == "range_temp_upper") {
            draw_param->range_temp_upper =
                convertCelsiusToRaw(atof(val.c_str()));
        } else if (key == "range_temp_lower") {
            draw_param->range_temp_lower =
                convertCelsiusToRaw(atof(val.c_str()));
        } else if (key == "cloud_token") {
            draw_param->cloud_token = val.c_str();
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
            // else if (key == "cloud_upload"      ) { draw_param->cloud_upload
            // .set(v); }
            else if (key == "cloud_interval") {
                draw_param->cloud_interval.set(v);
            }
        }
        // draw_param->saveNvs();
    }

    std::string strbuf;
    char cbuf[64];
    strbuf.append(
        cbuf,
        snprintf(cbuf, sizeof(cbuf), "{\n \"alarm_temperature\": \"%3.1f\"",
                 convertRawToCelsius(draw_param->alarm_temperature)));
    strbuf.append(cbuf,
                  snprintf(cbuf, sizeof(cbuf), ",\n \"alarm_mode\": \"%d\"",
                           draw_param->alarm_mode.get()));
    strbuf.append(
        cbuf, snprintf(cbuf, sizeof(cbuf), ",\n \"alarm_reference\": \"%d\"",
                       draw_param->alarm_reference.get()));
    strbuf.append(
        cbuf, snprintf(cbuf, sizeof(cbuf), ",\n \"sens_refreshrate\": \"%d\"",
                       draw_param->sens_refreshrate.get()));
    strbuf.append(
        cbuf, snprintf(cbuf, sizeof(cbuf), ",\n \"sens_noisefilter\": \"%d\"",
                       draw_param->sens_noisefilter.get()));
    strbuf.append(
        cbuf, snprintf(cbuf, sizeof(cbuf), ",\n \"sens_monitorarea\": \"%d\"",
                       draw_param->sens_monitorarea.get()));
    strbuf.append(
        cbuf, snprintf(cbuf, sizeof(cbuf), ",\n \"sens_emissivity\": \"%d\"",
                       draw_param->sens_emissivity.get()));
    strbuf.append(
        cbuf, snprintf(cbuf, sizeof(cbuf), ",\n \"range_autoswitch\": \"%d\"",
                       draw_param->range_autoswitch.get()));
    strbuf.append(
        cbuf,
        snprintf(cbuf, sizeof(cbuf), ",\n \"range_temp_upper\": \"%3.1f\"",
                 convertRawToCelsius(draw_param->range_temp_upper)));
    strbuf.append(
        cbuf,
        snprintf(cbuf, sizeof(cbuf), ",\n \"range_temp_lower\": \"%3.1f\"",
                 convertRawToCelsius(draw_param->range_temp_lower)));
    strbuf.append(
        cbuf, snprintf(cbuf, sizeof(cbuf), ",\n \"net_jpg_quality\": \"%d\"",
                       draw_param->net_jpg_quality.get()));
    strbuf.append(cbuf,
                  snprintf(cbuf, sizeof(cbuf), ",\n \"misc_cpuspeed\": \"%d\"",
                           draw_param->misc_cpuspeed.get()));
    strbuf.append(cbuf,
                  snprintf(cbuf, sizeof(cbuf), ",\n \"misc_volume\": \"%d\"",
                           draw_param->misc_volume.get()));
    strbuf.append(
        cbuf, snprintf(cbuf, sizeof(cbuf), ",\n \"misc_brightness\": \"%d\"",
                       draw_param->misc_brightness.get()));
    strbuf.append(cbuf,
                  snprintf(cbuf, sizeof(cbuf), ",\n \"misc_language\": \"%d\"",
                           draw_param->misc_language.get()));
    strbuf.append(cbuf,
                  snprintf(cbuf, sizeof(cbuf), ",\n \"misc_pointer\": \"%d\"",
                           draw_param->misc_pointer.get()));
    strbuf.append(cbuf,
                  snprintf(cbuf, sizeof(cbuf), ",\n \"misc_layout\": \"%d\"",
                           draw_param->misc_layout.get()));
    strbuf.append(cbuf,
                  snprintf(cbuf, sizeof(cbuf), ",\n \"misc_color\": \"%d\"",
                           draw_param->misc_color.get()));
    strbuf.append(cbuf,
                  snprintf(cbuf, sizeof(cbuf), ",\n \"cloud_interval\": \"%d\"",
                           draw_param->cloud_interval.get()));
    strbuf.append(cbuf,
                  snprintf(cbuf, sizeof(cbuf), ",\n \"cloud_token\": \"%s\"",
                           draw_param->cloud_token.c_str()));
    strbuf += "\n}\n\n";

    client->print(
        "HTTP/1.1 200 OK\nContent-Type: application/json; "
        "charset=UTF-8\nX-Content-Type-Options: nosniff\nConnection: "
        "keep-alive\nCache-Control: no-cache\n");
    client->printf("Content-Length: %d\n\n", strbuf.size());
    client->write(strbuf.c_str(), strbuf.size());
    client->print("\n");
    return true;
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
        "HTTP/1.1 200 OK\nContent-Type: application/json; "
        "charset=UTF-8\nX-Content-Type-Options: nosniff\nConnection: "
        "keep-alive\nCache-Control: no-cache\n");
    client->printf("Content-Length: %d\n\n", strbuf.size());
    client->write(strbuf.c_str(), strbuf.size());
    client->print("\n");
    return true;
}

static bool response_text(draw_param_t* draw_param, connection_t* conn) {
    auto client = &conn->client;
    auto t      = time(nullptr);
    auto gmt    = gmtime(&t);

    static constexpr const char html_1[] =
        "<!DOCTYPE html><html lang=\"en\"><head><meta charset=\"utf-8\">\n"
        "<meta name=\"viewport\" content=\"width=device-width, "
        "initial-scale=1.0\">\n"
        "<meta http-equiv=\"refresh\" content=\"1; URL=\">\n"
        "<title>T-Lite Text Info</title>\n</head>\n<body><table>\n";
    char cbuf[128];

    std::string strbuf = html_1;
    strbuf.append(cbuf,
                  snprintf(cbuf, sizeof(cbuf),
                           "<tr><th>date </th><td>%s, %d %s %04d</td></tr>",
                           wday_tbl[gmt->tm_wday], gmt->tm_mday,
                           mon_tbl[gmt->tm_mon], gmt->tm_year + 1900));
    strbuf.append(cbuf,
                  snprintf(cbuf, sizeof(cbuf),
                           "<tr><th>time </th><td>%02d:%02d:%02d GMT</td></tr>",
                           gmt->tm_hour, gmt->tm_min, gmt->tm_sec));
    strbuf.append(
        cbuf,
        snprintf(
            cbuf, sizeof(cbuf), "<tr><th>center </th><td>%3.1f</td></tr>\n",
            convertRawToCelsius(draw_param->frame->temp[framedata_t::center])));
    strbuf.append(cbuf,
                  snprintf(cbuf, sizeof(cbuf),
                           "<tr><th>highest</th><td>%3.1f</td></tr>\n",
                           convertRawToCelsius(
                               draw_param->frame->temp[framedata_t::highest])));
    strbuf.append(cbuf,
                  snprintf(cbuf, sizeof(cbuf),
                           "<tr><th>average</th><td>%3.1f</td></tr>\n",
                           convertRawToCelsius(
                               draw_param->frame->temp[framedata_t::average])));
    strbuf.append(
        cbuf,
        snprintf(
            cbuf, sizeof(cbuf), "<tr><th>lowest </th><td>%3.1f</td></tr>\n",
            convertRawToCelsius(draw_param->frame->temp[framedata_t::lowest])));
    strbuf += "</table></body></html>\n\n";

    client->print(
        "HTTP/1.1 200 OK\nContent-Type: text/html; "
        "charset=UTF-8\nX-Content-Type-Options: nosniff\nConnection: "
        "keep-alive\nCache-Control: no-cache\n");
    client->printf("Content-Length: %d\n\n", strbuf.size() - 1);
    client->write(strbuf.c_str(), strbuf.size());

    return true;
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

            char buf[64];
            decode_uri(buf, val.c_str(), sizeof(buf));
            if (key == "s") {
                ssid = buf;
            } else if (key == "p") {
                password = buf;
            }
        } while (!end);
        redirect_header(client, "/wifi");

        if (ssid.length()) {
            draw_param->sys_ssid         = ssid;
            draw_param->net_tmp_ssid     = ssid;
            draw_param->net_tmp_pwd      = password;
            draw_param->net_running_mode = draw_param->net_running_mode_offline;
            delay(64);
            draw_param->net_running_mode =
                draw_param->net_running_mode_lan_cloud;
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
        }
        return false;
    }

    static constexpr const char html_1[] =
        "<html><head><meta http-equiv=\"Content-Type\" content=\"text/html; "
        "charset=UTF-8\">\n"
        "<meta name=\"viewport\" content=\"width=device-width, "
        "initial-scale=1.0\">\n"
        "<title>T-Lite WiFi setup</title>\n"
        "<script>function s(a){var l=document.querySelectorAll('.list "
        "a');for(let i=0;i<l.length;i++){"
        "if(a===l[i]){a.classList.add('active')}else{l[i].classList.remove('"
        "active')}}"
        "document.getElementById('s').value=a.innerText||a.textContent;"
        "document.getElementById('p').focus();};"
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
    client->print(HTML_style);
    client->print(html_2);
    if (!draw_param->sys_ssid.empty()) {
        client->print("<div class='ls'><h2>Current SSID</h2>");
        client->printf(
            "<a href='javascript:void(0);' onclick='s(this)'> %s </a>",
            draw_param->sys_ssid.c_str());
        client->print("</div><hr>");
    }

    client->print(html_3);
    int i     = 0;
    int count = WiFi.scanComplete();
    for (int i = 0; i < count; ++i) {
        auto ssid = WiFi.SSID(i);
        client->printf(
            "<a href='javascript:void(0);' onclick='s(this)'> %s </a>",
            ssid.c_str());
        // WiFi.encryptionType(i) == WIFI_AUTH_OPEN ?
        // WiFi.RSSI(i) + "dBm";
    }
    client->print(html_4);
    client->print(HTML_footer);

    if (count != -1) {
        WiFi.scanNetworks(true);
    }

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
        "<div class='main'><div class='ls'><h2>Cloud</h2>";

    static constexpr const char html_3[] =
        "</div><hr><div class='ls'><h2>LAN</h2>";

    static constexpr const char html_4[] =
        "<a href=\"/main\">Browser control</a>\n"
        "<a href=\"/text\">Text infomation</a>\n"
        "<a href=\"/json\">JSON data</a>\n"
        "<a href=\"/stream\">Stream Image</a>\n"
        "</div></div>\n";

    client->print(HTTP_200_html);
    client->print(html_1);
    client->print(HTML_style);
    client->print(html_2);

    client->printf(
        "<a target='_blank' rel='noreferrer' href='%s'>Image "
        "(ConfirmCode:%s)</a>",
        draw_param->cloud_url.c_str(), draw_param->cloud_token.c_str());
    // if (WiFi.getMode() & WIFI_AP) {
    //     client->print(html_3);
    // }
    client->print(html_3);

    if (WiFi.getMode() & WIFI_AP) {
        client->print("<a href=\"/wifi\">WiFi setting</a>\n");
    }

    client->print(html_4);

    client->print(HTML_footer);
    return false;
}

static bool response_test(draw_param_t* draw_param, connection_t* conn) {
    static constexpr const char head[] =
        "HTTP/1.1 200 OK\n"
        "Content-Type: text/html\n"
        "Content-Length: 16\n"
        "Connection: keep-alive\n"
        "Cache-Control: no-store\n"
        "\n";
    auto client = &conn->client;

    client->print(head);
    client->print("0123456789abcdef\n\n\n");
    client->flush();
    return true;

    /*/
    static constexpr const char html[] = "HTTP/1.1 200 OK\nContent-Type:
    text/html\nConnection:close\n\n"
    "<!DOCTYPE HTML><html><head><meta charset=\"utf-8\">"
    "<meta name=\"viewport\" content=\"width=device-width,
    initial-scale=1\"><style>" "  html { font-family: Helvetica; display:
    inline-block; margin: 0px auto;text-align: center;} " "  h1
    {font-size:28px;} " " .btn_on { padding:12px 30px; text-decoration:none;
    font-size:24px; background-color: " "  #668ad8; color: #FFF; border-bottom:
    solid 4px #627295; border-radius: 2px;} " "      .btn_on:active {
    -webkit-transform: translateY(0px); transform: translateY(0px); " "
    border-bottom: none;} " "      .btn_off { background-color: #555555;
    border-bottom: solid 4px #333333;} " "      .slider { width: 200px;} " "
    </style><script
    src=\"https://ajax.googleapis.com/ajax/libs/jquery/3.4.1/jquery.min.js\"></script></head>"
    " <body><h1>M5TLite</h1> ";
                                client->print(html);
                                client->printf("<p>Brightness (<span
    id=\"emissivityValue\"></span>)</p>"
                                            "<input type=\"range\" min=\"5\"
    max=\"100\" step=\"1\" class=\"slider\" id=\"emissivityInput\"
    onchange=\"valueFunction(this.value)\" value=\"%d\" />",
    draw_param->perf_emissivity); client->print("<script> var obj =
    document.getElementById(\"emissivityInput\");" "var target =
    document.getElementById(\"emissivityValue\");" "target.innerHTML =
    obj.value;" "obj.oninput = function() { obj.value = this.value;
    target.innerHTML = this.value; } \n" " function valueFunction(val) {
    $.get(\"/?value=\" + val + '&'); { Connection: close}; }"
        "</script></body></html>");
    return false;
    //*/
}

struct response_table_t {
    const char* path;
    bool (*response_func)(draw_param_t*, connection_t*);
};

static constexpr const response_table_t response_table[] = {
    {"/", response_top},        {"/main", response_main},
    {"/json", response_json},   {"/text", response_text},
    {"/wifi", response_wifi},   {"/stream", response_stream},
    {"/param", response_param},
    // { "/test"   , response_test },
};

void webserverTask(void* arg) {
    auto draw_param = (draw_param_t*)arg;

    WiFiServer httpServer(80, 4);

    static constexpr const size_t connection_size = 8;
    connection_t connection[connection_size];
    uint8_t connection_index  = 0;
    uint32_t conn_idx         = 0;
    bool prev_connected       = false;
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
                MDNS.begin(draw_param->net_apmode_ssid);
                MDNS.addService("http", "tcp", 80);
                // ESP_EARLY_LOGD("DEBUG","httpServer begin");
            } else {
                MDNS.end();
                for (auto& conn : connection) {
                    conn.stop();
                }
                httpServer.end();
                // ESP_EARLY_LOGD("DEBUG","httpServer end");
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
