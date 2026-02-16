//! Copyright (c) M5Stack. All rights reserved.
//! Licensed under the MIT license.
//! See LICENSE file in the project root for full license information.

#if defined(WIFI_DISABLED)
void webserverTask(void*) {
}
#else

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
    "<div class='ft'>Copyright &copy;2022" 
    "</div></div>\n</body></html>\n\n";

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
    std::string strbuf;
    char cbuf[64];

    strbuf = "<html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>Device Control</title><style>"
        "body{font-family:sans-serif;margin:0;padding:10px;background:#1a1a1a;color:#eee}"
        "h1{margin:0 0 20px;text-align:center;font-size:24px;color:#2eb840}"
        ".grid{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-bottom:20px}"
        ".card{background:#2a2a2a;padding:15px;border-radius:8px;border:1px solid #444}"
        ".card h3{margin:0 0 10px;font-size:14px;color:#3DA7C7}"
        "select,input[type=range]{width:100%;padding:8px;margin:5px 0;border:1px solid #555;background:#333;color:#eee;border-radius:4px}"
        ".sentry{grid-column:1/-1;background:#0a3a2a;border:2px solid #2eb840;padding:20px;text-align:center;font-size:18px}"
        ".sentry select{font-size:16px;padding:10px}"
        ".group{margin:15px 0}"
        ".group label{display:block;font-weight:bold;margin-bottom:5px;font-size:12px;color:#aaa}"
        "button{width:100%;padding:10px;margin:10px 0 0;background:#2eb840;color:#000;border:none;border-radius:4px;font-weight:bold;cursor:pointer}"
        ".info{font-size:12px;color:#999;margin-top:10px;text-align:center}"
        "</style></head><body>"
        "<h1>DEVICE CONTROL</h1>";

    // Sentry Mode - Big prominent control
    strbuf += "<div class='sentry'><b>SENTRY MODE</b><br>";
    strbuf += "<select id='misc_sentry_mode' onchange='s(\"misc_sentry_mode\",this.value)'>";
    for (int i = 0; i < draw_param->misc_sentry_mode_max; ++i) {
        strbuf.append(cbuf, snprintf(cbuf, sizeof(cbuf),
                                     "<option value=\"%d\"%s>%s</option>", i,
                                     i == draw_param->misc_sentry_mode.get() ? " selected" : "",
                                     draw_param->misc_sentry_mode.getText(i)));
    }
    strbuf += "</select><div class='group' style='margin-top:10px'><label>Interval</label>";
    strbuf += "<select id='misc_sentry_interval' onchange='s(\"misc_sentry_interval\",this.value)'>";
    for (int i = 0; i < draw_param->misc_sentry_interval_max; ++i) {
        strbuf.append(cbuf, snprintf(cbuf, sizeof(cbuf),
                                     "<option value=\"%d\"%s>%s</option>", i,
                                     i == draw_param->misc_sentry_interval.get() ? " selected" : "",
                                     draw_param->misc_sentry_interval.getText(i)));
    }
    strbuf += "</select></div></div>";

    strbuf += "<div class='grid'>";
    
    // Alarm
    strbuf += "<div class='card'><h3>ALARM</h3>";
    strbuf += "<div class='group'><label>Mode</label>";
    strbuf += "<select id='alarm_mode' onchange='s(\"alarm_mode\",this.value)'>";
    for (int i = 0; i < draw_param->alarm_mode_max; ++i) {
        strbuf.append(cbuf, snprintf(cbuf, sizeof(cbuf),
                                     "<option value=\"%d\"%s>%s</option>", i,
                                     i == draw_param->alarm_mode.get() ? " selected" : "",
                                     draw_param->alarm_mode.getText(i)));
    }
    strbuf += "</select></div>";
    strbuf += "<div class='group'><label>Reference</label>";
    strbuf += "<select id='alarm_reference' onchange='s(\"alarm_reference\",this.value)'>";
    for (int i = 0; i < draw_param->alarm_reference_max; ++i) {
        strbuf.append(cbuf, snprintf(cbuf, sizeof(cbuf),
                                     "<option value=\"%d\"%s>%s</option>", i,
                                     i == draw_param->alarm_reference.get() ? " selected" : "",
                                     draw_param->alarm_reference.getText(i)));
    }
    strbuf += "</select></div>";
    strbuf += "<div class='group'><label>";
    strbuf.append(cbuf, snprintf(cbuf, sizeof(cbuf), "Temp: %.1fC",
                                 convertRawToCelsius(draw_param->alarm_temperature)));
    strbuf += "</label>";
    strbuf.append(cbuf, snprintf(cbuf, sizeof(cbuf),
                                 "<input type='range' min='-50' max='350' step='0.5' id='alarm_temperature' value='%.1f' onchange='s(\"alarm_temperature\",this.value)'>",
                                 convertRawToCelsius(draw_param->alarm_temperature)));
    strbuf += "</div></div>";

    // Sensor
    strbuf += "<div class='card'><h3>SENSOR</h3>";
    strbuf += "<div class='group'><label>Refresh</label>";
    strbuf += "<select id='sens_refreshrate' onchange='s(\"sens_refreshrate\",this.value)'>";
    for (int i = 0; i < draw_param->sens_refreshrate_max; ++i) {
        strbuf.append(cbuf, snprintf(cbuf, sizeof(cbuf),
                                     "<option value=\"%d\"%s>%s</option>", i,
                                     i == draw_param->sens_refreshrate.get() ? " selected" : "",
                                     draw_param->sens_refreshrate.getText(i)));
    }
    strbuf += "</select></div>";
    strbuf += "<div class='group'><label>Filter</label>";
    strbuf += "<select id='sens_noisefilter' onchange='s(\"sens_noisefilter\",this.value)'>";
    for (int i = 0; i < draw_param->sens_noisefilter_max; ++i) {
        strbuf.append(cbuf, snprintf(cbuf, sizeof(cbuf),
                                     "<option value=\"%d\"%s>%s</option>", i,
                                     i == draw_param->sens_noisefilter.get() ? " selected" : "",
                                     draw_param->sens_noisefilter.getText(i)));
    }
    strbuf += "</select></div>";
    strbuf += "<div class='group'><label>Area</label>";
    strbuf += "<select id='sens_monitorarea' onchange='s(\"sens_monitorarea\",this.value)'>";
    for (int i = 0; i < draw_param->sens_monitorarea_max; ++i) {
        strbuf.append(cbuf, snprintf(cbuf, sizeof(cbuf),
                                     "<option value=\"%d\"%s>%s</option>", i,
                                     i == draw_param->sens_monitorarea.get() ? " selected" : "",
                                     draw_param->sens_monitorarea.getText(i)));
    }
    strbuf += "</select></div>";
    strbuf += "<div class='group'><label>Emissivity: ";
    strbuf.append(cbuf, snprintf(cbuf, sizeof(cbuf), "%d%%", draw_param->sens_emissivity.get()));
    strbuf += "</label>";
    strbuf.append(cbuf, snprintf(cbuf, sizeof(cbuf),
                                 "<input type='range' min='20' max='100' id='sens_emissivity' value='%d' onchange='s(\"sens_emissivity\",this.value)'>",
                                 draw_param->sens_emissivity.get()));
    strbuf += "</div></div>";

    // Range
    strbuf += "<div class='card'><h3>RANGE</h3>";
    strbuf += "<div class='group'><label>Auto</label>";
    strbuf += "<select id='range_autoswitch' onchange='s(\"range_autoswitch\",this.value)'>";
    for (int i = 0; i < draw_param->range_autoswitch_max; ++i) {
        strbuf.append(cbuf, snprintf(cbuf, sizeof(cbuf),
                                     "<option value=\"%d\"%s>%s</option>", i,
                                     i == draw_param->range_autoswitch.get() ? " selected" : "",
                                     draw_param->range_autoswitch.getText(i)));
    }
    strbuf += "</select></div>";
    strbuf += "<div class='group'><label>";
    strbuf.append(cbuf, snprintf(cbuf, sizeof(cbuf), "High: %.1fC",
                                 convertRawToCelsius(draw_param->range_temp_upper)));
    strbuf += "</label>";
    strbuf.append(cbuf, snprintf(cbuf, sizeof(cbuf),
                                 "<input type='range' min='-50' max='350' step='0.5' id='range_temp_upper' value='%.1f' onchange='s(\"range_temp_upper\",this.value)'>",
                                 convertRawToCelsius(draw_param->range_temp_upper)));
    strbuf += "</div>";
    strbuf += "<div class='group'><label>";
    strbuf.append(cbuf, snprintf(cbuf, sizeof(cbuf), "Low: %.1fC",
                                 convertRawToCelsius(draw_param->range_temp_lower)));
    strbuf += "</label>";
    strbuf.append(cbuf, snprintf(cbuf, sizeof(cbuf),
                                 "<input type='range' min='-50' max='350' step='0.5' id='range_temp_lower' value='%.1f' onchange='s(\"range_temp_lower\",this.value)'>",
                                 convertRawToCelsius(draw_param->range_temp_lower)));
    strbuf += "</div></div>";

    // Display
    strbuf += "<div class='card'><h3>DISPLAY</h3>";
    strbuf += "<div class='group'><label>Color</label>";
    strbuf += "<select id='misc_color' onchange='s(\"misc_color\",this.value)'>";
    for (int i = 0; i < color_map_table_len; ++i) {
        strbuf.append(cbuf, snprintf(cbuf, sizeof(cbuf),
                                     "<option value=\"%d\"%s>%s</option>", i,
                                     i == draw_param->misc_color.get() ? " selected" : "",
                                     draw_param->misc_color.getText(i)));
    }
    strbuf += "</select></div>";
    strbuf += "<div class='group'><label>Pointer</label>";
    strbuf += "<select id='misc_pointer' onchange='s(\"misc_pointer\",this.value)'>";
    for (int i = 0; i < draw_param->misc_pointer_max; ++i) {
        strbuf.append(cbuf, snprintf(cbuf, sizeof(cbuf),
                                     "<option value=\"%d\"%s>%s</option>", i,
                                     i == draw_param->misc_pointer.get() ? " selected" : "",
                                     draw_param->misc_pointer.getText(i)));
    }
    strbuf += "</select></div>";
    strbuf += "<div class='group'><label>Brightness</label>";
    strbuf += "<select id='misc_brightness' onchange='s(\"misc_brightness\",this.value)'>";
    for (int i = 0; i < draw_param->misc_brightness_max; ++i) {
        strbuf.append(cbuf, snprintf(cbuf, sizeof(cbuf),
                                     "<option value=\"%d\"%s>%s</option>", i,
                                     i == draw_param->misc_brightness.get() ? " selected" : "",
                                     draw_param->misc_brightness.getText(i)));
    }
    strbuf += "</select></div></div>";

    // System
    strbuf += "<div class='card'><h3>SYSTEM</h3>";
    strbuf += "<div class='group'><label>CPU Speed</label>";
    strbuf += "<select id='misc_cpuspeed' onchange='s(\"misc_cpuspeed\",this.value)'>";
    for (int i = 0; i < draw_param->misc_cpuspeed_max; ++i) {
        strbuf.append(cbuf, snprintf(cbuf, sizeof(cbuf),
                                     "<option value=\"%d\"%s>%s</option>", i,
                                     i == draw_param->misc_cpuspeed.get() ? " selected" : "",
                                     draw_param->misc_cpuspeed.getText(i)));
    }
    strbuf += "</select></div>";
    strbuf += "<div class='group'><label>Volume</label>";
    strbuf += "<select id='misc_volume' onchange='s(\"misc_volume\",this.value)'>";
    for (int i = 0; i < draw_param->misc_volume_max; ++i) {
        strbuf.append(cbuf, snprintf(cbuf, sizeof(cbuf),
                                     "<option value=\"%d\"%s>%s</option>", i,
                                     i == draw_param->misc_volume.get() ? " selected" : "",
                                     draw_param->misc_volume.getText(i)));
    }
    strbuf += "</select></div>";
    strbuf += "<div class='group'><label>Stream Quality</label>";
    strbuf += "<select id='net_jpg_quality' onchange='s(\"net_jpg_quality\",this.value)'>";
    for (int i = 1; i <= 100; i += 10) {
        strbuf.append(cbuf, snprintf(cbuf, sizeof(cbuf),
                                     "<option value=\"%d\"%s>%d%%</option>", i,
                                     i == draw_param->net_jpg_quality.get() ? " selected" : "",
                                     i));
    }
    strbuf += "</select></div></div>";

    strbuf += "</div><div class='info'>Settings auto-save | <a href='/stream' style='color:#3DA7C7;text-decoration:none'>View Stream</a></div>";
    strbuf += "<script>function s(k,v){fetch('/param?'+k+'='+v)}</script></body></html>";

    client->print(
        "HTTP/1.1 200 OK\nContent-Type: text/html\nConnection: keep-alive\n");
    client->printf("Content-Length: %d\n\n", strbuf.size());
    client->write(strbuf.c_str(), strbuf.size());
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
            } else if (key == "misc_pointer") {
                draw_param->misc_pointer.set(v);
            } else if (key == "misc_layout") {
                draw_param->misc_layout.set(v);
                draw_param->in_config_mode = false;
            } else if (key == "misc_color") {
                draw_param->misc_color.set(v);
            } else if (key == "misc_sentry_mode") {
                draw_param->misc_sentry_mode.set(v);
                config_save_countdown = 60;
            } else if (key == "misc_sentry_interval") {
                draw_param->misc_sentry_interval.set(v);
                config_save_countdown = 60;
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
                  snprintf(cbuf, sizeof(cbuf), ",\n \"misc_pointer\": \"%d\"",
                           draw_param->misc_pointer.get()));
    strbuf.append(cbuf,
                  snprintf(cbuf, sizeof(cbuf), ",\n \"misc_layout\": \"%d\"",
                           draw_param->misc_layout.get()));
    strbuf.append(cbuf,
                  snprintf(cbuf, sizeof(cbuf), ",\n \"misc_color\": \"%d\"",
                           draw_param->misc_color.get()));
    strbuf.append(cbuf,
                  snprintf(cbuf, sizeof(cbuf), ",\n \"misc_sentry_mode\": \"%d\"",
                           draw_param->misc_sentry_mode.get()));
    strbuf.append(cbuf,
                  snprintf(cbuf, sizeof(cbuf), ",\n \"misc_sentry_interval\": \"%d\"",
                           draw_param->misc_sentry_interval.get()));
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
        "<title>Device Text Info</title>\n</head>\n<body><table>\n";
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
            draw_param->net_wifi_mode = draw_param->net_wifi_mode_off;
            delay(64);
            draw_param->net_wifi_mode =
                draw_param->net_wifi_mode_connect_saved;
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
            content=\"text/html; charset=UTF-8\"><title>Device</title>\n"
                            "<style>\n body,select,button{font-size:12vw
            !important;font-size:24px;}\n"
                            ".ft{flex:0 0 auto;padding:10px
            0;text-align:center}\n" "ul,li{list-style:none;padding-left:0;}\n"
                            "a{font-size:8vw;text-align:center;display:block;background:#abc;padding:8px;margin-bottom:3px;cursor:pointer;}\n"
                            "</style></head><body><div><h4>DEVICE</h4>\n<ul>"
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
        "<title>WiFi Setup</title>\n"
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
        "</head><body><div class='ct'><h1>WIFI SETUP</h1>"
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
        "<title>Device Menu</title>\n";

    static constexpr const char html_2[] =
        "</head><body><div class='ct'><h1>DEVICE MENU</h1>"
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

    // Cloud functionality removed

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

// Sentry Mode API endpoints
extern SentryData sentry_data;

static bool response_sentry_status(draw_param_t* draw_param, connection_t* conn) {
    auto client = &conn->client;
    char cbuf[256];
    std::string response;
    
    response += HTTP_200_json;
    response += "{\n";
    response.append(cbuf, snprintf(cbuf, sizeof(cbuf), 
        "  \"sentry_mode\": %s,\n", draw_param->misc_sentry_mode.get() != draw_param_t::misc_sentry_mode_t::misc_sentry_mode_off ? "true" : "false"));
    response.append(cbuf, snprintf(cbuf, sizeof(cbuf),
        "  \"avg_temp\": %.1f,\n", sentry_data.last_avg_temp));
    response.append(cbuf, snprintf(cbuf, sizeof(cbuf),
        "  \"min_temp\": %.1f,\n", sentry_data.last_min_temp));
    response.append(cbuf, snprintf(cbuf, sizeof(cbuf),
        "  \"max_temp\": %.1f,\n", sentry_data.last_max_temp));
    response.append(cbuf, snprintf(cbuf, sizeof(cbuf),
        "  \"battery_level\": %d\n", draw_param->battery_level));
    response += "}\n";
    
    client->print(response.c_str());
    client->flush();
    
    return true;
}

static constexpr const response_table_t response_table[] = {
    {"/", response_top},        {"/main", response_main},
    {"/json", response_json},   {"/text", response_text},
    {"/wifi", response_wifi},   {"/stream", response_stream},
    {"/param", response_param}, {"/api/sentry/status", response_sentry_status},
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
                MDNS.addServiceTxt("http", "tcp", "api", "sentry");
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
#endif
