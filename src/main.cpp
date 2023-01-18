//! Copyright (c) M5Stack. All rights reserved.
//! Licensed under the MIT license.
//! See LICENSE file in the project root for full license information.

#include "command_processor.hpp"
#include <esp_log.h>
#include <esp_sntp.h>
#include <esp_wifi.h>
#include <driver/gpio.h>

#include <vector>

#include <Preferences.h>
#include <WiFiServer.h>
#include <DNSServer.h>

#include <ArduinoJson.hpp>  // https://github.com/bblanchon/ArduinoJson/
#include <M5Unified.hpp>    // https://github.com/m5stack/M5Unified/
#include <lgfx/utility/lgfx_qrcode.h>

#include "common_header.h"
#include "screenshot_streamer.hpp"
#include "jpg/jpge.h"

#include "resource/bmp_logo.h"
#include "resource/jpg_staff.h"
#include "resource/wav_enter.h"

DNSServer dnsServer;

extern const unsigned char gWav_Click[112];

extern void webserverTask(void*);

auto& display = M5.Display;
// M5BurnerNVS burner_nvs;
screenshot_streamer_t screenshot_holder;
draw_param_t draw_param;

static constexpr const char* cloud_server_name = "ezdata.m5stack.com";

static constexpr const char* ntp_server[] = {"0.pool.ntp.org", "1.pool.ntp.org",
                                             "2.pool.ntp.org"};
volatile bool need_wifi_reconnect         = false;

extern "C" {
void esp_timer_impl_update_apb_freq(
    uint32_t apb_ticks_per_us);  // private in IDF
}

static constexpr const uint16_t label_ui_text_color = 0x8610u;

static constexpr const int32_t header_ui_height = 24;
static constexpr const int32_t battery_ui_width = 4;
static constexpr const uint8_t mlx_width        = 16;
static constexpr const uint8_t mlx_height       = 24;

static constexpr const char* graph_text_table[] = {"Cntr", "High", "Avrg",
                                                   "Low"};

static constexpr const uint32_t graph_color_table[] = {
    0x00909090u, 0x00FFCF00u, 0x0000A000u, 0x0000CFFFu};

static constexpr const int step_table[] = {1, 2, 5, 10, 20, 50, 100, 200};
static constexpr const size_t step_table_len =
    sizeof(step_table) / sizeof(step_table[0]);

const int32_t raw_step_offset = convertCelsiusToRaw(0.0f) - 128 * 1000;
// volatile size_t color_map_table_idx = 0;

static constexpr size_t framedata_len = 6;
framedata_t framedata[framedata_len];

volatile int idx_recv = -1;

static int smooth_move(int dst, int src) {
    return (dst == src) ? dst : ((dst + src + (src < dst ? 1 : 0)) >> 1);
    // return (dst == src) ? dst : ((dst * 3 + src * 5 + (src < dst ? 5 : 0)) >>
    // 3);
}

static void soundStartUp(void) {
    if (draw_param.misc_volume != draw_param.misc_volume_t::misc_volume_mute) {
        M5.Speaker.playRaw(wav_enter, sizeof(wav_enter), 48000);
        //     M5.Speaker.playRaw(wav_startup, sizeof(wav_startup), 24000);
    }
}
static void soundEnter(void) {
    if (draw_param.misc_volume != draw_param.misc_volume_t::misc_volume_mute) {
        M5.Speaker.playRaw(wav_enter, sizeof(wav_enter), 48000);
    }
}
static void soundExit(void) {
    if (draw_param.misc_volume != draw_param.misc_volume_t::misc_volume_mute) {
        M5.Speaker.playRaw(wav_enter, sizeof(wav_enter), 36000);
    }
}
static void soundMoveCursor(void) {
    if (draw_param.misc_volume != draw_param.misc_volume_t::misc_volume_mute) {
        M5.Speaker.playRaw(wav_enter, sizeof(wav_enter), 42000);
        // M5.Speaker.tone(4400, 80);
    }
}
static void soundValueChange(void) {
    if (draw_param.misc_volume != draw_param.misc_volume_t::misc_volume_mute) {
        M5.Speaker.playRaw(wav_enter, sizeof(wav_enter), 48000, false, 1, 0,
                           true);
        // M5.Speaker.tone(2200, 50, 0);
    }
}
static void soundOperate(void) {
    if (draw_param.misc_volume != draw_param.misc_volume_t::misc_volume_mute) {
        M5.Speaker.playRaw(wav_enter, sizeof(wav_enter), 48000);
        // M5.Speaker.playRaw(wav_confirm, sizeof(wav_confirm), 48000);
    }
}
static void soundUSBConnected(void) {
    if (draw_param.misc_volume != draw_param.misc_volume_t::misc_volume_mute) {
        M5.Speaker.tone(1046.502f, 64, 0, false);  // C6
        M5.Speaker.tone(880.000f, 64, 0, false);   // A5
        M5.Speaker.tone(1318.510f, 64, 0, false);  // E6
    }
}
static void soundUSBDisconnected(void) {
    if (draw_param.misc_volume != draw_param.misc_volume_t::misc_volume_mute) {
        M5.Speaker.tone(880.000f, 64, 0, false);  // A5
        M5.Speaker.tone(783.991f, 64, 0, false);  // G5
        M5.Speaker.tone(659.255f, 64, 0, false);  // E5
    }
}
static void soundWiFiConnected(void) {
    if (draw_param.misc_volume != draw_param.misc_volume_t::misc_volume_mute) {
        // M5.Speaker.playRaw(wav_connect, sizeof(wav_connect), 24000);
        M5.Speaker.tone(1046.502f, 64, 0, false);  // C6
        M5.Speaker.tone(1567.982f, 64, 0, false);  // G6
    }
}
static void soundWiFiDisconnected(void) {
    if (draw_param.misc_volume != draw_param.misc_volume_t::misc_volume_mute) {
        M5.Speaker.tone(1760.000f, 64, 0, false);  // A6
        M5.Speaker.tone(1174.659f, 64, 0, false);  // D6
    }
}
static void soundCloudSuccess(void) {
    if (draw_param.misc_volume != draw_param.misc_volume_t::misc_volume_mute) {
        // M5.Speaker.playRaw(wav_connect, sizeof(wav_connect), 24000);
        M5.Speaker.tone(2093.005f, 48, 0, false);  // C7
        M5.Speaker.tone(2637.020f, 48, 0, false);  // E7
        M5.Speaker.tone(3135.963f, 48, 0, false);  // G7
    }
}
static void soundCloudError(void) {
    if (draw_param.misc_volume != draw_param.misc_volume_t::misc_volume_mute) {
        M5.Speaker.tone(3520.000f, 48, 0, false);  // A7
        M5.Speaker.tone(2793.826f, 48, 0, false);  // F7
        M5.Speaker.tone(2349.318f, 48, 0, false);  // D7
    }
}

struct rect_t {
    int16_t x;
    int16_t y;
    int16_t w;
    int16_t h;
    constexpr rect_t(void) : x{0}, y{0}, w{0}, h{0} {
    }
    constexpr rect_t(int x_, int y_, int w_, int h_)
        : x{(int16_t)x_}, y{(int16_t)y_}, w{(int16_t)w_}, h{(int16_t)h_} {
    }
    inline constexpr int top(void) const {
        return y;
    }
    inline constexpr int left(void) const {
        return x;
    }
    inline constexpr int right(void) const {
        return x + w;
    }
    inline constexpr int bottom(void) const {
        return y + h;
    }
    inline constexpr bool empty(void) const {
        return w <= 0 || h <= 0;
    }

    bool smooth_move(const rect_t& src) {
        if (operator==(src)) return false;
        int new_b = ::smooth_move(src.bottom(), bottom());
        int new_y = ::smooth_move(src.y, y);
        h         = new_b - new_y;
        y         = new_y;
        int new_r = ::smooth_move(src.right(), right());
        int new_x = ::smooth_move(src.x, x);
        w         = new_r - new_x;
        x         = new_x;
        return true;
    }
    inline bool operator==(const rect_t& src) const {
        return x == src.x && y == src.y && w == src.w && h == src.h;
    }
    inline bool operator!=(const rect_t& src) const {
        return !operator==(src);
    }
};

static constexpr const char NVS_NAMESPACE[]         = "__tlite_nvs__";
static constexpr const char KEY_ALARM_TEMPERATURE[] = "alm_temp";
static constexpr const char KEY_ALARM_REFERENCE[]   = "alm_ref";
static constexpr const char KEY_ALARM_MODE[]        = "alm_mode";
// static constexpr const char KEY_ALARM_BEHAVIOR[]    = "alm_behavior";
static constexpr const char KEY_SENS_REFRESHRATE[] = "refreshrate";
static constexpr const char KEY_SENS_NOISEFILTER[] = "noisefilter";
static constexpr const char KEY_SENS_MONITORAREA[] = "monitorarea";
static constexpr const char KEY_SENS_EMISSIVITY[]  = "emissivity";
static constexpr const char KEY_RANGE_AUTOSWITCH[] = "range_auto";
static constexpr const char KEY_RANGE_UPPER[]      = "range_upper";
static constexpr const char KEY_RANGE_LOWER[]      = "range_lower";
static constexpr const char KEY_NET_RUNNING_MODE[] = "net_running";
static constexpr const char KEY_NET_JPGQUALITY[]   = "jpg_quality";
static constexpr const char KEY_CLOUD_UPLOAD[]     = "upload_ena";
static constexpr const char KEY_CLOUD_INTERVAL[]   = "upload_int";
static constexpr const char KEY_CLOUD_TOKEN[]      = "ezdata_token";
static constexpr const char KEY_NET_TIMEZONE[]     = "timezone";
static constexpr const char KEY_MISC_CPUSPEED[]    = "cpuspeed";
static constexpr const char KEY_MISC_BRIGHTNESS[]  = "brightness";
static constexpr const char KEY_MISC_VOLUME[]      = "volume";
static constexpr const char KEY_MISC_LANGUAGE[]    = "language";
static constexpr const char KEY_MISC_LAYOUT[]      = "layout";
static constexpr const char KEY_MISC_COLOR[]       = "color";
static constexpr const char KEY_MISC_POINTER[]     = "pointer";
// static constexpr const char KEY_MISC_ROTATION[]     = "msc_rotation";

std::string convert(const std::string& src) {
    std::string res;
    int i = src.length();
    for (auto c : src) {
        i = (i + 1) & 15;
        c = c ^ i;
        res.append(1, c);
    }
    return res;
}

void config_param_t::saveNvs(void) {
    ESP_LOGD("DEBUG", "saveNvs in");
    /// ToDo:FlashROMのキャッシュが無効化するため排他制御が必要
    Preferences pref;
    bool exist = pref.begin(NVS_NAMESPACE, false);
    pref.putUShort(KEY_ALARM_TEMPERATURE, alarm_temperature.get());
    pref.putUChar(KEY_ALARM_REFERENCE, alarm_reference);
    pref.putUChar(KEY_ALARM_MODE, alarm_mode);
    // pref.putUChar( KEY_ALARM_BEHAVIOR   , alarm_behavior       );
    pref.putUChar(KEY_MISC_CPUSPEED, misc_cpuspeed);
    pref.putUChar(KEY_SENS_REFRESHRATE, sens_refreshrate);
    pref.putUChar(KEY_SENS_NOISEFILTER, sens_noisefilter);
    pref.putUChar(KEY_SENS_MONITORAREA, sens_monitorarea);
    pref.putUChar(KEY_SENS_EMISSIVITY, sens_emissivity);
    pref.putUChar(KEY_RANGE_AUTOSWITCH, range_autoswitch);
    pref.putUShort(KEY_RANGE_UPPER, range_temp_upper);
    pref.putUShort(KEY_RANGE_LOWER, range_temp_lower);
    pref.putUChar(KEY_NET_RUNNING_MODE, net_running_mode);
    // pref.putUChar( KEY_NET_SETUP_MODE   , net_setup_mode       );
    // pref.putBool(  KEY_NET_WEBSERVER    , net_webserver        );
    pref.putUChar(KEY_NET_JPGQUALITY, net_jpg_quality);
    pref.putInt(KEY_NET_TIMEZONE, oncloud_timezone_sec);
    // pref.putBool(  KEY_CLOUD_UPLOAD     , cloud_upload         );
    pref.putUChar(KEY_CLOUD_INTERVAL, cloud_interval);
    pref.putUChar(KEY_MISC_BRIGHTNESS, misc_brightness);
    pref.putUChar(KEY_MISC_VOLUME, misc_volume);
    pref.putUChar(KEY_MISC_LANGUAGE, misc_language);
    pref.putUChar(KEY_MISC_POINTER, misc_pointer);
    pref.putUChar(KEY_MISC_LAYOUT, misc_layout);
    pref.putUChar(KEY_MISC_COLOR, misc_color);
    pref.putString(KEY_CLOUD_TOKEN, cloud_token.c_str());
    // pref.putString(KEY_NET_SSID         , net_ssid.c_str()     );
    // pref.putString(KEY_NET_PWD          , convert(net_pwd).c_str());
    // pref.putUChar( KEY_MISC_ROTATION    , misc_rotation        );
    pref.end();
    ESP_LOGD("DEBUG", "saveNvs out");
}

void config_param_t::loadNvs(void) {
    loadDefault();

    Preferences pref;
    if (pref.begin(NVS_NAMESPACE, true)) {
        alarm_temperature =
            pref.getUShort(KEY_ALARM_TEMPERATURE, alarm_temperature);
        alarm_reference = (alarm_reference_t)pref.getUChar(KEY_ALARM_REFERENCE,
                                                           alarm_reference);
        alarm_mode = (alarm_mode_t)pref.getUChar(KEY_ALARM_MODE, alarm_mode);
        // alarm_behavior       = (alarm_behavior_t)  pref.getUChar(
        // KEY_ALARM_BEHAVIOR   , alarm_behavior       );
        sens_refreshrate = (sens_refreshrate_t)pref.getUChar(
            KEY_SENS_REFRESHRATE, sens_refreshrate);
        sens_noisefilter = (sens_noisefilter_t)pref.getUChar(
            KEY_SENS_NOISEFILTER, sens_noisefilter);
        sens_monitorarea = (sens_monitorarea_t)pref.getUChar(
            KEY_SENS_MONITORAREA, sens_monitorarea);
        sens_emissivity  = pref.getUChar(KEY_SENS_EMISSIVITY, sens_emissivity);
        range_autoswitch = (range_autoswitch_t)pref.getUChar(
            KEY_RANGE_AUTOSWITCH, range_autoswitch);
        range_temp_upper = pref.getUShort(KEY_RANGE_UPPER, range_temp_upper);
        range_temp_lower = pref.getUShort(KEY_RANGE_LOWER, range_temp_lower);
        net_running_mode = (net_running_mode_t)pref.getUChar(
            KEY_NET_RUNNING_MODE, net_running_mode);
        // net_setup_mode       = (net_setup_mode_t)  pref.getUChar(
        // KEY_NET_SETUP_MODE   , net_setup_mode       ); net_webserver        =
        // pref.getBool(  KEY_NET_WEBSERVER    , net_webserver        );
        net_jpg_quality = pref.getUChar(KEY_NET_JPGQUALITY, net_jpg_quality);
        oncloud_timezone_sec =
            pref.getInt(KEY_NET_TIMEZONE, oncloud_timezone_sec);
        misc_cpuspeed =
            (misc_cpuspeed_t)pref.getUChar(KEY_MISC_CPUSPEED, misc_cpuspeed);
        misc_brightness = (misc_brightness_t)pref.getUChar(KEY_MISC_BRIGHTNESS,
                                                           misc_brightness);
        misc_volume =
            (misc_volume_t)pref.getUChar(KEY_MISC_VOLUME, misc_volume);
        misc_language =
            (misc_language_t)pref.getUChar(KEY_MISC_LANGUAGE, misc_language);
        misc_pointer =
            (misc_pointer_t)pref.getUChar(KEY_MISC_POINTER, misc_pointer);
        misc_layout = pref.getUChar(KEY_MISC_LAYOUT, misc_layout);
        misc_color  = (misc_color_t)pref.getUChar(KEY_MISC_COLOR, misc_color);
        // cloud_upload         = (cloud_upload_t)    pref.getUChar(
        // KEY_CLOUD_UPLOAD     , cloud_upload         );
        cloud_interval =
            (cloud_interval_t)pref.getUChar(KEY_CLOUD_INTERVAL, cloud_interval);
        cloud_token =
            pref.getString(KEY_CLOUD_TOKEN, cloud_token.c_str()).c_str();
        // net_ssid             = pref.getString(KEY_NET_SSID         ,
        // net_ssid.c_str()     ).c_str(); net_pwd              = convert(
        // pref.getString(KEY_NET_PWD          ,
        // convert(net_pwd).c_str()).c_str()).c_str(); misc_rotation        =
        // pref.getUChar( KEY_MISC_ROTATION    , misc_rotation        );
        pref.end();
    }

    config_save_countdown = 0;
}

void config_param_t::loadDefault(void) {
    // net_ssid.clear();
    // net_pwd.clear();
    net_running_mode  = net_running_mode_t::net_running_mode_offline;
    net_setup_mode    = net_setup_mode_t ::net_setup_mode_off;
    alarm_temperature = (100 + 64) * 128;
    alarm_mode        = alarm_mode_t ::alarm_mode_hightemp;
    alarm_reference   = alarm_reference_t ::alarm_reference_highest;
    // alarm_behavior    = alarm_behavior_t  ::alarm_behavior_beep_led ;
    sens_refreshrate = sens_refreshrate_t::sens_refreshrate_16;
    sens_noisefilter = sens_noisefilter_t::sens_noisefilter_medium;
    sens_monitorarea = sens_monitorarea_t::sens_monitorarea_30x24;
    sens_emissivity  = 98;
    range_autoswitch = range_autoswitch_t::range_autoswitch_on;
    range_temp_upper = (40 + 64) * 128;
    range_temp_lower = (20 + 64) * 128;
    misc_brightness  = misc_brightness_t ::misc_brightness_middle;
    misc_cpuspeed    = misc_cpuspeed_t ::misc_cpuspeed_160;
    misc_language    = misc_language_t ::misc_language_en;
    net_jpg_quality  = 60;
    cloud_interval   = cloud_interval_t ::cloud_interval_30sec;
    misc_layout      = 0;
    misc_color.setDefault();
    misc_pointer = misc_pointer_t ::misc_pointer_pointtxt;
    misc_volume  = misc_volume_t ::misc_volume_normal;
}

void config_param_t::setEmissivity(uint8_t emissivity) {
    sens_emissivity = emissivity;
    command_processor::setEmissivity(emissivity);
}

void config_param_t::setTimeZoneSec(int32_t sec) {
    oncloud_timezone_sec = sec;
    configTime(sec, 0, ntp_server[0], ntp_server[1], ntp_server[2]);
}

void config_param_t::referenceModeChange(int add) {
    alarm_reference.add(add);
    /*
        if (add) {
            int i = alarm_reference + add;
            if (i < 0)
                i = alarm_reference_t::alarm_reference_max - 1;
            else if (i >= alarm_reference_t::alarm_reference_max)
                i = 0;
            alarm_reference = (config_param_t::alarm_reference_t)i;
         // show_reference_name = 50; //
    alarmの対象温度値を変更した際、一定時間だけ温度値の代わりに名称を表示する
        }
    //*/
}

/*
struct graph_filter_t {
    static constexpr const size_t hist_max = 3;
    // 5 : 13
    // 1 3
    uint16_t _hist[hist_max];
    uint8_t _idx = 0;
    uint16_t exec(uint16_t new_value) {
        _hist[_idx] = new_value;
        if (++_idx >= hist_max) { _idx = 0; }

        uint32_t avg_temp = 0;
        for (size_t i = 0; i < hist_max; ++i) {
            size_t k = abs((int)(i - _idx) * 2 + 1);
            if (k > hist_max) {
                k = hist_max * 2 - k;
            }
            avg_temp += _hist[i] * k;
        }
// ESP_LOGE("DEBUG","input:%d  avg:%d", new_value, avg_temp);
        return avg_temp / 5;
        // return new_value;
    }
};
static graph_filter_t graph_filter[4];
//*/

void draw_param_t::setup(LovyanGFX* gfx_, framedata_t* frame_array_,
                         int frameindex) {
    _frame_array = frame_array_;
    frame        = &frame_array_[frameindex];
    _lowest_value.set(frame->temp[frame->lowest]);
    _highest_value.set(frame->temp[frame->highest]);
    update(frameindex);

    for (int i = 0; i < 4; ++i) {
        auto tmp = frame->temp[i];
        for (int j = 0; j < graph_data.data_len; ++j) {
            graph_data.temp_arrays[i][j] = tmp;
        }
        // while (tmp != graph_filter[i].exec(tmp));
    }
    _prev_frameindex = -1;
}

void draw_param_t::setFont(const m5gfx::IFont* font_) {
    font = font_;
    display.setFont(font_);
    lgfx::FontMetrics fm;
    font->getDefaultMetric(&fm);
    font_height = fm.height;
}

void draw_param_t::setColorTable(const uint16_t* tbl) {
    color_map = tbl;
}

void draw_param_t::setColorTable(size_t idx) {
    color_map = color_map_table[idx];
}

bool draw_param_t::update(int frameindex) {
    if (_prev_frameindex == frameindex) return false;
    _prev_frameindex = frameindex;
    frame            = &_frame_array[frameindex];
    ++update_count;
    return true;
}

bool draw_param_t::range_update(void) {
    if (!frame) return false;
    if (range_autoswitch.get() == range_autoswitch_off) return true;

    int32_t lowest  = frame->temp[frame->lowest];
    int32_t highest = frame->temp[frame->highest];
    int32_t margin  = ((highest - lowest) >> 4) + 1;
    lowest          = _lowest_value.exec(lowest - margin, margin);
    highest         = _highest_value.exec(highest + margin, margin);
    if (range_temp_lower != lowest || range_temp_upper != highest) {
        range_temp_lower = lowest;
        range_temp_upper = highest;
        temp_diff        = (highest - lowest) + 1;
        ++modify_count;
        return true;
    }
    return false;
}

int32_t value_smooth_t::exec(int32_t src, int32_t margin) {
    /*
            int32_t new_target = src << 8;
            if (abs(new_target - _target) > (margin << 8)) {
                _target = new_target;
            }
            int diff = _target - _current;

            _add = ((_add + diff + 2) >> 5);
            if (diff) {
                _add += diff < 0 ? -1 : 1;
            }
            _current += _add;

            return _current >> 8;
    /*/
    int32_t new_target = src << 8;
    if (abs(new_target - _target) > (margin << 8)) {
        _target = new_target;
    }
    int diff = _target - _current;

    if (diff) {
        int ta    = 1;
        int tdiff = abs(diff);
        while (0 < (tdiff -= ta)) {
            ta *= 2;
        }
        ta += tdiff;

        if (diff < 0) {
            ta = -ta;
        }

        _add = (_add + (ta >> 1)) >> 1;
    }

    // _add = ((_add + diff + 2) >> 5);
    // if (diff) {
    //     _add += diff < 0 ? -1 : 1;
    // }
    _current += _add;

    return _current >> 8;
    //*/
}
void value_smooth_t::set(int32_t default_value) {
    _target  = default_value << 8;
    _current = default_value << 8;
}

// static constexpr const uint32_t jpeg_len_max = 8192 * 3;

screenshot_streamer_t::screenshot_streamer_t(void) {
    // JPEGエンコード待ちキューの数 (メモリ節約のため1とする)
    _queue_canvas = xQueueCreate(1, sizeof(screenshot_streamer_t::queue_ss_t));

    // WiFiClientへのデータ送信待ちキューの数
    _queue_bufdata =
        xQueueCreate(2, sizeof(screenshot_streamer_t::queue_bufdata_t));

    // スクリーンショット更新の順番待ちができるクライアント（ブラウザ）の数
    _queue_client = xQueueCreate(4, sizeof(WiFiClient*));
}

void screenshot_streamer_t::requestScreenShot(WiFiClient* client) {
    xQueueSendToBack(_queue_client, &client, 0);
    _is_requested = true;
}
/*
#include <lwip/sockets.h> // for 'send' function
static size_t write_fd(int socketFileDescriptor, const uint8_t *buf, size_t
size)
{
    int retry = 2;
    size_t totalBytesSent = 0;
    size_t bytesRemaining = size;

    if(socketFileDescriptor < 0) {
        return 0;
    }

    while(retry--) {
        //use select to make sure the socket is ready for writing
        fd_set set;
        struct timeval tv;
        FD_ZERO(&set);        // empties the set
        FD_SET(socketFileDescriptor, &set); // adds FD to the set
        tv.tv_sec = 0;
        tv.tv_usec = 100; // WIFI_CLIENT_SELECT_TIMEOUT_US;

        if (select(socketFileDescriptor + 1, NULL, &set, NULL, &tv) < 0) {
            return 0;
        }

        if (FD_ISSET(socketFileDescriptor, &set)) {
            int res = send(socketFileDescriptor, (void*) buf, bytesRemaining,
MSG_DONTWAIT); if(res > 0) { totalBytesSent += res; if (totalBytesSent >= size)
{
                    //completed successfully
                    retry = 0;
                } else {
                    buf += res;
                    bytesRemaining -= res;
                    retry = 2; // WIFI_CLIENT_MAX_WRITE_RETRY;
                }
            }
            else if(res < 0) {
                ESP_LOGE("DEBUG", "fail on fd %d, errno: %d, \"%s\"",
socketFileDescriptor, errno, strerror(errno)); if(errno != EAGAIN) {
                    //if resource was busy, can try again, otherwise give up
                    // stop();
                    retry = 0;
                }
            }
        }
    }
    return totalBytesSent;
}
//*/
void screenshot_streamer_t::streamTask(void* arg) {
    auto me = (screenshot_streamer_t*)arg;
    queue_bufdata_t qd;
    auto& cl = me->_wifi_client;
    while (pdTRUE == xQueueReceive(me->_queue_bufdata, &qd, portMAX_DELAY)) {
        auto len = qd.len;
        auto buf = qd.bufdata;
        if (qd.y0) {
            xQueueReceive(me->_queue_client, &cl, 0);
        }
        if (cl == nullptr) {
            continue;
        }
        if (cl->connected()) {
            if (len == cl->write(buf, len)) {
                continue;
            }
            cl->stop();
        }
        cl = nullptr;
    }
    vTaskDelete(nullptr);
}

bool screenshot_streamer_t::put_buf(const void* Pbuf, int len) {
    if (len) {
        _q_bufdata.len     = len;
        _q_bufdata.bufdata = (const uint8_t*)Pbuf;
        xQueueSendToBack(_queue_bufdata, &_q_bufdata, portMAX_DELAY);
        return true;
    }
    if (_wifi_client != nullptr) {
        requestScreenShot(_wifi_client);
    }
    return true;
}

bool screenshot_streamer_t::initCapture(uint16_t width, uint16_t height) {
    if (!_is_requested && !uxQueueMessagesWaiting(_queue_client)) {
        return false;
    }
    _is_requested = false;

    if (_width != width || _height != height) {
        int num_channels          = 3;
        jpge::params comp_params  = jpge::params();
        comp_params.m_subsampling = jpge::H2V2;
        comp_params.m_quality     = draw_param.net_jpg_quality;
        if (!_jpeg_enc.init(this, width, height, num_channels, comp_params)) {
            return false;
        }
        _y      = 0;
        _width  = width;
        _height = height;
    }
    return true;
}

bool screenshot_streamer_t::addQueue(M5Canvas* canvas, uint16_t y) {
    queue_ss_t queue_ss;
    queue_ss.canvas = canvas;
    queue_ss.y      = y;
    if (pdPASS != xQueueSendToBack(_queue_canvas, &queue_ss, 64)) {
        // ESP_LOGD("DEBUG","queue_timeout:y:%d", y);
        // 時間内にキューが空かなかった場合は失敗終了とする。
        if (_wifi_client) {  // wifiクライアントが存続している場合は_is_requestedフラグをセットし、次回のフレームで処理が継続できるようにする
            _is_requested = true;
        }
        return false;
    }
    return true;
}

screenshot_streamer_t::process_result_t screenshot_streamer_t::processCapture(
    void) {
    queue_ss_t queue_ss;
    // この時点ではまだキューから取り出さない
    // (取り出すと描画側タスクの処理が進んでキュー内の画像が上書きされてしまうため)
    if (pdPASS != xQueuePeek(_queue_canvas, &queue_ss, 0)) {
        return pr_nothing;
    }

    if (_y != queue_ss.y) {
        // 座標の不一致が生じた場合はキューを破棄して帰る
        xQueueReceive(_queue_canvas, &queue_ss, 0);
        return pr_error;
    }
    if (queue_ss.y == 0) {
        bool request  = uxQueueMessagesWaiting(_queue_client);
        _is_requested = request;
        if (!request) {
            xQueueReceive(_queue_canvas, &queue_ss, 0);
            return process_result_t::pr_nothing;
        }

        static constexpr const char headtext[] =
            "\r\n--tlite\r\nContent-type: image/jpeg\r\n\r\n";
        _q_bufdata.bufdata = (const uint8_t*)headtext;
        _q_bufdata.len     = sizeof(headtext) - 1;
        _q_bufdata.y0      = true;
        xQueueSendToBack(_queue_bufdata, &_q_bufdata, portMAX_DELAY);
        _q_bufdata.y0 = false;

        // JPEGエンコーダ再初期化 (初期設定部分を省略)
        _jpeg_enc.reinit(draw_param.net_jpg_quality);
    }

    _y += queue_ss.canvas->height();
    bool success = true;
    // bool lineend = false;
    auto framebuffer = (uint16_t*)(queue_ss.canvas->getBuffer());
    for (int i = 0; i < queue_ss.canvas->height(); ++i) {
        success = _jpeg_enc.process_scanline565(
            &framebuffer[i * queue_ss.canvas->width()]);
        if (++queue_ss.y >= _height) {
            break;
        }
        if (!success) {
            break;
        }
    }
    // 画像データの読出しを終えた後でキューから取り出す
    xQueueReceive(_queue_canvas, &queue_ss, 0);

    // JPEGのエンコード処理部分を実行する
    if (success) {
        _jpeg_enc.process_mcu_row();
        if (_y >= _height) {
            _y = 0;
            _jpeg_enc.process_scanline565(nullptr);
        }
        return process_result_t::pr_progress;
    }
    // ESP_EARLY_LOGE("DEBUG","jpg encode error");
    return process_result_t::pr_error;

    //---------------------------------------------
}

struct ui_base_t {
    bool isModified(draw_param_t* param) const {
        return _prev_update_count != param->update_count ||
               _prev_modify_count != param->modify_count;
    }

    virtual void update(draw_param_t* param) {
    }

    virtual bool draw(draw_param_t* param) {
        bool res = _prev_update_count != param->update_count ||
                   _prev_modify_count != param->modify_count;

        _prev_update_count = param->update_count;
        _prev_modify_count = param->modify_count;

        return !_client_rect.empty() && res;
    }

    virtual void draw(draw_param_t* param, M5Canvas* canvas, int32_t y,
                      int32_t h) {
    }

    virtual bool smoothMove(void) {
        return _client_rect.smooth_move(_target_rect);
    }
    virtual void setTargetRect(const rect_t& rect) {
        _target_rect = rect;
    }
    void setClientRect(const rect_t& rect) {
        _client_rect = rect;
    }
    const rect_t& getTargetRect(void) const {
        return _target_rect;
    }
    const rect_t& getClientRect(void) const {
        return _client_rect;
    }

    void rotateCoordinate(int r, int& x, int& y, int& w, int& h) {
        if (r) {
            if (r & 1) {
                std::swap(x, y);
                std::swap(w, h);
            }
            if ((1u << r) & 0b0110) {
                x = _client_rect.w - (x + w);
            }
            if ((1u << r) & 0b1100) {
                y = _client_rect.h - (y + h);
            }
        }
    }

   protected:
    // inline void clearInvalidate(void) {
    //     _invalidate = false;
    // }
    // bool _invalidate;
    rect_t _client_rect;
    rect_t _target_rect;
    uint8_t _prev_update_count;
    uint8_t _prev_modify_count;
};

class qrcode_ui_t : public ui_base_t {
   public:
    std::string _qr_string = "teststring";
    M5Canvas _qr_canvas;
    void draw(draw_param_t* param, M5Canvas* canvas, int32_t canvas_y,
              int32_t h) override {
        canvas->fillScreen(TFT_WHITE);
        float zoom = (float)_client_rect.h / (_qr_canvas.height() + 4);
        _qr_canvas.pushRotateZoom(
            canvas, _client_rect.x + (_client_rect.w >> 1),
            _client_rect.y + (_client_rect.h >> 1) - canvas_y, 0, zoom, zoom);
    }
    void show(const char* data) {
        _qr_canvas.setColorDepth(1);
        _qr_string = data;
        for (uint8_t version = 1; version <= 40; ++version) {
            QRCode qrcode;
            auto qrcodeData =
                (uint8_t*)alloca(lgfx_qrcode_getBufferSize(version));
            if (0 !=
                lgfx_qrcode_initText(&qrcode, qrcodeData, version, 0, data))
                continue;
            _qr_canvas.createSprite(qrcode.size, qrcode.size);
            _qr_canvas.fillScreen(1);
            int_fast16_t thickness  = 1;
            int_fast16_t lineLength = qrcode.size;
            _qr_canvas.setColor(0);
            uint32_t y = 0;
            do {
                uint32_t x = 0;
                do {
                    if (lgfx_qrcode_getModule(&qrcode, x, y))
                        _qr_canvas.drawPixel(x, y);
                } while (++x < qrcode.size);
            } while (++y < qrcode.size);
            break;
        }

        int w = display.height();
        setTargetRect({(display.width() - w) >> 1, 0, w, w});
    }
    void hide(void) {
        setTargetRect({display.width() >> 1, display.height() >> 1, 0, 0});
    }
};

class overlay_ui_t : public ui_base_t {
    static constexpr const size_t lines_max = 6;
    std::string _text[lines_max];
    uint16_t _textwidth[lines_max];
    uint8_t _lines;
    uint8_t _countdown;

   public:
    void show(uint8_t count, const char* text0, const char* text1 = nullptr,
              const char* text2 = nullptr, const char* text3 = nullptr,
              const char* text4 = nullptr, const char* text5 = nullptr) {
        _countdown = count;
        _text[0]   = text0;
        _lines     = 1;
        if (text1) {
            _text[1] = text1;
            ++_lines;
            if (text2) {
                _text[2] = text2;
                ++_lines;
                if (text3) {
                    _text[3] = text3;
                    ++_lines;
                    if (text4) {
                        _text[4] = text4;
                        ++_lines;
                        if (text5) {
                            _text[5] = text5;
                            ++_lines;
                        }
                    }
                }
            }
        }
        int32_t w = 0;
        for (int32_t j = _lines - 1; j >= 0; --j) {
            int32_t tw    = display.textWidth(_text[j].c_str());
            _textwidth[j] = tw;
            if (w < tw) {
                w = tw;
            }
        }
        int32_t fh = display.fontHeight();
        w += fh;
        int32_t h = (_lines + 1) * fh;
        setTargetRect(
            {(display.width() - w) >> 1, (display.height() - h) >> 1, w, h});
    }

    void hide(void) {
        setTargetRect({display.width() >> 1, display.height() >> 1, 0, 0});
    }

    void update(draw_param_t* param) override {
        if (_countdown && (_countdown != 255)) {
            if (0 == --_countdown) {
                hide();
            }
        }
    }

    void draw(draw_param_t* param, M5Canvas* canvas, int32_t canvas_y,
              int32_t h) override {
        {
            auto canvas_w        = canvas->width();
            int32_t client_width = _client_rect.right() < canvas_w
                                       ? _client_rect.right()
                                       : canvas_w;
            int32_t client_x     = _client_rect.x < 0 ? 0 : _client_rect.x;
            client_width -= client_x;
            auto client_height = _client_rect.h;
            auto buf           = ((uint16_t*)canvas->getBuffer()) + client_x;

            int dy  = _client_rect.y - canvas_y;
            int dye = dy + client_height;
            if (dy < 0) dy = 0;
            if (dye > h) dye = h;

            uint32_t cw2 = (client_width + 1) >> 1;
            for (; dy < dye; ++dy) {
                auto b = (uint32_t*)(&buf[canvas_w * dy]);
                for (int x = 0; x < cw2; ++x) {
                    uint32_t tmp = b[x];
                    b[x] = (tmp >> 1) & 0x6F7B6F7B | ((tmp & 0x00010001) << 15);
                    // tmp = (tmp << 7 | tmp >> 9) & 0x7BEF;
                    // b[x] = tmp << 8 | tmp >> 8;
                }
            }
        }

        canvas->setTextSize(1);
        canvas->setTextColor(TFT_WHITE);
        canvas->setTextDatum(textdatum_t::top_left);
        auto fh = param->font_height;
        int32_t y =
            _client_rect.y - canvas_y + ((_client_rect.h - fh * _lines) >> 1);
        for (int j = 0; j < _lines; ++j, y += fh) {
            if (y > canvas->height() || 0 > y + fh) {
                continue;
            }
            if (0 == _textwidth[j]) {
                continue;
            }
            canvas->drawString(
                _text[j].c_str(),
                _client_rect.x + ((_client_rect.w - _textwidth[j]) >> 1), y);
        }
    }
};

qrcode_ui_t qrcode_ui;
overlay_ui_t overlay_ui;

void config_param_t::misc_backtofactory_func(uint8_t) {
    WiFi.mode(WIFI_MODE_STA);
    WiFi.disconnect(true, true);
    draw_param.loadDefault();
    draw_param.saveNvs();
    overlay_ui.show(64, "Reset Done.");
}

static rect_t _cursor_rect        = {0, 0, 240, 0};
static rect_t _cursor_target_rect = {0, 0, 240, 0};
class control_ui_t;
static control_ui_t* _cursor_target = nullptr;

class control_ui_t : public ui_base_t {
   protected:
    static control_ui_t* getCursorTarget(void) {
        return _cursor_target;
    }
    static void setCursorTarget(control_ui_t* target) {
        _cursor_target = target;
        if (target) {
            _cursor_target_rect = target->getDisplayTargetRect();
        }
    }

    rect_t getDisplayTargetRect(void) const {
        if (_parent) {
            auto r = _parent->getDisplayTargetRect();
            r.x += _target_rect.x;
            r.y += _target_rect.y;
            r.w = _target_rect.w;
            r.h = _target_rect.h;
            return r;
        }
        return {0, 0, _target_rect.w, _target_rect.h};
    }

    control_ui_t* _focus_target = nullptr;
    control_ui_t* _parent       = nullptr;

   public:
    control_ui_t* getTopParent(void) {
        return _parent ? _parent->getTopParent() : this;
    }

    // void invalidate(void) override {
    //     if (_parent)
    //         _parent->invalidate();
    //     else
    //         ui_base_t::invalidate();
    // }

    void setParent(control_ui_t* p) {
        _parent = p;
    }
    virtual void draw_inner(LovyanGFX* gfx, int offset_x, int offset_y) {
        // gfx->drawRect(_client_rect.x + offset_x, _client_rect.y + offset_y,
        // _client_rect.w, _client_rect.h, rand());
    }
    virtual bool loop(void) {
        return _focus_target ? _focus_target->loop() : false;
    }
    virtual void setup(void) {
    }
    virtual void relocation(void) {
    }
    virtual bool enter(control_ui_t* parent) {
        // M5.Speaker.tone(6600, 80);
        soundEnter();
        return true;
    }
    virtual void exit(void) {
        // M5.Speaker.tone(3300, 80);
        soundExit();
        setCursorTarget(this);
        qrcode_ui.hide();
        overlay_ui.hide();
    }
    bool smoothMove(void) override {
        bool res = ui_base_t::smoothMove();
        //    if (_focus_target) res += _focus_target->smoothMove();
        return res;
    }
};

class text_control_ui_t : public control_ui_t {
   public:
    uint16_t text_color = 0xFFFFu;

    void draw_text(LovyanGFX* gfx, const char* text, int offset_x,
                   int offset_y) {
        offset_x += _client_rect.x;
        offset_y += _client_rect.y;
        int ypos = 1 + offset_y;

        gfx->setTextColor(text_color);
        int32_t tw   = gfx->textWidth(text);
        int32_t xpos = (_client_rect.w - tw) >> 1;
        int32_t over = (tw - _client_rect.w);
        if (over >= 0) {
            over += 8;
            int scr = (int32_t)(draw_param.draw_count % (over + 128)) -
                      (over >> 1) - 64;
            over >>= 1;
            if (abs(scr) > over) {
                scr = (scr < 0) ? -over : over;
            }
            xpos -= scr;
        }
        gfx->drawString(text, xpos + offset_x, ypos);
    }
};

class itext_ui_t : public text_control_ui_t {
    const itext_t* _text = nullptr;

   public:
    itext_ui_t(void) = default;
    itext_ui_t(const itext_t* lt, control_ui_t* target = nullptr) : _text{lt} {
        _focus_target = target;
    }

    void draw_inner(LovyanGFX* gfx, int offset_x, int offset_y) override {
        text_control_ui_t::draw_inner(gfx, offset_x, offset_y);
        draw_text(gfx, _text->get(), offset_x, offset_y);
    }

    void relocation(void) override {
        auto r = getTargetRect();
        r.h    = (draw_param.font_height + 2);
        setTargetRect(r);
    }
};

class label_ui_t : public text_control_ui_t {
   public:
    label_ui_t(void) = default;
    label_ui_t(const char* text_, control_ui_t* target = nullptr)
        : text{text_} {
        _focus_target = target;
    }
    const char* text    = nullptr;
    uint16_t text_color = 0xFFFFu;

    void draw_inner(LovyanGFX* gfx, int offset_x, int offset_y) override {
        text_control_ui_t::draw_inner(gfx, offset_x, offset_y);
        draw_text(gfx, text, offset_x, offset_y);
    }

    void relocation(void) override {
        auto r = getTargetRect();
        r.h    = (draw_param.font_height + 2);
        setTargetRect(r);
    }
};

class container_ui_t : public control_ui_t {
   protected:
    std::vector<control_ui_t*> items;
    int _cursor_index = 0;
    int _scroll_y     = 0;
    int _scroll_limit = 0;
    bool _sub_control = false;

   public:
    void setTargetRect(const rect_t& rect) override {
        control_ui_t::setTargetRect(rect);
        relocation();
    }

    void relocation(void) override {
        control_ui_t::relocation();
        int ypos = 2;
        int w    = _target_rect.w - 2;
        for (auto& item : items) {
            item->relocation();
            auto r = item->getTargetRect();
            r.x    = 1;
            r.w    = w;
            r.y    = ypos;
            ypos   = r.bottom() + 1;
            item->setTargetRect(r);
        }
        _target_rect.h = ypos + 2;
    }

    void setup(void) override {
        for (auto& item : items) {
            item->setup();
        }
    }

    bool loop(void) override {
        if (_sub_control) {
            if (items[_cursor_index]->loop()) {
                return true;
            }
            _sub_control = false;
            relocation();
            items[_cursor_index]->exit();
            if (_parent && !M5.BtnPWR.wasHold() && !M5.BtnC.wasHold()) {
                return true;
            }
            //      setCursorTarget(items[_cursor_index]);
            setCursorTarget(nullptr);
            return false;
        }

        if (M5.BtnPWR.wasClicked() || M5.BtnPWR.wasHold() ||
            M5.BtnC.wasClicked() || M5.BtnC.wasHold()) {
            return false;
        }
        if (!items.empty()) {
            {
                //        if (M5_BtnB_wasClicked())
                // if (M5.BtnPWR.wasHold() || M5.BtnB.wasPressed() ||
                // M5.BtnB.isHolding())
                if (M5.BtnB.wasPressed() ||
                    ((M5.BtnB.isHolding() &&
                      _cursor_target_rect == _cursor_rect))) {
                    soundMoveCursor();
                    int add = 1;
                    //          int add = 1;
                    int idx = _cursor_index + add;
                    if (idx < 0)
                        idx = items.size() - 1;
                    else if (idx >= items.size())
                        idx = 0;
                    _cursor_index = idx;
                    setCursorTarget(items[_cursor_index]);
                    // invalidate();
                }
            }

            if (M5.BtnA.wasPressed()) {
                _sub_control = items[_cursor_index]->enter(this);
                relocation();
                // invalidate();
            }
        }
        return true;
    }

    bool enter(control_ui_t* parent) override {
        control_ui_t::enter(parent);
        relocation();
        setCursorTarget(items[_cursor_index]);
        return true;
    }

    void exit(void) override {
        control_ui_t::exit();
    }

    void draw_inner(LovyanGFX* gfx, int offset_x, int offset_y) override {
        control_ui_t::draw_inner(gfx, offset_x, offset_y);

        int32_t left, top, right, bottom;
        gfx->getClipRect(&left, &top, &right, &bottom);

        right  = right + left;
        bottom = bottom + top;

        auto rect = getClientRect();
        rect.x += offset_x;
        rect.y += offset_y + _scroll_y;

        //  gfx->drawRect(rect.x, rect.y, rect.w, rect.h, TFT_WHITE);

        if (top < rect.top()) top = rect.top();
        if (bottom > rect.bottom()) bottom = rect.bottom();
        if (top >= bottom) {
            return;
        }

        if (left < rect.left()) left = rect.left();
        if (right > rect.right()) right = rect.right();
        if (left >= right) {
            return;
        }

        offset_x += _client_rect.x;
        offset_y += _client_rect.y;

        for (auto& item : items) {
            auto r = item->getClientRect();
            r.y += offset_y;
            if (r.empty() || r.y >= bottom || r.bottom() <= top) {
                continue;
            }

            gfx->setClipRect(left, top, right - left, bottom - top);
            //    gfx->drawRect(left, top, right - left, bottom - top, rand());
            item->draw_inner(gfx, offset_x, offset_y);
        }
    }

    bool smoothMove(void) override {
        bool res = control_ui_t::smoothMove();
        for (auto& item : items) {
            res += item->smoothMove();
        }
        return res;
    }
    void addItem(control_ui_t* item_) {
        item_->setParent(this);
        items.push_back(item_);
    }
};

static int valueChangeSpan;
static int valueChangeWait;

class property_ui_t : public text_control_ui_t {
   public:
    property_ui_t(void) = default;
    property_ui_t(config_property_base_t* prop_) : property{prop_} {
        _focus_target = nullptr;
    }

    config_property_base_t* property;

    void draw_inner(LovyanGFX* gfx, int offset_x, int offset_y) override {
        text_control_ui_t::draw_inner(gfx, offset_x, offset_y);
        draw_text(gfx, property->getText(), offset_x, offset_y);
    }

    void relocation(void) override {
        auto r = getTargetRect();
        r.h    = (draw_param.font_height + 2);
        setTargetRect(r);
    }
};

class value_ui_t : public container_ui_t {
    bool _simple;

   public:
    itext_ui_t label;
    property_ui_t value;

    value_ui_t(void) = default;
    value_ui_t(const itext_t* text_, config_property_base_t* property,
               bool simple = false)
        : _simple{simple}, label{text_, &value}, value{property} {
        _focus_target = &value;
        _cursor_index = 1;
        addItem(&label);
        addItem(&value);
        label.text_color = label_ui_text_color;
        value.text_color = 0xFFFFu;
    }

    value_ui_t(config_property_base_t* property, bool simple = false)
        : _simple{simple}, label{property->getName(), &value}, value{property} {
        _focus_target = &value;
        _cursor_index = 1;
        addItem(&label);
        addItem(&value);
        label.text_color = label_ui_text_color;
        value.text_color = 0xFFFFu;
    }

    bool enter(control_ui_t* parent) override {
        if (_simple) {
            valueChange(-1);
            soundEnter();
            // M5.Speaker.tone(2200, 50, 0);
            return false;
        }

        container_ui_t::enter(parent);
        return true;
    }

    bool loop(void) override {
        if (M5.BtnPWR.wasClicked() || M5.BtnPWR.wasHold() ||
            M5.BtnC.wasClicked() || M5.BtnC.wasHold()) {
            return false;
        }
        int add = 0;
        if (M5.BtnA.wasPressed() || M5.BtnA.isHolding()) add = -1;
        if (M5.BtnB.wasPressed() || M5.BtnB.isHolding()) add = 1;
        if (add) {
            if (valueChangeWait) {
                --valueChangeWait;
            } else {
                valueChangeSpan = valueChangeSpan > 0 ? valueChangeSpan - 1 : 0;
                valueChangeWait = valueChangeSpan;
                if (valueChange(add)) {
                    // invalidate();
                    // M5.Speaker.tone(2200, 50, 0);
                    soundValueChange();
                }
            }
        } else {
            valueChangeSpan = 16;
            valueChangeWait = 0;
        }
        return true;
    }

    virtual bool valueChange(int add) {
        return value.property->add(add);
    }
};

class token_ui_t : public container_ui_t {
   public:
    itext_ui_t label;
    label_ui_t value;

    token_ui_t(void) = default;
    token_ui_t(const itext_t* text_, std::string* property)
        : label{text_, &value}, value{property->c_str()} {
        _focus_target = &value;
        _cursor_index = 1;
        addItem(&label);
        addItem(&value);
        label.text_color = label_ui_text_color;
        value.text_color = 0xFFFFu;
    }

    bool enter(control_ui_t* parent) override {
        soundEnter();
        // M5.Speaker.tone(2200, 50, 0);
        char cbuf[32];
        auto mi = micros();
        for (int i = 0; i < 4; ++i) {
            cbuf[i] = '0' + (mi % 10);
            mi >>= 4ull;
        }
        cbuf[4]                = '\0';
        draw_param.cloud_token = cbuf;

        return false;
    }
};

class qr_ui_t : public container_ui_t {
    const std::string* _property;

   public:
    itext_ui_t label;
    label_ui_t value;

    qr_ui_t(void) = default;
    qr_ui_t(const itext_t* text_, const std::string* property)
        : _property{property}, label{text_, &value}, value{property->c_str()} {
        _focus_target = &value;
        _cursor_index = 1;
        addItem(&label);
        addItem(&value);
        label.text_color = label_ui_text_color;
        value.text_color = 0xFFFFu;
    }

    bool loop(void) override {
        if (M5.BtnA.wasPressed() || M5.BtnB.wasPressed() ||
            M5.BtnPWR.wasClicked() || M5.BtnPWR.wasHold() ||
            M5.BtnC.wasClicked() || M5.BtnC.wasHold()) {
            return false;
        }
        return true;
    }
    bool enter(control_ui_t* parent) override {
        value.text = _property->c_str();
        qrcode_ui.show(value.text);
        return control_ui_t::enter(parent);
    }
};

class qr_ui2_t : public qr_ui_t {
   public:
    qr_ui2_t(const itext_t* text_, const std::string* property)
        : qr_ui_t{text_, property} {
    }

    bool enter(control_ui_t* parent) override {
        qrcode_ui.show(value.text);
        overlay_ui.show(255, "Confirm", "Code", "",
                        draw_param.cloud_token.c_str());
        auto r1   = qrcode_ui.getTargetRect();
        auto r2   = overlay_ui.getTargetRect();
        int32_t x = (display.width() - (r1.w + r2.w));
        if (x < 0) {
            r2.w += x;
            x = 0;
        }
        r1.x = x >> 1;
        r2.x = r1.right();

        qrcode_ui.setTargetRect(r1);
        overlay_ui.setTargetRect(r2);

        return control_ui_t::enter(parent);
    }
};

class staff_ui_t : public container_ui_t {
   public:
    itext_ui_t label;
    itext_ui_t value;

    staff_ui_t(void) = default;
    staff_ui_t(const itext_t* text_, const itext_t* text2_)
        : label{text_, &value}, value{text2_} {
        _focus_target = &value;
        _cursor_index = 1;
        addItem(&label);
        addItem(&value);
        label.text_color = label_ui_text_color;
        value.text_color = 0xFFFFu;
    }

    bool loop(void) override {
        if (M5.BtnA.wasPressed() || M5.BtnB.wasPressed() ||
            M5.BtnPWR.wasClicked() || M5.BtnPWR.wasHold() ||
            M5.BtnC.wasClicked() || M5.BtnC.wasHold()) {
            return false;
        }
        return true;
    }
    bool enter(control_ui_t* parent) override {
        draw_param.misc_staff = 1;
        return control_ui_t::enter(parent);
    }
    void exit(void) override {
        draw_param.misc_staff = 0;
        return control_ui_t::exit();
    }
};

class switch_ui_t : public container_ui_t {
    itext_ui_t title;
    bool _expanded = false;

   public:
    switch_ui_t(const localize_text_t* text_, control_ui_t* target = nullptr)
        : title{text_, this} {
        _cursor_index    = 1;
        _focus_target    = target;
        title.text_color = 0x87F8;
        addItem(&title);
        addItem(target);
        _target_rect.h = title.getTargetRect().h;
        auto r         = target->getTargetRect();
        r.y            = getClientRect().bottom();
        target->setTargetRect(r);
    }

    bool loop(void) override {
        if (_focus_target->loop()) {
            return true;
        } else {
            _expanded = false;
            relocation();
        }
        return false;
    }

    void draw_inner(LovyanGFX* gfx, int offset_x, int offset_y) override {
        auto rect = getClientRect();
        rect.x += offset_x;
        rect.y += offset_y;
        // gfx->drawFastHLine(rect.x, rect.bottom(), rect.w, 0x4208);
        container_ui_t::draw_inner(gfx, offset_x, offset_y);
    }

    void relocation(void) override {
        container_ui_t::relocation();
        if (!_expanded) {
            _target_rect.h = title.getTargetRect().bottom();
            auto r         = title.getTargetRect();
            r.y            = r.bottom();
            r.h            = 0;
            _focus_target->setTargetRect(r);
        }
    }

    bool enter(control_ui_t* parent) override {
        _expanded = true;
        _focus_target->enter(parent);
        return true;
    }
    //
    void exit(void) override {
        _expanded = false;
        container_ui_t::exit();
    }
};

class config_ui_t : public container_ui_t {
    container_ui_t top_config_ui;
    container_ui_t network_config_ui;
    container_ui_t alarm_config_ui;
    container_ui_t cloud_config_ui;
    container_ui_t sens_config_ui;
    container_ui_t range_config_ui;
    container_ui_t misc_config_ui;

    static void cloud_timezone_func(int32_t v) {
        draw_param.setTimeZoneSec(v);
    }
    static const char* cloud_timezone_text_func(int32_t v) {
        static char text_buf[8];
        snprintf(text_buf, sizeof(text_buf), "%6.2f", (float)v / 3600);
        return text_buf;
    }

   public:
    template <typename T>
    int getIndex(const T* data, T target, size_t len) {
        int res = 0;
        do {
            if (data[res] == target) {
                return res;
            }
        } while (++res < len);
        return 0;
    }

    void setup(void) override {
        static constexpr const localize_text_t lt_Network = {"Network", "网络",
                                                             "ネットワーク"};
        static constexpr const localize_text_t lt_Cloud   = {
            "Cloud", "云监控配置", "クラウド"};
        static constexpr const localize_text_t lt_Alarm = {"Alarm", "报警设置",
                                                           "アラーム"};
        static constexpr const localize_text_t lt_Sensor = {
            "Sensor", "传感器设置", "センサー"};
        static constexpr const localize_text_t lt_Range  = {"Range", "温度量程",
                                                           "温度レンジ"};
        static constexpr const localize_text_t lt_Others = {
            "Others", "其他设置", "その他"};

        static constexpr const localize_text_t lt_LAN_QR_IP = {
            "LAN Monitor (IP)", "局域网监控（IP方法）", "LANモニタ（IP）"};
        static constexpr const localize_text_t lt_LAN_QR_mDNS = {
            "LAN Monitor (mDNS)", "局域网监控（mDNS）", "LANモニタ（mDNS）"};
        static constexpr const localize_text_t lt_Cloud_Online_QR = {
            "Cloud Online QR", "云监控-网址", "クラウド監視"};
        static constexpr const localize_text_t lt_Cloud_Confirm_Code = {
            "Cloud Confirm Code", "云监控-配对码", "クラウド認証コード"};
        static constexpr const localize_text_t lt_Temperature = {
            "Temperature", "设定温度", "設定温度"};
        static constexpr const localize_text_t lt_Emissivity = {
            "Emissivity", "辐射率", "放射率"};
        static constexpr const single_text_t lt_Language = {"Language"};
        static constexpr const localize_text_t lt_LAN_Stream_Quality = {
            "LAN Stream Quality", "局域网视频流画质", "LANモニタの画質"};
        static constexpr const localize_text_t lt_Factory_Reset = {
            "Factory Reset", "恢复出厂设置", "出荷時設定に戻す"};
        static constexpr const localize_text_t lt_Staff = {"Staff", "参与者",
                                                           "開発スタッフ"};
        static constexpr const localize_text_t lt_Staff_option = {
            "Display Info", "显示信息", "情報表示"};
        static constexpr const localize_text_t lt_Sens_TempHighest = {
            "Upper Temperature", "上限温度", "上限温度"};
        static constexpr const localize_text_t lt_Sens_TempLowest = {
            "Lower Temperature", "下限温度", "下限温度"};

        top_config_ui.addItem(new switch_ui_t{&lt_Network, &network_config_ui});
        top_config_ui.addItem(new switch_ui_t{&lt_Cloud, &cloud_config_ui});
        top_config_ui.addItem(new switch_ui_t{&lt_Alarm, &alarm_config_ui});
        top_config_ui.addItem(new switch_ui_t{&lt_Sensor, &sens_config_ui});
        top_config_ui.addItem(new switch_ui_t{&lt_Range, &range_config_ui});
        top_config_ui.addItem(new switch_ui_t{&lt_Others, &misc_config_ui});

        addItem(&top_config_ui);

        network_config_ui.addItem(
            new value_ui_t{&draw_param.net_running_mode, true});
        network_config_ui.addItem(
            new value_ui_t{&draw_param.net_setup_mode, true});
        network_config_ui.addItem(
            new qr_ui_t{&lt_LAN_QR_IP, &draw_param.net_url_ip});
        network_config_ui.addItem(
            new qr_ui_t{&lt_LAN_QR_mDNS, &draw_param.net_url_mdns});
        network_config_ui.addItem(
            new qr_ui2_t{&lt_Cloud_Online_QR, &draw_param.cloud_url});
        network_config_ui.addItem(
            new token_ui_t{&lt_Cloud_Confirm_Code, &draw_param.cloud_token});
        cloud_config_ui.addItem(
            new value_ui_t{&draw_param.cloud_interval, true});
        alarm_config_ui.addItem(new value_ui_t{&draw_param.alarm_mode, true});
        alarm_config_ui.addItem(
            new value_ui_t{&lt_Temperature, &draw_param.alarm_temperature});
        alarm_config_ui.addItem(
            new value_ui_t{&draw_param.alarm_reference, true});
        sens_config_ui.addItem(
            new value_ui_t{&draw_param.sens_refreshrate, true});
        sens_config_ui.addItem(
            new value_ui_t{&draw_param.sens_noisefilter, true});
        sens_config_ui.addItem(
            new value_ui_t{&draw_param.sens_monitorarea, true});
        sens_config_ui.addItem(
            new value_ui_t{&lt_Emissivity, &draw_param.sens_emissivity});
        range_config_ui.addItem(new value_ui_t{&draw_param.range_autoswitch});
        range_config_ui.addItem(
            new value_ui_t{&lt_Sens_TempHighest, &draw_param.range_temp_upper});
        range_config_ui.addItem(
            new value_ui_t{&lt_Sens_TempLowest, &draw_param.range_temp_lower});
        misc_config_ui.addItem(new value_ui_t{&draw_param.misc_language, true});
        misc_config_ui.addItem(new value_ui_t{&draw_param.misc_cpuspeed, true});
        misc_config_ui.addItem(new value_ui_t{&draw_param.misc_volume, true});
        misc_config_ui.addItem(
            new value_ui_t{&draw_param.misc_brightness, true});
        misc_config_ui.addItem(new value_ui_t{&lt_LAN_Stream_Quality,
                                              &draw_param.net_jpg_quality});
        misc_config_ui.addItem(new value_ui_t{
            &lt_Factory_Reset, &draw_param.misc_backtofactory, true});
        misc_config_ui.addItem(new staff_ui_t{&lt_Staff, &lt_Staff_option});

        top_config_ui.setup();

        container_ui_t::setup();

        _target_rect.w = display.width();
        _target_rect.h = display.height();
        // enter();
        do {
            update(nullptr);
        } while (smoothMove());
        _target_rect.h = 0;
        _client_rect.h = 0;
    }

    void relocation(void) override {
        auto r = getTargetRect();
        r.y    = 0;
        top_config_ui.setTargetRect(r);

        setCursorTarget(getCursorTarget());
    }

    bool enter(control_ui_t* parent = nullptr) override {
        relocation();
        _sub_control = true;
        return top_config_ui.enter(this);
    }

    void update(draw_param_t* param) override {
        if (_client_rect.empty()) {
            return;
        }

        // auto rect = items[_cursor_index]->getTargetRect();
        if (_cursor_target) {
            if (_cursor_rect.smooth_move(_cursor_target_rect)) {
                _scroll_limit =
                    top_config_ui.getTargetRect().bottom() - _target_rect.h;
                if (_scroll_limit < 0) _scroll_limit = 0;
            }
        }

        int new_scroll_y = _scroll_y;
        if (new_scroll_y > _scroll_limit) {
            new_scroll_y = _scroll_limit;
        }
        int pos_top = _cursor_rect.y - (_client_rect.h >> 3);
        if (pos_top < 0) pos_top = 0;
        int pos_bottom = _cursor_rect.bottom() - (_client_rect.h * 7 >> 3);
        if (pos_bottom > _scroll_limit) pos_bottom = _scroll_limit;

        if (new_scroll_y > pos_top != new_scroll_y < pos_bottom) {
            if (new_scroll_y > pos_top) {
                new_scroll_y = pos_top;
            } else {
                new_scroll_y = pos_bottom;
            }
        }

        if (_scroll_y != new_scroll_y) {
            _scroll_y = smooth_move(new_scroll_y, _scroll_y);
        }
    }

    void draw(draw_param_t* param, M5Canvas* canvas, int32_t canvas_y,
              int32_t h) override {
        canvas->fillScreen(TFT_BLACK);

        int32_t y = _client_rect.y - (canvas_y + _scroll_y);

        canvas->fillRect(_cursor_rect.x + 2, _cursor_rect.y + 2 + y,
                         _cursor_rect.w - 4, _cursor_rect.h - 4, 0x00007Fu);
        canvas->drawRect(_cursor_rect.x, _cursor_rect.y + y, _cursor_rect.w,
                         _cursor_rect.h, 0xC0C0C0u);

        canvas->setTextDatum(textdatum_t::top_left);
        canvas->setTextSize(1);

        canvas->setTextColor(0xB0B0A0u);
        top_config_ui.draw_inner(canvas, 0, y);
    }
};
config_ui_t config_ui;

void config_param_t::net_running_mode_func(net_running_mode_t v) {
    switch (v) {
        case net_running_mode_t::net_running_mode_offline:
            draw_param.request_wifi_state = 0;
            break;

        case net_running_mode_t::net_running_mode_lan_cloud:
        case net_running_mode_t::net_running_mode_lan:
            draw_param.request_wifi_state |=
                net_running_mode_t::net_running_mode_lan;
            break;

        default:
            draw_param.request_wifi_state &=
                ~net_running_mode_t::net_running_mode_lan;
            break;
    }
}

void config_param_t::misc_brightness_func(misc_brightness_t v) {
    // display.setBrightness(misc_brightness_value[v]);
}

void config_param_t::misc_language_func(misc_language_t v) {
    display.setFont(misc_language_value[v]);
    draw_param.setFont(misc_language_value[v]);
    localize_text_t::localize_index = v;
    config_ui.relocation();
}

void config_param_t::misc_volume_func(misc_volume_t v) {
    M5.Speaker.setVolume(misc_volume_value[v]);
}

void config_param_t::misc_cpuspeed_func(misc_cpuspeed_t v) {
    // draw_param.misc_cpuspeed = v;
    // setCpuFrequencyMhz(misc_cpuspeed_value[v]);
    rtc_cpu_freq_config_t _cpu_freq_conf;
    rtc_clk_cpu_freq_get_config(&_cpu_freq_conf);
    auto new_cpuspeed = misc_cpuspeed_value[v];
    bool reconnect    = _cpu_freq_conf.freq_mhz > 160 || new_cpuspeed > 160;
    rtc_clk_cpu_freq_mhz_to_config(new_cpuspeed, &_cpu_freq_conf);
    rtc_clk_cpu_freq_set_config_fast(&_cpu_freq_conf);
    uint32_t apb = 80 * MHZ;
    if (_cpu_freq_conf.freq_mhz < 80) {
        apb = _cpu_freq_conf.source_freq_mhz * MHZ / _cpu_freq_conf.div;
    }
    rtc_clk_apb_freq_update(apb);
    esp_timer_impl_update_apb_freq(apb / MHZ);

    if (reconnect) {
        need_wifi_reconnect = true;
    }
}

void config_param_t::sens_refreshrate_func(sens_refreshrate_t v) {
    command_processor::setRate(sens_refreshrate_value[v]);
}
void config_param_t::sens_noisefilter_func(sens_noisefilter_t v) {
    command_processor::setFilter(sens_noisefilter_value[v]);
}
void config_param_t::perf_emissivity_func(uint8_t v) {
    command_processor::setEmissivity(v);
}
void config_param_t::range_temperature_func(int32_t) {
    if (draw_param.range_temp_upper.get() <=
        draw_param.range_temp_lower.get()) {
        auto tmp = (draw_param.range_temp_upper.get() +
                    draw_param.range_temp_lower.get()) /
                   2;
        draw_param.range_temp_upper.set(tmp + 16);
        draw_param.range_temp_lower.set(tmp - 16);
    }
    draw_param.temp_diff = abs(draw_param.range_temp_upper.get() -
                               draw_param.range_temp_lower.get());
}

void config_param_t::misc_color_func(misc_color_t v) {
    draw_param.color_map = color_map_table[v];
}

class battery_ui_t : public ui_base_t {
    int8_t prev_battery_level = 0;

   public:
    void update(draw_param_t* param) override {
        prev_battery_level =
            smooth_move(param->battery_level, prev_battery_level);
    }

    void draw(draw_param_t* param, M5Canvas* canvas, int32_t canvas_y,
              int32_t h) override {
        int32_t x     = _client_rect.x;
        int32_t y     = _client_rect.y - canvas_y;
        int32_t bat_h = (100 - prev_battery_level) * _client_rect.h / 100;

        uint32_t fg = param->battery_state ? 0x00FF00u : 0x8080FFu;
        canvas->fillRect(x, y, _client_rect.w, bat_h, 0xFF0000u);
        canvas->fillRect(x, y + bat_h, _client_rect.w, _client_rect.h, fg);
    }
};

class header_ui_t : public ui_base_t {
    std::string _text;
    int32_t _text_width;
    int32_t _text_pos = 0;

   public:
    void update(draw_param_t* param) override {
        if (!param->in_config_mode) {
            if (_text_width) {
                _text_width = 0;
                _text_pos   = 0;
                _text.clear();
            }
        } else {
            if ((param->draw_count & 3) == 0) {
                auto status = WiFi.status();

                char cbuf[64];

                if (WiFi.getMode() == WIFI_AP ||
                    WiFi.getMode() == WIFI_AP_STA) {
                    _text.assign(cbuf,
                                 snprintf(cbuf, sizeof(cbuf), "SSID:%s  /  ",
                                          draw_param.net_apmode_ssid));
                    _text.append(cbuf,
                                 snprintf(cbuf, sizeof(cbuf), "PWD:%s  /  ",
                                          draw_param.net_apmode_pass));
                    _text.append(
                        cbuf,
                        snprintf(
                            cbuf, sizeof(cbuf), "AP IP:%s  /  ",
                            draw_param.net_apmode_ipaddr.toString().c_str()));
                } else if (WiFi.getMode() == WIFI_STA) {
                    if (draw_param.sys_ssid.empty()) {
                        if (param->net_setup_mode ==
                            param->net_setup_mode_smartconfig) {
                            _text =
                                "Please download and use \"ESP TOUCH\" app.  ";
                        } else {
                            _text = "Please setting WiFi at first.  ";
                        }
                    } else if (WiFi.isConnected()) {
                        _text.assign(
                            cbuf, snprintf(cbuf, sizeof(cbuf), "SSID:%s  /  ",
                                           draw_param.sys_ssid.c_str()));
                        _text.append(
                            cbuf,
                            snprintf(cbuf, sizeof(cbuf), "mDNS:%s.local  /  ",
                                     draw_param.net_apmode_ssid));
                        _text.append(
                            cbuf, snprintf(cbuf, sizeof(cbuf), "STA IP:%s  /  ",
                                           WiFi.localIP().toString().c_str()));
                    } else {
                        static constexpr const char* status_tbl[] = {
                            "idle",        "no ssid avail",  "scan completed",
                            "connected",   "connect failed", "connection lost",
                            "disconnected"};
                        if ((uint32_t)status < SIZEOF_ARRAY(status_tbl)) {
                            _text = status_tbl[status];
                        } else {
                            _text.empty();
                        }
                    }
                } else {
                    _text = "Config";
                }
                _text_width = display.textWidth(_text.c_str());
            }
            --_text_pos;
            if (_text_width + _text_pos < 0) {
                _text_pos += _text_width;
            }
        }
    }

    void draw(draw_param_t* param, M5Canvas* canvas, int32_t canvas_y,
              int32_t h) override {
        // if (_client_rect.empty()) { return; }
        // {
        //     int32_t x, y, w, h;
        //     canvas->setClipRect(_client_rect.x, _client_rect.y - canvas_y,
        //     _client_rect.w, _client_rect.h); canvas->getClipRect(&x, &y, &w,
        //     &h); if (0 == (w + h)) { return; }
        // }

        for (int i = 0; i < _client_rect.h; ++i) {
            canvas->drawFastHLine(
                _client_rect.x, i + _client_rect.y - canvas_y, _client_rect.w,
                canvas->color565(0, 128 - (i << 7) / _client_rect.h, 0));
        }
        canvas->setTextColor(TFT_WHITE);

        enum show_mode_t {
            show_none,
            show_apssid,
            show_passwd,
            show_apip,
            show_staip,
            show_stassid,
            show_mdns,
            show_need_setup,
        };
        show_mode_t show_mode = show_none;

        int xpos = _client_rect.right();
        {
            size_t level = 0;
            if (param->net_running_mode & param->net_running_mode_lan) {
                level = 1;
            }
            if (WiFi.status() == WL_CONNECTED) {
                auto rssi = WiFi.RSSI();
                level     = (rssi <= -96)   ? 2
                            : (rssi <= -85) ? 3
                            : (rssi <= -75) ? 4
                                            : 5;
            }
            xpos -= 14;
            // canvas->drawBitmap(xpos+1, _client_rect.y - canvas_y,
            // icon_wifi[level], 16, 12, TFT_WHITE);
            canvas->pushImage(xpos, _client_rect.y - canvas_y, 16, 14,
                              icon_wifi565[level], 0x2002);
        }

        if (param->net_running_mode & param->net_running_mode_cloud) {
            int idx = -1;
            switch (param->cloud_status) {
                case param->cloud_status_t::cloud_connection:
                    idx = 0;
                    break;
                case param->cloud_status_t::cloud_uploading:
                    idx = (param->draw_count >> 3) & 3;
                    break;
                case param->cloud_status_t::cloud_complete:
                    idx = 4;
                    break;
                case param->cloud_status_t::cloud_error:
                    idx = 5;
                    break;
                case param->cloud_status_t::cloud_timerwait:
                    idx = 6;
                    break;
                default:
                    break;
            }
            if (idx >= 0) {
                xpos -= 16;
                // canvas->drawBitmap(xpos+1, _client_rect.y - canvas_y,
                // icon_cloud[idx], 16, 12, TFT_WHITE);
                canvas->pushImage(xpos, _client_rect.y - canvas_y, 16, 14,
                                  icon_cloud565[idx], 0x2002);
                if (idx == 6) {
                    xpos -= 1;
                    canvas->setTextSize(1);
                    canvas->setTextColor(TFT_WHITE);
                    canvas->setTextDatum(textdatum_t::top_right);
                    xpos -= canvas->drawNumber(param->cloud_countdown_sec, xpos,
                                               _client_rect.y - canvas_y - 1,
                                               &fonts::Font2);
                }
            }
        }

        if (param->in_config_mode) {
            {
                int32_t x, y, w, h;
                canvas->getClipRect(&x, &y, &w, &h);
                w = xpos - _client_rect.x - x - 2;
                canvas->setClipRect(x, y, w, h);
            }

            canvas->setTextDatum(middle_left);
            int x = _text_pos + _client_rect.x;
            int y = _client_rect.y + (_client_rect.h >> 1) - 1 - canvas_y;

            bool scroll = xpos < _text_width;
            if (!scroll) {
                x = (_client_rect.w - _text_width) >> 1;
                if (x > xpos - _text_width) {
                    x = xpos - _text_width;
                }
                x += _client_rect.x;
            }
            canvas->drawString(_text.c_str(), x, y);
            if (scroll) {
                x += _text_width;
                canvas->drawString(_text.c_str(), x, y);
            }
        }
    }
};

class image_ui_t : public ui_base_t {
    struct marker_t {
        M5Canvas txtimg;
        const m5gfx::IFont* font = nullptr;

        int16_t mark_x = 64;
        int16_t mark_y = 64;

        int16_t text_x = 64;
        int16_t text_y = 64;

        value_smooth_t smooth_tx;
        value_smooth_t smooth_ty;

        uint16_t raw = UINT16_MAX;
        bool update(int raw_, const rect_t& rect, int x_, int y_,
                    const m5gfx::IFont* font_) {
            int mx      = x_ * rect.w / (frame_width - 1);
            int my      = y_ * rect.h / (frame_height - 1);
            bool result = mark_x != mx || mark_y != my;
            if (result) {
                mark_x = mx;
                mark_y = my;
            }

            if (font != font_) {
                font = font_;
                txtimg.setFont(font_);
                raw = 0;
            }
            /*
                        if (draw_param.show_reference_name)
                        {
                            raw_ = --draw_param.show_reference_name > 40 ? 1 :
               0;
                        }
            */

            if (raw != raw_) {
                raw    = raw_;
                result = true;
                char text[8];
                snprintf(text, sizeof(text), "%5.1f ",
                         convertRawToCelsius(raw_));

                const char* text_ptr = text;
                /*
                                if (draw_param.show_reference_name)
                                {
                                    text_ptr =
                   config_param_t::alarm_reference_text[draw_param.alarm_reference];
                                }
                */
                txtimg.setColorDepth(2);
                txtimg.createSprite(txtimg.textWidth(text_ptr) + 2,
                                    txtimg.fontHeight() + 2);
                txtimg.setPivot(txtimg.width() / 2.0f, txtimg.height() / 2.0f);
                txtimg.setPaletteColor(1, TFT_BLACK);
                txtimg.setPaletteColor(3, TFT_WHITE);
                txtimg.setTextColor(1);
                static constexpr const uint8_t xytbl[] = {
                    0x00, 0x01, 0x02, 0x12, 0x22, 0x21, 0x20, 0x10, 0x11};
                for (uint_fast8_t xy : xytbl) {
                    if (xy == 0x11u) {
                        txtimg.setTextColor(3);
                    }
                    txtimg.drawString(text_ptr, xy >> 4, xy & 3);
                }
            }

            int tx;
            int ty;
            if (x_ < 0) {  // || draw_param.show_reference_name) {
                tx = (frame_width << 7) / (frame_width - 1);
                ty = (frame_height << 7) / (frame_height - 1);
                smooth_tx.set(tx);
                smooth_ty.set(ty);
            } else {
                tx = (x_ << 8) / (frame_width - 1);
                ty = (y_ << 8) / (frame_height - 1);
                // if (draw_param.rotation & 1) {
                //     tx += txtimg.height() * (x_ < (frame_width >> 1) ? 2 :
                //     -2);
                // } else
                { ty += txtimg.height() * (y_ < (frame_height >> 1) ? 2 : -2); }
            }
            tx = smooth_tx.exec(tx, 0);
            ty = smooth_ty.exec(ty, 0);
            // if (draw_param.rotation & 1) {
            //     tx = ((tx *  rect.w                   + 128) >> 8) + 2;
            //     ty = ((ty * (rect.h - txtimg.width()) + 128) >> 8) +
            //     (txtimg.width() >> 1);
            // } else
            {
                tx = ((tx * (rect.w - txtimg.width()) + 128) >> 8) +
                     (txtimg.width() >> 1);
                ty = ((ty * rect.h + 128) >> 8) + 2;
            }
            result |= text_x != tx || text_y != ty;
            text_x = tx;
            text_y = ty;

            return result;
        }
    };
    marker_t _marker;

   public:
    void pointerChange(void) {
        draw_param.misc_pointer.add(1);
    }

    void update(draw_param_t* param) override {
        {
            int mark_x;
            int mark_y;
            switch (param->alarm_reference) {
                case draw_param_t::alarm_reference_lowest:
                    mark_x = param->frame->low_x;
                    mark_y = param->frame->low_y;
                    break;

                case draw_param_t::alarm_reference_highest:
                    mark_x = param->frame->high_x;
                    mark_y = param->frame->high_y;
                    break;

                case draw_param_t::alarm_reference_average:
                    mark_x = -1;
                    mark_y = -1;
                    break;

                default:
                    mark_x = frame_width >> 1;
                    mark_y = frame_height >> 1;
                    break;
            }
            uint32_t temp =
                mark_x < 0
                    ? param->frame->temp[param->frame->average]
                    : param->frame->pixel_raw[mark_x + (frame_width * mark_y)];
            bool txtmod = (temp != _marker.raw);
            if (_marker.update(temp, _client_rect, mark_x, mark_y,
                               param->font)) {
                // invalidate();
            }
        }

        // clearInvalidate();
    }

    void draw(draw_param_t* param, M5Canvas* canvas, int32_t canvas_y,
              int32_t h) override {
        // if (_client_rect.empty()) { return; }
        // {
        //     int32_t x, y, w, h;
        //     canvas->setClipRect(_client_rect.x, _client_rect.y - canvas_y,
        //     _client_rect.w, _client_rect.h); canvas->getClipRect(&x, &y, &w,
        //     &h); if (0 == (w + h)) { return; }
        // }

        int32_t y1 = _client_rect.y;
        for (int32_t fy = 1; fy < frame_height; ++fy) {
            int32_t y0 = y1;
            y1 = _client_rect.y + (fy * _client_rect.h) / (frame_height - 1);

            if (y1 - canvas_y < 0) {
                continue;
            }
            if (y0 - canvas_y >= _client_rect.bottom()) {
                break;
            }

            int32_t boxHeight = y1 - y0;
            if (boxHeight == 0) continue;

            int32_t v0;
            int32_t v1 = ((param->frame->pixel_raw[(fy - 1) * frame_width] -
                           param->range_temp_lower)
                          << 16) /
                         boxHeight;
            int32_t v2;
            int32_t v3 = ((param->frame->pixel_raw[(fy)*frame_width] -
                           param->range_temp_lower)
                          << 16) /
                         boxHeight;

            int32_t x1 = 0;
            for (int32_t fx = 1; fx < frame_width; ++fx) {
                int32_t x0       = x1;
                x1               = (fx * _client_rect.w) / (frame_width - 1);
                int32_t boxWidth = x1 - x0;
                v0               = v1;
                v1 = ((param->frame->pixel_raw[fx + (fy - 1) * frame_width] -
                       param->range_temp_lower)
                      << 16) /
                     boxHeight;
                v2 = v3;
                v3 = ((param->frame->pixel_raw[fx + (fy)*frame_width] -
                       param->range_temp_lower)
                      << 16) /
                     boxHeight;
                if (boxWidth == 0) continue;
                int32_t divider = boxWidth * param->temp_diff;

                int32_t ypos = y0 - canvas_y;
                int32_t by   = 0;
                if (ypos < 0) {
                    by   = -ypos;
                    ypos = 0;
                }
                for (; by < boxHeight && ypos < canvas->height();
                     ++by, ++ypos) {
                    int32_t v02 = (v0 * (boxHeight - by) + v2 * by) / divider;
                    int32_t v13 = (v1 * (boxHeight - by) + v3 * by) / divider;
                    auto img_buf =
                        &((m5gfx::swap565_t*)
                              canvas->getBuffer())[_client_rect.x + x0 +
                                                   ypos * canvas->width()];
                    for (int32_t bx = 0; bx < boxWidth; ++bx) {
                        int32_t v   = (v02 * (boxWidth - bx) + v13 * bx) >> 8;
                        img_buf[bx] = m5gfx::getSwap16(
                            (param->color_map[(v < 0)     ? 0
                                              : (v > 255) ? 255
                                                          : v]));
                    }
                }
            }
        }
        if (draw_param.misc_pointer !=
            draw_param.misc_pointer_t::misc_pointer_off) {
            int y = _marker.mark_y + _client_rect.y - canvas_y;
            // if ((y << 1) - (canvas->height()) < 20)
            {
                int x = _marker.mark_x + _client_rect.x;
                canvas->setColor(abs(15 - (int)(31 & draw_param.draw_count)) *
                                 0x0F0F0Fu);
                canvas->drawCircle(x, y, 4);
                canvas->drawFastVLine(x, y - 6, 13);
                canvas->drawFastHLine(x - 6, y, 13);
            }
        }

        if (draw_param.misc_pointer ==
            draw_param.misc_pointer_t::
                misc_pointer_pointtxt) {  // ||
                                          // draw_param.show_reference_name)
                                          // {
            int32_t marker_h = _marker.txtimg.height();
            int32_t y        = _client_rect.y + _marker.text_y - canvas_y;
            if (((y - marker_h) << 1) < canvas->height()) {
                int32_t x = _client_rect.x + _marker.text_x;
                _marker.txtimg.pushSprite(canvas,
                                          x - (_marker.txtimg.width() >> 1),
                                          y - (marker_h >> 1), 0);
            }
        }
    }
};

class graph_ui_t : public ui_base_t {
    int32_t _range_lowest;
    int32_t _range_highest;
    uint16_t _current_index = 0;
    int32_t _step_raw;
    uint8_t _last_update_count;

   public:
    void setup(draw_param_t* param) {
        _range_lowest  = param->frame->temp[framedata_t::lowest];
        _range_highest = param->frame->temp[framedata_t::highest];
    }

    void update(draw_param_t* param) override {
        _current_index = param->graph_data.current_idx;

        if (!_client_rect.empty()) {  // Obtain the maximum and minimum values
                                      // within the displayed range.
            int32_t t0  = UINT16_MAX;
            int32_t t1  = 0;
            uint8_t idx = _current_index - _client_rect.w;
            for (int i = 0; i < _client_rect.w; ++i) {
                t0 = std::min<int32_t>(
                    t0,
                    param->graph_data.temp_arrays[framedata_t::lowest][idx]);
                t1 = std::max<int32_t>(
                    t1,
                    param->graph_data.temp_arrays[framedata_t::highest][idx]);
                ++idx;
            }
            int32_t diff    = (t1 - t0) >> 3;
            int32_t new_low = (_range_lowest * 3 + t0 - diff) >> 2;
            if (new_low < 0) new_low = 0;
            int32_t new_high = (_range_highest * 3 + t1 + diff + 3) >> 2;
            if (new_high > UINT16_MAX) new_high = UINT16_MAX;
            if (new_low != _range_lowest || new_high != _range_highest) {
                // invalidate();
                _range_lowest  = new_low;
                _range_highest = new_high;
            }
            int32_t graph_temp_diff = _range_highest - _range_lowest + 1;

            size_t step_index = 0;
            int fontHeight    = draw_param.font_height;
            while ((graph_temp_diff * fontHeight) >
                       (_client_rect.h * step_table[step_index] << 7) &&
                   step_index < (step_table_len - 1)) {
                ++step_index;
            }
            _step_raw = step_table[step_index] * 128;
        }
    }

    void draw(draw_param_t* param, M5Canvas* canvas, int32_t canvas_y,
              int32_t h) override {
        int32_t graph_temp_diff = _range_highest - _range_lowest + 1;

        {
            int32_t xs = _current_index & 15;
            int ystart = canvas_y - _client_rect.y;
            int yend   = ystart + h;

            canvas->setTextDatum(textdatum_t::bottom_left);
            canvas->setTextSize(1);
            int fontHeight = draw_param.font_height;

            int32_t raw =
                ((_client_rect.h - ystart) * graph_temp_diff / _client_rect.h) +
                _range_lowest;
            int32_t line_idx = (raw - raw_step_offset) / _step_raw;
            for (int y = ystart; y < yend + fontHeight; ++y) {
                int32_t prev_raw = line_idx;
                int32_t i        = _client_rect.h - (y + 1);
                raw = (i * graph_temp_diff / _client_rect.h) + _range_lowest;
                int v =
                    ((raw - param->range_temp_lower) << 8) / param->temp_diff;
                line_idx         = (raw - raw_step_offset) / _step_raw;
                v                = (v < 0) ? 0 : (v > 255) ? 255 : v;
                uint16_t color   = param->color_map[v];
                uint16_t bgcolor = (color >> 2) & 0x39E7;
                int32_t draw_y   = y + _client_rect.y - canvas_y;
                if (prev_raw == line_idx) {
                    if (draw_y < h) {
                        canvas->drawFastHLine(_client_rect.x, draw_y,
                                              _client_rect.w, bgcolor);
                        canvas->setColor(bgcolor + 0x2104);
                        for (int x = _client_rect.w - xs; x >= 0; x -= 16) {
                            canvas->drawPixel(_client_rect.x + x, draw_y);
                        }
                    }
                } else {
                    int gauge_value = convertRawToCelsius(prev_raw * _step_raw +
                                                          raw_step_offset);
                    canvas->setTextColor(((color >> 1) & 0x7BEF) + 0x630C);
                    canvas->drawNumber(gauge_value, _client_rect.x + 1, draw_y);
                    if (draw_y >= h) {
                        break;
                    }
                    canvas->drawFastHLine(_client_rect.x, draw_y,
                                          _client_rect.w, bgcolor + 0x2104);
                }
            }

            for (int i = 0; i < 4; ++i) {
                uint8_t idx = (_current_index - _client_rect.w);
                canvas->setColor(graph_color_table[i]);
                int y = _client_rect.h -
                        (1 + (int32_t)((param->graph_data.temp_arrays[i][idx] -
                                        _range_lowest) *
                                       _client_rect.h) /
                                 graph_temp_diff);
                for (int gi = 0; gi < _client_rect.w; ++gi) {
                    uint8_t prev_idx = idx;
                    ++idx;
                    int prev_y = y;
                    y          = _client_rect.h -
                        (1 + (int32_t)((param->graph_data.temp_arrays[i][idx] -
                                        _range_lowest) *
                                       _client_rect.h) /
                                 graph_temp_diff);
                    int y0 = (y < prev_y) ? y : prev_y;
                    int y1 = (y > prev_y) ? y : prev_y;
                    canvas->fillRect(_client_rect.x + gi,
                                     _client_rect.y + y0 - canvas_y, 1,
                                     y1 - y0 + 1);
                }
            }
        }
    }
};

class infotext_ui_t : public ui_base_t {
    static constexpr int _text_count = 4;
    char _value_text[_text_count][8];
    int _value_x10[_text_count];
    int16_t _text_width = 0;
    bool _text_mod[_text_count];
    bool _two_line    = false;
    bool _draw_title  = false;
    float _textsize_x = 1.0f;
    float _textsize_y = 1.0f;

   public:
    void update(draw_param_t* param) override {
        if (isModified(param)) {
            for (int i = 0; i < _text_count; ++i) {
                float ftmp = convertRawToCelsius(param->frame->temp[i]);
                int tmp =
                    roundf(convertRawToCelsius(param->frame->temp[i]) * 10);
                bool mod     = (_value_x10[i] != tmp);
                _text_mod[i] = mod;
                if (mod) {
                    _value_x10[i] = tmp;
                    snprintf(_value_text[i], sizeof(_value_text[0]), "%5.1f",
                             ftmp);
                }
            }
        }

        size_t tw1  = display.textWidth("High");
        size_t tw2  = display.textWidth("888.8");
        float sw1   = (float)_client_rect.w / (tw1 + tw2);
        float sw2   = (float)_client_rect.w / tw2;
        float sh    = (float)_client_rect.h / (draw_param.font_height * 8);
        _two_line   = (sw1 < 1.0f) && (sh > 1.0f);
        _text_width = _client_rect.w;
        _draw_title = true;
        if (_two_line) {
            _textsize_x = sw2 < 1.0f ? 1.0f : sw2;
            _textsize_y = sh;
        } else {
            if (sw1 < 1.0f) {
                _draw_title = false;
                sw1         = sw2;
            } else {
                _text_width = tw2 * sw1;
            }
            _textsize_x = sw1 < 1.0f ? 1.0f : sw1;
            _textsize_y = sh * 2;
        }
    }

    void draw(draw_param_t* param, M5Canvas* canvas, int32_t canvas_y,
              int32_t h) override {
        canvas->fillScreen(TFT_BLACK);
        for (uint_fast8_t i = 0; i < _text_count; ++i) {
            canvas->setTextColor(graph_color_table[i]);
            if (_draw_title) {
                canvas->setTextSize(1, _textsize_y);
                canvas->setTextDatum(textdatum_t::top_left);
                canvas->drawString(graph_text_table[i], _client_rect.x,
                                   _client_rect.y +
                                       (_client_rect.h * i / _text_count) -
                                       canvas_y);
            }

            canvas->setTextSize(_textsize_x, _textsize_y);
            canvas->setTextDatum(textdatum_t::top_right);
            int32_t y0 = _client_rect.h * (i) / _text_count;
            if (_two_line) {
                int32_t y1 = _client_rect.h * (i + 1) / _text_count;
                y0         = (y1 + y0) / 2;
            }
            canvas->drawString(_value_text[i], _client_rect.right(),
                               _client_rect.y + y0 - canvas_y);
        }
    }
};

class hist_ui_t : public ui_base_t {
    uint16_t* _histgram    = nullptr;
    uint16_t* _prev_hist_x = nullptr;
    bool* _prev_hist_line  = nullptr;
    uint16_t _hist_len     = 0;

   public:
    void update(draw_param_t* param) override {
        if (_client_rect.empty()) {
            return;
        }

        int hist_len = _client_rect.h;
        if (_hist_len != hist_len) {
            _hist_len = hist_len;
            if (_histgram) free(_histgram);

            _histgram =
                (typeof(_histgram))malloc(hist_len * sizeof(_histgram[0]));
        }

        size_t step_index = 0;
        int fontHeight    = param->font_height;
        while ((param->temp_diff * fontHeight) >
                   (hist_len * step_table[step_index] << 7) &&
               step_index < (step_table_len - 1)) {
            ++step_index;
        }
        _step_raw = step_table[step_index] * 128;

        memset(_histgram, 0, _hist_len * sizeof(_histgram[0]));

        {  // Histogram Aggregation.
            int hist_max  = _hist_len - 1;
            int temp_low  = param->range_temp_lower;
            int temp_diff = param->temp_diff;
            for (int idx = 0; idx < frame_width * frame_height; ++idx) {
                int hist_idx =
                    (param->frame->pixel_raw[idx] - temp_low) * (hist_max + 1);
                hist_idx = (hist_idx << 6) / temp_diff;
                int rate = hist_idx & 63;
                hist_idx >>= 6;
                hist_idx = (hist_idx < 0)          ? 0
                           : (hist_idx > hist_max) ? hist_max
                                                   : hist_idx;
                _histgram[hist_idx] += 64 - rate;
                _histgram[++hist_idx > hist_max ? hist_max : hist_idx] += rate;
            }
        }
    }
    int32_t _step_raw;

    void draw(draw_param_t* param, M5Canvas* canvas, int32_t canvas_y,
              int32_t h) override {
        int32_t drawX      = _client_rect.x;
        int32_t drawY      = _client_rect.y - canvas_y;
        int32_t drawWidth  = _client_rect.w;
        int32_t drawHeight = _client_rect.h;
        canvas->setTextSize(1);
        int32_t fontHeight = param->font_height;

        // rotateCoordinate(param->rotation, drawX, drawY, drawWidth,
        // drawHeight);
        // param->gfx->setClipRect(drawX, drawY, drawWidth, drawHeight);
        // param->gfx->clearClipRect();
        canvas->setTextDatum(textdatum_t::bottom_left);

        int32_t x_offset = 0;

        int32_t y_value = canvas_y - _client_rect.y;
        int32_t y       = 0;
        if (y_value < 0) {
            y       = -y_value;
            y_value = 0;
        }
        int32_t i   = drawHeight - (y_value);
        int32_t raw = ((i * param->temp_diff / drawHeight) +
                       param->range_temp_lower - raw_step_offset) /
                      _step_raw;
        for (; y < h + fontHeight; ++y, ++y_value) {
            i                = drawHeight - (y_value + 1);
            int32_t prev_raw = raw;
            raw              = ((i * param->temp_diff / drawHeight) +
                   param->range_temp_lower - raw_step_offset) /
                  _step_raw;
            bool drawline = (prev_raw != raw);
            uint16_t color =
                param->color_map[((i < 0 ? 0 : i) << 8) / (drawHeight + 1)];
            if (drawline) {
                int gauge_value =
                    convertRawToCelsius(prev_raw * _step_raw + raw_step_offset);
                // img->setTextColor(((color >> 1) & 0x7BEF) + 0x630C);
                // img->drawNumber(gauge_value, 1, y);
                canvas->setTextColor(((color >> 1) & 0x7BEF) + 0x630C);
                canvas->drawNumber(gauge_value, _client_rect.x, y);
            }

            if (i >= 0) {
                int cw = drawWidth;
                int ch = drawHeight;
                int x  = (_histgram[i] * cw * ch + (1 << 18)) >> 19;
                // int px          = _prev_hist_x[i];
                x = (x < 0) ? 0 : (x > cw) ? cw : x;
                // _prev_hist_x[i] = x;

                uint16_t bgcolor = (color >> 2) & 0x39E7;
                if (drawline) {
                    bgcolor += 0x2104;
                }
                {
                    // int rx = 0;
                    // int ry = y;
                    // int rw = drawWidth - x;
                    // int rh = 1;
                    // rotateCoordinate(param->rotation, rx, ry, rw, rh);
                    canvas->fillRect(_client_rect.x, y, drawWidth - x, 1,
                                     bgcolor);
                    // if (_prev_hist_line[i] != drawline || isInvalidated())
                    {
                        // _prev_hist_line[i] = drawline;
                        canvas->fillRect(_client_rect.x + drawWidth - x, y,
                                         drawWidth, 1, color);
                        // px = 0;
                    }
                }
            }
        }
    }
};

infotext_ui_t text_ui;
hist_ui_t hist_ui;
image_ui_t image_ui;
graph_ui_t graph_ui;
header_ui_t header_ui;
battery_ui_t battery_ui;

void changeLayout_Config(void) {
    draw_param.in_config_mode = true;
    // draw_param.misc_layout = draw_param.misc_layout | 0x80;
    overlay_ui.setTargetRect(
        {display.width() >> 1, display.height() >> 1, 0, 0});
}

void changeLayout_Normal(bool next = true) {
    if (next) {
        draw_param.misc_layout.add(1);
    }
    draw_param.in_config_mode = false;
}

uint8_t changeLayout(uint8_t layout_idx) {
    int disp_w = display.width();
    int disp_h = display.height();

    battery_ui.setTargetRect(
        {disp_w - battery_ui_width, 0, battery_ui_width, disp_h});
    // battery_ui.setTargetRect({disp_w + 2, 0, battery_line_width, disp_h});
    const int32_t ox = 1;
    const int32_t oy = 1;
    disp_w -= (ox * 2) + battery_ui_width;
    disp_h -= (oy * 2);

    // in_config_mode = layout_idx & 0x80;

    rect_t header_rect = {ox, oy, disp_w, header_ui_height};

    if (draw_param.in_config_mode) {
        // static constexpr const int32_t header_height = 24;

        {
            config_ui.setTargetRect(
                {ox, oy + header_ui_height, disp_w, disp_h - header_ui_height});
            //*
            ui_base_t* ui_list[] = {&image_ui, &hist_ui, &graph_ui, &text_ui};
            for (auto ui : ui_list) {
                auto r = ui->getTargetRect();
                r.y    = oy + header_ui_height;
                r.h    = 0;
                ui->setTargetRect(r);
            }
            //*/
        }
    } else {
        rect_t image_rect;
        rect_t hist_rect;
        rect_t text_rect;
        rect_t graph_rect;

        // if (disp_w < disp_h) {
        //     disp_h -= battery_line_width;

        //     battery_ui.setTargetRect({0, disp_h, disp_w,
        //     battery_line_width});

        //     int image_w = disp_w;
        //     int image_h = disp_h;

        //     if (layout_idx >= 4) {
        //         layout_idx = 0;
        //     }
        //     image_h    = image_w * 4 / 3;
        //     image_rect = {0, 0, image_w, image_h};
        //     switch (layout_idx) {
        //         default:
        //             hist_rect  = {0, image_h, disp_w, disp_h - image_h};
        //             graph_rect = {0, image_h, 0, hist_rect.h};
        //             text_rect  = {0, image_h, 0, hist_rect.h};
        //             break;

        //         case 1:
        //             hist_rect  = {disp_w, image_h, 0, disp_h - image_h};
        //             graph_rect = {0, image_h, disp_w, disp_h - image_h};
        //             text_rect  = {disp_w, image_h, 0, hist_rect.h};
        //             break;

        //         case 2:
        //             image_rect = {0, 0, disp_h * 3 / 8, disp_h >> 1};
        //             hist_rect = {image_rect.right(), 0, disp_w -
        //             image_rect.right(),
        //                         image_rect.h};
        //             graph_rect = {0, image_rect.bottom(), image_rect.w,
        //                         disp_h - image_rect.bottom()};
        //             text_rect  = {graph_rect.right(), graph_rect.y,
        //                         disp_w - graph_rect.right(), graph_rect.h};
        //             break;

        //         case 3:
        //             hist_rect  = {disp_w, image_h, 0, disp_h - image_h};
        //             graph_rect = {0, image_h, 0, disp_h - image_h};
        //             text_rect  = {0, image_h, disp_w, disp_h - image_h};
        //             break;
        //     }
        //     config_ui.setTargetRect({0, 0, 0, disp_h});
        // } else
        {
            int image_w = disp_w;
            int image_h = disp_h;

            // image_w    = image_h * 4 / 3;
            // image_w    = display.width() * 3 >> 2;
            // image_h    = image_w * 3 >> 2;
            image_h    = ((display.width() * 9) >> 4) - (oy * 2);
            image_w    = image_h * 4 / 3;
            image_rect = {ox, oy, image_w, image_h};
            hist_rect  = {ox + disp_w, oy, 0, disp_h};
            graph_rect = {ox + disp_w, oy, 0, disp_h};
            text_rect  = {ox + disp_w, oy, 0, disp_h};

            header_rect.x = image_rect.right() + 1;
            header_rect.w = disp_w - image_rect.right();
            header_rect.h = 15;

            // header_rect.h = 0;

            switch (layout_idx) {
                default:
                    layout_idx = 0;
                    hist_rect  = {
                        image_rect.right() + 1, header_rect.bottom() + 1,
                        disp_w - image_rect.right(),
                        image_rect.bottom() - header_rect.bottom() - 1};
                    if (image_rect.h < disp_h) {
                        graph_rect = {image_rect.x, image_rect.bottom() + 1,
                                      disp_w >> 1,
                                      disp_h - image_rect.bottom()};
                        text_rect  = {graph_rect.right() + 1, graph_rect.y,
                                     disp_w - graph_rect.right(), graph_rect.h};
                    }
                    break;

                case 1:
                    graph_rect = {
                        image_rect.right() + 1, header_rect.bottom() + 1,
                        disp_w - image_rect.right(),
                        image_rect.bottom() - header_rect.bottom() - 1};
                    if (image_rect.h < disp_h) {
                        text_rect = {image_rect.x, image_rect.bottom() + 1,
                                     disp_w >> 1, disp_h - image_rect.bottom()};
                        hist_rect = {text_rect.right() + 1, text_rect.y,
                                     disp_w - text_rect.right(), text_rect.h};
                    }
                    break;

                case 2:
                    text_rect = {
                        image_rect.right() + 1, header_rect.bottom() + 1,
                        disp_w - image_rect.right(),
                        image_rect.bottom() - header_rect.bottom() - 1};
                    if (image_rect.h < disp_h) {
                        hist_rect  = {image_rect.x, image_rect.bottom() + 1,
                                     disp_w >> 1, disp_h - image_rect.bottom()};
                        graph_rect = {hist_rect.right() + 1, hist_rect.y,
                                      disp_w - hist_rect.right(), hist_rect.h};
                    }
                    break;

                case 3:
                    header_rect.w = disp_w / 3;
                    header_rect.x = (disp_w + ox) - header_rect.w;
                    image_rect    = {header_rect.x, header_rect.bottom() + 1,
                                  header_rect.w, header_rect.w * 3 >> 2};
                    graph_rect    = {header_rect.x, image_rect.bottom() + 1,
                                  header_rect.w, disp_h - image_rect.bottom()};
                    hist_rect = {ox, oy, disp_w - header_rect.w - 1, disp_h};
                    break;

                case 4:
                    header_rect.w = disp_w / 3;
                    header_rect.x = (disp_w + ox) - header_rect.w;
                    image_rect    = {header_rect.x, header_rect.bottom() + 1,
                                  header_rect.w, header_rect.w * 3 >> 2};
                    hist_rect     = {header_rect.x, image_rect.bottom() + 1,
                                 header_rect.w, disp_h - image_rect.bottom()};
                    graph_rect = {ox, oy, disp_w - header_rect.w - 1, disp_h};
                    break;

                case 5:
                    header_rect.w = disp_w / 3;
                    header_rect.x = (disp_w + ox) - header_rect.w;
                    image_rect    = {header_rect.x, header_rect.bottom() + 1,
                                  header_rect.w, header_rect.w * 3 >> 2};
                    text_rect     = {header_rect.x, image_rect.bottom() + 1,
                                 header_rect.w, disp_h - image_rect.bottom()};
                    graph_rect    = {ox, oy, disp_w - header_rect.w - 1,
                                  disp_h >> 1};
                    hist_rect     = {ox, graph_rect.bottom() + 1, graph_rect.w,
                                 disp_h - graph_rect.bottom()};
                    break;

                    // case 3:
                    //     image_rect.w = disp_h * 2 / 3;
                    //     image_rect.h = disp_h >> 1;
                    //     graph_rect = {image_rect.x, image_rect.bottom()+1,
                    //     image_rect.w, disp_h - image_rect.bottom()};
                    //     hist_rect = {image_rect.right()+1, image_rect.top(),
                    //     disp_w - image_rect.right(), disp_h}; break;

                    // case 4:
                    //     image_rect.w = disp_h * 2 / 3;
                    //     image_rect.h = disp_h >> 1;
                    //     hist_rect = {image_rect.x, image_rect.bottom()+1,
                    //     image_rect.w, disp_h - image_rect.bottom()};
                    //     graph_rect = {image_rect.right()+1, image_rect.top(),
                    //     disp_w - image_rect.right(), disp_h}; break;

                    // case 5:
                    //     image_rect.w = disp_h * 2 / 3;
                    //     image_rect.h = disp_h >> 1;
                    //     graph_rect = {image_rect.right()+1, image_rect.y,
                    //                 disp_w - image_rect.right(),
                    //                 image_rect.h};
                    //     hist_rect  = {image_rect.x, image_rect.bottom()+1,
                    //     image_rect.w,
                    //                 disp_h - image_rect.bottom()};
                    //     text_rect  = {image_rect.right()+1,
                    //     image_rect.bottom()+1,
                    //                 disp_w - image_rect.right(),
                    //                 disp_h - image_rect.bottom()};
                    //     break;
            }
            config_ui.setTargetRect({0, disp_h, disp_w, 0});
        }
        graph_ui.setTargetRect(graph_rect);
        image_ui.setTargetRect(image_rect);
        text_ui.setTargetRect(text_rect);
        hist_ui.setTargetRect(hist_rect);
    }
    header_ui.setTargetRect(header_rect);

    return layout_idx;
}

void drawTask(void*) {
    ui_base_t* ui_list[] = {&battery_ui, &text_ui,   &hist_ui,
                            &image_ui,   &graph_ui,  &config_ui,
                            &header_ui,  &qrcode_ui, &overlay_ui};
    static constexpr const uint32_t disp_buf_height = 16;
    static constexpr const size_t disp_buf_count =
        3;  // 描画バッファの数。jpegエンコーダのqueueにセットする分があるため3とする
    M5Canvas disp_buf[disp_buf_count];
    uint8_t disp_buf_idx   = 0;
    uint8_t disp_queue_idx = 0;

    int32_t disp_width  = display.width();
    int32_t disp_height = display.height();
    auto depth          = display.getColorDepth();
    for (int i = 0; i < disp_buf_count; ++i) {
        disp_buf[i].deleteSprite();
        disp_buf[i].setPsram(false);
        disp_buf[i].setColorDepth(depth);
        disp_buf[i].createSprite(disp_width, disp_buf_height);
        disp_buf[i].startWrite();
    }
    {
        rect_t rect = {disp_width >> 1, disp_height >> 1, 0, 0};
        for (auto ui : ui_list) {
            ui->setClientRect(rect);
        }
    }
    do {
        delay(1);
    } while (idx_recv < 2);

    // uint8_t prev_color_table_idx = 0;
    uint32_t prev_msec = millis();
    uint32_t prev_wdt  = 0;

    draw_param.setup(&display, framedata, 2);
    // draw_param.setColorTable(color_map_table[0]);
    graph_ui.setup(&draw_param);

    bool prev_misc_staff = false;

    uint8_t prev_layout = 255;

    display.startWrite();
    for (;;) {
        ++draw_param.draw_count;

        // {
        //     auto r = config_ui.getTargetRect();
        //     r.x = (draw_param.draw_count & 63);
        //     r.y = (draw_param.draw_count & 63);
        //     config_ui.setTargetRect(r);
        // }

        // if (display.getRotation() != draw_param.misc_rotation) {
        //     display.setRotation(draw_param.misc_rotation);
        // }

        // if (disp_width != display.width()) {
        //     disp_width = display.width();
        //     disp_height = display.height();
        //     auto depth = display.getColorDepth();
        //     for (int i = 0; i < disp_buf_count; ++i) {
        //         disp_buf[i].deleteSprite();
        //         disp_buf[i].setPsram(false);
        //         disp_buf[i].setColorDepth(depth);
        //         disp_buf[i].createSprite(disp_width, disp_buf_height);
        //     }
        // }
        if (disp_buf[0].getFont() != draw_param.font) {
            for (int i = 0; i < disp_buf_count; ++i) {
                disp_buf[i].setFont(draw_param.font);
            }
        }
        // if (prev_color_table_idx != color_map_table_idx) {
        //     prev_color_table_idx = color_map_table_idx;
        //     draw_param.setColorTable(color_map_table[prev_color_table_idx]);
        // }

        if (prev_layout !=
            (draw_param.misc_layout | draw_param.in_config_mode << 7)) {
            draw_param.misc_layout = changeLayout(draw_param.misc_layout);
            prev_layout = draw_param.misc_layout | draw_param.in_config_mode
                                                       << 7;
        }

        uint32_t msec = millis();
        uint8_t wdt   = msec >> 6;
        prev_msec += 30;
        int32_t limit_delay = prev_msec - msec;
        if (prev_wdt != wdt) {
            prev_wdt = wdt;
            if (limit_delay <= 0) {
                limit_delay = 1;
            }
        }
        if (limit_delay > 0) {
            delay(limit_delay);
        } else {
            prev_msec += (-limit_delay) >> 1;
        }

        for (auto ui : ui_list) {
            ui->smoothMove();
        }

        draw_param.range_update();
        if (draw_param.update(idx_recv)) {
        }
        for (auto ui : ui_list) {
            ui->update(&draw_param);
        }
        uint32_t h = disp_buf_height;
        if (prev_misc_staff != (bool)draw_param.misc_staff) {
            prev_misc_staff = !prev_misc_staff;
            if (prev_misc_staff) {
                display.drawJpg(jpg_staff, sizeof(jpg_staff), 0, 0,
                                display.width(), display.height(), 0, 0, 1.0f,
                                1.0f, datum_t::middle_center);
            }
        }

        bool screenshot =
            screenshot_holder.initCapture(disp_width, disp_height);
        for (uint32_t y = 0; y < disp_height; y += h) {
            if (h >= disp_height - y) {
                h = disp_height - y;
            }

            do {
                disp_buf_idx =
                    (disp_buf_idx < disp_buf_count - 1) ? disp_buf_idx + 1 : 0;
                // JPGストリームキュー待機中のバッファの使用を避ける
                // これにより、通信エラー時にブラウザの表示画像が乱れることを防止する
            } while (disp_buf_idx == disp_queue_idx);

            auto canvas = &disp_buf[disp_buf_idx];
            canvas->clearClipRect();
            canvas->fillScreen(draw_param.background_color);
            int32_t dummy, wid, hei;
            for (auto ui : ui_list) {
                auto rect = ui->getClientRect();
                if (rect.empty()) {
                    continue;
                }
                canvas->setClipRect(rect.x, rect.y - y, rect.w, rect.h);
                canvas->getClipRect(&dummy, &dummy, &wid, &hei);
                if (wid <= 0 || hei <= 0) {
                    continue;
                }

                ui->draw(&draw_param, canvas, y, h);
            }
            if (!prev_misc_staff) {
                canvas->pushSprite(&display, 0, y);
            }
            if (screenshot) {
                screenshot = screenshot_holder.addQueue(canvas, y);
                if (screenshot) {
                    disp_queue_idx = disp_buf_idx;
                }
            }
        }
    }
    display.endWrite();
}

static bool sync_rtc_ntp(void) {
    if (sntp_get_sync_status() != SNTP_SYNC_STATUS_COMPLETED) return false;

    time_t t = time(nullptr) + 1;        // Advance one second.
    while (t > time(nullptr)) delay(1);  /// Synchronization in seconds
    M5.Rtc.setDateTime(gmtime(&t));

    return true;
}

// volatile bool requestWiFi = false;

static void wifiTask(void*) {
    bool rtc_sync = false;
    config_param_t::net_setup_mode_t prev_net_setup_mode =
        config_param_t::net_setup_mode_off;
    bool prev_ap_connected  = false;
    bool prev_sta_connected = false;
    int connecting_retry    = 0;

    for (;;) {
        delay(1);
        if (prev_sta_connected != WiFi.isConnected()) {
            prev_sta_connected = !prev_sta_connected;
            if (!prev_sta_connected) {
                soundWiFiDisconnected();
                connecting_retry = 0;
            } else {
                soundWiFiConnected();
                configTime(draw_param.oncloud_timezone_sec, 0, ntp_server[0],
                           ntp_server[1], ntp_server[2]);
                std::string strbuf = "http://";
                strbuf += WiFi.localIP().toString().c_str();
                strbuf += "/";
                draw_param.net_url_ip = strbuf;

                if (!draw_param.net_tmp_ssid.empty()) {
                    draw_param.net_tmp_ssid.clear();
                    draw_param.net_tmp_pwd.clear();
                    if (draw_param.net_setup_mode !=
                        draw_param.net_setup_mode_off) {
                        draw_param.net_setup_mode =
                            draw_param.net_setup_mode_off;
                        // qrcode_ui.hide();
                    }
                }
                wifi_config_t current_conf;
                if (esp_wifi_get_config((wifi_interface_t)ESP_IF_WIFI_STA,
                                        &current_conf) == ESP_OK) {
                    draw_param.sys_ssid = (char*)(current_conf.sta.ssid);
                }
                if (!rtc_sync) {
                    rtc_sync = sync_rtc_ntp();
                }
                if (0u == draw_param.cloud_ip) {
                    WiFiGenericClass::hostByName(cloud_server_name,
                                                 draw_param.cloud_ip);
                }
            }
        }

        if (prev_net_setup_mode != draw_param.net_setup_mode) {
            qrcode_ui.hide();
            switch (prev_net_setup_mode) {
                default:
                    break;

                case config_param_t::net_setup_mode_smartconfig:
                    WiFi.stopSmartConfig();
                    break;

                case config_param_t::net_setup_mode_accesspoint:
                    WiFi.scanDelete();
                    dnsServer.stop();
                    break;
            }
            prev_ap_connected = false;

            prev_net_setup_mode = draw_param.net_setup_mode;
            switch (prev_net_setup_mode) {
                default:
                case config_param_t::net_setup_mode_off:
                    WiFi.mode((wifi_mode_t)(WiFi.getMode() &
                                            ~wifi_mode_t::WIFI_MODE_AP));
                    break;

                case config_param_t::net_setup_mode_smartconfig:
                    draw_param.net_running_mode =
                        draw_param.net_running_mode_offline;
                    WiFi.mode(wifi_mode_t::WIFI_MODE_STA);
                    WiFi.beginSmartConfig();
                    break;

                case config_param_t::net_setup_mode_accesspoint:
                    draw_param.net_running_mode =
                        draw_param.net_running_mode_offline;
                    WiFi.softAP(draw_param.net_apmode_ssid,
                                draw_param.net_apmode_pass);
                    WiFi.softAPConfig(draw_param.net_apmode_ipaddr,
                                      draw_param.net_apmode_ipaddr,
                                      draw_param.net_apmode_subnet);
                    WiFi.mode(wifi_mode_t::WIFI_MODE_APSTA);
                    WiFi.scanNetworks(true);
                    dhcps_dns_setserver(&(draw_param.dnsip));
                    static const uint16_t DNS_PORT = 53;
                    dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
                    dnsServer.start(DNS_PORT, "*",
                                    draw_param.net_apmode_ipaddr);
                    break;
            }
        }

        if (prev_net_setup_mode != draw_param.net_setup_mode_off) {
            switch (prev_net_setup_mode) {
                case draw_param.net_setup_mode_accesspoint:
                    dnsServer.processNextRequest();
                    if (prev_ap_connected != (bool)WiFi.softAPgetStationNum()) {
                        prev_ap_connected = !prev_ap_connected;
                        if (prev_ap_connected) {
                            soundWiFiConnected();
                            qrcode_ui.show(draw_param.net_ap_url.c_str());
                            // qrcode_ui.show(draw_param.net_url.c_str());
                        } else {
                            qrcode_ui.hide();
                        }
                    }
                    break;

                case draw_param.net_setup_mode_smartconfig:
                    if (WiFi.smartConfigDone()) {
                        draw_param.net_tmp_ssid.clear();
                        draw_param.net_tmp_pwd.clear();
                        if (draw_param.net_running_mode ==
                            draw_param.net_running_mode_offline) {
                            draw_param.net_running_mode =
                                draw_param.net_running_mode_lan_cloud;
                        }
                        draw_param.net_setup_mode =
                            draw_param.net_setup_mode_off;
                    }
                    continue;

                default:
                    break;
            }

            // requestWiFi = draw_param.net_running_mode !=
            // draw_param.net_running_mode_offline;
        }

        // {
        //     int retry = 128;
        //     do { delay(1); } while (((bool)draw_param.request_wifi_state) ==
        //     WiFi.isConnected() && !need_wifi_reconnect && --retry); if (retry
        //     == 0) { continue; }
        // }

        if (!need_wifi_reconnect &&
            (((bool)draw_param.request_wifi_state) == WiFi.isConnected())) {
            continue;
        }

        if ((need_wifi_reconnect || !((bool)draw_param.request_wifi_state))) {
            WiFi.disconnect(need_wifi_reconnect ||
                            draw_param.net_setup_mode ==
                                draw_param.net_setup_mode_off);
            need_wifi_reconnect = false;
            // WiFi.mode(WIFI_MODE_NULL);
        } else {
            if (connecting_retry) {
                --connecting_retry;
            }
            if (connecting_retry == 0) {
                if (!draw_param.net_tmp_ssid.empty()) {
                    WiFi.begin(draw_param.net_tmp_ssid.c_str(),
                               draw_param.net_tmp_pwd.c_str());
                    connecting_retry = 64;
                } else {
                    WiFi.begin();
                    connecting_retry = 512;
                }
                ESP_EARLY_LOGD("DEBUG", "WiFi begin() status:%d",
                               WiFi.status());
            }
        }
    }
}

static void cloudTask(void*) {
    std::string json_frame;

    static constexpr const int32_t prepare_sec = 3;

    delay(1024);

    time_t time_next_upload = 0;
    time_t time_prev_upload = time(nullptr);
    {
        auto interval_sec =
            config_param_t::cloud_interval_value[draw_param.cloud_interval];
        time_prev_upload = (time_prev_upload / interval_sec) * interval_sec;
    }

    draw_param.cloud_status =
        (draw_param.net_running_mode &
         draw_param.net_running_mode_t::net_running_mode_cloud)
            ? draw_param.cloud_status_t::cloud_timerwait
            : draw_param.cloud_status_t::cloud_disable;
    for (;;) {
        delay(64);
        if (!(draw_param.net_running_mode &
              draw_param.net_running_mode_cloud)) {
            if ((bool)(draw_param.request_wifi_state &
                       draw_param_t::net_running_mode_cloud)) {
                draw_param.request_wifi_state &=
                    ~draw_param_t::net_running_mode_cloud;
            }
        } else {
            switch (draw_param.cloud_status) {
                default:
                case draw_param.cloud_status_t::cloud_complete:
                case draw_param.cloud_status_t::cloud_disable:
                    draw_param.cloud_status =
                        draw_param.cloud_status_t::cloud_timerwait;
                    [[fallthrough]];
                case draw_param.cloud_status_t::cloud_timerwait:

                    // if (time_diff > prepare_sec) {
                    //     if (!(draw_param.net_running_mode &
                    //     draw_param.net_running_mode_lan)) { //
                    //     次の送信までの秒数までの待ち時間が指定秒数以上あればWiFiを切断
                    //         request_wifi = false;
                    //     }
                    // } else
                    {
                        auto t            = time(nullptr);
                        auto interval_sec = config_param_t::cloud_interval_value
                            [draw_param.cloud_interval];
                        int32_t time_diff = (time_prev_upload - t);
                        if (time_diff >= 0) {
                            time_diff = 0;
                        }
                        time_diff += interval_sec;

                        draw_param.cloud_countdown_sec =
                            (0 <= time_diff) ? time_diff : 0;

                        if (0 > time_diff) {
                            draw_param.cloud_status =
                                draw_param.cloud_status_t::cloud_connection;
                            time_prev_upload =
                                (t / interval_sec) * interval_sec;
#if defined(ESP_LOGD)
                            auto tm = localtime(&t);
                            ESP_LOGD(
                                "DEBUG",
                                "upload time : %04d/%02d/%02d  %02d:%02d:%02d",
                                tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
                                tm->tm_hour, tm->tm_min, tm->tm_sec);

                            tm = gmtime(&t);
                            ESP_LOGD("DEBUG",
                                     "gmt : %04d/%02d/%02d  %02d:%02d:%02d\n",
                                     tm->tm_year + 1900, tm->tm_mon + 1,
                                     tm->tm_mday, tm->tm_hour, tm->tm_min,
                                     tm->tm_sec);
#endif
                            // ezdata_step = 1;

                            // JSON生成中に別タスクの操作によってデータが変化する可能性があるため、ローカル変数にコピーしてから行う
                            framedata_t frame = *draw_param.frame;

                            // WiFi接続時の出力変動で画像が乱れる事があるため、フレームデータをローカルコピーした後でWiFi要求を行う。
                            draw_param.request_wifi_state |=
                                draw_param_t::net_running_mode_cloud;
                            json_frame = "{ \"payload\": ";
                            json_frame += frame.getJsonData();
                            json_frame += "}\r\n";
                        } else {
                            if ((time_diff > prepare_sec) &&
                                ((bool)(draw_param.request_wifi_state &
                                        draw_param_t::
                                            net_running_mode_cloud))) {
                                draw_param.request_wifi_state &=
                                    ~draw_param_t::net_running_mode_cloud;
                            }
                            delay(64);
                        }
                    }
                    break;

                case draw_param.cloud_status_t::cloud_error:
                case draw_param.cloud_status_t::cloud_connection:
                case draw_param.cloud_status_t::cloud_uploading:
                    if (WiFi.isConnected() && 0u != draw_param.cloud_ip) {
                        /*
                                            static constexpr const char* host =
                        "http://ezdata.m5stack.com/api/M5StickT-Lite-Data/";

                                            HTTPClient http_client;
                                            // http_client.setReuse(false);
                                            uint_fast16_t timeout = 4096;
                                            do {
                                                http_client.setTimeout(timeout);
                                                if (http_client.begin(host)) {
                                                    http_client.addHeader("Content-Type",
                        "application/json");
                                                    // Send HTTP POST request
                                                    draw_param.cloud_status =
                        draw_param.cloud_status_t::cloud_uploading; int
                        httpResponseCode =
                        http_client.POST((uint8_t*)json_frame.c_str(),
                        json_frame.length()); http_client.end();
                                                    if(httpResponseCode ==
                        HTTP_CODE_OK){ soundCloudSuccess();
                                                        draw_param.cloud_status
                        = draw_param.cloud_status_t::cloud_complete;
                                                        ESP_LOGD("DEBUG",
                        "Success upload data to ezdata"); break; } else {
                                                        soundCloudError();
                                                        draw_param.cloud_status
                        = draw_param.cloud_status_t::cloud_error;
                                                        ESP_LOGD("DEBUG", "Fail
                        to upload data,response code:%d\n",httpResponseCode);
                        draw_param.cloud_status =
                        draw_param.cloud_status_t::cloud_complete; break;
                                                    }
                                                }
                                                timeout = timeout * 3 >> 1;
                                            } while (timeout <= 8192);
                        //*/
                        WiFiClient wifi_client;
                        // ESP_EARLY_LOGD("DEBUG","CLOUD 1");
                        if (1 == wifi_client.connect(draw_param.cloud_ip, 80,
                                                     6144)) {
                            wifi_client.setTimeout(5);
                            wifi_client.setNoDelay(true);
                            // ESP_EARLY_LOGD("DEBUG","CLOUD 2");
                            draw_param.cloud_status =
                                draw_param.cloud_status_t::cloud_uploading;
                            wifi_client.print(
                                "POST /api/M5StickT-Lite-Data/ HTTP/1.1\r\n"
                                "Accept: */*\r\n"
                                "Connection: keep-alive\r\n"
                                "Content-Type: application/json; "
                                "charset=utf-8\r\n"
                                "DNT: 1\r\n"
                                "Origin: null\r\n"
                                "User-Agent: ESP32\r\n");
                            wifi_client.printf(
                                "Host: %s:80\r\nContent-Length: %d\r\n\r\n",
                                cloud_server_name, json_frame.length());
                            // ESP_EARLY_LOGD("DEBUG","CLOUD 3");
                            size_t len = json_frame.length();
                            auto p     = json_frame.c_str();
                            do {
                                size_t l = 1436 < len ? 1436 : len;
                                if (l != wifi_client.write(p, l)) {
                                    break;
                                }
                                // ESP_EARLY_LOGD("DEBUG","remain:%d", len);
                                p += l;
                                len -= l;
                            } while (len);
                            // if (wifi_client.write(json_frame.c_str(),
                            // json_frame.length()) == json_frame.length()) {
                            if (len) {
                                draw_param.cloud_status =
                                    draw_param.cloud_status_t::cloud_error;
                                soundCloudError();
                            } else {
                                // ESP_EARLY_LOGD("DEBUG","CLOUD 4");
                                wifi_client.print("\r\n\r\n");
                                bool success = false;
                                std::string linebuf;
                                int retry = 2048;
                                // ESP_EARLY_LOGD("DEBUG","CLOUD 5");
                                do {
                                    delay(1);
                                } while (wifi_client.connected() &&
                                         wifi_client.available() == 0 &&
                                         --retry >= 0);
                                if (wifi_client.connected()) {
                                    if (retry >= 0) {
                                        wifi_client.print("\r\n");
                                    }
                                    retry = 2048;
                                    do {
                                        delay(1);
                                        int c;
                                        while (0 <= (c = wifi_client.read())) {
                                            if (c == '\r') {
                                                continue;
                                            }
                                            if (c != '\n') {
                                                linebuf.append(1, (char)c);
                                                ++retry;
                                            } else {
                                                if (linebuf.size() == 0) {
                                                    retry = 0;
                                                    break;
                                                }
                                                // ESP_EARLY_LOGD("DEBUG", "%s",
                                                // linebuf.c_str());
                                                if (linebuf.compare(
                                                        "HTTP/1.1 200 OK") ==
                                                    0) {
                                                    // ESP_EARLY_LOGD("DEBUG",
                                                    // "upload success",
                                                    // linebuf.c_str());
                                                    success = true;
                                                }
                                                linebuf.clear();
                                            }
                                        }
                                    } while (wifi_client.connected() &&
                                             --retry >= 0);
                                }
                                if (success) {
                                    draw_param.cloud_status =
                                        draw_param
                                            .cloud_status_t::cloud_complete;
                                    soundCloudSuccess();
                                } else {
                                    draw_param.cloud_status =
                                        draw_param.cloud_status_t::cloud_error;
                                    soundCloudError();
                                }
                            }
                            wifi_client.stop();
                        }
                    }
                    //*/

                    break;
            }
        }
    }
}

/*
void connect_ezdata(void) {
    if (draw_param.oncloud_conf_valid) {
        if (setupWifi(str_ssid.c_str(), str_pass.c_str())) {
            int value = 0;
            getData(str_token.c_str(), "testData", value);

            if (setData(str_token.c_str(), "testData", ++value)) {
            }
        }
    }
}
*/

void setup(void) {
    // 最大の連続メモリ領域を後に残すため、敢えてここで確保しておき、準備が終わった後に解放する。
    auto dummy_alloc = heap_caps_malloc(
        heap_caps_get_largest_free_block(MALLOC_CAP_8BIT), MALLOC_CAP_8BIT);

    display.setBrightness(32);
    M5.begin();
    display.setRotation(1);
    display.drawBmp(bmp_logo, sizeof(bmp_logo), 0, 0, display.width(),
                    display.height(), 0, 0, 1.0f, 1.0f, datum_t::middle_center);

    {
        auto cfg             = M5.Speaker.config();
        cfg.sample_rate      = 48000;
        cfg.task_priority    = 3;
        cfg.task_pinned_core = APP_CPU_NUM;
        cfg.dma_buf_count    = 16;
        M5.Speaker.config(cfg);
        // M5.Speaker.setVolume(0);
    }
    /*
        gpio_config_t io_conf;
        io_conf.mode         = GPIO_MODE_DISABLE;
        io_conf.intr_type    = GPIO_INTR_DISABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
        io_conf.pin_bit_mask =
            1 << GPIO_NUM_0 | 1 << GPIO_NUM_2 | 1 << GPIO_NUM_4 | 1 <<
    GPIO_NUM_5 | 1 << GPIO_NUM_9 | 1 << GPIO_NUM_10 | 1 << GPIO_NUM_12 | 1 <<
    GPIO_NUM_13 | 1 << GPIO_NUM_14 | 1 << GPIO_NUM_15 | 1 << GPIO_NUM_18 | 1 <<
    GPIO_NUM_19 | 1 << GPIO_NUM_23 | 1 << GPIO_NUM_25 | 1 << GPIO_NUM_26 #if
    CORE_DEBUG_LEVEL == 0 | 1 << GPIO_NUM_1 | 1 << GPIO_NUM_3 #endif
            ;
        gpio_config(&io_conf);
    //*/

    command_processor::setup();
    for (int i = 0; i < 4; ++i) {
        draw_param.graph_data.temp_arrays[i] = (uint16_t*)malloc(
            draw_param.graph_data.data_len * sizeof(uint16_t));
    }

    // webサーバタスクは loopと同じ APP_CPUプライオリティ1
    // を指定、優劣をつけない
    xTaskCreatePinnedToCore(webserverTask, "webTask", 6144, &draw_param, 1,
                            nullptr, APP_CPU_NUM);

    // xTaskCreatePinnedToCore(screenshot_streamer_t::streamTask, "stream",
    // 2048, &screenshot_holder, 1, nullptr,
    //                         PRO_CPU_NUM);
    xTaskCreate(screenshot_streamer_t::streamTask, "stream", 2048,
                &screenshot_holder, 1, nullptr);

    auto macaddr = draw_param.macaddr;
    esp_read_mac(macaddr, ESP_MAC_WIFI_SOFTAP);
    snprintf(draw_param.net_apmode_ssid, sizeof(draw_param.net_apmode_ssid),
             "T-Lite_%02x%02x", macaddr[4], macaddr[5]);

    draw_param.net_hostname = draw_param.net_apmode_ssid;
    draw_param.net_hostname += ".local";

    char cbuf[32];

    // 初期トークン 4文字の数字を設定
    auto mi = micros() + macaddr[0] + macaddr[1] + macaddr[2] + macaddr[3] +
              macaddr[4] + macaddr[5];
    for (int i = 0; i < 4; ++i) {
        cbuf[i] = '0' + (mi % 10);
        mi >>= 4ull;
    }
    cbuf[4]                = '\0';
    draw_param.cloud_token = cbuf;

    draw_param.net_url_mdns = "http://";
    draw_param.net_url_mdns += draw_param.net_hostname;
    draw_param.net_url_mdns += "/";

    draw_param.net_url_ip = draw_param.net_url_mdns;

    // snprintf(cbuf, sizeof(cbuf), "http://%s/",
    // draw_param.net_apmode_ipaddr.toString().c_str()); draw_param.net_ap_url =
    // cbuf;
    draw_param.net_ap_url = draw_param.net_url_mdns + "wifi";

    snprintf(cbuf, sizeof(cbuf), "%02x%02x%02x%02x%02x%02x", macaddr[0],
             macaddr[1], macaddr[2], macaddr[3], macaddr[4], macaddr[5]);
    draw_param.cloud_url = "https://T-Lite.m5stack.com/";
    draw_param.cloud_url += cbuf;

    draw_param.loadNvs();

    display.setBrightness(
        draw_param.misc_brightness_value[draw_param.misc_brightness]);

    soundStartUp();

    xTaskCreatePinnedToCore(drawTask, "drawTask", 4096, nullptr, 1, nullptr,
                            PRO_CPU_NUM);

    delay(16);

    // check_oncloud();
    config_ui.setup();
    qrcode_ui.setTargetRect(
        {display.width() >> 1, display.height() >> 1, 0, 0});

    heap_caps_free(dummy_alloc);

    {
        wifi_config_t current_conf;
        WiFi.setHostname(draw_param.net_hostname.c_str());
        WiFi.setAutoReconnect(false);
        if (WiFi.begin()) {
            if (esp_wifi_get_config((wifi_interface_t)ESP_IF_WIFI_STA,
                                    &current_conf) == ESP_OK) {
                draw_param.sys_ssid = (char*)(current_conf.sta.ssid);
            }
        }
    }

    xTaskCreatePinnedToCore(wifiTask, "wifiTask", 4096, nullptr, 3, nullptr,
                            PRO_CPU_NUM);
    delay(512);

    {
        static constexpr size_t line_len = 64;
        char lines[6][line_len]          = {""};
        snprintf(lines[0], line_len, "ver:%d.%d.%d", firmware_ver_major,
                 firmware_ver_minor, firmware_ver_patch);
        size_t line_idx = 1;
        snprintf(lines[line_idx++], line_len, "Sensor:%s/%d%%",
                 draw_param.sens_refreshrate.getText(),
                 draw_param.sens_emissivity.get());
        if (draw_param.net_running_mode & draw_param.net_running_mode_cloud) {
            snprintf(lines[line_idx++], line_len, "Mode:%s(%s)",
                     draw_param.net_running_mode.getText(),
                     draw_param.cloud_interval.getText());
        } else {
            snprintf(lines[line_idx++], line_len, "Mode:%s",
                     draw_param.net_running_mode.getText());
        }
        if (draw_param.alarm_mode) {
            snprintf(lines[line_idx++], line_len, "Alarm:%s %3.1fC",
                     draw_param.alarm_mode.getText(),
                     convertRawToCelsius(draw_param.alarm_temperature));
        } else {
            snprintf(lines[line_idx++], line_len, "Alarm:%s",
                     draw_param.alarm_reference.getText());
        }

        if (draw_param.sys_ssid.length()) {
            snprintf(lines[line_idx++], line_len, "WiFi:%s",
                     draw_param.sys_ssid.c_str());
        } else {
            snprintf(lines[line_idx++], line_len, "WiFi: -- unset --");
        }
        snprintf(lines[line_idx++], line_len, "MAC:%02x%02x%02x%02x%02x%02x",
                 macaddr[0], macaddr[1], macaddr[2], macaddr[3], macaddr[4],
                 macaddr[5]);

        // overlay_ui.setClientRect({ display.width() >> 1, display.height() >>
        // 1, 0, 0});
        overlay_ui.show(128, lines[0], lines[1], lines[2], lines[3], lines[4],
                        lines[5]);
    }

    xTaskCreatePinnedToCore(cloudTask, "cloudTask", 8192, nullptr, 3, nullptr,
                            APP_CPU_NUM);

    // debug
    // pinMode(GPIO_NUM_32, OUTPUT);
    // pinMode(GPIO_NUM_33, OUTPUT);
}

void loop(void) {
    if (config_save_countdown) {
        auto br = draw_param.misc_brightness_value[draw_param.misc_brightness];
        if (display.getBrightness() != br) {
            display.setBrightness(br);
        }
        if (0 == --config_save_countdown) {
            draw_param.saveNvs();
        }
    }

    //     static uint8_t loop_count = 192;
    //     if (++loop_count == 0) {
    // // ESP_LOGE("DEBUG","largest_free:%d",
    // heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    //     }

    uint32_t msec = millis();

    {
        static uint32_t prev_sec;
        uint32_t sec = msec >> 10;
        if (prev_sec != sec) {
            prev_sec = sec;
            // debug
            {
                static uint32_t prev_dc;
                auto dc = draw_param.draw_count;
                ESP_EARLY_LOGD("DEBUG", "draw count:%d", dc - prev_dc);
                prev_dc = dc;
            }

            delay(1);
            if (M5.Power.getType() == m5::Power_Class::pmic_axp192) {
                draw_param.battery_state = M5.Power.isCharging();
                draw_param.battery_level = M5.Power.getBatteryLevel();
                static bool prev_acin;
                if (prev_acin != M5.Power.Axp192.isVBUS()) {
                    prev_acin = !prev_acin;
                    if (prev_acin) {
                        soundUSBConnected();
                    } else {
                        soundUSBDisconnected();
                    }
                }
            }
        }
    }

    static uint32_t _alarm_last_time = 0;
    static uint32_t _alarm_interval  = 500;
    // 温度アラーム判定
    if (((msec - _alarm_last_time) > _alarm_interval)) {
        auto frame   = &framedata[idx_recv];
        int temp_idx = 0;
        switch (draw_param.alarm_reference) {
            case draw_param_t::alarm_reference_highest:
                temp_idx = frame->highest;
                break;
            case draw_param_t::alarm_reference_lowest:
                temp_idx = frame->lowest;
                break;
            case draw_param_t::alarm_reference_center:
                temp_idx = frame->center;
                break;
            case draw_param_t::alarm_reference_average:
                temp_idx = frame->average;
                break;
        }

        enum alarm_state_t {
            alarm_none,
            alarm_on1,
            alarm_on2,
        };

        static alarm_state_t _current_alarm_state = alarm_state_t::alarm_none;

        if (_current_alarm_state == alarm_state_t::alarm_none) {
            _alarm_last_time = msec;
            _alarm_interval  = (_alarm_interval + 50) >> 1;
        } else {
            _alarm_last_time += _alarm_interval;
        }
        int temp   = (frame->temp[temp_idx]);
        bool alarm = false;
        switch (draw_param.alarm_mode) {
            case draw_param_t::alarm_mode_t::alarm_mode_hightemp:
                alarm = temp > draw_param.alarm_temperature;
                break;
            case draw_param_t::alarm_mode_t::alarm_mode_lowtemp:
                alarm = temp < draw_param.alarm_temperature;
                break;
        }
        alarm_state_t alarm_state = alarm_none;
        if (alarm) {
            alarm_state =
                (_current_alarm_state == alarm_on1) ? alarm_on2 : alarm_on1;
        }
        if (_current_alarm_state != alarm_state) {
            _current_alarm_state = alarm_state;

            static constexpr const uint16_t alarm_background_color[] = {
                0,
                0xF800,
                0xFFE0,
            };
            draw_param.background_color = alarm_background_color[alarm_state];

            bool buzzer = false;
            bool led    = false;

            if (draw_param.alarm_mode) {
                switch (alarm_state) {
                    default:
                        break;

                    case alarm_state_t::alarm_on1:
                        buzzer = true;
                        break;

                    case alarm_state_t::alarm_on2:
                        led = true;
                        break;
                }
            }

            // switch (draw_param.alarm_behavior) {
            //     case draw_param_t::alarm_behavior_t::alarm_behavior_beep:
            //         led = false;
            //         break;
            //     case draw_param_t::alarm_behavior_t::alarm_behavior_led:
            //         buzzer = false;
            //         break;
            //     default:
            //         break;
            // }

            _alarm_interval = 100;
            if (buzzer &&
                draw_param.misc_volume != draw_param.misc_volume_mute) {
                M5.Speaker.tone(4000, _alarm_interval);
            }
            M5.Power.setLed(led ? 128 : 0);
        }
    }

    /*
        uint32_t prev_sec = 0;
        uint32_t sec = msec / 1000;
        if (prev_sec != sec) {
            prev_sec = sec;
            // Serial.printf("fps: %d : upd %d : recv %d\n",
    draw_param.draw_count,
            // draw_param.update_count, command_processor::getRecvCount());
            // draw_param.draw_count   = 0;
            // draw_param.update_count = 0;
            if (draw_param.cloud_upload && draw_param.oncloud_conf_valid &&
                draw_param.cloud_interval) {
                if (!rtc_sync) {
                    rtc_sync = sync_rtc_ntp();
                }
                if (rtc_sync) {
                    request_wifi = false;
                }
            }
        }
    //*/

    M5.update();
    /*
        if (M5.BtnPWR.wasClicked()) {
            confmode = !confmode;
            if (confmode) {
                layoutChangeConfig();
            }else {
                layoutChange(false);
            }
        }
    //*/

    if (draw_param.in_config_mode) {
        if (!config_ui.loop() || M5.BtnPWR.wasHold() || M5.BtnC.wasHold()) {
            changeLayout_Normal(false);
        }
    } else {
        if (M5.BtnPWR.wasHold()) {  // PWR KEY hold 2sec to poweroff.
            if (config_save_countdown) {
                draw_param.saveNvs();
            }
            M5.Power.powerOff();
        }

        if (M5.BtnPWR.wasClicked() || M5.BtnC.wasClicked()) {
            changeLayout_Config();
            config_ui.enter();
        }

        else if (draw_param.in_pause_state) {
            if (M5.BtnA.wasReleased()) {
                ++draw_param.in_pause_state;
            }
            if (M5.BtnB.wasReleased()) {
                ++draw_param.in_pause_state;
            }
            if (draw_param.in_pause_state > 3) {
                soundOperate();
                draw_param.in_pause_state = 0;
                overlay_ui.show(32, "Pause", "Off");
            }
        } else {
            bool color_change          = M5.BtnA.wasClicked();
            bool reference_mode_change = M5.BtnA.wasHold();
            bool marker_visible_change = M5.BtnB.wasClicked();
            bool layout_change         = M5.BtnB.wasHold();

            if (!color_change && !reference_mode_change &&
                !marker_visible_change && !layout_change) {
                if (M5.BtnA.isPressed() && M5.BtnB.isPressed()) {
                    soundOperate();
                    draw_param.in_pause_state = 1;
                    overlay_ui.show(32, "Pause", "On");
                }
            } else {
                if (color_change) {
                    soundOperate();
                    draw_param.misc_color = (config_param_t::misc_color_t)(
                        (draw_param.misc_color + 1) %
                        config_param_t::misc_color_max);
                    overlay_ui.show(48, draw_param.misc_color.getName()->get(),
                                    draw_param.misc_color.getText());
                }
                if (layout_change) {
                    soundOperate();
                    changeLayout_Normal();
                }
                if (reference_mode_change) {
                    soundOperate();
                    draw_param.referenceModeChange(1);
                    overlay_ui.show(48,
                                    draw_param.alarm_reference.getName()->get(),
                                    draw_param.alarm_reference.getText());
                }
                if (marker_visible_change) {
                    soundOperate();
                    image_ui.pointerChange();
                    overlay_ui.show(48,
                                    draw_param.misc_pointer.getName()->get(),
                                    draw_param.misc_pointer.getText());
                }
                //*
            }
        }
        //*/
    }

    // 温度センサからデータ取得
    if (!command_processor::loop()) {
        delay(8);
    } else if (!draw_param.in_pause_state) {
        int idx_recv_next = (idx_recv + 1) % framedata_len;
        auto frame        = &framedata[idx_recv_next];
        auto prev_frame   = &framedata[idx_recv % framedata_len];
        memcpy(frame, prev_frame, sizeof(framedata_t));
        // Obtain temperature data structure.
        auto temp_data = command_processor::getTemperatureData();

        uint32_t search_lowest  = UINT16_MAX;
        uint32_t search_highest = 0;
        // search_lowest = convertCelsiusToRaw(-80);
        // search_highest = convertCelsiusToRaw(100);

        uint32_t search_total = 0;
        uint32_t search_count = 0;

        uint16_t diff[mlx_width * mlx_height];
        // Pixel data is held in an array. Array size is 384. (16x24)
        bool subpage   = temp_data->subpage;
        frame->subpage = subpage;

        uint8_t moniy =
            draw_param.sens_monitorarea_value[draw_param.sens_monitorarea];
        uint8_t monix = moniy >> 4;
        moniy &= 0x0F;

        for (int idx = 0; idx < mlx_width * mlx_height; ++idx) {
            // // XとYの入れ替えと上下・左右の反転を行う
            // uint_fast8_t x = frame_width - (1 + (idx >> 4));
            // uint_fast8_t y =
            //     frame_height - (1 + (((idx & 15) << 1) + ((x & 1) ==
            //     subpage)));
            uint_fast8_t y = idx >> 4;
            uint_fast8_t x = ((mlx_width - 1 - (idx - (y << 4))) << 1) +
                             ((y & 1) == subpage);
            uint_fast16_t xy     = x + y * frame_width;
            int32_t raw          = temp_data->data[idx];
            int d                = raw - (int32_t)frame->pixel_raw[xy];
            diff[xy >> 1]        = abs(d);
            frame->pixel_raw[xy] = raw;

            // if (x && y && x < (frame_width - 1) && y < (frame_height - 1)) {
            if (((moniy + y - (mlx_height >> 1)) < (moniy << 1)) &&
                ((monix + x - (mlx_width)) <
                 (monix
                  << 1))) {  // 最高・最低温度の更新。最外周ピクセルは極端な外れ値を出すことがあるため除外する。
                search_total += raw;
                ++search_count;
                if (search_lowest > raw) {
                    search_lowest = raw;
                    frame->low_x  = x;
                    frame->low_y  = y;
                }
                if (search_highest < raw) {
                    search_highest = raw;
                    frame->high_x  = x;
                    frame->high_y  = y;
                }
            }
        }
        // Interpolation is performed from surrounding pixels where the
        // temperature change is large. (Areas with little temperature change
        // inherit values from the previous frame.)
        for (int idx = 0; idx < 384; ++idx) {
            // uint_fast8_t x = frame_width - (1 + (idx >> 4));
            // uint_fast8_t y =
            //     frame_height - (1 + (((idx & 15) << 1) + ((x & 1) !=
            //     subpage)));
            uint_fast8_t y = idx >> 4;
            uint_fast8_t x = ((mlx_width - 1 - (idx - (y << 4))) << 1) +
                             ((y & 1) != subpage);
            uint_fast16_t xy = x + y * frame_width;

            uint32_t diff_sum = 0;
            size_t count      = 0;
            if (x > 0) {
                ++count;
                diff_sum += diff[(xy - 1) >> 1];
            }
            if (x < (frame_width - 1)) {
                ++count;
                diff_sum += diff[(xy + 1) >> 1];
            }
            if (y > 0) {
                ++count;
                diff_sum += diff[(xy - frame_width) >> 1];
            }
            if (y < (frame_height - 1)) {
                ++count;
                diff_sum += diff[(xy + frame_width) >> 1];
            }
            diff_sum /= count;

            int32_t raw = frame->pixel_raw[xy];

            uint32_t sum = 0;
            if (x > 0) {
                sum += frame->pixel_raw[xy - 1];
            }
            if (x < (frame_width - 1)) {
                sum += frame->pixel_raw[xy + 1];
            }
            if (y > 0) {
                sum += frame->pixel_raw[xy - frame_width];
            }
            if (y < (frame_height - 1)) {
                sum += frame->pixel_raw[xy + frame_width];
            }
            raw = (sum + (count >> 1)) / count;

            // 温度変化量が小さい箇所は前回値の継承効果を高くする。
            // 温度変化量が大きい箇所は補間処理効果を高くする。
            if (diff_sum > 256) {
                diff_sum = 256;
            }
            raw =
                (frame->pixel_raw[xy] * (256 - diff_sum) + diff_sum * raw) >> 8;
            frame->pixel_raw[xy] = raw;

            // 最高・最低温度の更新。最外周ピクセルは極端な外れ値を出すことがあるため除外する。
            if (((moniy + y - (mlx_height >> 1)) < (moniy << 1)) &&
                ((monix + x - (mlx_width)) <
                 (monix
                  << 1))) {  // 最高・最低温度の更新。最外周ピクセルは極端な外れ値を出すことがあるため除外する。
                search_total += raw;
                ++search_count;
                if (search_lowest > raw) {
                    search_lowest = raw;
                    frame->low_x  = x;
                    frame->low_y  = y;
                }
                if (search_highest < raw) {
                    search_highest = raw;
                    frame->high_x  = x;
                    frame->high_y  = y;
                }
            }
        }
        //  if ( abs((int32_t)(frame->temp[frame->lowest]  - search_lowest )) >
        //  1*128
        //  ||  abs((int32_t)(frame->temp[frame->highest] - search_highest)) >
        //  1*128) {
        //     draw_param.in_pause_state = 1;
        //  }

        frame->temp[frame->lowest]  = search_lowest;
        frame->temp[frame->highest] = search_highest;
        frame->temp[frame->average] = search_total / search_count;
        frame->temp[frame->center] =
            frame->pixel_raw[(frame_width >> 1) +
                             (frame_width * (frame_height >> 1))];

        uint8_t idx = draw_param.graph_data.current_idx + 1;
        for (uint_fast8_t i = 0; i < 4; ++i) {
            draw_param.graph_data.temp_arrays[i][idx] = frame->temp[i];
            // draw_param.graph_data.temp_arrays[i][idx] =
            // graph_filter[i].exec(frame->temp[i]);
        }
        draw_param.graph_data.current_idx = idx;
        idx_recv                          = idx_recv_next;
    }
}

std::string framedata_t::getJsonData(void) const {
    // auto usec = micros();

    auto t   = time(nullptr);
    auto gmt = gmtime(&t);
    char cbuf[64];
    std::string result;
    result.reserve(768 * 6 + 512);
    result.append(cbuf,
                  snprintf(cbuf, sizeof(cbuf), "{\r\n \"pwd\": \"%s\",\r\n",
                           draw_param.cloud_token.c_str()));
    result.append(
        cbuf,
        snprintf(cbuf, sizeof(cbuf),
                 " \"datetime\": \"%s, %d %s %04d %02d:%02d:%02d GMT\",\r\n",
                 wday_tbl[gmt->tm_wday], gmt->tm_mday, mon_tbl[gmt->tm_mon],
                 gmt->tm_year + 1900, gmt->tm_hour, gmt->tm_min, gmt->tm_sec));
    result.append(
        cbuf,
        snprintf(cbuf, sizeof(cbuf), " \"interval\": %d,\r\n",
                 draw_param.cloud_interval_value[draw_param.cloud_interval]));

    auto macaddr = draw_param.macaddr;
    result.append(
        cbuf, snprintf(cbuf, sizeof(cbuf),
                       " \"macaddr\": \"%02x:%02x:%02x:%02x:%02x:%02x\",\r\n",
                       macaddr[0], macaddr[1], macaddr[2], macaddr[3],
                       macaddr[4], macaddr[5]));
    result.append(cbuf, snprintf(cbuf, sizeof(cbuf), " \"center\": %3.1f,\r\n",
                                 convertRawToCelsius(temp[center])));
    result.append(cbuf, snprintf(cbuf, sizeof(cbuf), " \"average\": %3.1f,\r\n",
                                 convertRawToCelsius(temp[average])));
    result.append(cbuf, snprintf(cbuf, sizeof(cbuf), " \"highest\": %3.1f,\r\n",
                                 convertRawToCelsius(temp[highest])));
    result.append(cbuf, snprintf(cbuf, sizeof(cbuf), " \"lowest\": %3.1f,\r\n",
                                 convertRawToCelsius(temp[lowest])));
    result.append(cbuf, snprintf(cbuf, sizeof(cbuf), " \"frame\": [%3.1f",
                                 convertRawToCelsius(pixel_raw[0])));
    for (uint_fast16_t i = 1; i < frame_width * frame_height; ++i) {
        result.append(cbuf, snprintf(cbuf, sizeof(cbuf), ",%3.1f",
                                     convertRawToCelsius(pixel_raw[i])));
    }
    result += "]\r\n}\r\n";

    // ESP_LOGE("DEBUG","JSON CREATE:%d usec", micros() - usec);

    return result;
}

#if !defined(ARDUINO)

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

extern "C" {
void loopTask(void*) {
    setup();
    for (;;) {
        loop();
        taskYIELD();
    }
    vTaskDelete(NULL);
}

void app_main() {
    xTaskCreatePinnedToCore(loopTask, "loopTask", 8192, NULL, 1, NULL,
                            APP_CPU_NUM);
}
}
#endif
