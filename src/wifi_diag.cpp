#include "wifi_diag.hpp"

#if TLITE_WIFI_DIAG

#include <Arduino.h>
#include <WiFi.h>
#include <esp_event.h>
#include <esp_netif.h>
#include <esp_netif_sta_list.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace {

// Events are recorded into a small ring buffer from the event tasks and
// printed by a separate low-priority task, so the event tasks themselves do
// no formatting/printing (sys_evt has a small stack and must not be slowed).
enum source_t : uint8_t { src_sys, src_ard };

struct record_t {
    uint32_t ms;
    int32_t id;
    uint32_t ip;
    uint8_t mac[6];
    uint8_t aid;
    source_t src;
    bool is_ip_event;
};

constexpr size_t ring_len = 64;
record_t ring[ring_len];
size_t ring_head      = 0;  // next write
size_t ring_tail      = 0;  // next read
uint32_t ring_dropped = 0;
portMUX_TYPE ring_mux = portMUX_INITIALIZER_UNLOCKED;

void push(const record_t& r) {
    portENTER_CRITICAL(&ring_mux);
    size_t next = (ring_head + 1) % ring_len;
    if (next == ring_tail) {
        ++ring_dropped;
    } else {
        ring[ring_head] = r;
        ring_head       = next;
    }
    portEXIT_CRITICAL(&ring_mux);
}

bool pop(record_t& r) {
    bool ok = false;
    portENTER_CRITICAL(&ring_mux);
    if (ring_tail != ring_head) {
        r         = ring[ring_tail];
        ring_tail = (ring_tail + 1) % ring_len;
        ok        = true;
    }
    portEXIT_CRITICAL(&ring_mux);
    return ok;
}

const char* sys_wifi_event_name(int32_t id) {
    switch (id) {
        case WIFI_EVENT_WIFI_READY:
            return "WIFI_READY";
        case WIFI_EVENT_SCAN_DONE:
            return "SCAN_DONE";
        case WIFI_EVENT_STA_START:
            return "STA_START";
        case WIFI_EVENT_STA_STOP:
            return "STA_STOP";
        case WIFI_EVENT_STA_CONNECTED:
            return "STA_CONNECTED";
        case WIFI_EVENT_STA_DISCONNECTED:
            return "STA_DISCONNECTED";
        case WIFI_EVENT_AP_START:
            return "AP_START";
        case WIFI_EVENT_AP_STOP:
            return "AP_STOP";
        case WIFI_EVENT_AP_STACONNECTED:
            return "AP_STACONNECTED";
        case WIFI_EVENT_AP_STADISCONNECTED:
            return "AP_STADISCONNECTED";
        case WIFI_EVENT_AP_PROBEREQRECVED:
            return "AP_PROBEREQRECVED";
        default:
            return "?";
    }
}

const char* sys_ip_event_name(int32_t id) {
    switch (id) {
        case IP_EVENT_STA_GOT_IP:
            return "STA_GOT_IP";
        case IP_EVENT_STA_LOST_IP:
            return "STA_LOST_IP";
        case IP_EVENT_AP_STAIPASSIGNED:
            return "AP_STAIPASSIGNED";
        default:
            return "?";
    }
}

// Runs in the sys_evt task (ESP-IDF default event loop).
void sys_handler(void*, esp_event_base_t base, int32_t id, void* data) {
    record_t r = {};
    r.ms       = millis();
    r.id       = id;
    r.src      = src_sys;
    if (base == WIFI_EVENT) {
        if (id == WIFI_EVENT_AP_STACONNECTED && data) {
            auto e = (wifi_event_ap_staconnected_t*)data;
            memcpy(r.mac, e->mac, 6);
            r.aid = e->aid;
        } else if (id == WIFI_EVENT_AP_STADISCONNECTED && data) {
            auto e = (wifi_event_ap_stadisconnected_t*)data;
            memcpy(r.mac, e->mac, 6);
            r.aid = e->aid;
        }
    } else {
        r.is_ip_event = true;
        if (id == IP_EVENT_AP_STAIPASSIGNED && data) {
            r.ip = ((ip_event_ap_staipassigned_t*)data)->ip.addr;
        }
    }
    push(r);
}

// Runs in Arduino's arduino_events task.
void ard_handler(arduino_event_id_t id, arduino_event_info_t info) {
    record_t r = {};
    r.ms       = millis();
    r.id       = id;
    r.src      = src_ard;
    switch (id) {
        case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
            memcpy(r.mac, info.wifi_ap_staconnected.mac, 6);
            r.aid = info.wifi_ap_staconnected.aid;
            break;
        case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
            memcpy(r.mac, info.wifi_ap_stadisconnected.mac, 6);
            r.aid = info.wifi_ap_stadisconnected.aid;
            break;
        case ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED:
            r.ip = info.wifi_ap_staipassigned.ip.addr;
            break;
        default:
            break;
    }
    push(r);
}

char state_letter(TaskHandle_t h) {
    if (!h) return '-';
    switch (eTaskGetState(h)) {
        case eRunning:
            return 'X';
        case eReady:
            return 'R';
        case eBlocked:
            return 'B';
        case eSuspended:
            return 'S';
        case eDeleted:
            return 'D';
        default:
            return '?';
    }
}

void print_task(const char* name) {
    TaskHandle_t h = xTaskGetHandle(name);
    if (!h) {
        log_printf(" %s=-", name);
        return;
    }
    log_printf(" %s=%c/%u", name, state_letter(h),
                  (unsigned)uxTaskGetStackHighWaterMark(h));
}

void print_record(const record_t& r) {
    const char* name;
    if (r.src == src_ard) {
        name = WiFi.eventName((arduino_event_id_t)r.id);
    } else {
        name = r.is_ip_event ? sys_ip_event_name(r.id)
                             : sys_wifi_event_name(r.id);
    }
    log_printf("[%7u][APDIAG] %s %-3s %s", (unsigned)r.ms,
                  r.src == src_sys ? "sys" : "ard",
                  r.is_ip_event ? "ip" : "", name);
    bool has_mac = false;
    for (auto b : r.mac) has_mac |= b != 0;
    if (has_mac) {
        log_printf(" mac=%02x:%02x:%02x:%02x:%02x:%02x aid=%u", r.mac[0],
                      r.mac[1], r.mac[2], r.mac[3], r.mac[4], r.mac[5],
                      r.aid);
    }
    if (r.ip) {
        log_printf(" ip=" IPSTR, IP2STR((esp_ip4_addr_t*)&r.ip));
    }
    log_printf(" (printed +%ums)\n", (unsigned)(millis() - r.ms));
}

// Prints the soft-AP configuration actually held by the driver, the real
// CPU/APB clocks, and how fast the FreeRTOS tick runs relative to millis().
// (main.cpp's misc_cpuspeed_func changes clocks without updating the tick
// divisor; a skewed tick would show up here as ticks_ms != wall_ms.)
void print_ap_config(void) {
    wifi_config_t c = {};
    if (esp_wifi_get_config(WIFI_IF_AP, &c) != ESP_OK) {
        log_printf("[APDIAG] cfg: esp_wifi_get_config(AP) failed\n");
        return;
    }
    char ssid[33] = {};
    memcpy(ssid, c.ap.ssid, c.ap.ssid_len ? c.ap.ssid_len : 32);
    char pass[65] = {};
    memcpy(pass, c.ap.password, 64);
    log_printf(
        "[APDIAG] cfg: ssid='%s' pass='%s' (len %u) ch=%u authmode=%d "
        "cipher=%d hidden=%u maxconn=%u beacon=%u\n",
        ssid, pass, (unsigned)strlen(pass), c.ap.channel, (int)c.ap.authmode,
        (int)c.ap.pairwise_cipher, c.ap.ssid_hidden, c.ap.max_connection,
        c.ap.beacon_interval);
    wifi_country_t country = {};
    esp_wifi_get_country(&country);
    int8_t txpower = 0;
    esp_wifi_get_max_tx_power(&txpower);
    wifi_ps_type_t ps = WIFI_PS_NONE;
    esp_wifi_get_ps(&ps);
    log_printf(
        "[APDIAG] cfg: country=%.2s schan=%u nchan=%u txpower=%d(x0.25dBm) "
        "ps=%d cpu=%uMHz apb=%uHz\n",
        country.cc, country.schan, country.nchan, txpower, (int)ps,
        (unsigned)getCpuFrequencyMhz(), (unsigned)getApbFrequency());
}

void print_tick_rate(void) {
    static uint32_t last_ms    = 0;
    static TickType_t last_tic = 0;
    uint32_t ms                = millis();
    TickType_t tic             = xTaskGetTickCount();
    if (last_ms) {
        log_printf("[APDIAG] clock: wall_ms=%u ticks_ms=%u (tick %uHz)\n",
                   (unsigned)(ms - last_ms),
                   (unsigned)((tic - last_tic) * 1000u / configTICK_RATE_HZ),
                   (unsigned)configTICK_RATE_HZ);
    }
    last_ms  = ms;
    last_tic = tic;
}

void heartbeat(void) {
    static bool cfg_printed = false;
    static uint8_t tick_div = 0;
    if ((tick_div++ % 5) == 0) print_tick_rate();  // every ~10 s

    wifi_mode_t mode = WIFI_MODE_NULL;
    esp_wifi_get_mode(&mode);
    if (!(mode & WIFI_MODE_AP)) {
        cfg_printed = false;
        return;
    }
    if (!cfg_printed) {
        cfg_printed = true;
        print_ap_config();
    }

    wifi_sta_list_t wl     = {};
    esp_netif_sta_list_t nl = {};
    bool have_list = esp_wifi_ap_get_sta_list(&wl) == ESP_OK &&
                     esp_netif_get_sta_list(&wl, &nl) == ESP_OK;

    uint8_t primary            = 0;
    wifi_second_chan_t second = WIFI_SECOND_CHAN_NONE;
    esp_wifi_get_channel(&primary, &second);

    log_printf("[%7u][APDIAG] hb ch=%u scan=%d heap=%u min=%u stations=%d",
                  (unsigned)millis(), primary, (int)WiFi.scanComplete(),
                  (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap(),
                  have_list ? nl.num : -1);
    if (have_list) {
        for (int i = 0; i < nl.num; ++i) {
            auto& s = nl.sta[i];
            log_printf(" [%02x:%02x:%02x:%02x:%02x:%02x " IPSTR
                          " rssi=%d]",
                          s.mac[0], s.mac[1], s.mac[2], s.mac[3], s.mac[4],
                          s.mac[5], IP2STR(&s.ip), wl.sta[i].rssi);
        }
    }
    log_printf("\n[APDIAG]   tasks(state/stackfree):");
    print_task("sys_evt");
    print_task("arduino_events");
    print_task("tiT");
    print_task("wifi");
    print_task("wifiTask");
    print_task("drawTask");
    print_task("webTask");
    print_task("mlxTask");
    uint32_t dropped;
    portENTER_CRITICAL(&ring_mux);
    dropped = ring_dropped;
    portEXIT_CRITICAL(&ring_mux);
    log_printf(" dropped=%u\n", (unsigned)dropped);
}

void diag_task(void*) {
    uint32_t next_hb = millis();
    for (;;) {
        record_t r;
        while (pop(r)) {
            print_record(r);
        }
        if (int32_t(millis() - next_hb) >= 0) {
            next_hb += 2000;
            heartbeat();
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

}  // namespace

void wifi_diag_begin(void) {
    // Arduino creates the default event loop lazily; make sure it exists.
    esp_err_t err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        log_printf("[APDIAG] event loop create failed: %d\n", err);
    }
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, sys_handler,
                               nullptr);
    esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, sys_handler,
                               nullptr);
    WiFi.onEvent(ard_handler);
    xTaskCreatePinnedToCore(diag_task, "apdiag", 3072, nullptr, 1, nullptr,
                            PRO_CPU_NUM);
    log_printf("[APDIAG] Wi-Fi diagnostics enabled\n");
}

#endif
