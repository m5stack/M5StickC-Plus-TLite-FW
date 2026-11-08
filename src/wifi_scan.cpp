#include "wifi_scan.hpp"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>

#include <mutex>

namespace wifi_scan {
namespace {

// A full active scan of channels 1-13 takes about 4 s, longer in AP+STA
// mode because the radio returns to the AP channel between channels. If no
// SCAN_DONE has arrived after this long (e.g. Wi-Fi was stopped mid-scan),
// the scan is considered dead so that requests are not ignored forever.
constexpr uint32_t stall_ms = 20000;

std::mutex mtx;
bool scan_running   = false;
uint32_t started_ms = 0;
bool have_results   = false;
std::vector<std::string> ssids;

// Runs in the arduino_events task after WiFiScanClass::_scanDone() has
// copied the driver's records into WiFi's scan result buffer.
void on_scan_done(arduino_event_id_t, arduino_event_info_t info) {
    std::lock_guard<std::mutex> lock(mtx);
    scan_running = false;
    const int16_t n = WiFi.scanComplete();
    ESP_LOGD("wifi_scan", "scan done: status=%u records=%d",
             (unsigned)info.wifi_scan_done.status, (int)n);
    // status != 0: the scan was aborted (stopped, or interrupted by a
    // connection attempt) and its list is incomplete. Keep the previous one.
    if (info.wifi_scan_done.status == 0 && n >= 0) {
        ssids.clear();
        for (int i = 0; i < n; ++i) {
            // The driver returns records sorted by RSSI, strongest first.
            const String s = WiFi.SSID(i);
            if (s.isEmpty()) continue;  // hidden network
            bool duplicate = false;
            for (const auto& known : ssids) {
                if (known == s.c_str()) {
                    duplicate = true;
                    break;
                }
            }
            if (!duplicate) ssids.emplace_back(s.c_str());
        }
        have_results = true;
    }
    // The list has been copied; free WiFi's copy of the records.
    WiFi.scanDelete();
}

}  // namespace

void begin(void) {
    WiFi.onEvent(on_scan_done, ARDUINO_EVENT_WIFI_SCAN_DONE);
}

bool request(void) {
    std::lock_guard<std::mutex> lock(mtx);
    if (scan_running) {
        if (millis() - started_ms < stall_ms) {
            ESP_LOGD("wifi_scan", "request ignored: scan in progress");
            return true;
        }
        ESP_LOGW("wifi_scan", "scan did not finish within %u ms",
                 (unsigned)stall_ms);
        scan_running = false;
    }
    wifi_mode_t mode = WIFI_MODE_NULL;
    if (esp_wifi_get_mode(&mode) != ESP_OK || !(mode & WIFI_MODE_STA)) {
        return false;  // scanning needs the STA interface
    }
    // Same parameters the Arduino API uses for scanNetworks(true).
    wifi_scan_config_t config        = {};
    config.show_hidden               = false;
    config.scan_type                 = WIFI_SCAN_TYPE_ACTIVE;
    config.scan_time.active.min      = 100;
    config.scan_time.active.max      = 300;
    const esp_err_t err              = esp_wifi_scan_start(&config, false);
    if (err != ESP_OK) {
        // e.g. ESP_ERR_WIFI_STATE while a STA connection is in progress.
        ESP_LOGD("wifi_scan", "esp_wifi_scan_start failed: %d", err);
        return false;
    }
    // Set while still holding the lock, so a SCAN_DONE for this scan
    // cannot be processed before the flag is set.
    scan_running = true;
    started_ms   = millis();
    ESP_LOGD("wifi_scan", "scan started");
    return true;
}

bool running(void) {
    std::lock_guard<std::mutex> lock(mtx);
    return scan_running && millis() - started_ms < stall_ms;
}

bool has_results(void) {
    std::lock_guard<std::mutex> lock(mtx);
    return have_results;
}

std::vector<std::string> results(void) {
    std::lock_guard<std::mutex> lock(mtx);
    return ssids;
}

void cancel_and_clear(void) {
    if (running()) {
        esp_wifi_scan_stop();
        const uint32_t timeout_at = millis() + 100;
        while (running() && int32_t(millis() - timeout_at) < 0) {
            delay(1);
        }
    }
    std::lock_guard<std::mutex> lock(mtx);
    scan_running = false;
    have_results = false;
    std::vector<std::string>().swap(ssids);  // release the memory
}

}  // namespace wifi_scan
