// Single owner for Wi-Fi network scans (setup-AP SSID list).
//
// Both wifiTask and the web server used to drive the Arduino scan API
// directly. A page reload could start a scan while another was still
// running, and Arduino reports WIFI_SCAN_FAILED after a fixed 6 s timeout
// even though the driver is still scanning, which let a second
// esp_wifi_scan_start() abort the first. The setup page then showed
// "No networks found" or a partial list.
//
// This module tracks the scan state itself (from esp_wifi_scan_start() until
// the SCAN_DONE event), ignores requests while a scan is in progress, and
// keeps the SSID list of the last scan that completed successfully, so an
// aborted scan never replaces a good list.
#pragma once

#include <string>
#include <vector>

namespace wifi_scan {

// Registers the SCAN_DONE handler. Call once, after WiFi is initialised.
void begin(void);

// Starts an asynchronous scan unless one is already running; a request made
// while a scan is in progress is ignored. Returns true if a scan is running
// when the call returns (started now or already in progress).
bool request(void);

// True from a successful request() until the scan finishes.
bool running(void);

// True once a scan has completed successfully since the last clear().
bool has_results(void);

// Unique, non-empty SSIDs of the last successful scan, strongest first.
std::vector<std::string> results(void);

// Stops a running scan (waits up to 100 ms for it to end) and frees the
// stored list. Used before starting a STA connection and when leaving
// setup-AP mode.
void cancel_and_clear(void);

}  // namespace wifi_scan
