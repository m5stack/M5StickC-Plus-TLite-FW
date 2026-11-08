// Temporary Wi-Fi / soft-AP diagnostics.
//
// Built only when TLITE_WIFI_DIAG is non-zero (default: debug builds, i.e.
// CORE_DEBUG_LEVEL >= 4). Logs every WIFI_EVENT / IP_EVENT twice:
//   "sys"  - when ESP-IDF's default event loop (sys_evt task) dispatches it
//   "ard"  - when Arduino's arduino_events task processes it
// plus a periodic heartbeat with the driver's station list, DHCP-assigned
// IPs and the state of the event/network tasks. Comparing the two timestamps
// shows whether either event task stalls.
#pragma once

#ifndef TLITE_WIFI_DIAG
#if defined(CORE_DEBUG_LEVEL) && (CORE_DEBUG_LEVEL >= 4)
#define TLITE_WIFI_DIAG 1
#else
#define TLITE_WIFI_DIAG 0
#endif
#endif

#if TLITE_WIFI_DIAG
void wifi_diag_begin(void);
#else
static inline void wifi_diag_begin(void) {
}
#endif
