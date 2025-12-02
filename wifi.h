#pragma once

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialize Wi-Fi in station mode and start connecting.
// ssid and pass are null-terminated strings.
esp_err_t wifi_init_sta(const char *ssid, const char *pass);

// Returns true if we have a valid IP (got IP event occurred).
bool wifi_is_connected(void);

// Get current station IP as "xxx.xxx.xxx.xxx" string.
// buf must be at least 16 bytes ("255.255.255.255\0").
esp_err_t wifi_get_ip_str(char *buf, size_t len);

// Get current SSID string we are connected to.
// buf must be at least 33 bytes (32-byte SSID + '\0').
esp_err_t wifi_get_ssid(char *buf, size_t len);

#ifdef __cplusplus
}
#endif
