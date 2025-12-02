#include <string.h>
#include <stdio.h>

#include "wifi.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"

static const char *TAG = "WIFI";

static esp_netif_t *s_sta_netif = NULL;
static bool s_got_ip = false;
static char s_current_ssid[33] = {0};

// =================== Event Handler ===================

static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WiFi STA started, connecting...");
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi disconnected, reconnecting...");
        s_got_ip = false;
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        s_got_ip = true;
        char ip_str[16];
        snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Got IP: %s", ip_str);
    }
}

// =================== Public API ======================

esp_err_t wifi_init_sta(const char *ssid, const char *pass)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Create default station
    s_sta_netif = esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register handlers
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
                                               ESP_EVENT_ANY_ID,
                                               &wifi_event_handler,
                                               NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT,
                                               IP_EVENT_STA_GOT_IP,
                                               &wifi_event_handler,
                                               NULL));

    wifi_config_t wifi_config = { 0 };
    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, pass, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    // Save current SSID locally for wifi_get_ssid()
    memset(s_current_ssid, 0, sizeof(s_current_ssid));
    strncpy(s_current_ssid, ssid, sizeof(s_current_ssid) - 1);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi STA initialized, connecting to SSID: '%s'", ssid);

    return ESP_OK;
}

bool wifi_is_connected(void)
{
    return s_got_ip;
}

esp_err_t wifi_get_ip_str(char *buf, size_t len)
{
    if (!s_got_ip || s_sta_netif == NULL) {
        return ESP_FAIL;
    }

    esp_netif_ip_info_t ip_info;
    esp_err_t err = esp_netif_get_ip_info(s_sta_netif, &ip_info);
    if (err != ESP_OK) {
        return err;
    }

    if (len < 16) {
        return ESP_ERR_INVALID_SIZE;
    }

    snprintf(buf, len, IPSTR, IP2STR(&ip_info.ip));
    return ESP_OK;
}

esp_err_t wifi_get_ssid(char *buf, size_t len)
{
    size_t needed = strnlen(s_current_ssid, sizeof(s_current_ssid)) + 1;
    if (len < needed) {
        return ESP_ERR_INVALID_SIZE;
    }
    strncpy(buf, s_current_ssid, len);
    buf[len - 1] = '\0';
    return ESP_OK;
}
