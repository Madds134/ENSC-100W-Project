/*
 * iBeacon receiver: prints only when the target beacon is within ~50 cm
 * Uses fixed RSSI threshold of -55 dBm (tune if needed)
 */

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>

#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"
#include "esp_ibeacon_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "stepper/stepper.h"

static const char *DEMO_TAG = "SCHRODINGER_SYSTEMS";
extern esp_ble_ibeacon_vendor_t vendor_config;

// === Configuration ==========================================================
#define ENTER_DBM       (-55)   // print only when RSSI >= this value
#define USE_EMA_SMOOTHING 1
#define EMA_ALPHA       0.25f   // 0.0–1.0, lower = smoother

// Target beacon UUID + major/minor
static const uint8_t TARGET_UUID[ESP_UUID_LEN_128] = {
    0x8b,0x3d,0x81,0xae,0x11,0xc1,0x4b,0xa0,
    0x87,0x30,0x79,0x60,0xde,0xd5,0x0e,0xfa
};
#define TARGET_MAJOR 0x0001
#define TARGET_MINOR 0x0065


volatile bool beacon_nearby = false;
// ============================================================================

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

#if (IBEACON_MODE == IBEACON_RECEIVER)
static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

#elif (IBEACON_MODE == IBEACON_SENDER)
static esp_ble_adv_params_t ble_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_NONCONN_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};
#endif


static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    esp_err_t err;

    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
    #if (IBEACON_MODE == IBEACON_SENDER)
        esp_ble_gap_start_advertising(&ble_adv_params);
    #endif
        break;

    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
    #if (IBEACON_MODE == IBEACON_RECEIVER)
        esp_ble_gap_start_scanning(0); // scan indefinitely
    #endif
        break;

    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        if ((err = param->scan_start_cmpl.status) != ESP_BT_STATUS_SUCCESS)
            ESP_LOGE(DEMO_TAG, "Scan start failed: %s", esp_err_to_name(err));
        else
            ESP_LOGI(DEMO_TAG, "Scan started");
        break;

    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        if (scan_result->scan_rst.search_evt != ESP_GAP_SEARCH_INQ_RES_EVT)
            break;

        if (!esp_ble_is_ibeacon_packet(scan_result->scan_rst.ble_adv,
                                       scan_result->scan_rst.adv_data_len))
            break;

        esp_ble_ibeacon_t *ibeacon = (esp_ble_ibeacon_t *)scan_result->scan_rst.ble_adv;
        uint16_t major = ENDIAN_CHANGE_U16(ibeacon->ibeacon_vendor.major);
        uint16_t minor = ENDIAN_CHANGE_U16(ibeacon->ibeacon_vendor.minor);

        // Filter for your beacon
        if (memcmp(ibeacon->ibeacon_vendor.proximity_uuid, TARGET_UUID, ESP_UUID_LEN_128) == 0 &&
            major == TARGET_MAJOR && minor == TARGET_MINOR) {

#if USE_EMA_SMOOTHING
            static float rssi_ema = NAN;
            float rssi_now = (float)scan_result->scan_rst.rssi;
            if (isnan(rssi_ema))
                rssi_ema = rssi_now;
            else
                rssi_ema = EMA_ALPHA * rssi_now + (1.0f - EMA_ALPHA) * rssi_ema;
            float rssi_val = rssi_ema;
#else
            float rssi_val = (float)scan_result->scan_rst.rssi;
#endif

            // Only print when beacon is within 50 cm (RSSI >= -55 dBm)
            if (rssi_val >= (float)ENTER_DBM) {
                
                // Indicate to app_main that beacon is nearby
                beacon_nearby = true; // set flag to true, will detect later in app_main

                // ESP_LOGI(DEMO_TAG, "\n/------ Beacon within 50 cm ------\\");
                // ESP_LOGI(DEMO_TAG, "Device: "ESP_BD_ADDR_STR"", ESP_BD_ADDR_HEX(scan_result->scan_rst.bda));
                // ESP_LOGI(DEMO_TAG, "RSSI: %.1f dBm (threshold %.1f dBm)", rssi_val, (float)ENTER_DBM);
                // ESP_LOG_BUFFER_HEX("UUID", ibeacon->ibeacon_vendor.proximity_uuid, ESP_UUID_LEN_128);
                // //ESP_LOGI(DEMO_TAG, "UUID:", ibeacon->ibeacon_vendor.proximity_uuid, ESP_UUID_LEN_128);
                // ESP_LOGI(DEMO_TAG, "Major: %u  Minor: %u", major, minor);
            }
            else {
                beacon_nearby = false; // set flag to false
            }
        }
        break;
    }

    default:
        break;
    }
}

void ble_ibeacon_appRegister(void)
{
    esp_err_t status = esp_ble_gap_register_callback(esp_gap_cb);
    if (status != ESP_OK)
        ESP_LOGE(DEMO_TAG, "gap register error: %s", esp_err_to_name(status));
}

void ble_ibeacon_init(void)
{
    esp_bluedroid_init();
    esp_bluedroid_enable();
    ble_ibeacon_appRegister();
}

//static const char *TAG = "MAIN";
static stepper_t motor;

// STEPER MOTOR CONTROL MAIN ENTRY POINT
// This function initializes the stepper motor and continuously rotates it
// clockwise and counterclockwise, logging each action.
// void app_main(void)
// {
//     stepper_init(&motor, GPIO_NUM_4, GPIO_NUM_5, GPIO_NUM_6, GPIO_NUM_7);
//     stepper_set_rpm(&motor, 120.0f);  // alias to stepper_set_speed_rpm

//     while (1) {
//         ESP_LOGI(TAG, "CW one revolution");
//         for(int i = 0; i <= 10; i++)
//         {
//             stepper_one_rev_cw(&motor);
//         }
//         ESP_LOGI(TAG, "CCW one revolution");
//         stepper_one_rev_ccw(&motor);
        
//     }
// }

// IBEACON MAIN ENTRY POINT
// This function initializes the Bluetooth stack and configures the device
// as either an iBeacon sender or receiver based on the defined mode.
// void app_main(void)
// {
//     ESP_ERROR_CHECK(nvs_flash_init());
//     ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

//     esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
//     ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
//     ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

//     ble_ibeacon_init();

// #if (IBEACON_MODE == IBEACON_RECEIVER)
//     esp_ble_gap_set_scan_params(&ble_scan_params);
// #elif (IBEACON_MODE == IBEACON_SENDER)
//     esp_ble_ibeacon_t adv_data;
//     esp_err_t st = esp_ble_config_ibeacon_data(&vendor_config, &adv_data);
//     if (st == ESP_OK)
//         esp_ble_gap_config_adv_data_raw((uint8_t *)&adv_data, sizeof(adv_data));
//     else
//         ESP_LOGE(DEMO_TAG, "Config iBeacon data failed: %s", esp_err_to_name(st));
// #endif
// }


// Combined main entry point for both stepper motor control and iBeacon functionality
// void app_main(void)
// {
//     // Initialize NVS flash for Bluetooth
//     ESP_ERROR_CHECK(nvs_flash_init());
//     ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

//     // Initialize Bluetooth controller
//     esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
//     ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
//     ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

//     // Initialize iBeacon functionality 
//     ble_ibeacon_init();

// #if (IBEACON_MODE == IBEACON_RECEIVER)
//     esp_ble_gap_set_scan_params(&ble_scan_params);
// #elif (IBEACON_MODE == IBEACON_SENDER)
//     esp_ble_ibeacon_t adv_data;
//     esp_err_t st = esp_ble_config_ibeacon_data(&vendor_config, &adv_data);
//     if (st == ESP_OK)
//         esp_ble_gap_config_adv_data_raw((uint8_t *)&adv_data, sizeof(adv_data));
//     else
//         ESP_LOGE(DEMO_TAG, "Config iBeacon data failed: %s", esp_err_to_name(st));
// #endif

//     // Initialize stepper motor
//     stepper_init(&motor, GPIO_NUM_4, GPIO_NUM_5, GPIO_NUM_6, GPIO_NUM_7);
//     stepper_set_rpm(&motor, 120.0f);  // alias to stepper_set_speed_rpm

//     // Main loop for stepper motor control
//     while (1) {
//         ESP_LOGI(TAG, "CW one revolution");
//         for(int i = 0; i <= 10; i++)
//         {
//             stepper_one_rev_cw(&motor);
//         }
//         ESP_LOGI(TAG, "CCW one revolution");
//         stepper_one_rev_ccw(&motor);
        
//     }
// }


// Combined entry point of iBeacon receiver and stepper motor control, but
// motor only spins once CW and once CCW, when iBeacon is detected within range

void app_main(void)
{
    // Initialize NVS flash for Bluetooth
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    // Initialize Bluetooth controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    // Initialize iBeacon functionality 
    ble_ibeacon_init();
#if (IBEACON_MODE == IBEACON_RECEIVER)
    esp_ble_gap_set_scan_params(&ble_scan_params);
#elif (IBEACON_MODE == IBEACON_SENDER)
    esp_ble_ibeacon_t adv_data;
    esp_err_t st = esp_ble_config_ibeacon_data(&vendor_config, &adv_data);
    if (st == ESP_OK)
        esp_ble_gap_config_adv_data_raw((uint8_t *)&adv_data, sizeof(adv_data));
    else
        ESP_LOGE(DEMO_TAG, "Config iBeacon data failed: %s", esp_err_to_name(st));
#endif

    // Initialize stepper motor
    stepper_init(&motor, GPIO_NUM_4, GPIO_NUM_5, GPIO_NUM_6, GPIO_NUM_7);
    stepper_set_rpm(&motor, 120.0f);  // alias to stepper_set_speed_rpm

    // Main loop for stepper motor control triggered by iBeacon detection
    while (1){ 
        ble_ibeacon_appRegister();
        if (beacon_nearby == true) { // beacon_nearby flag from esp_gap_cb
            ESP_LOGI(DEMO_TAG, "Opening Window: CW 3 revolution");

            for (int i = 0; i < 3 ; i++) { // Rotate 3 times clockwise to open
                stepper_one_rev_cw(&motor);
            }
            while (beacon_nearby == true) {
                // STOP ROTATION, DO NOTHING WHILE BEACON IS STILL WITHIN RANGE
                ESP_LOGI(DEMO_TAG, "Beacon still nearby - motor idle");
                vTaskDelay(2000 / portTICK_PERIOD_MS); // Wait for 2 seconds
                ble_ibeacon_appRegister();
                if(beacon_nearby == false) break; // exit for loop
            }
            if (beacon_nearby == false) {
                // wait 2 seconds, rotate counter clockwise to close
                vTaskDelay(2000 / portTICK_PERIOD_MS); // Wait for 2 seconds
                ESP_LOGI(DEMO_TAG, "Beacon out of range - Closing Window: CCW 3 revolution");
                for (int i = 0; i < 3 ; i++) {
                    stepper_one_rev_ccw(&motor);
                }
            }
        }
        // If BLE Beacon is not detected
        else {
            ESP_LOGI(DEMO_TAG, "No beacon detected within range - motor idle");            
            vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait for 1 second
        }        
    }
}
