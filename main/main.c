#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gatt_common_api.h"

#define TAG "BLE"
#define DEVICE_NAME "ESP32-BLE"

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "BLE advertising started successfully");
            } else {
                ESP_LOGE(TAG, "Failed to start BLE advertising, error code: %d", param->adv_start_cmpl.status);
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            ESP_LOGI(TAG, "BLE advertising stopped");
            break;
        default:
            ESP_LOGI(TAG, "GAP event received: %d", event);
            break;
    }
}

void app_main() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));

    // Set the BLE device name
    ESP_ERROR_CHECK(esp_ble_gap_set_device_name(DEVICE_NAME));

    // Generate and set a random address
    esp_bd_addr_t rand_addr = {0};
    // Generate a random address using the default API
    
    rand_addr[0] = 0x02;  // Set the address type to random
    rand_addr[1] = 0x00;  // Fill in the rest of the address (you can set your own byte values)
    rand_addr[2] = 0x11;
    rand_addr[3] = 0x22;
    rand_addr[4] = 0x33;
    rand_addr[5] = 0x44;

    ESP_ERROR_CHECK(esp_ble_gap_set_rand_addr(rand_addr));  // Set the random address

    // Define advertisement data
    esp_ble_adv_data_t adv_data = {
        .set_scan_rsp = false,
        .include_name = true,
        .include_txpower = true,
        .min_interval = 0x20,
        .max_interval = 0x40,
        .appearance = 0x00,
        .manufacturer_len = 0,
        .p_manufacturer_data = NULL,
        .service_data_len = 0,
        .p_service_data = NULL,
        .service_uuid_len = 0,
        .p_service_uuid = NULL,
        .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
    };

    // Configure advertisement data
    ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&adv_data));

    // Advertising parameters
    esp_ble_adv_params_t adv_params = {
        .adv_int_min        = 0x20,
        .adv_int_max        = 0x40,
        .adv_type           = ADV_TYPE_IND,
        .own_addr_type      = BLE_ADDR_TYPE_RANDOM,  // Use RANDOM if PUBLIC doesn't work
        .channel_map        = ADV_CHNL_ALL,
        .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY
    };

    ESP_LOGI(TAG, "Starting BLE advertising...");
    ESP_ERROR_CHECK(esp_ble_gap_start_advertising(&adv_params));

    while (true) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

