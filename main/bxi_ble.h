#ifndef BXI_BLE_HEADER
#define BXI_BLE_HEADER
extern "C"{
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
extern esp_ble_adv_data_t adv_data;
extern const uint16_t primary_service_uuid;
extern const uint16_t character_declaration_uuid;
extern const uint16_t character_client_config_uuid;
extern const uint16_t character_description_uuid;
extern const uint8_t char_prop_read_notify;
extern const uint8_t char_prop_read_write;
extern const uint8_t char_prop_read;
struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
};
extern esp_ble_adv_params_t adv_params;
extern esp_ble_adv_data_t adv_data;
void bxi_ble_init();
}
#endif