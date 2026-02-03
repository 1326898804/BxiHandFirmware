#include "ble_hand_app.h"
#include "FakeSerial.h"
#include "bxi_ble.h"
#include "esp_err.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include <stdbool.h>
#include <stdint.h>
extern "C"{
#define BXI_HAND_TAG "BXI_HAND"
static uint16_t hand_handle_table[BXI_HAND_APP_IDX_NB];
// 当前连接id
static uint16_t hand_conn_id = 0xffff;
static bool is_connected = false;
static bool enable_hand_state_nf=false;
static bool enable_joint_state_nf=false;
static esp_gatt_if_t hand_gatts_if = 0xff;
static uint32_t hand_mtu_size = 20;
// 服务的uuid
static const uint16_t bxi_hand_server_uuid = 0xABB0;
// 关节状态uuid
static const uint16_t bxi_hand_joint_state_uuid = 0xABB1;
#define JOINT_STATE_BUF_SIZE 128
static uint8_t joint_state_buf[JOINT_STATE_BUF_SIZE] = {0};
static uint8_t bxi_hand_joint_state_config[2] = {0, 0};
static uint8_t bxi_hand_joint_state_description[] = "joint state";
// 关节命令
static const uint16_t bxi_hand_joint_command_uuid = 0xABB2;
#define JOINT_COMMAND_BUF_SIZE 128
static uint8_t joint_command_buf[JOINT_COMMAND_BUF_SIZE] = {0};
static uint8_t bxi_hand_joint_command_description[] = "joint command";
// 手部状态状态uuid
static const uint16_t bxi_hand_state_uuid = 0xABB3;
#define HAND_STATE_BUF_SIZE 128
static uint8_t hand_state_buf[HAND_STATE_BUF_SIZE] = {0};
static uint8_t hand_state_description[] = "hand state";
static uint8_t hand_state_config[2] = {0, 0};

static const esp_gatts_attr_db_t gatt_db[BXI_HAND_APP_IDX_NB] = {
    // 主服务声明
    [BXI_HAND_APP_IDX_SVC] = 
    {{ESP_GATT_AUTO_RSP},
                      {ESP_UUID_LEN_16,
                        (uint8_t *)&primary_service_uuid,
                        ESP_GATT_PERM_READ, sizeof(bxi_hand_server_uuid),
                        sizeof(bxi_hand_server_uuid),
                        (uint8_t *)&bxi_hand_server_uuid}},
    // 关节状态
    [BXI_HAND_APP_IDX_JOINT_STATE_NOTIFY_CHAR] =
        {{ESP_GATT_AUTO_RSP},
         {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid,
          ESP_GATT_PERM_READ, sizeof(char_prop_read_notify),
          sizeof(char_prop_read_notify), (uint8_t *)&char_prop_read_notify}},
    [BXI_HAND_APP_IDX_JOINT_STATE_NTY_VAL] =
        {{ESP_GATT_AUTO_RSP},
         {ESP_UUID_LEN_16, (uint8_t *)&bxi_hand_joint_state_uuid,
          ESP_GATT_PERM_READ, JOINT_STATE_BUF_SIZE, JOINT_STATE_BUF_SIZE,
          (uint8_t *)&joint_state_buf}},
    [BXI_HAND_APP_IDX_JOINT_STATE_NTF_CFG] =
        {{ESP_GATT_AUTO_RSP},
         {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid,
          ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t),
          sizeof(bxi_hand_joint_state_config),
          (uint8_t *)bxi_hand_joint_state_config}},
    [BXI_HAND_APP_IDX_JOINT_STATE_NTF_DESCRIPTION] =
        {{ESP_GATT_AUTO_RSP},
         {ESP_UUID_LEN_16, (uint8_t *)&character_description_uuid,
          ESP_GATT_PERM_READ, sizeof(bxi_hand_joint_state_description),
          sizeof(bxi_hand_joint_state_description),
          (uint8_t *)bxi_hand_joint_state_description}},
    // 关节命令
    [BXI_HAND_APP_IDX_JOINT_COMMAND_CHAR] =
        {{ESP_GATT_AUTO_RSP},
         {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid,
          ESP_GATT_PERM_READ, sizeof(char_prop_read_write),
          sizeof(char_prop_read_write), (uint8_t *)&char_prop_read_write}},
    [BXI_HAND_APP_IDX_JOINT_COMMAND_VAL] =
        {{ESP_GATT_AUTO_RSP},
         {ESP_UUID_LEN_16, (uint8_t *)&bxi_hand_joint_command_uuid,
          ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, JOINT_COMMAND_BUF_SIZE,
          JOINT_COMMAND_BUF_SIZE, (uint8_t *)&joint_command_buf}},
    [BXI_HAND_APP_IDX_JOINT_COMMAND_NTF_DESCRIPTION] =
        {{ESP_GATT_AUTO_RSP},
         {ESP_UUID_LEN_16, (uint8_t *)&character_description_uuid,
          ESP_GATT_PERM_READ, sizeof(bxi_hand_joint_command_description),
          sizeof(bxi_hand_joint_command_description),
          (uint8_t *)bxi_hand_joint_command_description}},
    // 手部状态
    [BXI_HAND_APP_IDX_HAND_STATUS_CHAR] =
        {{ESP_GATT_AUTO_RSP},
         {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid,
          ESP_GATT_PERM_READ, sizeof(char_prop_read_notify), sizeof(char_prop_read_notify),
          (uint8_t *)&char_prop_read_notify}},
    [BXI_HAND_APP_IDX_HAND_STATUS_VAL] =
        {{ESP_GATT_AUTO_RSP},
         {ESP_UUID_LEN_16, (uint8_t *)&bxi_hand_state_uuid,
          ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, HAND_STATE_BUF_SIZE,
          HAND_STATE_BUF_SIZE, (uint8_t *)&hand_state_buf}},
    [BXI_HAND_APP_IDX_HAND_STATUS_NTF_CFG] =
        {{ESP_GATT_AUTO_RSP},
         {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid,
          ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t),
          sizeof(hand_state_config),
          (uint8_t *)hand_state_config}},
    [BXI_HAND_APP_IDX_HAND_STATUS_DESCRIPTION] =
        {{ESP_GATT_AUTO_RSP},
         {ESP_UUID_LEN_16, (uint8_t *)&character_description_uuid,
          ESP_GATT_PERM_READ, sizeof(hand_state_description),
          sizeof(hand_state_description), (uint8_t *)hand_state_description}},
};
static uint8_t find_char_and_desr_index(uint16_t handle) {
  uint8_t error = 0xff;

  for (int i = 0; i < BXI_HAND_APP_IDX_NB; i++) {
    if (handle == hand_handle_table[i]) {
      return i;
    }
  }

  return error;
}
void ble_hand_app_gatts_cb(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                           esp_ble_gatts_cb_param_t *param) {
  esp_ble_gatts_cb_param_t *p_data = (esp_ble_gatts_cb_param_t *)param;
  switch (event) {
  case ESP_GATTS_REG_EVT: { // config adv data
    esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
    if (ret) {
      ESP_LOGE(BXI_HAND_TAG, "config adv data failed, error code = %x", ret);
      break;
    }
    ESP_LOGI(BXI_HAND_TAG, "gatt regsitered success");
    esp_ble_gap_set_device_name("BxiHand");
    ret = esp_ble_gap_config_adv_data(&adv_data);
    if (ret) {
      ESP_LOGE(BXI_HAND_TAG, "config adv data failed, error code = %x", ret);
      break;
    }
    esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, BXI_HAND_APP_IDX_NB, 0);
    break;
  }
  case ESP_GATTS_READ_EVT: {
    ESP_LOGI(BXI_HAND_TAG, "Characteristic read");
    break;
  }
  case ESP_GATTS_WRITE_EVT: {
    uint8_t res = find_char_and_desr_index(p_data->write.handle);
    // ESP_LOGI(BXI_HAND_TAG, "Characteristic write, value len %u, value handle_idx:%u",
    //          param->write.len,res);
    // ESP_LOG_BUFFER_HEX(BXI_HAND_TAG, param->write.value, param->write.len);
    if (res == BXI_HAND_APP_IDX_JOINT_COMMAND_VAL) {
      memcpy(joint_command_buf, p_data->write.value, p_data->write.len);
      FakeSerialpushDataToBuf(joint_command_buf, param->write.len);
    } else if (res == BXI_HAND_APP_IDX_HAND_STATUS_NTF_CFG) {
        if ((p_data->write.len == 2) && (p_data->write.value[0] == 0x01) && (p_data->write.value[1] == 0x00)) {
            ESP_LOGI(BXI_HAND_TAG, "Hand state notification enable");
            enable_hand_state_nf = true;
        } else if ((p_data->write.len == 2) && (p_data->write.value[0] == 0x02) && (p_data->write.value[1] == 0x00)) {
            ESP_LOGI(BXI_HAND_TAG, "Hand state indication enable");
            enable_hand_state_nf = true;
        } else if ((p_data->write.len == 2) && (p_data->write.value[0] == 0x00) && (p_data->write.value[1] == 0x00)) {
            esp_ble_gap_start_advertising(&adv_params);
            ESP_LOGI(BXI_HAND_TAG, "Hand state notification/indication disable");
            enable_hand_state_nf = false;
        }
    }  else if (res == BXI_HAND_APP_IDX_JOINT_STATE_NTF_CFG) {
        if ((p_data->write.len == 2) && (p_data->write.value[0] == 0x01) && (p_data->write.value[1] == 0x00)) {
            ESP_LOGI(BXI_HAND_TAG, "Joint state notification enable");
            enable_joint_state_nf = true;
        } else if ((p_data->write.len == 2) && (p_data->write.value[0] == 0x02) && (p_data->write.value[1] == 0x00)) {
            ESP_LOGI(BXI_HAND_TAG, "Joint state indication enable");
            enable_joint_state_nf = true;
        } else if ((p_data->write.len == 2) && (p_data->write.value[0] == 0x00) && (p_data->write.value[1] == 0x00)) {
            esp_ble_gap_start_advertising(&adv_params);
            ESP_LOGI(BXI_HAND_TAG, "Joint state notification/indication disable");
            enable_joint_state_nf = false;
        }
    } 
    break;
  }
  case ESP_GATTS_DELETE_EVT:
    break;
  case ESP_GATTS_START_EVT:
    ESP_LOGI(BXI_HAND_TAG, "Service start, status %d, service_handle %d",
             param->start.status, param->start.service_handle);
    break;
  case ESP_GATTS_STOP_EVT:
    break;
  case ESP_GATTS_CONNECT_EVT: {
    ESP_LOGI(BXI_HAND_TAG, "Connected, conn_id %u, remote " ESP_BD_ADDR_STR "",
             param->connect.conn_id,
             ESP_BD_ADDR_HEX(param->connect.remote_bda));
    esp_ble_conn_update_params_t conn_params;
    memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
    conn_params.latency = 0;
    conn_params.max_int = 0x06;    // 7.5ms (最小值)
    conn_params.min_int = 0x06;    // 7.5ms
    conn_params.timeout = 400;     // 4s
    esp_ble_gap_update_conn_params(&conn_params);

    esp_ble_gatt_set_local_mtu(517);
    hand_conn_id = p_data->connect.conn_id;
    is_connected = true;
    hand_gatts_if=gatts_if;
    break;
  }
  case ESP_GATTS_DISCONNECT_EVT: {
    ESP_LOGI(BXI_HAND_TAG,
             "Disconnected, remote " ESP_BD_ADDR_STR ", reason 0x%02x",
             ESP_BD_ADDR_HEX(param->disconnect.remote_bda),
             param->disconnect.reason);
    is_connected = false;
    esp_ble_gap_start_advertising(&adv_params);
    break;
  }
  case ESP_GATTS_MTU_EVT:
    ESP_LOGI(BXI_HAND_TAG, "MTU exchange, MTU %d", param->mtu.mtu);
    hand_mtu_size = p_data->mtu.mtu;
  break;
  case ESP_GATTS_CONF_EVT: {
    // ESP_LOGI(BXI_HAND_TAG, "Confirm receive, status %d, attr_handle %d",
    //          param->conf.status, param->conf.handle);
    break;
  }
  case ESP_GATTS_CREAT_ATTR_TAB_EVT: {
    ESP_LOGI(BXI_HAND_TAG, "The number handle %x",
             param->add_attr_tab.num_handle);
    if (param->add_attr_tab.status != ESP_GATT_OK) {
      ESP_LOGE(BXI_HAND_TAG, "Create attribute table failed, error code 0x%x",
               param->add_attr_tab.status);
    } else if (param->add_attr_tab.num_handle != BXI_HAND_APP_IDX_NB) {
      ESP_LOGE(BXI_HAND_TAG,
               "Create attribute table abnormally, num_handle (%d) doesn't "
               "equal to HRS_IDX_NB(%d)",
               param->add_attr_tab.num_handle, BXI_HAND_APP_IDX_NB);
    } else {
      memcpy(hand_handle_table, param->add_attr_tab.handles,
             sizeof(hand_handle_table));
      esp_ble_gatts_start_service(hand_handle_table[BXI_HAND_APP_IDX_SVC]);
    }
    break;
  }
  default:
    break;
  }
}
bool sendHandState(TxSerialPack_u pack) {
    if (is_connected && enable_hand_state_nf) {
        if (hand_mtu_size < (sizeof(TxSerialPack) + 3)) {
            ESP_LOGW("BLE_HAND", "mtu to small!");
            return false;
        }
        esp_err_t ret = esp_ble_gatts_send_indicate(
            hand_gatts_if, 
            hand_conn_id, 
            hand_handle_table[BXI_HAND_APP_IDX_HAND_STATUS_VAL],
            sizeof(TxSerialPack), 
            pack.bytes, 
            false 
        );
        return (ret == ESP_OK);
    }
    return false;
}
bool sendJointState(JointStateSerialPack_u pack) {
    if (is_connected && enable_joint_state_nf) {
        if (hand_mtu_size < (sizeof(JointStateSerialPack_u) + 3)) {
            ESP_LOGW("BLE_HAND", "mtu to small!");
            return false;
        }
        esp_err_t ret = esp_ble_gatts_send_indicate(
            hand_gatts_if, 
            hand_conn_id, 
            hand_handle_table[BXI_HAND_APP_IDX_JOINT_STATE_NTY_VAL],
            sizeof(JointStateSerialPack_u), 
            pack.bytes, 
            false 
        );
        return (ret == ESP_OK);
    }
    return false;
}
}