#ifndef BXI_HAND_APP_HEADER
#define BXI_HAND_APP_HEADER
#include "FakeSerial.h"
extern "C"{
#include "freertos/task.h"
#include "esp_gatts_api.h"

#define BXI_HAND_APP_ID 0
enum{
    //主服务UUID
    BXI_HAND_APP_IDX_SVC,
    //关节状态
    BXI_HAND_APP_IDX_JOINT_STATE_NOTIFY_CHAR,
    BXI_HAND_APP_IDX_JOINT_STATE_NTY_VAL,
    BXI_HAND_APP_IDX_JOINT_STATE_NTF_CFG,
    BXI_HAND_APP_IDX_JOINT_STATE_NTF_DESCRIPTION,
    //关节命令
    BXI_HAND_APP_IDX_JOINT_COMMAND_CHAR,
    BXI_HAND_APP_IDX_JOINT_COMMAND_VAL,
    BXI_HAND_APP_IDX_JOINT_COMMAND_NTF_DESCRIPTION,
    // //手部状态
    BXI_HAND_APP_IDX_HAND_STATUS_CHAR,
    BXI_HAND_APP_IDX_HAND_STATUS_VAL,
    BXI_HAND_APP_IDX_HAND_STATUS_NTF_CFG,
    BXI_HAND_APP_IDX_HAND_STATUS_DESCRIPTION,

    BXI_HAND_APP_IDX_NB,
};

void ble_hand_app_gatts_cb(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
bool sendHandState(TxSerialPack_u pack);
bool sendJointState(JointStateSerialPack_u pack);
}
#endif