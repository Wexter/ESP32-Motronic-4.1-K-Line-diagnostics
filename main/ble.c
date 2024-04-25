#include "ble.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"

#include "string.h"

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

void (*__spp_data_recv_callback)(esp_ble_gatts_cb_param_t *) = NULL;
void (*__client_connected_callback)() = NULL;

static uint16_t spp_mtu_size = 23;
static uint16_t spp_conn_id = 0xffff;
static esp_gatt_if_t spp_gatts_if = 0xff;

static bool enable_data_ntf = false;
static esp_bd_addr_t spp_remote_bda = {0x0,};

static uint16_t spp_handle_table[SPP_IDX_NB];

static esp_ble_adv_params_t spp_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst spp_profile_tab[SPP_PROFILE_NUM] = {
    [SPP_PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

void ble_send_notification(spp_message_t *spp_message)
{
    if (!enable_data_ntf) {
        ESP_LOGE(__FUNCTION__, "Data notify disabled");

        return;
    }

    esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL], spp_message->size, spp_message->data, false);
}

static uint8_t find_char_and_desr_index(uint16_t handle)
{
    uint8_t error = 0xff;

    for (int i = 0; i < SPP_IDX_NB ; i++) {
        if (handle == spp_handle_table[i])
            return i;
    }

    return error;
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(__FUNCTION__, "EVT %d, gatts if %d", event, gatts_if);

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            spp_profile_tab[SPP_PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            ESP_LOGI(__FUNCTION__, "Reg app failed, app_id %04x, status %d",param->reg.app_id, param->reg.status);
            return;
        }
    }

    for (int idx = 0; idx < SPP_PROFILE_NUM; idx++) {
        if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                gatts_if == spp_profile_tab[idx].gatts_if) {
            if (spp_profile_tab[idx].gatts_cb) {
                spp_profile_tab[idx].gatts_cb(event, gatts_if, param);
            }
        }
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    esp_err_t err;

    ESP_LOGE(__FUNCTION__, "GAP_EVT, event %d", event);

    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            esp_ble_gap_start_advertising(&spp_adv_params);
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            //advertising start complete event to indicate advertising start successfully or failed
            if ((err = param->adv_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(__FUNCTION__, "Advertising start failed: %s", esp_err_to_name(err));
            }
            break;
        default:
            break;
    }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    esp_ble_gatts_cb_param_t *p_data = (esp_ble_gatts_cb_param_t *) param;

    uint8_t res = 0xff;

    switch (event) {
        case ESP_GATTS_REG_EVT:
                esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);
                esp_ble_gap_config_adv_data_raw((uint8_t *)spp_adv_data, sizeof(spp_adv_data));
                esp_ble_gatts_create_attr_tab(spp_gatt_db, gatts_if, SPP_IDX_NB, SPP_SVC_INST_ID);
            break;

        case ESP_GATTS_READ_EVT:
            // res = find_char_and_desr_index(p_data->read.handle);
            // ESP_LOGI(__FUNCTION__, "ESP_GATTS_READ_EVT : handle = %d", res);
            break;

        case ESP_GATTS_WRITE_EVT:
            if (p_data->write.is_prep == true)
                break;

            res = find_char_and_desr_index(p_data->write.handle);

            ESP_LOGI(__FUNCTION__, "ESP_GATTS_WRITE_EVT : handle = %d", res);

            if (res == SPP_IDX_SPP_DATA_NTF_CFG) {
                if ((p_data->write.len == 2) && (p_data->write.value[1] == 0x00))
                    enable_data_ntf = p_data->write.value[0] == 0x01;
            } else if (res == SPP_IDX_SPP_DATA_RECV_VAL) {
                if (__spp_data_recv_callback != NULL)
                    __spp_data_recv_callback(p_data);
            }
            break;

        case ESP_GATTS_EXEC_WRITE_EVT:
            ESP_LOGI(__FUNCTION__, "ESP_GATTS_EXEC_WRITE_EVT");
            break;

        case ESP_GATTS_MTU_EVT:
            spp_mtu_size = p_data->mtu.mtu;
            break;

        case ESP_GATTS_CONF_EVT:
            break;

        case ESP_GATTS_UNREG_EVT:
                break;

        case ESP_GATTS_DELETE_EVT:
                break;

        case ESP_GATTS_START_EVT:
                break;

        case ESP_GATTS_STOP_EVT:
                break;

        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(__FUNCTION__, "Client connected");
            spp_conn_id = p_data->connect.conn_id;
            spp_gatts_if = gatts_if;
            memcpy(&spp_remote_bda, &p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
            if (__client_connected_callback != NULL)
                __client_connected_callback();
            break;

        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(__FUNCTION__, "Client disconnected");
            enable_data_ntf = false;
            esp_ble_gap_start_advertising(&spp_adv_params);
            break;

        case ESP_GATTS_OPEN_EVT:
            break;

        case ESP_GATTS_CANCEL_OPEN_EVT:
            break;

        case ESP_GATTS_CLOSE_EVT:
            break;

        case ESP_GATTS_LISTEN_EVT:
            break;

        case ESP_GATTS_CONGEST_EVT:
            break;

        case ESP_GATTS_CREAT_ATTR_TAB_EVT:
            ESP_LOGI(__FUNCTION__, "The number handle =%x",param->add_attr_tab.num_handle);
            if (param->add_attr_tab.status != ESP_GATT_OK) {
                ESP_LOGE(__FUNCTION__, "Create attribute table failed, error code=0x%x", param->add_attr_tab.status);
            } else if (param->add_attr_tab.num_handle != SPP_IDX_NB) {
                ESP_LOGE(__FUNCTION__, "Create attribute table abnormally, num_handle (%d) doesn't equal to HRS_IDX_NB(%d)", param->add_attr_tab.num_handle, SPP_IDX_NB);
            } else {
                memcpy(spp_handle_table, param->add_attr_tab.handles, sizeof(spp_handle_table));
                esp_ble_gatts_start_service(spp_handle_table[SPP_IDX_SVC]);
            }
            break;

        default:
            break;
    }
}

void ble_set_spp_data_recv_callback(void *callback)
{
    if (__spp_data_recv_callback != NULL) return;

    __spp_data_recv_callback = (void (*)(esp_ble_gatts_cb_param_t *))callback;
}

bool ble_spp_init()
{
    esp_err_t ret;
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(__FUNCTION__, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return false;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(__FUNCTION__, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return false;
    }

    ESP_LOGI(__FUNCTION__, "%s init bluetooth", __func__);

    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();

    ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg);
    if (ret) {
        ESP_LOGE(__FUNCTION__, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return false;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(__FUNCTION__, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return false;
    }

    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(ESP_SPP_APP_ID);

    return true;
}