/**
 * For ECU connection & packets details see https://gist.github.com/Wexter/e82a5607d0599efa7484552379848761
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include <stdio.h>
#include <stdlib.h>

#define GATTS_TABLE_TAG  "GATTS_SPP_DEMO"

#define SPP_PROFILE_NUM             1
#define SPP_PROFILE_APP_IDX         0
#define ESP_SPP_APP_ID              0x56
#define SAMPLE_DEVICE_NAME          "ESP_SPP_SERVER"    //The Device Name Characteristics in GAP
#define SPP_SVC_INST_ID             0

#define K_LINE_DUMP_PACKETS

#ifndef GPIO_LEVEL_LOW
#define GPIO_LEVEL_LOW 0
#endif

#ifndef GPIO_LEVEL_HIGH
#define GPIO_LEVEL_HIGH 1
#endif

#define UART_NUMBER UART_NUM_2
#define UART_RX_BUF_SIZE 1024
#define UART_TXD_PIN GPIO_NUM_17
#define UART_RXD_PIN GPIO_NUM_16
#define UART_BAUD_RATE 8860
#define UART_DEBUG_PIN GPIO_NUM_5

#define MS_TICKS(ms) (ms / portTICK_PERIOD_MS)

#define ISO9141_ECHO_BYTE_TIMEOUT_TICKS (MS_TICKS(100)) // 100ms
#define ISO9141_ECHO_BYTE_RETRY_COUNT 3
#define ISO9141_INIT_ECU_DST_ADDRESS 0x10
#define ISO9141_PACKET_RECV_TIMEOUT_TICKS (MS_TICKS(100)) // 100ms

#define delay(ms) vTaskDelay(MS_TICKS(ms));

#define ECU_NO_DATA              0x00
#define ECU_END_SESSION          0x01
#define ECU_READ_EPROM           0x02
#define ECU_GET_AFR              0x06
#define ECU_GET_VBAT             0x07
#define ECU_GET_INT_AIR_TEMP     0x08
#define ECU_GET_COOLANT_TEMP     0x09
#define ECU_ERASE_ERR_CODES      0x0A
#define ECU_GET_CO_POT           0x0C
#define ECU_GET_O2_SENSOR        0x0D
#define ECU_GET_IGN_TIME         0x0F
#define ECU_GET_ERROR_CODES      0x11
#define ECU_GET_RPM              0x12
#define ECU_GET_TPS              0x13
#define ECU_GET_ENG_LOAD         0x14
#define ECU_GET_INJ_TIME         0x15
#define ECU_GET_AC_DRV_SW        0x19
#define ECU_GET_O2_REG           0x1A
#define ECU_GET_FPUMP_RELAY      0x1B
#define ECU_GET_ADSORBER_VALVE   0x1C
#define ECU_ENABLE_INJECTOR      0x1D
#define ECU_ENABLE_ADSORB_VALVE  0x1E
#define ECU_ENABLE_IDLE_VALVE    0x1F
#define ECU_MAX_REQUEST          0x1F

#define ECU_ERROR_CODE_13         13 // O2 sensor slow signal change
#define ECU_ERROR_CODE_14         14 // Coolant temp sensor low voltage (Temp > 140 C)
#define ECU_ERROR_CODE_15         15 // Coolant temp sensor high voltage (Temp < -35.4 C & Intake temp < -20)
#define ECU_ERROR_CODE_44         44 // O2 sensor low voltage (<0.09V)
#define ECU_ERROR_CODE_45         45 // O2 sensor high voltage (>1.099V)
#define ECU_ERROR_CODE_48         48 // ECU low input voltage (<10V)
#define ECU_ERROR_CODE_49         49 // ECU high input voltage (>16V)
#define ECU_ERROR_CODE_51         51 // ECU mailfunction.
#define ECU_ERROR_CODE_55         55 // ECU mailfunction.
#define ECU_ERROR_CODE_65         65 // AFR CO pot low voltage
#define ECU_ERROR_CODE_66         66 // AFR CO pot high voltage
#define ECU_ERROR_CODE_67         67 // TPS idle position stuck
#define ECU_ERROR_CODE_69         69 // Intake air temp high voltage (Temp > 140 C)
#define ECU_ERROR_CODE_71         71 // Intake air temp low voltage (Temp < 35 C)
#define ECU_ERROR_CODE_72         72 // TPS kick-down position stuck
#define ECU_ERROR_CODE_73         73 // AFR low voltage
#define ECU_ERROR_CODE_74         74 // AFR high voltage
#define ECU_ERROR_CODE_75         75 // Torque control low voltage

#define ECU_ERROR_STATE_1              0x40 // 0100 0000
#define ECU_ERROR_STATE_ACTIVE         0x60 // 0110 0000
#define ECU_ERROR_STATE_PAST           0xA0 // 1010 0000
#define ECU_ERROR_STATE_ACTIVE_REPEAT  0xE0 // 1110 0000
#define spp_sprintf(s,...)         sprintf((char*)(s), ##__VA_ARGS__)
#define SPP_DATA_MAX_LEN           (512)
#define SPP_CMD_MAX_LEN            (20)
#define SPP_STATUS_MAX_LEN         (20)
#define SPP_DATA_BUFF_MAX_LEN      (2*1024)

///Attributes State Machine
enum{
    SPP_IDX_SVC,

    SPP_IDX_SPP_DATA_RECV_CHAR,
    SPP_IDX_SPP_DATA_RECV_VAL,

    SPP_IDX_SPP_DATA_NOTIFY_CHAR,
    SPP_IDX_SPP_DATA_NTY_VAL,
    SPP_IDX_SPP_DATA_NTF_CFG,

    SPP_IDX_NB,
};

static const char* TAG = "esp-k-line";

enum EcuRequestID {
    NoData, // 0x00
    EndSession, // 0x01
    ReadEPROM, // 0x02
    GetAFR, // 0x06
    GetVBat, // 0x07
    GetIntakeAirTemp, // 0x08
    GetCoolantTemp, // 0x09
    EraseErrorCodes, // 0x0A
    GetCOPot, // 0x0C
    GetO2Sensor, // 0x0D
    GetIgnitionTime, // 0x0F
    GetErrorCodes, // 0x11
    GetRPM, // 0x12
    GetTPS, // 0x13
    GetEngineLoad, // 0x14
    GetInjectionTime, // 0x15
    GetACParams, // 0x19
    GetO2Params, // 0x1A
    GetFuelPumpParams, // 0x1B
    GetAdsorberParams, // 0x1C
    EnableInjector, // 0x1D
    EnableAdsorberValve, // 0x1E
    EnableIdleValve, // 0x1F
    EcuRequestMax
};

uint8_t ecu_recv_buffer[32];

uint8_t const ECU_REQUESTS[][8] = {
   { 0x03, 0x00, 0x09, 0x03, 0x00, 0x00, 0x00, 0x00 }, // 0x00
   { 0x03, 0x00, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00 }, // 0x01
   { 0x06, 0x00, 0x03, 0x0D, 0x00, 0x00, 0x03, 0x00 }, // 0x02
   { 0x04, 0x00, 0x08, 0x00, 0x03, 0x00, 0x00, 0x00 }, // 0x03
   { 0x04, 0x00, 0x08, 0x01, 0x03, 0x00, 0x00, 0x00 }, // 0x04
   { 0x04, 0x00, 0x08, 0x02, 0x03, 0x00, 0x00, 0x00 }, // 0x05
   { 0x04, 0x00, 0x08, 0x03, 0x03, 0x00, 0x00, 0x00 }, // 0x06
   { 0x03, 0x00, 0x05, 0x03, 0x00, 0x00, 0x00, 0x00 }, // 0x07
   { 0x04, 0x00, 0x08, 0x04, 0x03, 0x00, 0x00, 0x00 }, // 0x08
   { 0x04, 0x00, 0x08, 0x05, 0x03, 0x00, 0x00, 0x00 }, // 0x09
   { 0x04, 0x00, 0x08, 0x07, 0x03, 0x00, 0x00, 0x00 }, // 0x0A
   { 0x03, 0x00, 0x07, 0x03, 0x00, 0x00, 0x00, 0x00 }, // 0x0B
   { 0x06, 0x00, 0x01, 0x01, 0x00, 0x3a, 0x03, 0x00 }, // 0x0C
   { 0x06, 0x00, 0x01, 0x01, 0x00, 0x20, 0x03, 0x00 }, // 0x0D
   { 0x06, 0x00, 0x01, 0x01, 0x00, 0x42, 0x03, 0x00 }, // 0x0E
   { 0x06, 0x00, 0x01, 0x02, 0x00, 0x62, 0x03, 0x00 }, // 0x0F

   // ML1.5 commands
   { 0x03, 0x00, 0x12, 0x03, 0x00, 0x00, 0x00, 0x00 }, // 0x10
   { 0x03, 0x00, 0x1c, 0x03, 0x00, 0x00, 0x00, 0x00 }, // 0x11
   { 0x06, 0x00, 0x01, 0x01, 0x01, 0xF8, 0x03, 0x00 }, // 0x12

   // ML4.1 additional parameters
   { 0x06, 0x00, 0x01, 0x01, 0x00, 0x22, 0x03, 0x00 }, // 0x13
   { 0x06, 0x00, 0x01, 0x01, 0x00, 0x29, 0x03, 0x00 }, // 0x14
   { 0x06, 0x00, 0x01, 0x01, 0x01, 0x90, 0x03, 0x00 }, // 0x15
   { 0x06, 0x00, 0x01, 0x01, 0x01, 0xb0, 0x03, 0x00 }, // 0x16

   // ML4.1 devices test
   { 0x04, 0x00, 0x04, 0x0e, 0x03, 0x00, 0x00, 0x00 }, // 0x17
   { 0x04, 0x00, 0x04, 0x1f, 0x03, 0x00, 0x00, 0x00 }, // 0x18
   { 0x04, 0x00, 0x04, 0x21, 0x03, 0x00, 0x00, 0x00 }, // 0x19

   // ML1.5 devices test
   /*
   { 0x04, 0x00, 0x04, 0x10, 0x03, 0x00, 0x00, 0x00 }, // 0x1A
   { 0x04, 0x00, 0x04, 0x05, 0x03, 0x00, 0x00, 0x00 }, // 0x1B
   { 0x04, 0x00, 0x04, 0x04, 0x03, 0x00, 0x00, 0x00 }, // 0x1C
   { 0x04, 0x00, 0x04, 0x17, 0x03, 0x00, 0x00, 0x00 }, // 0x1D
   */
};

/*
const short temperature_values_map[] = {
    197, 194, 191, 189, 186, 184, 181, 178, 
    176, 173, 171, 168, 165, 163, 160, 158, 
    155, 152, 150, 147, 145, 142, 139, 137, 
    134, 132, 129, 126, 124, 121, 119, 116, 
    114, 112, 111, 110, 109, 108, 107, 106, 
    105, 104, 103, 102, 101, 100, 99, 98, 
    97, 95, 94, 93, 92, 91, 90, 89, 
    88, 87, 86, 85, 84, 83, 82, 81, 
    80, 79, 78, 78, 77, 77, 76, 76, 
    75, 74, 74, 73, 73, 72, 72, 71, 
    71, 70, 69, 69, 68, 68, 67, 67, 
    66, 65, 65, 64, 64, 63, 63, 62, 
    62, 61, 61, 60, 60, 59, 59, 58, 
    58, 57, 57, 56, 56, 55, 55, 54, 
    54, 53, 53, 52, 52, 51, 51, 50, 
    50, 49, 49, 48, 48, 47, 47, 46, 
    46, 45, 44, 44, 43, 43, 42, 42, 
    41, 41, 40, 40, 39, 39, 38, 38, 
    37, 36, 36, 35, 35, 34, 34, 33, 
    33, 32, 32, 31, 31, 30, 30, 29, 
    29, 28, 28, 27, 27, 26, 26, 25, 
    25, 24, 24, 23, 23, 22, 22, 21, 
    21, 20, 20, 19, 19, 18, 18, 17, 
    17, 17, 16, 16, 15, 15, 14, 14, 
    13, 13, 12, 12, 11, 11, 10, 10, 
    9, 9, 8, 8, 7, 7, 6, 6, 
    6, 5, 4, 3, 3, 2, 1, 0, 
    0, 0, -1, -2, -3, -3, -4, -5, 
    -6, -6, -7, -8, -9, -10, -10, -11, 
    -12, -13, -14, -14, -15, -16, -17, -18, 
    -19, -20, -21, -22, -24, -25, -26, -28, 
    -29, -30, -32, -33, -34, -36, -37, -38, 
};

const short ignition_time_values_map[] = {
    10, 108, 107, 106, 105, 105, 104, 103, 
    102, 102, 101, 100, 99, 99, 98, 97, 
    96, 96, 95, 94, 93, 93, 92, 91, 
    90, 90, 89, 88, 87, 87, 86, 85, 
    84, 84, 83, 82, 81, 81, 80, 79, 
    78, 78, 77, 76, 75, 75, 74, 73, 
    72, 72, 71, 70, 69, 69, 68, 67, 
    66, 66, 65, 64, 63, 63, 62, 61, 
    60, 60, 59, 58, 57, 57, 56, 55, 
    54, 54, 53, 52, 51, 51, 50, 49, 
    48, 48, 47, 46, 45, 45, 44, 43, 
    42, 42, 41, 40, 39, 39, 38, 37, 
    36, 36, 35, 34, 33, 33, 32, 31, 
    30, 30, 29, 28, 27, 27, 26, 25, 
    24, 24, 23, 22, 21, 21, 20, 19, 
    18, 18, 17, 16, 15, 15, 14, 13, 
    12, 12, 11, 10, 9, 9, 8, 7, 
    6, 6, 5, 4, 3, 3, 2, 1, 
    0, 0, -1, -2, -3, -3, -4, -5, 
    -6, -6, -7, -8, -9, -9, -10, -11, 
    -12, -12, -13, -14, -15, -15, -16, -17, 
    -18, -18, -19, -20, -21, -21, -22, -23, 
    -24, -24, -25, -26, -27, -27, -28, -29, 
    -30, -30, -31, -32, -33, -33, -34, -35, 
    -36, -36, -37, -38, -39, -39, -40, -41, 
    -42, -42, -43, -44, -45, -45, -46, -47, 
    -48, -48, -49, -50, -51, -51, -52, -53, 
    -54, -54, -55, -56, -57, -57, -58, -59, 
    -60, -60, -61, -62, -63, -63, -64, -65, 
    -66, -66, -67, -68, -69, -69, -70, -71, 
    -72, -72, -73, -74, -75, -75, -76, -77, 
    -78, -78, -79, -80, -81, -81, -82, -83,
};
*/

struct iso9141_connection
{
    uint8_t packet_id;
    uint8_t last_sent_packet_id;
} iso9141_connection;

struct iso9141_connection ecu_connection = {
    .packet_id = 0,
    .last_sent_packet_id = 0,
};

static const uint8_t spp_adv_data[23] = {
    /* Flags */
    0x02,0x01,0x06,
    /* Complete List of 16-bit Service Class UUIDs */
    0x03,0x03,0xF0,0xAB,
    /* Complete Local Name in advertising */
    0x0F,0x09, 'E', 'S', 'P', '_', 'S', 'P', 'P', '_', 'S', 'E', 'R','V', 'E', 'R'
};

static uint16_t spp_mtu_size = 23;
static uint16_t spp_conn_id = 0xffff;
static esp_gatt_if_t spp_gatts_if = 0xff;
static QueueHandle_t ml41_request_queue = NULL;

static bool enable_data_ntf = false;
static bool is_connected = false;
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

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

typedef struct spp_data {
    uint16_t size;
    uint8_t *data;
} spp_data_t;

typedef enum {
    Disconnected,
    Initialization,
    Connected
} ECUConnectionState_t;

#define LED_GPIO 2
#define LED_BLINK_DELAY 200

void enable_led(void)
{
    gpio_set_level(LED_GPIO, 1);
}

void disable_led(void)
{
    gpio_set_level(LED_GPIO, 0);
}

void blink_led(uint8_t count)
{
    disable_led();

    do {
        delay(LED_BLINK_DELAY);
        enable_led();
        delay(LED_BLINK_DELAY);
        disable_led();
    } while (--count > 0);
}

void configure_led(void)
{
    gpio_reset_pin(LED_GPIO);

    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
}

static int ecu_connection_state = Disconnected;

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst spp_profile_tab[SPP_PROFILE_NUM] = {
    [SPP_PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

/*
 *  SPP PROFILE ATTRIBUTES
 ****************************************************************************************
 */
#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ|ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_WRITE_NR|ESP_GATT_CHAR_PROP_BIT_READ;

/// SPP Service
static const uint16_t spp_service_uuid = 0xABF0;

///SPP Service - data receive characteristic, read&write without response
static const uint16_t spp_data_receive_uuid = 0xABF1;
static const uint8_t  spp_data_receive_val[20] = { 0x00 };

///SPP Service - data notify characteristic, notify&read
static const uint16_t spp_data_notify_uuid = 0xABF2;
static const uint8_t  spp_data_notify_val[20] = { 0x00 };
static const uint8_t  spp_data_notify_ccc[2] = { 0x00, 0x00 };

///Full HRS Database Description - Used to add attributes into the database
static const esp_gatts_attr_db_t spp_gatt_db[SPP_IDX_NB] =
{
    //SPP -  Service Declaration
    [SPP_IDX_SVC] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *) &primary_service_uuid, ESP_GATT_PERM_READ,
    sizeof(spp_service_uuid), sizeof(spp_service_uuid), (uint8_t *) &spp_service_uuid}},

    //SPP -  data receive characteristic Declaration
    [SPP_IDX_SPP_DATA_RECV_CHAR] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *) &character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *) &char_prop_read_write}},

    //SPP -  data receive characteristic Value
    [SPP_IDX_SPP_DATA_RECV_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *) &spp_data_receive_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    SPP_DATA_MAX_LEN,sizeof(spp_data_receive_val), (uint8_t *) spp_data_receive_val}},

    //SPP -  data notify characteristic Declaration
    [SPP_IDX_SPP_DATA_NOTIFY_CHAR] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *) &character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *) &char_prop_read_notify}},

    //SPP -  data notify characteristic Value
    [SPP_IDX_SPP_DATA_NTY_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *) &spp_data_notify_uuid, ESP_GATT_PERM_READ,
    SPP_DATA_MAX_LEN, sizeof(spp_data_notify_val), (uint8_t *) spp_data_notify_val}},

    // //SPP -  data notify characteristic - Client Characteristic Configuration Descriptor
    [SPP_IDX_SPP_DATA_NTF_CFG] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    sizeof(uint16_t),sizeof(spp_data_notify_ccc), (uint8_t *)spp_data_notify_ccc}},
};

static uint8_t find_char_and_desr_index(uint16_t handle)
{
    uint8_t error = 0xff;

    for (int i = 0; i < SPP_IDX_NB ; i++) {
        if (handle == spp_handle_table[i])
            return i;
    }

    return error;
}

void send_notification(spp_data_t *spp_message)
{
    if (!enable_data_ntf) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s do not enable data Notify", __func__);

        return;
    }

    esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL], spp_message->size, spp_message->data, false);
}

void set_connection_state(ECUConnectionState_t newState)
{
    ecu_connection_state = newState;

    // spp_data_t spp_message = {
    //     .data = { 0x01, (uint8_t) newState },
    //     .size = 2
    // };

    // send_notification(&spp_message);
}

uint8_t spp_end_session_data[5] = { 0x02, 0x03, 0x00, 0x06, 0x03, };
uint8_t spp_no_data_data[5] = { 0x04, 0x03, 0x00, 0x09, 0x03, };

spp_data_t spp_end_session = {
    .size = 5,
    .data = spp_end_session_data
};

spp_data_t no_data_notify = {
    .size = 5,
    .data = spp_no_data_data
};

static void ml41_connection_task()
{
    spp_data_t *spp_message;

    set_connection_state(Initialization);

    blink_led(2);

    set_connection_state(Connected);

    enable_led();

    while (is_connected) {
        if (xQueueReceive(ml41_request_queue, &spp_message, 50 / portTICK_PERIOD_MS)) {
            ESP_LOGI(GATTS_TABLE_TAG, "message size: %d message ptr: %p data ptr: %p \r\n", spp_message->size, spp_message, spp_message->data);

            ESP_LOG_BUFFER_HEXDUMP(GATTS_TABLE_TAG, spp_message->data, spp_message->size, ESP_LOG_INFO);

            if (spp_message->data[0] == 0x02 && spp_message->data[1] == 0x01)
            {
                send_notification(&spp_end_session);
                free(spp_message->data);
                free(spp_message);
                break;
            }

            send_notification(spp_message);

            free(spp_message->data);
            free(spp_message);
        } else {
            send_notification(&no_data_notify);
        }
    }

    disable_led();

    blink_led(3);

    vTaskDelete(NULL);
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(GATTS_TABLE_TAG, "EVT %d, gatts if %d", event, gatts_if);

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            spp_profile_tab[SPP_PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            ESP_LOGI(GATTS_TABLE_TAG, "Reg app failed, app_id %04x, status %d",param->reg.app_id, param->reg.status);
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

bool k_line_send_byte(const uint8_t send_byte, bool wait_echo_byte);
uint8_t k_line_read_byte(uint8_t* buffer, TickType_t read_timeout, bool send_echo);
void ml41_dump_packet(uint8_t * packet);

// void UpdateML41Parameters(uint8_t * ecu_response)
// {
//     /**
//     * ecu_response[0] - Request packet ID
//     * ecu_response[1] - first byte of ecu response
//     */
//     /* SendPacket ID */
//     switch(ecu_response[0]) {
//        /* AFR & COP  */
//        case 6:
//        case 0xc:
//           float value = ecu_response[5] * 5 / 256.f;
//           break;
//        /* Battery voltage */
//        case 7:
//           float value = 0.4 + ecu_response[5] / 15.0f;
//           break;
//        /* Coolant & Intake air temp */
//        /** 
//         * Temperature calculation example
//         * ecu_response[5] = 0xEE
//         * SYMBOLS_ARRAY = 0x554910
//         * ecu_response[5] * 0x04 + SYMBOLS_ARRAY = 0x554cc8
//         * [0x554cc8] = 0xFFFFFFEF = -17
//        */
//        case 8:
//        case 9:
//           result = TEMP_VALUES_ARRAY[ecu_response[5]];
//           break;
//        /* Lambda */
//        case 0xd:
//           result = ecu_response[5] * 4.87;
//           break;
//        /* Ignition time */
//        case 0xf:
//           local_c = ignition_time_values_map[ecu_response[4]];
//           break;
//        /* Engine RPM */
//        /**
//         * ecu_reponse[4] = 0xFF
//         * ecu_response[4] << 3 = 0x07f8
//         * result = (ecu_response[4] << 3) + ((ecu_response[4] << 3) * 4)
//         * result = ecu_response[4] * 0x28 = 10200 rpm
//        */
//        case 0x12:
//           local_14 = ecu_response[4] * 0x28;
//           break;
//        /* TPS */
//        case 0x13:
//           /**
//            * 0x12 - IDLE (ecu_response[4] & 3 == 2)
//            * 0x10 - MID (ecu_response[4] & 3 == 0)
//            * 0x11 - FULL (ecu_response[4] & 3 == 1)
//            * ERR (ecu_response[4] & 3 == 3)
//           */
//           // bVar5 = (ecu_response[4]) & 3; // bits 0 & 1 0000 0011
//           // if (bVar5 == 0) { } // MID
//           // else if (bVar5 == 1) { } // FULL
//           // else if (bVar5 == 2) { } // IDLE
//           // else if (bVar5 == 3) { } // ERR

//           /* Transmission type */
//           if (((ecu_response[4]) & 4) == 0) { } // Manual 
//           else { } // Auto

//           /* O2 Sensor */
//           if (((ecu_response[4]) & 0x20) == 0) { } // Present
//           else { } // Absent
//           break;
//        /* Engine Load */
//        case 0x14:
//           local_c = (0.05f * ecu_response[4]);
//           break;
//        /* InjectionTime */
//        case 0x15:
//           time = (0.3f * (((ecu_response[4]) * -0x100 + 0xffff) - ecu_response[5]) / 250.f);
//           break;
//        /* AC drive & switch */
//        case 0x19:
//           /* edACDrive */
//           if (((ecu_response[4]) & 8) == 0) { } // Off
//           else { } // On

//           /* edACSwitch */
//           if (((ecu_response[4]) & 0x10) == 0) { } // Off
//           else { } // On
//        /* O2 Sensor regulation */
//        case 0x1a:
//           if (hasO2Sensor) {
//              if (((ecu_response[4]) & 0x20) == 0) { } // Open
//              else { } // Close
//           } else { } // Absent
//        /* Fuel pump relay & engine torque control */
//        case 0x1b:
//           /* Fuel pump relay state */
//           if (((ecu_response[4]) & 4) == 0) { } // On
//           else { } // Off

//           /* Engine torque control */
//           if (((ecu_response[4]) & 0x20) == 0) { } // On
//           else { } // Off
//        /* Adsorber valve state*/
//        case 0x1c:
//           if (((ecu_response[4]) & 0x20) == 0) { } // Open
//           else { } // Close
//     }
// }

// void process_ecu_response(uint8_t * ecu_response)
// {
//     switch(ecu_response[3]) {
//        case 0xee:
//        case 0xf4:
//        UpdateML15Parameters(ecu_response);
//        break;
//        case 0xf6:
//        UpdateMainForm(ecu_response);
//        break;
//        case 0xfb:
//        UpdateML41Parameters(ecu_response);
//        break;
//        case 0xfc:
//        ListErrors(ecu_response);
//        break;
//        case 0xfd:
//        pdateEPROMReadProgress(ecu_response,-1);
//        break;
//        case 0xfe:
//        if ((ecu_response[0] == '\x0f') || ((ecu_response[0] - 0x12U) < 4)) {
//           UpdateML41Parameters(ecu_response);
//        } else if (ecu_response[0] == '\x18') {
//           UpdateFuelPumpStatus(ecu_response);
//        } else if ((ecu_response[0]  - 0x19U) < 4)
//           UpdateML41Parameters(ecu_response);
//     }

//     // clear ecu response
//     for (size_t i = 1; i < 24; i++)
//        ecu_response[i] = 0;
//     /* iVar1 = 0;
//     do {
//      *(undefined *)(ecu_response + iVar1 + 0x45) = 0;
//      iVar1 = iVar1 + 1;
//     } while (iVar1 != 0x24); */
// }

bool k_line_send_byte(const uint8_t send_byte, bool wait_echo_byte)
{
    uint8_t echo_byte = 0x00;

    for (uint8_t retry_count = 0; retry_count < ISO9141_ECHO_BYTE_RETRY_COUNT; retry_count++)
    {
        uart_disable_rx_intr(UART_NUMBER);
        uart_write_bytes(UART_NUMBER, &send_byte, 1);
        uart_wait_tx_done(UART_NUMBER, MS_TICKS(10));
        uart_flush(UART_NUMBER);
        uart_enable_rx_intr(UART_NUMBER);

        if (!wait_echo_byte)
            return true;

        if (1 > k_line_read_byte(&echo_byte, MS_TICKS(20), false))
            continue;

        if (echo_byte + send_byte == 0xFF)
            return true;
    }

    return false;
}

uint8_t k_line_read_byte(uint8_t* buffer, TickType_t read_timeout, bool send_echo)
{
    uint8_t* last_byte_ptr = buffer;

    if (1 > uart_read_bytes(UART_NUMBER, last_byte_ptr, 1, read_timeout))
    {
        return 0; // no byte received
    }

    if (send_echo) // Send echo for all except last one
        k_line_send_byte(~(*last_byte_ptr), false);

    return 1;
}

bool ml41_make_request(uint8_t* rx_buff, uint8_t request_id)
{
    if (request_id > ECU_MAX_REQUEST) // invalid request id
        return false;

    uint8_t packet[8];

    memcpy(&packet, ECU_REQUESTS[request_id], 8);

    const uint8_t packet_length = packet[0];

    if (packet_length == 0)
        return false;

    packet[1] = ++ecu_connection.packet_id;

#ifdef K_LINE_DUMP_PACKETS
    ESP_LOGI(TAG, "Sending packet length: %d bytes", packet[0]);
    ml41_dump_packet(packet);
#endif

    for (uint8_t idx = 0; idx <= packet_length; idx++)
        if (!k_line_send_byte(packet[idx], idx < packet_length))
            return false;

    ecu_connection.last_sent_packet_id = request_id;

    return true;
}

uint8_t ml41_recv_packet(uint8_t* buffer)
{
    if (1 > k_line_read_byte(buffer, MS_TICKS(100), true))
        return 0;

    ESP_LOGI(TAG, "Incoming packet length: %d bytes", (int) buffer[0]);

    for (uint8_t idx = 1; idx <= buffer[0]; idx++)
    {
        if (1 > k_line_read_byte(buffer + idx, MS_TICKS(100), idx != buffer[0]))
        {
            return 0;
        }
    }

#ifdef K_LINE_DUMP_PACKETS
    ml41_dump_packet(buffer);
#endif

    ecu_connection.packet_id++;

    return buffer[0];
}

void ml41_send_slow_init_wakeup()
{
    // Bit-bang 5baud init byte
    gpio_reset_pin(UART_TXD_PIN);

    gpio_set_direction(UART_TXD_PIN, GPIO_MODE_OUTPUT);

    gpio_set_level(UART_TXD_PIN, GPIO_LEVEL_LOW);

    delay(1000);

    gpio_set_level(UART_TXD_PIN, GPIO_LEVEL_HIGH);

    delay(200);

    gpio_set_level(UART_TXD_PIN, GPIO_LEVEL_LOW);

    delay(600);

    gpio_set_level(UART_TXD_PIN, GPIO_LEVEL_HIGH);
}

bool ml41_start_full_speed()
{
    const uart_config_t uart_config = {
       .baud_rate = UART_BAUD_RATE,
       .data_bits = UART_DATA_8_BITS,
       .parity = UART_PARITY_DISABLE,
       .stop_bits = UART_STOP_BITS_1,
       .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
       .source_clk = UART_SCLK_DEFAULT,
    };

    uart_driver_install(UART_NUMBER, UART_RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUMBER, &uart_config);
    uart_set_rx_full_threshold(UART_NUMBER, 1);
    uart_set_pin(UART_NUMBER, UART_TXD_PIN, UART_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    ESP_LOGI(TAG, "UART configured");

    uint8_t rx_byte;

    int bytes_read = uart_read_bytes(UART_NUMBER, &rx_byte, 1, MS_TICKS(1000));
    if (0 < bytes_read && rx_byte == 0x55)
    {
        return true;
    }

    return false;
}

bool ml41_recv_keywords()
{
    uint8_t rx_byte;

    if (1 > k_line_read_byte(&rx_byte, MS_TICKS(100), true))
        return false;

    if (1 > k_line_read_byte(&rx_byte, MS_TICKS(100), true))
        return false;

    return true;
}

void ml41_dump_packet(uint8_t* packet)
{
    char packet_str[64] = { 0 };

    uint8_t packet_length = packet[0];

    for (uint8_t idx = 0; idx <= packet_length; idx++)
        sprintf(packet_str + idx * 3, "%02X ", packet[idx]);
        // ESP_LOGI(TAG, "%02X", packet[idx]);

    ESP_LOGI(TAG, "Packet: %s", packet_str);
}

bool ml41_read_ecu_init_data()
{
    uint8_t rx_buff[24] = { 0 };

    // ECU EPROM code
    if (1 > ml41_recv_packet(rx_buff))
    {
        ESP_LOGI(TAG, "EPROM code read error");

        return false;
    }

    rx_buff[rx_buff[0]] = '\x0';

    ESP_LOGI(TAG, "EPROM code: %s", rx_buff + 3);

    delay(15);

    if (1 > ml41_make_request(rx_buff, ECU_NO_DATA))
    {
        ESP_LOGI(TAG, "BOSCH code read error");

        return false;
    }

    // ECU BOSCH code
    if (1 > ml41_recv_packet(rx_buff))
    {
    }

    rx_buff[rx_buff[0]] = '\x0';

    ESP_LOGI(TAG, "BOSCH code: %s", rx_buff + 3);

    delay(15);

    // GM CODE + ALFA code
    ml41_make_request(rx_buff, ECU_NO_DATA);

    // ECU BOSCH code
    if (1 > ml41_recv_packet(rx_buff))
    {
        ESP_LOGI(TAG, "GM CODE code read error");

        return false;
    }

    rx_buff[rx_buff[0]] = '\x0';

    ESP_LOGI(TAG, "GM CODE code: %s", rx_buff + 3);

    while (true) {
        delay(50);

        ml41_make_request(rx_buff, ECU_NO_DATA);

        if (1 > ml41_recv_packet(rx_buff))
        {
            ESP_LOGI(TAG, "ml41_recv_packet failed");

            return false;
        }
        delay(50);
        ml41_make_request(rx_buff, ECU_GET_AFR);
        if (1 > ml41_recv_packet(rx_buff))
        {
            ESP_LOGI(TAG, "ml41_recv_packet failed");

            return false;
        }
        delay(50);
        ml41_make_request(rx_buff, ECU_GET_VBAT);
        if (1 > ml41_recv_packet(rx_buff))
        {
            ESP_LOGI(TAG, "ml41_recv_packet failed");

            return false;
        }
        delay(50);
        ml41_make_request(rx_buff, ECU_GET_INT_AIR_TEMP);
        if (1 > ml41_recv_packet(rx_buff))
        {
            ESP_LOGI(TAG, "ml41_recv_packet failed");

            return false;
        }
        delay(50);
        ml41_make_request(rx_buff, ECU_GET_COOLANT_TEMP);
        if (1 > ml41_recv_packet(rx_buff))
        {
            ESP_LOGI(TAG, "ml41_recv_packet failed");

            return false;
        }
        delay(50);
        ml41_make_request(rx_buff, ECU_ERASE_ERR_CODES);
        if (1 > ml41_recv_packet(rx_buff))
        {
            ESP_LOGI(TAG, "ml41_recv_packet failed");

            return false;
        }
        delay(50);
        ml41_make_request(rx_buff, ECU_GET_CO_POT);
        if (1 > ml41_recv_packet(rx_buff))
        {
            ESP_LOGI(TAG, "ml41_recv_packet failed");

            return false;
        }
        delay(50);
        ml41_make_request(rx_buff, ECU_GET_O2_SENSOR);
        if (1 > ml41_recv_packet(rx_buff))
        {
            ESP_LOGI(TAG, "ml41_recv_packet failed");

            return false;
        }
        delay(50);
        ml41_make_request(rx_buff, ECU_GET_IGN_TIME);
        if (1 > ml41_recv_packet(rx_buff))
        {
            ESP_LOGI(TAG, "ml41_recv_packet failed");

            return false;
        }
        // delay(50);
        // ml41_make_request(rx_buff, ECU_GET_ERROR_CODES);
        //         if (1 > ml41_recv_packet(rx_buff))
        // {
        //     ESP_LOGI(TAG, "ml41_recv_packet failed");

        //     return false;
        // }
        delay(50);
        ml41_make_request(rx_buff, ECU_GET_RPM);
        if (1 > ml41_recv_packet(rx_buff))
        {
            ESP_LOGI(TAG, "ml41_recv_packet failed");

            return false;
        }
        delay(50);
        ml41_make_request(rx_buff, ECU_GET_TPS);
        if (1 > ml41_recv_packet(rx_buff))
        {
            ESP_LOGI(TAG, "ml41_recv_packet failed");

            return false;
        }
        delay(50);
        ml41_make_request(rx_buff, ECU_GET_ENG_LOAD);
        if (1 > ml41_recv_packet(rx_buff))
        {
            ESP_LOGI(TAG, "ml41_recv_packet failed");

            return false;
        }
        delay(50);
        ml41_make_request(rx_buff, ECU_GET_INJ_TIME);
        if (1 > ml41_recv_packet(rx_buff))
        {
            ESP_LOGI(TAG, "ml41_recv_packet failed");

            return false;
        }
        delay(50);
        ml41_make_request(rx_buff, ECU_GET_AC_DRV_SW);
        if (1 > ml41_recv_packet(rx_buff))
        {
            ESP_LOGI(TAG, "ml41_recv_packet failed");

            return false;
        }
        delay(50);
        ml41_make_request(rx_buff, ECU_GET_O2_REG);
        if (1 > ml41_recv_packet(rx_buff))
        {
            ESP_LOGI(TAG, "ml41_recv_packet failed");

            return false;
        }
        delay(50);
        ml41_make_request(rx_buff, ECU_GET_FPUMP_RELAY);
        if (1 > ml41_recv_packet(rx_buff))
        {
            ESP_LOGI(TAG, "ml41_recv_packet failed");

            return false;
        }
        delay(50);
        ml41_make_request(rx_buff, ECU_GET_ADSORBER_VALVE);
        if (1 > ml41_recv_packet(rx_buff))
        {
            ESP_LOGI(TAG, "ml41_recv_packet failed");

            return false;
        }

        UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );

        ESP_LOGI(TAG, "Task stack high water mark: %d", uxHighWaterMark);
    }

    delay(15);

    ml41_make_request(rx_buff, ECU_END_SESSION);

    if (1 > ml41_recv_packet(rx_buff))
    {
        ESP_LOGI(TAG, "ml41_recv_packet failed");

        return false;
    }

    return true;
}

void ml41_init_connection(void *)
{
    ml41_send_slow_init_wakeup();

    if (!ml41_start_full_speed())
    {
        ESP_LOGI(TAG, "ECU connection error: no sync received");

        delay(5000);

        return;
    }

    if (!ml41_recv_keywords())
    {
        ESP_LOGI(TAG, "ECU connection error: no KW received");
        
        delay(5000);
        return;
    }

    if (!ml41_read_ecu_init_data())
    {
        delay(5000);
        return;
    }
}


static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    esp_err_t err;

    ESP_LOGE(GATTS_TABLE_TAG, "GAP_EVT, event %d", event);

    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            esp_ble_gap_start_advertising(&spp_adv_params);
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            //advertising start complete event to indicate advertising start successfully or failed
            if ((err = param->adv_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TABLE_TAG, "Advertising start failed: %s", esp_err_to_name(err));
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
            // ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_READ_EVT : handle = %d", res);
            break;

        case ESP_GATTS_WRITE_EVT:
            if (p_data->write.is_prep == true)
                break;

            res = find_char_and_desr_index(p_data->write.handle);

            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_WRITE_EVT : handle = %d", res);

            if (res == SPP_IDX_SPP_DATA_NTF_CFG) {
                if ((p_data->write.len == 2) && (p_data->write.value[1] == 0x00))
                    enable_data_ntf = p_data->write.value[0] == 0x01;
            } else if (res == SPP_IDX_SPP_DATA_RECV_VAL) {
                if (p_data->write.len > 1) {
                    spp_data_t *spp_message = (spp_data_t *) malloc(sizeof(spp_data_t));

                    spp_message->size = p_data->write.len;
                    spp_message->data = (uint8_t *) malloc(p_data->write.len);

                    memcpy(spp_message->data, p_data->write.value, p_data->write.len);

                    xQueueSend(ml41_request_queue, &spp_message, 10 / portTICK_PERIOD_MS);
                    break;
                }

                switch (p_data->write.value[0])
                {
                    case 0x01: // start ecu connection
                        // start ECU connection task
                        xTaskCreate(ml41_init_connection, "ml41_init_connection", 16384, NULL, configMAX_PRIORITIES - 2, NULL);
                        break;
                    case 0x02:
                        spp_data_t *spp_message = (spp_data_t *) malloc(sizeof(spp_data_t));

                        spp_message->size = 2;
                        spp_message->data = (uint8_t *) malloc(spp_message->size);

                        memcpy(spp_message->data, (uint8_t[2]) { 0x02, 0x01 }, 2);

                        xQueueSend(ml41_request_queue, &spp_message, 10/ portTICK_PERIOD_MS);
                    default:
                        break;
                }
            }
            break;

        case ESP_GATTS_EXEC_WRITE_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
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
            spp_conn_id = p_data->connect.conn_id;
            spp_gatts_if = gatts_if;
            is_connected = true;
            memcpy(&spp_remote_bda, &p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
                break;

        case ESP_GATTS_DISCONNECT_EVT:
            is_connected = false;
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
            ESP_LOGI(GATTS_TABLE_TAG, "The number handle =%x",param->add_attr_tab.num_handle);
            if (param->add_attr_tab.status != ESP_GATT_OK) {
                ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table failed, error code=0x%x", param->add_attr_tab.status);
            } else if (param->add_attr_tab.num_handle != SPP_IDX_NB) {
                ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table abnormally, num_handle (%d) doesn't equal to HRS_IDX_NB(%d)", param->add_attr_tab.num_handle, SPP_IDX_NB);
            } else {
                memcpy(spp_handle_table, param->add_attr_tab.handles, sizeof(spp_handle_table));
                esp_ble_gatts_start_service(spp_handle_table[SPP_IDX_SVC]);
            }
            break;

        default:
            break;
    }
}


void app_main(void)
{
    delay(2000);

    configure_led();

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
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(GATTS_TABLE_TAG, "%s init bluetooth", __func__);

    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();

    ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(ESP_SPP_APP_ID);

    ml41_request_queue = xQueueCreate(32, sizeof(void *));
}
