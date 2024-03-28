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

#ifndef GPIO_LEVEL_LOW
#define GPIO_LEVEL_LOW 0
#endif

#ifndef GPIO_LEVEL_HIGH
#define GPIO_LEVEL_HIGH 1
#endif

#define UART_NUMBER UART_NUM_0
#define UART_RX_BUF_SIZE 255
#define UART_TXD_PIN GPIO_NUM_1
#define UART_RXD_PIN GPIO_NUM_3
#define UART_BAUD_RATE 8860

#define KWP2000_ECHO_BYTE_TIMEOUT_TICKS (100 / portTICK_PERIOD_MS) // 100ms
#define KWP2000_ECHO_BYTE_RETRY_COUNT 3
#define KWP2000_INIT_ECU_DST_ADDRESS 0x10
#define KWP2000_PACKET_RECV_TIMEOUT_TICKS (100 / portTICK_PERIOD_MS) // 100ms

#define delay(ms) vTaskDelay(ms / portTICK_PERIOD_MS);

// 0x00
const uint8_t no_data_request[] = { 0x03, 0x00, 0x09, 0x00, };
// 0x02 EPROM read request
const uint8_t ml41_eprom_read_request[] = { 0x06, 0x00, 0x03, 0x0D, 0x00, 0x00, 0x03,};
// 0x06
const uint8_t get_afr_request[] = { 0x04, 0x00, 0x08, 0x00, 0x03, };
// 0x07
const uint8_t get_battery_voltage_request[] = { 0x04, 0x00, 0x08, 0x01, 0x03, };
// 0x08
const uint8_t get_intake_air_temp_request[] = { 0x04, 0x00, 0x08, 0x02, 0x03, };
// 0x09
const uint8_t get_coolant_temp_request[] = { 0x04, 0x00, 0x08, 0x03, 0x03, };
// 0x0A
const uint8_t erase_ecu_error_codes_request[] = { 0x03, 0x00, 0x05, 0x03, };
// 0x0C
const uint8_t get_co_pot_value_request[] = { 0x04, 0x00, 0x08, 0x04, 0x03, };
// 0x0D
const uint8_t get_o2_sensor_value_request[] = { 0x04, 0x00, 0x08, 0x05, 0x03, };
// 0x0F
const uint8_t get_ignition_time_request[] = { 0x04, 0x00, 0x08, 0x07, 0x03, };
// 0x11
const uint8_t get_ecu_error_codes_request[] = { 0x03, 0x00, 0x07, 0x03, };
// 0x12
const uint8_t get_engine_rpm_request[] = { 0x06, 0x00, 0x01, 0x01, 0x00, 0x3a, 0x03, };
// 0x13 TPS + O2 sensor
const uint8_t get_throttle_pos_request[] = { 0x06, 0x00, 0x01, 0x01, 0x00, 0x20, 0x03, };
// 0x14
const uint8_t get_engine_load_request[] = { 0x06, 0x00, 0x01, 0x01, 0x00, 0x42, 0x03, };
// 0x15
const uint8_t get_injection_time_request[] = { 0x06, 0x00, 0x01, 0x02, 0x00, 0x62, 0x03, };

// -------------- ML1.5 commands --------------
// 0x16
// const uint8_t ml15_get_engine_rpm_request[] = { 0x03, 0x00, 0x12, 0x03, };
// 0x17
// const uint8_t ml15_get_ecu_ad_param1_request[] = { 0x03, 0x00, 0x1c, 0x03, };
// 0x18
// const uint8_t ml15_get_ecu_ad_param2_request[] = { 0x06, 0x00, 0x01, 0x01, 0x01, 0xF8, 0x03, };

// -------------- ML4.1 commands --------------
// 0x19 AC Switch & Drive request
const uint8_t ml41_get_ac_drive_switch_request[] = { 0x06, 0x00, 0x01, 0x01, 0x00, 0x22, 0x03, };
// 0x1a Lambda regulation open/close
const uint8_t ml41_get_lambda_reg_request[] = { 0x06, 0x00, 0x01, 0x01, 0x00, 0x29, 0x03, };
// 0x1b Fuel pump relay
const uint8_t ml41_get_ad_param3_request[] = { 0x06, 0x00, 0x01, 0x01, 0x00, 0x90, 0x03, };
// 0x1c
const uint8_t ml41_get_adsorber_request[] = { 0x06, 0x00, 0x01, 0x01, 0x00, 0xb0, 0x03, };
// 0x1d
const uint8_t ml41_enable_injector_request[] = { 0x04, 00, 0x04, 0x0e, 0x03, };
// 0x1e
const uint8_t ml41_enable_adsorber_valve_request[] = { 0x04, 0x00, 0x04, 0x1f, 0x03, };
// 0x1f
const uint8_t ml41_enable_idle_valve_request[] = { 0x04, 0x00, 0x04, 0x21, 0x03, };

// 0x20
// const uint8_t ml15_enable_injector_request[] = { 0x04, 0x00, 0x04, 0x10, 0x03, };
// 0x21
// const uint8_t ml15_enable_adsorber_valve_request[] = { 0x04, 0x00, 0x04, 0x05, 0x03, };
// 0x22
// const uint8_t ml15_enable_idle_valve_request[] = { 0x04, 0x00, 0x04, 0x04, 0x03, };
// 0x23
// const uint8_t ml15_enable_???_request[] = { 0x04, 0x00, 0x04, 0x17, 0x03, };


#define EEPROME_CODE_SIZE 11
#define BOSCH_CODE_SIZE 11
#define GM_CODE_SIZE 8
#define ALPHA_CODE_SIZE 3

typedef struct ml41_ecu_data {
    uint8_t packet_id;
    uint8_t eeprom_code[EEPROME_CODE_SIZE];
    uint8_t bosch_code[BOSCH_CODE_SIZE];
    uint8_t gm_code[GM_CODE_SIZE];
    uint8_t alpha_code[ALPHA_CODE_SIZE];
} ml41_ecu_data;

struct ml41_ecu_data ecu_data = {
    .packet_id = 0,
};


void UpdateMainForm(uint8_t *ecu_response)
{

}

void UpdateML41Parameters(uint8_t * ecu_response)
{
    /**
     * ecu_response[0] - Request packet ID
     * ecu_response[1] - first byte of ecu response
    */
    /* SendPacket ID */
    switch(ecu_response[0]) {
        /* AFR & COP  */
        case 6:
        case 0xc:
            float value = ecu_response[5] * 5 / 256.f;
            break;
        /* Battery voltage */
        case 7:
            float value = 0.4 + ecu_response[5] / 15.0f;
            FloatToStrF(2,4,1,(int *)&local_10);
            break;
        /* Coolant & Intake air temp */
        /** 
         * Temperature calculation example
         * ecu_response[5] = 0xEE
         * SYMBOLS_ARRAY = 0x554910
         * ecu_response[5] * 0x04 + SYMBOLS_ARRAY = 0x554cc8
         * [0x554cc8] = 0xFFFFFFEF = -17
        */
        case 8:
        case 9:
            local_c = (double)*(int *)(SYMBOLS_ARRAY + (ecu_response[5]) * 4);
            FloatToStr((int *)&local_10);
            break;
        /* Lambda */
        case 0xd:
            local_14 = (uint)(ecu_response[5]);
            local_1c = system.@ROUND();
            local_c = (double)CONCAT44(extraout_EDX,local_1c);
            FloatToStr((int *)&local_10);
            break;
        /* Ignition time */
        case 0xf:
            local_c = (double)SYMBOLS_ARRAY2[(ecu_response[4])];
            FloatToStr((int *)&local_10);
            break;
        /* Engine RPM */
        /**
         * ecu_reponse[4] = 0xFF
         * ecu_response[4] << 3 = 0x07f8
         * result = (ecu_response[4] << 3) + ((ecu_response[4] << 3) * 4)
         * result = ecu_response[4] * 28 = 10200 rpm
        */
        case 0x12:
            local_14 = (uint)(ecu_response[4]) * 0x28;
            local_c = (double)local_14;
            FloatToStr((int *)&local_10);
            break;
        /* TPS */
        case 0x13:
            /**
             * 0x12 - IDLE (ecu_response[4] & 3 == 2)
             * 0x10 - MID (ecu_response[4] & 3 == 0)
             * 0x11 - FULL (ecu_response[4] & 3 == 1)
             * ERR (ecu_response[4] & 3 == 3)
            */
            // bVar5 = (ecu_response[4]) & 3;
            // if (bVar5 == 0) { } // MID
            // else if (bVar5 == 1) { } // FULL
            // else if (bVar5 == 2) { } // IDLE
            // else if (bVar5 == 3) { } // ERR

            /* edTrans */
            if (((ecu_response[4]) & 4) == 0) { } // Manual
            else { } // Auto

            /* edLZond */
            if (((ecu_response[4]) & 0x20) == 0) {
                TControl.SetText(*(int *)(*(int *)TfmAdParams + 0x314),DAT_0055ef94);
            }
            else {
                TControl.SetText(*(int *)(*(int *)TfmAdParams + 0x314),DAT_0055ef98);
            }
            break;
        case 0x14:
            /* Engine Load */
            local_c = (0.05f * ecu_response[4]);
            FloatToStrF(2,4,2,(int *)&local_10);
            break;
        case 0x15:
            /* InjectionTime */
            local_14 = ((uint)(ecu_response[4]) * -0x100 + 0xffff) - (uint)(ecu_response[5]);
            local_c = (double)((0.3f * local_14) / 250.f);
            FloatToStrF(2,4,2,(int *)&local_10);
            break;
        case 0x19:
            /* ECU Additional Param 1 */
            if (*(char *)(*(int *)TfmAdParams + 0x1a6) != '\0') {
                /* edACDrive */
                if (((ecu_response[4]) & 8) == 0) {
                    TControl.SetText(*(int *)(*(int *)TfmAdParams + 0x32c),DAT_0055ef70);
                }
                else {
                    TControl.SetText(*(int *)(*(int *)TfmAdParams + 0x32c),DAT_0055ef6c);
                }
                                /* edACSwitch */
                if (((ecu_response[4]) & 0x10) == 0) {
                    TControl.SetText(*(int *)(*(int *)TfmAdParams + 0x334),DAT_0055ef70);
                }
                else {
                    TControl.SetText(*(int *)(*(int *)TfmAdParams + 0x334),DAT_0055ef6c);
                }
            }
            goto LAB_0054af5f;
        case 0x1a:
            /* ECU Additional Param 2 */
            uVar6 = *(char *)(*(int *)TfmAdParams + 0x1a6) == '\0';
            if (!(bool)uVar6) {
                /* edLZond */
                TControl.GetText(*(int *)(*(int *)TfmAdParams + 0x314),(int *)&local_20);
                @LStrCmp(local_20,DAT_0055ef94);
                                /* edLZondReg */
                if ((bool)uVar6) {
                    if (((ecu_response[4]) & 0x20) == 0) {
                    TControl.SetText(*(int *)(*(int *)TfmAdParams + 0x318),DAT_0055ef74);
                    }
                    else {
                    TControl.SetText(*(int *)(*(int *)TfmAdParams + 0x318),DAT_0055ef78);
                    }
                } else {
                    TControl.SetText(*(int *)(*(int *)TfmAdParams + 0x318),DAT_0055ef98);
                }
            }
            goto LAB_0054af5f;
        case 0x1b:
            if (*(char *)(*(int *)TfmAdParams + 0x1a6) != '\0') {
                            /* edFuelPump */
            if (((ecu_response[4]) & 4) == 0) {
                TControl.SetText(*(int *)(*(int *)TfmAdParams + 0x33c),DAT_0055ef6c);
            }
            else {
                TControl.SetText(*(int *)(*(int *)TfmAdParams + 0x33c),DAT_0055ef70);
            }
                            /* edEngPwr */
            if (((ecu_response[4]) & 0x20) == 0) {
                TControl.SetText(*(int *)(*(int *)TfmAdParams + 0x31c),DAT_0055ef70);
            }
            else {
                TControl.SetText(*(int *)(*(int *)TfmAdParams + 0x31c),DAT_0055ef6c);
            }
            }
            goto LAB_0054af5f;
        case 0x1c:
            if (*(char *)(*(int *)TfmAdParams + 0x1a6) != '\0') {
                            /* Adsorber */
            if (((ecu_response[4]) & 0x20) == 0) {
                TControl.SetText(*(int *)(*(int *)TfmAdParams + 0x320),DAT_0055ef74);
            }
            else {
                TControl.SetText(*(int *)(*(int *)TfmAdParams + 0x320),DAT_0055ef78);
            }
            }
            goto LAB_0054af5f;
    }
    /*puVar4 = TList.Get(ParametersList,TList_var_index);
    puVar4[2] = (uint)local_c;
    puVar4[3] = local_c._4_4_;
    puVar4 = TList.Get(ParametersList,TList_var_index);
    @LStrAsg((int *)(puVar4 + 4),local_10);
    TList_var_index = TList_var_index + 1;
    if ((int)ParametersList[2] <= TList_var_index) {
        TList_var_index = 0;
        list_size = ParametersList[2];
        if (-1 < list_size + -1) {
        list_index = 0;
        do {
            puVar4 = TList.Get(ParametersList,list_index);
            uVar10 = (undefined2)puVar4[4];
            uVar11 = (undefined2)(puVar4[4] >> 0x10);
            puVar4 = TList.Get(ParametersList,list_index);
            TControl.SetText(**(int **)(puVar4[0xc] + 8),(uint *)CONCAT22(uVar11,uVar10));
            dVar1 = (double)StrangeParametersCounter;
            uVar3 = SUB84(dVar1,0);
            uVar10 = (undefined2)((ulonglong)dVar1 >> 0x20);
            uVar11 = (undefined2)((ulonglong)dVar1 >> 0x30);
            puVar4 = TList.Get(ParametersList,list_index);
            uVar9 = puVar4[3];
            uVar8 = puVar4[2];
            puVar4 = TList.Get(ParametersList,list_index);
            sgr_FUN_004a64dc(**(int **)(puVar4[0xc] + 0x10),extraout_EDX_00,extraout_ECX,uVar8,uVar9,
                            uVar3,CONCAT22(uVar11,uVar10));
            uVar7 = 0x3ff00000;
            uVar3 = 0;
            puVar4 = TList.Get(ParametersList,list_index);
            sgr_FUN_004a2f1c(*(int *)(**(int **)(puVar4[0xc] + 0xc) + 0x218),extraout_EDX_01,
                            extraout_ECX_00,uVar3,uVar7);
            list_index = list_index + 1;
            list_size = list_size + -1;
        } while (list_size != 0);
        }
        StrangeParametersCounter = StrangeParametersCounter + 1;
    }*/
}

void process_ecu_response(uint8_t * ecu_response)
{
    switch(ecu_response[3]) {
        case 0xee:
        case 0xf4:
        UpdateML15Parameters(ecu_response);
        break;
        case 0xf6:
        UpdateMainForm(ecu_response);
        break;
        case 0xfb:
        UpdateML41Parameters(ecu_response);
        break;
        case 0xfc:
        ListErrors(ecu_response);
        break;
        case 0xfd:
        UpdateEPROMReadProgress(ecu_response,-1);
        break;
        case 0xfe:
        if ((ecu_response[0] == '\x0f') || ((ecu_response[0] - 0x12U) < 4)) {
            UpdateML41Parameters(ecu_response);
        } else if (ecu_response[0] == '\x18') {
            UpdateFuelPumpStatus(ecu_response);
        } else if ((ecu_response[0]  - 0x19U) < 4)
            UpdateML41Parameters(ecu_response);
    }

    // clear ecu response
    for (size_t i = 1; i < 24; i++)
        ecu_response[i] = 0;
    /* iVar1 = 0;
    do {
      *(undefined *)(ecu_response + iVar1 + 0x45) = 0;
      iVar1 = iVar1 + 1;
    } while (iVar1 != 0x24); */
}

bool k_line_send_byte(const uint8_t send_byte, bool wait_echo_byte)
{
    uart_disable_rx_intr(UART_NUMBER);
    uart_write_bytes(UART_NUMBER, &send_byte, 1);
    uart_enable_rx_intr(UART_NUMBER);

    if (!wait_echo_byte)
        return true;

    uint8_t echo_byte = 0x00,
        bytes_read = 0;

    for (uint8_t retry_count = 0; retry_count < KWP2000_ECHO_BYTE_RETRY_COUNT; retry_count++)
    {
        bytes_read = uart_read_bytes(UART_NUMBER, &echo_byte, 1, KWP2000_ECHO_BYTE_TIMEOUT_TICKS);

        if (bytes_read > 0 && echo_byte == ~send_byte)
            return true;
    }

    return false;
}

int k_line_read_bytes(uint8_t* bytes, int bytes_count, bool send_echo, TickType_t read_timeout)
{
    uint8_t bytes_read = 0,
        last_byte = 0;

    do {
        if (0 >= uart_read_bytes(UART_NUMBER, &last_byte, 1, read_timeout))
            return bytes_read; // no byte received

        bytes[bytes_read] = last_byte;

        if (bytes_read + 1 < bytes_count) // Send echo for all except last one
            k_line_send_byte(~last_byte, false);
    } while (++bytes_read < bytes_count);

    return bytes_read;
}

bool ml41_send_packet(ml41_ecu_data* connection,const uint8_t* packet)
{
    const uint8_t packet_length = packet[0];

    // Send packet length
    if (!k_line_send_byte(packet_length, true))
        return false;

    // Send packet sequence number
    if (!k_line_send_byte(connection->packet_id++, true))
        return false;

    // Send packet data
    // Skip first 2 bytes (length & packet id) and last byte (EOP)
    for (uint8_t i = 2; i < packet_length - 1; i++)
        if (!k_line_send_byte(packet[i], true))
            return false;

    // Send packet end mark 0x03 without waiting for echo
    k_line_send_byte(0x03, false);

    return true;
}

void send_nop_packet(ml41_ecu_data* connection)
{
    ml41_send_packet(connection, no_data_cmd);
}

// will read packet and return it's data
int ml41_recv_packet(ml41_ecu_data* connection, uint8_t * rx_buffer)
{
    connection->packet_id++;

    return 0;
}

void ml41_send_slow_init_wakeup()
{
    // Bit-bang 5baud init byte
    gpio_reset_pin(UART_TXD_PIN);

    gpio_set_direction(UART_TXD_PIN, GPIO_MODE_OUTPUT);

    gpio_set_level(UART_TXD_PIN, GPIO_LEVEL_LOW);

    delay(1200);

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

    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUMBER, UART_RX_BUF_SIZE, 0, 0, NULL, 0);
    uart_param_config(UART_NUMBER, &uart_config);
    uart_set_pin(UART_NUMBER, UART_TXD_PIN, UART_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uint8_t rx_byte;

    if (0 < k_line_read_bytes(&rx_byte, 1, false, 1000 / portTICK_PERIOD_MS) && rx_byte == 0x55)
        return false;

    return true;
}

bool ml41_recv_keywords()
{
    uint8_t rx_buffer[2];

    if (2 > k_line_read_bytes(rx_buffer, 2, true, KWP2000_PACKET_RECV_TIMEOUT_TICKS))
        return false;

    return true;
}

void ml41_read_ecu_init_data(ml41_ecu_data* connection)
{
    // ECU EEPROM code
    // ml41_recv_packet();

    // ECU BOSCH code

    // GM CODE + ALFA code
}

ml41_ecu_data* ml41_init_connection(void *)
{
    ml41_send_slow_init_wakeup();

    if (!ml41_start_full_speed());

    ml41_recv_keywords();

    ml41_read_ecu_init_data(&ml41_conn);

    // Start ecu send/recv queue
    // xTaskCreate();
    return &ml41_conn;

    // start ECU connection task
    xTaskCreate(ml41_init_connection, "ml41_init_connection", 1024 * 2, NULL, configMAX_PRIORITIES - 2, NULL);
}

void init(void)
{
    // wait 5 seconds
    delay(5000);

}

void app_main(void)
{
    init();
}
