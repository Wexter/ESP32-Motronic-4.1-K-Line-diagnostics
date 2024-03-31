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
#define ECU_ERROR_STATE_PAST           0x0A // 1010 0000
#define ECU_ERROR_STATE_ACTIVE_REPEAT  0xE0 // 1110 0000

static const char* TAG = "esp-k-line";

uint8_t ecu_recv_buffer[32];

uint8_t const ECU_REQUESTS[][8] = {
   { 0x03, 0x00, 0x09, 0x03, 0x00, 0x00, 0x00, 0x00 }, // 0x00
   { 0x03, 0x00, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00 }, // 0x01
   { 0x06, 0x00, 0x03, 0x0D, 0x00, 0x00, 0x03, 0x00 }, // 0x02
   { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, // 0x03
   { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, // 0x04
   { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, // 0x05
   { 0x04, 0x00, 0x08, 0x00, 0x03, 0x00, 0x00, 0x00 }, // 0x06
   { 0x04, 0x00, 0x08, 0x01, 0x03, 0x00, 0x00, 0x00 }, // 0x07
   { 0x04, 0x00, 0x08, 0x02, 0x03, 0x00, 0x00, 0x00 }, // 0x08
   { 0x04, 0x00, 0x08, 0x03, 0x03, 0x00, 0x00, 0x00 }, // 0x09
   { 0x03, 0x00, 0x05, 0x03, 0x00, 0x00, 0x00, 0x00 }, // 0x0A
   { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, // 0x0B
   { 0x04, 0x00, 0x08, 0x04, 0x03, 0x00, 0x00, 0x00 }, // 0x0C
   { 0x04, 0x00, 0x08, 0x05, 0x03, 0x00, 0x00, 0x00 }, // 0x0D
   { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, // 0x0E
   { 0x04, 0x00, 0x08, 0x07, 0x03, 0x00, 0x00, 0x00 }, // 0x0F
   { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, // 0x10
   { 0x03, 0x00, 0x07, 0x03, 0x00, 0x00, 0x00, 0x00 }, // 0x11
   { 0x06, 0x00, 0x01, 0x01, 0x00, 0x3a, 0x03, 0x00 }, // 0x12
   { 0x06, 0x00, 0x01, 0x01, 0x00, 0x20, 0x03, 0x00 }, // 0x13
   { 0x06, 0x00, 0x01, 0x01, 0x00, 0x42, 0x03, 0x00 }, // 0x14
   { 0x06, 0x00, 0x01, 0x02, 0x00, 0x62, 0x03, 0x00 }, // 0x15

   // ML1.5 commands
   { 0x03, 0x00, 0x12, 0x03, 0x00, 0x00, 0x00, 0x00 }, // 0x16
   { 0x03, 0x00, 0x1c, 0x03, 0x00, 0x00, 0x00, 0x00 }, // 0x17
   { 0x06, 0x00, 0x01, 0x01, 0x01, 0xF8, 0x03, 0x00 }, // 0x18

   // ML4.1 additional parameters
   { 0x06, 0x00, 0x01, 0x01, 0x00, 0x22, 0x03, 0x00 }, // 0x19
   { 0x06, 0x00, 0x01, 0x01, 0x00, 0x29, 0x03, 0x00 }, // 0x1a
   { 0x06, 0x00, 0x01, 0x01, 0x00, 0x90, 0x03, 0x00 }, // 0x1b
   { 0x06, 0x00, 0x01, 0x01, 0x00, 0xb0, 0x03, 0x00 }, // 0x1c

   // ML4.1 devices test
   { 0x04, 0x00, 0x04, 0x0e, 0x03, 0x00, 0x00, 0x00 }, // 0x1d
   { 0x04, 0x00, 0x04, 0x1f, 0x03, 0x00, 0x00, 0x00 }, // 0x1e
   { 0x04, 0x00, 0x04, 0x21, 0x03, 0x00, 0x00, 0x00 }, // 0x1f

   // ML1.5 devices test
   /*
   { 0x04, 0x00, 0x04, 0x10, 0x03, 0x00, 0x00, 0x00 }, // 0x20
   { 0x04, 0x00, 0x04, 0x05, 0x03, 0x00, 0x00, 0x00 }, // 0x21
   { 0x04, 0x00, 0x04, 0x04, 0x03, 0x00, 0x00, 0x00 }, // 0x22
   { 0x04, 0x00, 0x04, 0x17, 0x03, 0x00, 0x00, 0x00 }, // 0x23
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

typedef struct queue {
   uint8_t items[256];
   uint8_t read_idx;
   uint8_t write_idx;
   uint8_t size;
} queue;

queue ecu_request_queue = {
   .items = { 0xFF },
   .size = 0,
   .read_idx = 0,
   .write_idx = 0
};

bool k_line_send_byte(const uint8_t send_byte, bool wait_echo_byte);
uint8_t k_line_read_byte(uint8_t* buffer, TickType_t read_timeout, bool send_echo);

bool ecu_request_queue_add(uint8_t ecu_request_id)
{
   if (ecu_request_queue.size >= 255)
      return false; // queue full

   ecu_request_queue.items[ecu_request_queue.write_idx++] = ecu_request_id;

   return true;
}

uint8_t ecu_request_queue_get()
{
   if (ecu_request_queue.size == 0)
      return 0xFF; // queue empty

   ecu_request_queue.size--;

   return ecu_request_queue.items[ecu_request_queue.read_idx++];
}

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
//           // bVar5 = (ecu_response[4]) & 3;
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
        return 0; // no byte received

    if (send_echo) // Send echo for all except last one
        k_line_send_byte(~(*last_byte_ptr), false);

    return 1;
}

bool ml41_send_request(uint8_t request_id)
{
    if (request_id > ECU_MAX_REQUEST) // invalid request id
        return false;

    uint8_t packet[8];

    memcpy(&packet, ECU_REQUESTS[request_id], 8);

    const uint8_t packet_length = packet[0];

    if (packet_length == 0)
        return false;

    packet[1] = ++ecu_connection.packet_id;

    // Send packet data
    for (uint8_t idx = 0; idx <= packet_length; idx++)
        if (!k_line_send_byte(packet[idx], idx < packet_length))
            return false;

    ecu_connection.last_sent_packet_id = request_id;

    return true;
}

void send_nop_packet()
{
    ml41_send_request(ECU_NO_DATA);
}

// will read packet and return it's data
uint8_t ml41_recv_packet(uint8_t* buffer)
{
    if (1 > k_line_read_byte(buffer, MS_TICKS(100), true))
        return 0;

    ESP_LOGI(TAG, "Incoming packet length: %d bytes", (int) buffer[0]);

    for (uint8_t idx = 1; idx <= buffer[0]; idx++)
    {
        // ESP_LOGI(TAG, "ml41_recv_packet: reading byte %d of %d. send_echo: %d. Buffer addr: 0x%#08x", idx, buffer[0], idx != buffer[0], (unsigned int) (buffer + idx));

        if (1 > k_line_read_byte(buffer + idx, MS_TICKS(100), idx != buffer[0]))
        {
            return 0;
        }
    }

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

    ml41_send_request(ECU_NO_DATA);

    // ECU BOSCH code
    if (1 > ml41_recv_packet(rx_buff))
    {
        ESP_LOGI(TAG, "BOSCH code read error");

        return false;
    }

    rx_buff[rx_buff[0]] = '\x0';

    ESP_LOGI(TAG, "BOSCH code: %s", rx_buff + 3);

    delay(15);

    // GM CODE + ALFA code
    ml41_send_request(ECU_NO_DATA);

    // ECU BOSCH code
    if (1 > ml41_recv_packet(rx_buff))
    {
        ESP_LOGI(TAG, "GM CODE code read error");

        return false;
    }

    rx_buff[rx_buff[0]] = '\x0';

    ESP_LOGI(TAG, "GM CODE code: %s", rx_buff + 3);

    /*while (true) {
        delay(15);

        ml41_send_request(ECU_NO_DATA);

        if (1 > ml41_recv_packet(rx_buff))
        {
            ESP_LOGI(TAG, "ml41_recv_packet failed");

            return false;
        }
    }*/

    delay(15);

    ml41_send_request(ECU_END_SESSION);

    if (1 > ml41_recv_packet(rx_buff))
    {
        ESP_LOGI(TAG, "ml41_recv_packet failed");

        return false;
    }

    return true;
}

void ml41_init_connection(void *)
{
    ESP_LOGI(TAG, "Start slow init");
    ml41_send_slow_init_wakeup();

    ESP_LOGI(TAG, "Start full speed UART");
    if (!ml41_start_full_speed())
    {
        ESP_LOGI(TAG, "ECU connection error: no sync received");

        delay(5000);

        return;
    }

    ESP_LOGI(TAG, "Sync received");

    if (!ml41_recv_keywords())
    {
        ESP_LOGI(TAG, "ECU connection error: no KW received");
        
        delay(5000);
        return;
    }

    // ESP_LOGI(TAG, "KW received");

    if (!ml41_read_ecu_init_data())
    {
        // ESP_LOGI(TAG, "ECU init data read error");
        
        delay(5000);
        return;
    }

    // Start ecu send/recv queue
    // xTaskCreate();
}

void app_main(void)
{
    delay(2000);
    gpio_reset_pin(UART_DEBUG_PIN);

    gpio_set_direction(UART_DEBUG_PIN, GPIO_MODE_OUTPUT);

    gpio_set_level(UART_DEBUG_PIN, GPIO_LEVEL_LOW);

    // ml41_start_full_speed();
    // xTaskCreate(ml41_start_full_speed, "ml41_start_full_speed", 1024 * 2, NULL, configMAX_PRIORITIES - 2, NULL);
   // start ECU connection task
   xTaskCreate(ml41_init_connection, "ml41_init_connection", 1024 * 2, NULL, configMAX_PRIORITIES - 2, NULL);
}
