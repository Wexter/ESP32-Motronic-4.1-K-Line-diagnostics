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
#define UART_RX_BUF_SIZE 256
#define UART_TXD_PIN GPIO_NUM_1
#define UART_RXD_PIN GPIO_NUM_3
#define UART_BAUD_RATE 8860

#define KWP2000_ECHO_BYTE_TIMEOUT_TICKS (100 / portTICK_PERIOD_MS) // 100ms
#define KWP2000_ECHO_BYTE_RETRY_COUNT 3
#define KWP2000_INIT_ECU_DST_ADDRESS 0x10
#define KWP2000_PACKET_RECV_TIMEOUT_TICKS (100 / portTICK_PERIOD_MS) // 100ms

#define delay(ms) vTaskDelay(ms / portTICK_PERIOD_MS);

const uint8_t no_data[] = { 0x03, 0x00, 0x09, 0x00, };
const uint8_t request_throttle_pos[] = { 0x06, 0x00, 0x01, 0x01, 0x00, 0x20, 0x03, };
const uint8_t request_engine_rpm[] = { 0x06, 0x00, 0x01, 0x01, 0x00, 0x3a, 0x03, };
const uint8_t request_engine_load[] = { 0x06, 0x00, 0x01, 0x01, 0x00, 0x42, 0x03, };
const uint8_t request_injection_time[] = { 0x06, 0x00, 0x01, 0x02, 0x00, 0x62, 0x03, };
const uint8_t request_airflow_meter_value[] = { 0x04, 0x00, 0x08, 0x00, 0x03, };
const uint8_t request_battery_voltage[] = { 0x04, 0x00, 0x08, 0x01, 0x03, };
const uint8_t request_intake_air_temp[] = { 0x04, 0x00, 0x08, 0x02, 0x03, };
const uint8_t request_coolant_temp[] = { 0x04, 0x00, 0x08, 0x03, 0x03, };
const uint8_t request_co_pot_value[] = { 0x04, 0x00, 0x08, 0x04, 0x03, };
const uint8_t request_o2_sensor_value[] = { 0x04, 0x00, 0x08, 0x05, 0x03, };
const uint8_t request_ignition_time[] = { 0x04, 0x00, 0x08, 0x07, 0x03, };

uint8_t kwp2000_session_packet_id = 0;

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

bool kwp2000_send_packet(const uint8_t* packet)
{
    const uint8_t packet_length = packet[0];

    // Send packet length
    if (!k_line_send_byte(packet_length, true))
        return false;

    // Send packet sequence number
    if (!k_line_send_byte(kwp2000_session_packet_id++, true))
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

void send_nop_packet()
{
    kwp2000_send_packet(no_data);
}

// will read packet and return it's data
int kwp2000_recv_packet(uint8_t * rx_buffer)
{

    kwp2000_session_packet_id++;

    return 0;
}

void kwp2000_send_slow_init_wakeup()
{
    // Bit-bang 5baud init byte
    gpio_reset_pin(UART_TXD_PIN);

    gpio_set_direction(UART_TXD_PIN, GPIO_MODE_OUTPUT);

    gpio_set_level(UART_TXD_PIN, GPIO_LEVEL_LOW);

    vTaskDelay(1200 / portTICK_PERIOD_MS);

    gpio_set_level(UART_TXD_PIN, GPIO_LEVEL_HIGH);

    vTaskDelay(200 / portTICK_PERIOD_MS);

    gpio_set_level(UART_TXD_PIN, GPIO_LEVEL_LOW);

    vTaskDelay(600 / portTICK_PERIOD_MS);

    gpio_set_level(UART_TXD_PIN, GPIO_LEVEL_HIGH);
}

void kwp2000_start_full_speed()
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
}

bool kwp2000_wait_handshake()
{
    uint8_t rx_buffer[2];

    if (2 > k_line_read_bytes(rx_buffer, 2, true, KWP2000_PACKET_RECV_TIMEOUT_TICKS))
        return false;

    return true;
}

void kwp2000_read_ecu_init_data()
{

}

void ecu_connect_kwp2000(void *)
{
    kwp2000_session_packet_id = 0;

    kwp2000_send_slow_init_wakeup();

    kwp2000_start_full_speed();

    kwp2000_wait_handshake();

    kwp2000_read_ecu_init_data();

    // Start ecu send/recv queue
    // xTaskCreate();
}

void init(void)
{
    // wait 5 seconds
    delay(5000);

    // start ECU connection task
    xTaskCreate(ecu_connect_kwp2000, "ecu_connect_kwp2000", 1024 * 2, NULL, configMAX_PRIORITIES - 2, NULL);
}

void app_main(void)
{
    init();
}
