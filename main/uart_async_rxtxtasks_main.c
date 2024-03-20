#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

#define UART_NUMBER UART_NUM_0
#define UART_RX_BUF_SIZE 256
#define UART_TXD_PIN GPIO_NUM_1
#define UART_RXD_PIN GPIO_NUM_3
#define UART_BAUD_RATE 8860

#define KWP2000_ECHO_BYTE_TIMEOUT_TICKS (100 / portTICK_PERIOD_MS)
#define KWP2000_ECHO_BYTE_RETRY_COUNT 3
#define KWP2000_INIT_ECU_DST_ADDRESS 0x10

#define delay(ms) vTaskDelay(ms / portTICK_PERIOD_MS);

const char nop_data[] = { 0x03, 0x00, 0x09, 0x00 };

uint8_t kwp2000_session_packet_id = 0;

bool k_line_send_byte(const char send_byte, bool wait_for_ack)
{
    uart_disable_rx_intr(UART_NUMBER);
    uart_write_bytes(UART_NUMBER, &send_byte, 1);
    uart_enable_rx_intr(UART_NUMBER);

    if (!wait_for_ack)
        return true;

    char received_byte = 0x00;

    char bytes_read = 0;

    for (char retry_count = 0; retry_count < KWP2000_ECHO_BYTE_RETRY_COUNT; retry_count++)
    {
        bytes_read = uart_read_bytes(UART_NUMBER, &received_byte, 1, KWP2000_ECHO_BYTE_TIMEOUT_TICKS);

        if (bytes_read > 0 && received_byte + send_byte == 0xFF)
            return true;
    }

    return false;
}

int k_line_read_bytes(char* bytes, int bytes_count, bool send_ack)
{
    uint8_t bytes_read = 0,
        last_byte = 0;

    for (; bytes_read < bytes_count; bytes_read++)
    {
        if (0 < uart_read_bytes(UART_NUMBER, &last_byte, bytes_count, KWP2000_ECHO_BYTE_TIMEOUT_TICKS)) 
        {
            bytes[bytes_read] = last_byte;

            k_line_send_byte(0xFF - last_byte, false);
        }
    }

    return bytes_read;
}

bool kwp2000_send_packet(const char * packet)
{
    const uint8_t packet_length = packet[0];

    // Send packet length
    if (!k_line_send_byte(packet_length, true))
        return false;

    // Send packet sequence number
    if (!k_line_send_byte(kwp2000_session_packet_id++, true))
        return false;

    // Send packet data
    for (uint8_t i = 2; i < packet_length; i++)
        if (!k_line_send_byte(packet[i], true))
            return false;

    // Send packet end mark 0x03 not wait ack
    k_line_send_byte(0x03, false);

    return true;
}

inline void send_nop_packet()
{
    kwp2000_send_packet(&nop_data);
}

// will read packet and return it's data
int kwp2000_recv_packet(char * rx_buffer)
{
    kwp2000_session_packet_id++;

    return 0;
}

inline void kwp2000_start_full_speed()
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

inline void kwp2000_send_slow_init_wakeup()
{
    const uart_config_t uart_config = {
        .baud_rate = 5,
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

    k_line_send_byte(KWP2000_INIT_ECU_DST_ADDRESS, false);
}

void ecu_connect_kwp2000()
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
