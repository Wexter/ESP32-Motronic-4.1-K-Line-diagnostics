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

#define KWP2000_ECHO_BYTE_TIMEOUT_MS 100
#define KWP2000_ECHO_BYTE_RETRY_COUNT 3
#define KWP2000_INIT_ECU_DST_ADDRESS 0x10

#define K_LINE_RX_BUFFER_SIZE 2

#define delay(ms) vTaskDelay(ms / portTICK_PERIOD_MS);

const char nop_data = 0x09;

uint8_t connection_packet_id = 0;

bool kwp2000_send_byte(const char send_byte, bool wait_for_ack)
{
    uart_write_bytes(UART_NUMBER, &send_byte, 1);

    if (!wait_for_ack)
        return true;

    char received_bytes[K_LINE_RX_BUFFER_SIZE] = { 0x00, 0x00 };

    char bytes_read = 0;

    for (char retry_count = 0; retry_count < KWP2000_ECHO_BYTE_RETRY_COUNT; retry_count++)
    {
        bytes_read = uart_read_bytes(UART_NUMBER, &received_bytes, K_LINE_RX_BUFFER_SIZE, KWP2000_ECHO_BYTE_TIMEOUT_MS / portTICK_PERIOD_MS);

        if (bytes_read > 0 && received_bytes[1] + send_byte == 0xFF)
            return true;
    }

    return false;
}

int kwp2000_read_bytes(char* bytes, int bytes_count, bool send_ack)
{
    uint8_t bytes_read = 0,
        last_byte = 0;

    for (; bytes_read < bytes_count; bytes_read++)
    {
        if(0 > uart_read_bytes(UART_NUMBER, &last_byte, bytes_count, KWP2000_ECHO_BYTE_TIMEOUT_MS / portTICK_PERIOD_MS))
        {
            bytes[bytes_read] = last_byte;

            kwp2000_send_byte(0xFF - last_byte, false);
        }
    }

    return bytes_read;
}

bool kwp2000_send_packet(const char * data, uint8_t data_length)
{
    const uint8_t packet_length = data_length + 2; // packet id + data + packet end (0x03)

    // Send packet length
    if (!kwp2000_send_byte(packet_length, true))
        return false;

    // Send packet sequence number
    if (!kwp2000_send_byte(connection_packet_id++, true))
        return false;

    // Send packet data
    for (uint8_t i = 0; i < data_length; i++)
        if (!kwp2000_send_byte(data[i], true))
            return false;

    // Send packet end mark 0x03 not wait ack
    kwp2000_send_byte(0x03, false);

    return true;
}

inline void send_nop_packet()
{
    kwp2000_send_packet(&nop_data, 1);
}

// will read packet and return it's data
int recv_packet(char * rx_buffer)
{
    // connection_packet_id++
    return 0;
}

inline void kwp2000_()
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
    uart_driver_install(UART_NUMBER, RX_BUF_SIZE, 0, 0, NULL, 0);
    uart_param_config(UART_NUMBER, &uart_config);
    uart_set_pin(UART_NUMBER, UART_TXD_PIN, UART_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

inline void kwp2000_send_wakeup_slow()
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

    kwp2000_send_byte(KWP2000_INIT_ECU_DST_ADDRESS, false);
}

void ecu_connect_kwp2000()
{
    kwp2000_send_wakeup_slow();

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
