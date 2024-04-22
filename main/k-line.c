#include "k-line.h"
#include "esp_log.h"

bool k_line_send_byte(const uint8_t send_byte, bool wait_echo_byte)
{
    for (uint8_t retry_count = 0; retry_count < K_LINE_ECHO_BYTE_RETRY_COUNT; retry_count++)
    {
        ESP_LOGI(__FUNCTION__, "Tx: %X", send_byte);

        uart_disable_rx_intr(UART_NUMBER);
        uart_write_bytes(UART_NUMBER, &send_byte, 1);
        uart_wait_tx_done(UART_NUMBER, MS_TICKS(10));
        uart_flush(UART_NUMBER);
        uart_enable_rx_intr(UART_NUMBER);

        if (!wait_echo_byte)
            return true;

        uint8_t echo_byte = 0x00;

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

    ESP_LOGI(__FUNCTION__, "Rx: %X", *last_byte_ptr);

    if (send_echo) // Send echo for all except last one
        k_line_send_byte(~(*last_byte_ptr), false);

    return 1;
}
