#ifndef KLINE_H
#define KLINE_H

#include "utils.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#define UART_BAUD_RATE 8860
#define UART_RX_BUF_SIZE 1024

#if CONFIG_IDF_TARGET_ESP32C3
#define UART_NUMBER UART_NUM_1
#define UART_TXD_PIN GPIO_NUM_4
#define UART_RXD_PIN GPIO_NUM_5
#else
#define UART_NUMBER UART_NUM_2
#define UART_TXD_PIN GPIO_NUM_17
#define UART_RXD_PIN GPIO_NUM_16
#define UART_DEBUG_PIN GPIO_NUM_5
#endif

#define K_LINE_ECHO_BYTE_RETRY_COUNT 3
#define K_LINE_INIT_ECU_DST_ADDRESS 0x10
#define K_LINE_PACKET_RECV_TIMEOUT_TICKS (MS_TICKS(100)) // 100ms

bool k_line_send_byte(const uint8_t send_byte, bool wait_echo_byte);

uint8_t k_line_read_byte(uint8_t* buffer, TickType_t read_timeout, bool send_echo);

#endif