#include "stdbool.h"
#include "string.h"
#include "stdio.h"
#include "esp_log.h"

#include "ml41.h"
#include "ble.h"
#include "k-line.h"

static uint8_t spp_end_session_data[5] = { 0x02, 0x03, 0x00, 0x06, 0x03, };
static uint8_t spp_no_data_data[5] = { 0x04, 0x03, 0x00, 0x09, 0x03, };

static spp_data_t spp_end_session = {
    .size = 5,
    .data = spp_end_session_data
};

static spp_data_t no_data_notify = {
    .size = 5,
    .data = spp_no_data_data
};

bool ml41_make_request(uint8_t* rx_buff, uint8_t request_id)
{
    if (request_id > ECU_MAX_REQUEST) // invalid request id
        return false;

    uint8_t packet[8];

    memcpy(&packet, ECU_REQUESTS[request_id], 8);

    const uint8_t packet_length = packet[0];

    if (packet_length == 0)
        return false;

    // packet[1] = ++ecu_connection.packet_id;

#ifdef ML41_DUMP_PACKETS
    ESP_LOGI(__FUNCTION__, "Sending packet length: %d bytes", packet[0]);
    ml41_dump_packet(packet);
#endif

    for (uint8_t idx = 0; idx <= packet_length; idx++)
        if (!k_line_send_byte(packet[idx], idx < packet_length))
            return false;

    // ecu_connection.last_sent_packet_id = request_id;

    return true;
}

uint8_t ml41_recv_packet(uint8_t* buffer)
{
    if (1 > k_line_read_byte(buffer, MS_TICKS(100), true))
        return 0;

    ESP_LOGI(__FUNCTION__, "Incoming packet length: %d bytes", (int) buffer[0]);

    for (uint8_t idx = 1; idx <= buffer[0]; idx++)
    {
        if (1 > k_line_read_byte(buffer + idx, MS_TICKS(100), idx != buffer[0]))
        {
            return 0;
        }
    }

#ifdef ML41_DUMP_PACKETS
    ml41_dump_packet(buffer);
#endif

    // ecu_connection.packet_id++;

    return buffer[0];
}

void ml41_send_slow_init_wakeup()
{
    uart_disable_rx_intr(UART_NUMBER);
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

    uart_enable_rx_intr(UART_NUMBER);
}

bool ml41_start_full_speed()
{
    uart_set_pin(UART_NUMBER, UART_TXD_PIN, UART_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uint8_t rx_byte;

    int bytes_read = uart_read_bytes(UART_NUMBER, &rx_byte, 1, MS_TICKS(1000));

    ESP_LOGI(__FUNCTION__, "Sync: %X", rx_byte);

    return (0 < bytes_read && rx_byte == 0x55);
}

bool ml41_recv_keywords()
{
    uint8_t rx_byte;

    int len = k_line_read_byte(&rx_byte, MS_TICKS(100), true);

    ESP_LOGI(__FUNCTION__, "KW1: %X", rx_byte);

    if (1 > len)
        return false;

    rx_byte = 0xFF;

    len = k_line_read_byte(&rx_byte, MS_TICKS(100), true);

    ESP_LOGI(__FUNCTION__, "KW2: %X", rx_byte);

    rx_byte = 0xFF;

    len = k_line_read_byte(&rx_byte, MS_TICKS(100), true);

    ESP_LOGI(__FUNCTION__, "KW2: %X", rx_byte);

    rx_byte = 0xFF;

    len = k_line_read_byte(&rx_byte, MS_TICKS(100), true);

    ESP_LOGI(__FUNCTION__, "KW2: %X", rx_byte);

    rx_byte = 0xFF;

    len = k_line_read_byte(&rx_byte, MS_TICKS(100), true);

    ESP_LOGI(__FUNCTION__, "KW2: %X", rx_byte);

    if (1 > len)
        return false;

    return true;
}

void ml41_dump_packet(uint8_t* packet)
{
    char packet_str[64] = { 0 };

    uint8_t packet_length = packet[0];

    for (uint8_t idx = 0; idx <= packet_length; idx++)
        sprintf(packet_str + idx * 3, "%02X ", packet[idx]);
        // ESP_LOGI(__FUNCTION__, "%02X", packet[idx]);

    ESP_LOGI(__FUNCTION__, "Packet: %s", packet_str);
}

bool ml41_read_ecu_init_data()
{
    uint8_t rx_buff[24] = { 0 };

    // ECU EPROM code
    if (1 > ml41_recv_packet(rx_buff))
    {
        ESP_LOGI(__FUNCTION__, "EPROM code read error");

        return false;
    }

    rx_buff[rx_buff[0]] = '\x0';

    ESP_LOGI(__FUNCTION__, "EPROM code: %s", rx_buff + 3);

    delay(15);

    if (1 > ml41_make_request(rx_buff, ECU_NO_DATA))
    {
        // ESP_LOGI(__FUNCTION__, "BOSCH code read error");

        return false;
    }

    // ECU BOSCH code
    if (1 > ml41_recv_packet(rx_buff))
    {
        ESP_LOGI(__FUNCTION__, "BOSCH code read error");

        return false;
    }

    rx_buff[rx_buff[0]] = '\x0';

    ESP_LOGI(__FUNCTION__, "BOSCH code: %s", rx_buff + 3);

    delay(15);

    // GM CODE + ALFA code
    ml41_make_request(rx_buff, ECU_NO_DATA);

    // ECU BOSCH code
    if (1 > ml41_recv_packet(rx_buff))
    {
        ESP_LOGI(__FUNCTION__, "GM CODE code read error");

        return false;
    }

    rx_buff[rx_buff[0]] = '\x0';

    ESP_LOGI(__FUNCTION__, "GM CODE code: %s", rx_buff + 3);

    /* while (true) {
        delay(50);

        ml41_make_request(rx_buff, ECU_NO_DATA);

        if (1 > ml41_recv_packet(rx_buff))
        {
            ESP_LOGI(__FUNCTION__, "ml41_recv_packet failed");

            return false;
        }
        delay(50);
        ml41_make_request(rx_buff, ECU_GET_AFR);
        if (1 > ml41_recv_packet(rx_buff))
        {
            ESP_LOGI(__FUNCTION__, "ml41_recv_packet failed");

            return false;
        }
        delay(50);
        ml41_make_request(rx_buff, ECU_GET_VBAT);
        if (1 > ml41_recv_packet(rx_buff))
        {
            ESP_LOGI(__FUNCTION__, "ml41_recv_packet failed");

            return false;
        }
        delay(50);
        ml41_make_request(rx_buff, ECU_GET_INT_AIR_TEMP);
        if (1 > ml41_recv_packet(rx_buff))
        {
            ESP_LOGI(__FUNCTION__, "ml41_recv_packet failed");

            return false;
        }
        delay(50);
        ml41_make_request(rx_buff, ECU_GET_COOLANT_TEMP);
        if (1 > ml41_recv_packet(rx_buff))
        {
            ESP_LOGI(__FUNCTION__, "ml41_recv_packet failed");

            return false;
        }
        delay(50);
        ml41_make_request(rx_buff, ECU_ERASE_ERR_CODES);
        if (1 > ml41_recv_packet(rx_buff))
        {
            ESP_LOGI(__FUNCTION__, "ml41_recv_packet failed");

            return false;
        }
        delay(50);
        ml41_make_request(rx_buff, ECU_GET_CO_POT);
        if (1 > ml41_recv_packet(rx_buff))
        {
            ESP_LOGI(__FUNCTION__, "ml41_recv_packet failed");

            return false;
        }
        delay(50);
        ml41_make_request(rx_buff, ECU_GET_O2_SENSOR);
        if (1 > ml41_recv_packet(rx_buff))
        {
            ESP_LOGI(__FUNCTION__, "ml41_recv_packet failed");

            return false;
        }
        delay(50);
        ml41_make_request(rx_buff, ECU_GET_IGN_TIME);
        if (1 > ml41_recv_packet(rx_buff))
        {
            ESP_LOGI(__FUNCTION__, "ml41_recv_packet failed");

            return false;
        }
        // delay(50);
        // ml41_make_request(rx_buff, ECU_GET_ERROR_CODES);
        //         if (1 > ml41_recv_packet(rx_buff))
        // {
        //     ESP_LOGI(__FUNCTION__, "ml41_recv_packet failed");

        //     return false;
        // }
        delay(50);
        ml41_make_request(rx_buff, ECU_GET_RPM);
        if (1 > ml41_recv_packet(rx_buff))
        {
            ESP_LOGI(__FUNCTION__, "ml41_recv_packet failed");

            return false;
        }
        delay(50);
        ml41_make_request(rx_buff, ECU_GET_TPS);
        if (1 > ml41_recv_packet(rx_buff))
        {
            ESP_LOGI(__FUNCTION__, "ml41_recv_packet failed");

            return false;
        }
        delay(50);
        ml41_make_request(rx_buff, ECU_GET_ENG_LOAD);
        if (1 > ml41_recv_packet(rx_buff))
        {
            ESP_LOGI(__FUNCTION__, "ml41_recv_packet failed");

            return false;
        }
        delay(50);
        ml41_make_request(rx_buff, ECU_GET_INJ_TIME);
        if (1 > ml41_recv_packet(rx_buff))
        {
            ESP_LOGI(__FUNCTION__, "ml41_recv_packet failed");

            return false;
        }
        delay(50);
        ml41_make_request(rx_buff, ECU_GET_AC_DRV_SW);
        if (1 > ml41_recv_packet(rx_buff))
        {
            ESP_LOGI(__FUNCTION__, "ml41_recv_packet failed");

            return false;
        }
        delay(50);
        ml41_make_request(rx_buff, ECU_GET_O2_REG);
        if (1 > ml41_recv_packet(rx_buff))
        {
            ESP_LOGI(__FUNCTION__, "ml41_recv_packet failed");

            return false;
        }
        delay(50);
        ml41_make_request(rx_buff, ECU_GET_FPUMP_RELAY);
        if (1 > ml41_recv_packet(rx_buff))
        {
            ESP_LOGI(__FUNCTION__, "ml41_recv_packet failed");

            return false;
        }
        delay(50);
        ml41_make_request(rx_buff, ECU_GET_ADSORBER_VALVE);
        if (1 > ml41_recv_packet(rx_buff))
        {
            ESP_LOGI(__FUNCTION__, "ml41_recv_packet failed");

            return false;
        }
    } */

    // delay(15);

    // ml41_make_request(rx_buff, ECU_END_SESSION);

    // if (1 > ml41_recv_packet(rx_buff))
    // {
    //     ESP_LOGI(__FUNCTION__, "ml41_recv_packet failed");

    //     return false;
    // }

    return true;
}

static void ml41_process_ecu_requests(void *params)
{
    spp_data_t *spp_message;

    QueueHandle_t *ml41_request_queue = (QueueHandle_t *) params;

    while (true)
    {
        if (xQueueReceive(*ml41_request_queue, &spp_message, 50 / portTICK_PERIOD_MS)) {
            ESP_LOGI(__FUNCTION__, "message size: %d message ptr: %p data ptr: %p \r\n", spp_message->size, spp_message, spp_message->data);

            ESP_LOG_BUFFER_HEXDUMP(__FUNCTION__, spp_message->data, spp_message->size, ESP_LOG_INFO);

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
    
    vTaskDelete(NULL);
}

void ml41_init_connection(QueueHandle_t *request_queue)
{
    ESP_LOGI(__FUNCTION__, "UART configured");

    ml41_send_slow_init_wakeup();

    blink_led(1);

    delay(200);

    if (!ml41_start_full_speed())
    {
        ESP_LOGI(__FUNCTION__, "ECU connection error: no sync received");

        delay(5000);

        return;
    }

    delay(50);

    if (!ml41_recv_keywords())
    {
        ESP_LOGI(__FUNCTION__, "ECU connection error: no KW received");
        
        delay(5000);
        return;
    }

    delay(50);

    if (!ml41_read_ecu_init_data())
    {
        delay(5000);
        return;
    }

    xTaskCreate(ml41_process_ecu_requests, "ml41_process_ecu_requests", 16384, NULL, configMAX_PRIORITIES - 2, NULL);
}

void ml41_init()
{
    const uart_config_t uart_config = {
       .baud_rate = UART_BAUD_RATE,
       .data_bits = UART_DATA_8_BITS,
       .parity = UART_PARITY_DISABLE,
       .stop_bits = UART_STOP_BITS_1,
       .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
       .source_clk = UART_SCLK_DEFAULT,
    };

    if (!uart_is_driver_installed(UART_NUMBER))
        uart_driver_install(UART_NUMBER, UART_RX_BUF_SIZE * 2, 0, 0, NULL, 0);

    uart_param_config(UART_NUMBER, &uart_config);
    uart_set_rx_full_threshold(UART_NUMBER, 1);
    // uart_flush_input(UART_NUMBER);
}