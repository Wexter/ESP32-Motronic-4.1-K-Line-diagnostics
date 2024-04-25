/**
 * For ECU connection & packets details see https://gist.github.com/Wexter/e82a5607d0599efa7484552379848761
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "string.h"
#include "ble.h"
#include "ml41.h"
#include "utils.h"

ml41_connection_t *ecu_connection = NULL;

static void start_ecu_connection(void *);

static void send_connection_state(ECUConnectionState_t state)
{
    uint8_t spp_data[] = { 0x01, state };

    spp_message_t message = {
        .size = 2,
        .data = (uint8_t *) &spp_data
    };

    ble_send_notification(&message);
}

void ble_data_recv_callback(esp_ble_gatts_cb_param_t *p_data)
{
    switch (p_data->write.value[0])
    {
        case 0x00:
            send_connection_state(ecu_connection->state);
            break;

        case 0x01: // start ecu connection
            if (ecu_connection->state != Disconnected) break;

            // start ECU connection task
            xTaskCreate(start_ecu_connection, "start_ecu_connection", 16384, NULL, configMAX_PRIORITIES - 2, NULL);
            break;

        case 0x02:
            if (p_data->write.len < 2) break;

            ml41_add_request(p_data->write.value[1]);
            break;

        case 0x03:
            break;

        default:
            break;
    }
}

void start_ecu_connection(void *)
{
    if (ecu_connection->state != Disconnected) goto ecu_connection_task_end;

    ecu_connection->packet_id = 0;

    if (!ml41_start_connection(ecu_connection)) goto ecu_connection_task_end;

    ml41_set_connection_state(Connected);

    uint8_t ml41_recv_buff[32] = { 0 };

    uint8_t request_idx;

    while (true)
    {
        if (xQueueReceive(ecu_connection->request_queue, &request_idx, MS_TICKS(100)))
        {
            if (!ml41_send_request(EndSession))
            {
                ESP_LOGE(__FUNCTION__, "Failed to make request %d", EndSession);

                break;
            }

            if (request_idx == EndSession) // don't wait response
                break;

            if (!ml41_recv_packet(ml41_recv_buff))
            {
                ESP_LOGE(__FUNCTION__, "Failed to recv packet");

                break;
            }
        }
        else
        {
            if (!ml41_send_request(NoData))
            {
                ESP_LOGE(__FUNCTION__, "Failed to make request %d", NoData);
                break;
            }

            if (!ml41_recv_packet(ml41_recv_buff))
            {
                ESP_LOGE(__FUNCTION__, "Failed to recv packet");
                break;
            }
        }
    }

ecu_connection_task_end:

    ml41_set_connection_state(Disconnected);

    vTaskDelete(NULL);
}

void ecu_connection_state_cb(ECUConnectionState_t state)
{
    send_connection_state(state);
}

void app_main(void)
{
    configure_led();

    ecu_connection = ml41_create_connection();

    ble_set_spp_data_recv_callback((void *) &ble_data_recv_callback);

    ml41_set_connection_state_change_cb((void *) &ecu_connection_state_cb);

    ble_spp_init();
}
