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

QueueHandle_t ml41_request_queue;

void data_recv_callback(esp_ble_gatts_cb_param_t *p_data)
{
    if (p_data->write.len > 1) {
        spp_data_t *spp_message = (spp_data_t *) malloc(sizeof(spp_data_t));

        spp_message->size = p_data->write.len;
        spp_message->data = (uint8_t *) malloc(p_data->write.len);

        memcpy(spp_message->data, p_data->write.value, p_data->write.len);

        xQueueSend(ml41_request_queue, &spp_message, 10 / portTICK_PERIOD_MS);
        return;
    }

    switch (p_data->write.value[0])
    {
        case 0x01: // start ecu connection
            // start ECU connection task
            ml41_init_connection(&ml41_request_queue);
            // xTaskCreate(ml41_init_connection, "ml41_init_connection", 16384, NULL, configMAX_PRIORITIES - 2, NULL);
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

void app_main(void)
{
    ml41_request_queue = xQueueCreate(32, sizeof(void *));

    configure_led();

    ml41_init();

    ble_set_spp_data_recv_callback((void *) &data_recv_callback);

    ble_spp_init();
}
