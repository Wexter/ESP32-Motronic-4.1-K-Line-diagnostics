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

void data_recv_callback(esp_ble_gatts_cb_param_t *p_data)
{
    switch (p_data->write.value[0])
    {
        case 0x01: // start ecu connection
            // start ECU connection task
            ml41_start_connection(ecu_connection);
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

void app_main(void)
{
    configure_led();

    ecu_connection = ml41_create_connection();

    ble_set_spp_data_recv_callback((void *) &data_recv_callback);

    ble_spp_init();
}
