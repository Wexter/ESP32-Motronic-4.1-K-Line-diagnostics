#include "utils.h"

void enable_led(void)
{
    gpio_set_level(LED_GPIO, 1);
}

void disable_led(void)
{
    gpio_set_level(LED_GPIO, 0);
}

void blink_led(uint8_t count)
{
    disable_led();

    do {
        delay(LED_BLINK_DELAY);
        enable_led();
        delay(LED_BLINK_DELAY);
        disable_led();
    } while (--count > 0);
}

void configure_led(void)
{
    gpio_reset_pin(LED_GPIO);

    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
}