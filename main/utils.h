#ifndef UTILS_H
#define UTILS_H

#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"

#define MS_TICKS(ms) (ms / portTICK_PERIOD_MS)

#define delay(ms) vTaskDelay(MS_TICKS(ms));

#ifndef GPIO_LEVEL_LOW
#define GPIO_LEVEL_LOW 0
#endif

#ifndef GPIO_LEVEL_HIGH
#define GPIO_LEVEL_HIGH 1
#endif

#define LED_GPIO 2
#define LED_BLINK_DELAY 200

void enable_led(void);

void disable_led(void);

void blink_led(uint8_t count);

void configure_led(void);

#endif