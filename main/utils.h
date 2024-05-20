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

#if CONFIG_IDF_TARGET_ESP32C3
#define LED_ENABLED 0
#define LED_DISABLED 1

#define LED_GPIO 8
#else
#define LED_ENABLED 1
#define LED_DISABLED 0

#define LED_GPIO 2
#endif
#define LED_BLINK_DELAY 200

void enable_led(void);

void disable_led(void);

void blink_led(uint8_t count);

void configure_led(void);

/* void UpdateML41Parameters(uint8_t * ecu_response)
{
    // * ecu_response[0] - Request packet ID
    // * ecu_response[1] - first byte of ecu response
    // SendPacket ID
    switch(ecu_response[0]) {
    //    AFR & COP 
       case 6:
       case 0xc:
          float value = ecu_response[5] * 5 / 256.f;
          break;
    //    Battery voltage
       case 7:
          float value = 0.4 + ecu_response[5] / 15.0f;
          break;
    //    Coolant & Intake air temp
        // * Temperature calculation example
        // * ecu_response[5] = 0xEE
        // * SYMBOLS_ARRAY = 0x554910
        // * ecu_response[5] * 0x04 + SYMBOLS_ARRAY = 0x554cc8
        // * [0x554cc8] = 0xFFFFFFEF = -17
       case 8:
       case 9:
          result = TEMP_VALUES_ARRAY[ecu_response[5]];
          break;
    //    Lambda
       case 0xd:
          result = ecu_response[5] * 4.87;
          break;
    //    Ignition time
       case 0xf:
          local_c = ignition_time_values_map[ecu_response[4]];
          break;
    //    Engine RPM
        // * ecu_reponse[4] = 0xFF
        // * ecu_response[4] << 3 = 0x07f8
        // * result = (ecu_response[4] << 3) + ((ecu_response[4] << 3) * 4)
        // * result = ecu_response[4] * 0x28 = 10200 rpm
      
       case 0x12:
          local_14 = ecu_response[4] * 0x28;
          break;
    //    TPS
       case 0x13:
        //    * 0x12 - IDLE (ecu_response[4] & 3 == 2)
        //    * 0x10 - MID (ecu_response[4] & 3 == 0)
        //    * 0x11 - FULL (ecu_response[4] & 3 == 1)
        //    * ERR (ecu_response[4] & 3 == 3)
            
            // bVar5 = (ecu_response[4]) & 3; // bits 0 & 1 0000 0011
            // if (bVar5 == 0) { } // MID
            // else if (bVar5 == 1) { } // FULL
            // else if (bVar5 == 2) { } // IDLE
            // else if (bVar5 == 3) { } // ERR

        //   Transmission type
          if (((ecu_response[4]) & 4) == 0) { } // Manual 
          else { } // Auto

        //   O2 Sensor
          if (((ecu_response[4]) & 0x20) == 0) { } // Present
          else { } // Absent
          break;
    //    Engine Load
       case 0x14:
          local_c = (0.05f * ecu_response[4]);
          break;
    //    InjectionTime
       case 0x15:
          time = (0.3f * (((ecu_response[4]) * -0x100 + 0xffff) - ecu_response[5]) / 250.f);
          break;
    //    AC drive & switch
       case 0x19:
        //   edACDrive
          if (((ecu_response[4]) & 8) == 0) { } // Off
          else { } // On

        //   edACSwitch
          if (((ecu_response[4]) & 0x10) == 0) { } // Off
          else { } // On
    //    O2 Sensor regulation
       case 0x1a:
          if (hasO2Sensor) {
             if (((ecu_response[4]) & 0x20) == 0) { } // Open
             else { } // Close
          } else { } // Absent
    //    Fuel pump relay & engine torque control
       case 0x1b:
        //   Fuel pump relay state
          if (((ecu_response[4]) & 4) == 0) { } // On
          else { } // Off

        //   Engine torque control
          if (((ecu_response[4]) & 0x20) == 0) { } // On
          else { } // Off
    //    Adsorber valve state
       case 0x1c:
          if (((ecu_response[4]) & 0x20) == 0) { } // Open
          else { } // Close
    }
} */

/* void process_ecu_response(uint8_t * ecu_response)
{
    switch(ecu_response[3]) {
       case 0xee:
       case 0xf4:
       UpdateML15Parameters(ecu_response);
       break;
       case 0xf6:
       UpdateMainForm(ecu_response);
       break;
       case 0xfb:
       UpdateML41Parameters(ecu_response);
       break;
       case 0xfc:
       ListErrors(ecu_response);
       break;
       case 0xfd:
       pdateEPROMReadProgress(ecu_response,-1);
       break;
       case 0xfe:
       if ((ecu_response[0] == '\x0f') || ((ecu_response[0] - 0x12U) < 4)) {
          UpdateML41Parameters(ecu_response);
       } else if (ecu_response[0] == '\x18') {
          UpdateFuelPumpStatus(ecu_response);
       } else if ((ecu_response[0]  - 0x19U) < 4)
          UpdateML41Parameters(ecu_response);
    }

    // clear ecu response
    for (size_t i = 1; i < 24; i++)
       ecu_response[i] = 0;
} */

#endif