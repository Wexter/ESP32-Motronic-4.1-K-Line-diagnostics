#ifndef KLINE_H
#define KLINE_H

#include "utils.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#define UART_NUMBER UART_NUM_2
#define UART_RX_BUF_SIZE 1024
#define UART_TXD_PIN GPIO_NUM_17
#define UART_RXD_PIN GPIO_NUM_16
#define UART_BAUD_RATE 8860
#define UART_DEBUG_PIN GPIO_NUM_5

#define K_LINE_ECHO_BYTE_RETRY_COUNT 3
#define K_LINE_INIT_ECU_DST_ADDRESS 0x10
#define K_LINE_PACKET_RECV_TIMEOUT_TICKS (MS_TICKS(100)) // 100ms

bool k_line_send_byte(const uint8_t send_byte, bool wait_echo_byte);

uint8_t k_line_read_byte(uint8_t* buffer, TickType_t read_timeout, bool send_echo);

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
    iVar1 = 0;
} */


#endif