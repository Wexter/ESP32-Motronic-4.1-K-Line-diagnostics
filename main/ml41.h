#ifndef ML41_H
#define ML41_h

#include "freertos/FreeRTOS.h"
#include "stdint.h"
#include "stdbool.h"

#define ECU_NO_DATA              0x00
#define ECU_END_SESSION          0x01
#define ECU_READ_EPROM           0x02
#define ECU_GET_AFR              0x06
#define ECU_GET_VBAT             0x07
#define ECU_GET_INT_AIR_TEMP     0x08
#define ECU_GET_COOLANT_TEMP     0x09
#define ECU_ERASE_ERR_CODES      0x0A
#define ECU_GET_CO_POT           0x0C
#define ECU_GET_O2_SENSOR        0x0D
#define ECU_GET_IGN_TIME         0x0F
#define ECU_GET_ERROR_CODES      0x11
#define ECU_GET_RPM              0x12
#define ECU_GET_TPS              0x13
#define ECU_GET_ENG_LOAD         0x14
#define ECU_GET_INJ_TIME         0x15
#define ECU_GET_AC_DRV_SW        0x19
#define ECU_GET_O2_REG           0x1A
#define ECU_GET_FPUMP_RELAY      0x1B
#define ECU_GET_ADSORBER_VALVE   0x1C
#define ECU_ENABLE_INJECTOR      0x1D
#define ECU_ENABLE_ADSORB_VALVE  0x1E
#define ECU_ENABLE_IDLE_VALVE    0x1F
#define ECU_MAX_REQUEST          0x1F

#define ECU_ERROR_CODE_13         13 // O2 sensor slow signal change
#define ECU_ERROR_CODE_14         14 // Coolant temp sensor low voltage (Temp > 140 C)
#define ECU_ERROR_CODE_15         15 // Coolant temp sensor high voltage (Temp < -35.4 C & Intake temp < -20)
#define ECU_ERROR_CODE_44         44 // O2 sensor low voltage (<0.09V)
#define ECU_ERROR_CODE_45         45 // O2 sensor high voltage (>1.099V)
#define ECU_ERROR_CODE_48         48 // ECU low input voltage (<10V)
#define ECU_ERROR_CODE_49         49 // ECU high input voltage (>16V)
#define ECU_ERROR_CODE_51         51 // ECU mailfunction.
#define ECU_ERROR_CODE_55         55 // ECU mailfunction.
#define ECU_ERROR_CODE_65         65 // AFR CO pot low voltage
#define ECU_ERROR_CODE_66         66 // AFR CO pot high voltage
#define ECU_ERROR_CODE_67         67 // TPS idle position stuck
#define ECU_ERROR_CODE_69         69 // Intake air temp high voltage (Temp > 140 C)
#define ECU_ERROR_CODE_71         71 // Intake air temp low voltage (Temp < 35 C)
#define ECU_ERROR_CODE_72         72 // TPS kick-down position stuck
#define ECU_ERROR_CODE_73         73 // AFR low voltage
#define ECU_ERROR_CODE_74         74 // AFR high voltage
#define ECU_ERROR_CODE_75         75 // Torque control low voltage

#define ECU_ERROR_STATE_1              0x40 // 0100 0000
#define ECU_ERROR_STATE_ACTIVE         0x60 // 0110 0000
#define ECU_ERROR_STATE_PAST           0xA0 // 1010 0000
#define ECU_ERROR_STATE_ACTIVE_REPEAT  0xE0 // 1110 0000

typedef enum {
    Disconnected,
    Initialization,
    Connected
} ECUConnectionState_t;

typedef struct ml41_connection_t
{
    uint8_t packet_id;
    ECUConnectionState_t state;
    QueueHandle_t request_queue;
    uint8_t ecu_eprom_code[11];
    uint8_t ecu_bosch_code[11];
    uint8_t ecu_gm_code[11];
} ml41_connection_t;

static uint8_t const ECU_REQUESTS[][8] = {
   { 0x03, 0x00, 0x09, 0x03, },                     // 0x00
   { 0x03, 0x00, 0x06, 0x03, },                     // 0x01
   { 0x06, 0x00, 0x03, 0x0D, 0x00, 0x00, 0x03, },   // 0x02
   { 0x04, 0x00, 0x08, 0x00, 0x03, },               // 0x03
   { 0x04, 0x00, 0x08, 0x01, 0x03, },               // 0x04
   { 0x04, 0x00, 0x08, 0x02, 0x03, },               // 0x05
   { 0x04, 0x00, 0x08, 0x03, 0x03, },               // 0x06
   { 0x03, 0x00, 0x05, 0x03, },                     // 0x07
   { 0x04, 0x00, 0x08, 0x04, 0x03, },               // 0x08
   { 0x04, 0x00, 0x08, 0x05, 0x03, },               // 0x09
   { 0x04, 0x00, 0x08, 0x07, 0x03, },               // 0x0A
   { 0x03, 0x00, 0x07, 0x03, },                     // 0x0B
   { 0x06, 0x00, 0x01, 0x01, 0x00, 0x3a, 0x03, },   // 0x0C
   { 0x06, 0x00, 0x01, 0x01, 0x00, 0x20, 0x03, },   // 0x0D
   { 0x06, 0x00, 0x01, 0x01, 0x00, 0x42, 0x03, },   // 0x0E
   { 0x06, 0x00, 0x01, 0x02, 0x00, 0x62, 0x03, },   // 0x0F

   // ML1.5 commands
   { 0x03, 0x00, 0x12, 0x03, },                     // 0x10
   { 0x03, 0x00, 0x1c, 0x03, },                     // 0x11
   { 0x06, 0x00, 0x01, 0x01, 0x01, 0xF8, 0x03, },   // 0x12

   // ML4.1 additional parameters
   { 0x06, 0x00, 0x01, 0x01, 0x00, 0x22, 0x03, },   // 0x13
   { 0x06, 0x00, 0x01, 0x01, 0x00, 0x29, 0x03, },   // 0x14
   { 0x06, 0x00, 0x01, 0x01, 0x01, 0x90, 0x03, },   // 0x15
   { 0x06, 0x00, 0x01, 0x01, 0x01, 0xb0, 0x03, },   // 0x16

   // ML4.1 devices test
   { 0x04, 0x00, 0x04, 0x0e, 0x03, },               // 0x17
   { 0x04, 0x00, 0x04, 0x1f, 0x03, },               // 0x18
   { 0x04, 0x00, 0x04, 0x21, 0x03, },               // 0x19

   // ML1.5 devices test
   /*
   { 0x04, 0x00, 0x04, 0x10, 0x03, },               // 0x1A
   { 0x04, 0x00, 0x04, 0x05, 0x03, },               // 0x1B
   { 0x04, 0x00, 0x04, 0x04, 0x03, },               // 0x1C
   { 0x04, 0x00, 0x04, 0x17, 0x03, },               // 0x1D
   */
};

#ifdef CONFIG_ML41_EMULATE_ECU
static uint8_t ECU_RESPONSES[][24] = {
    { 0x03, 0x00, 0x09, 0x03, }, // NoData, // 0x00
    { 0x03, 0x00, 0x06, 0x03, }, // EndSession, // 0x01
    { 0x03, 0x00, 0x09, 0x03, }, // ReadEPROM, // 0x02
    { 0x05, 0x00, 0xFB, 0x00, 0x06, 0x03, }, // GetAFR, // 0x06
    { 0x05, 0x00, 0xFB, 0x00, 0xad, 0x03, }, // GetVBat, // 0x07
    { 0x05, 0x00, 0xFB, 0x00, 0xfd, 0x03, }, // GetIntakeAirTemp, // 0x08
    { 0x05, 0x00, 0xFB, 0x00, 0xfd, 0x03, }, // GetCoolantTemp, // 0x09
    { 0x03, 0x00, 0x09, 0x03, }, // EraseErrorCodes, // 0x0A
    { 0x05, 0x00, 0xFB, 0x00, 0x04, 0x03, }, // GetCOPot, // 0x0C
    { 0x05, 0x00, 0xFB, 0x00, 0x5c, 0x03, }, // GetO2Sensor, // 0x0D
    { 0x05, 0x00, 0xFB, 0x00, 0xfd, 0x03, }, // GetIgnitionTime, // 0x0F
    { 0x04, 0x00, 0xFC, 0x00, 0x03, }, // GetErrorCodes, // 0x11
    { 0x04, 0x00, 0xFE, 0x00, 0x03, }, // GetRPM, // 0x12
    // [3] & 3: 0 - MID, 1 - FULL, 2 - IDLE, 3 - ERR. [3] & 0x04 == 0: Manual transmission. [3] & 0x20 == 0: O2 sensor present
    { 0x04, 0x00, 0xFE, 0x10, 0x03, }, // GetTPS, // 0x13
    // [3] & 0x04 == 0: Fuel pump on, [3] & 0x20 == 0: engine torque control off
    { 0x04, 0x00, 0xFE, 0x00, 0x03, }, // GetEngineLoad, // 0x14
    { 0x05, 0x00, 0xFE, 0x00, 0x00, 0x03, }, // GetInjectionTime, // 0x15
    // [3] & 0x08 == 0: AC drive off, [3] & 0x10 == 0: AC switch off
    { 0x04, 0x00, 0xFE, 0x00, 0x03, }, // GetACParams, // 0x19
    // [3] & 0x20 == 0: open
    { 0x04, 0x00, 0xFE, 0x04, 0x03, }, // GetO2Params, // 0x1A
    { 0x04, 0x00, 0xFE, 0x07, 0x03, }, // GetFuelPumpParams, // 0x1B
    // [3] & 0x20 == 0: Valve open
    { 0x04, 0x00, 0xFE, 0xF7, 0x03, }, // GetAdsorberParams, // 0x1C
    { 0x03, 0x00, 0x09, 0x03, }, // EnableInjector, // 0x1D
    { 0x03, 0x00, 0x09, 0x03, }, // EnableAdsorberValve, // 0x1E
    { 0x03, 0x00, 0x09, 0x03, }, // EnableIdleValve, // 0x1F
};

static uint8_t ecu_eprom_code[]    = { 0x0D, 0x01, 0xF6, 0x31, 0x33, 0x31, 0x30, 0x30, 0x32, 0x31, 0x36, 0x32, 0x30, 0x03, };
static uint8_t ecu_bosch_code[]    = { 0x0D, 0x03, 0xF6, 0x33, 0x34, 0x34, 0x36, 0x35, 0x33, 0x37, 0x36, 0x32, 0x31, 0x03, };
static uint8_t ecu_gm_code[]       = { 0x0D, 0x05, 0xF6, 0x30, 0x33, 0x33, 0x34, 0x32, 0x33, 0x30, 0x39, 0x54, 0x46, 0x03, };

#endif

typedef enum {
    NoData, // 0x00
    EndSession, // 0x01
    ReadEPROM, // 0x02
    GetAFR, // 0x06
    GetVBat, // 0x07
    GetIntakeAirTemp, // 0x08
    GetCoolantTemp, // 0x09
    EraseErrorCodes, // 0x0A
    GetCOPot, // 0x0C
    GetO2Sensor, // 0x0D
    GetIgnitionTime, // 0x0F
    GetErrorCodes, // 0x11
    GetRPM, // 0x12
    GetTPS, // 0x13
    GetEngineLoad, // 0x14
    GetInjectionTime, // 0x15
    GetACParams, // 0x19
    GetO2Params, // 0x1A
    GetFuelPumpParams, // 0x1B
    GetAdsorberParams, // 0x1C
    EnableInjector, // 0x1D
    EnableAdsorberValve, // 0x1E
    EnableIdleValve, // 0x1F
    EcuRequestMax
} EcuRequestID;

/*
const short temperature_values_map[] = {
    197, 194, 191, 189, 186, 184, 181, 178,
    176, 173, 171, 168, 165, 163, 160, 158,
    155, 152, 150, 147, 145, 142, 139, 137,
    134, 132, 129, 126, 124, 121, 119, 116,
    114, 112, 111, 110, 109, 108, 107, 106,
    105, 104, 103, 102, 101, 100,  99,  98,
     97,  95,  94,  93,  92,  91,  90,  89,
     88,  87,  86,  85,  84,  83,  82,  81,
     80,  79,  78,  78,  77,  77,  76,  76,
     75,  74,  74,  73,  73,  72,  72,  71,
     71,  70,  69,  69,  68,  68,  67,  67,
     66,  65,  65,  64,  64,  63,  63,  62,
     62,  61,  61,  60,  60,  59,  59,  58, 
     58,  57,  57,  56,  56,  55,  55,  54, 
     54,  53,  53,  52,  52,  51,  51,  50, 
     50,  49,  49,  48,  48,  47,  47,  46, 
     46,  45,  44,  44,  43,  43,  42,  42, 
     41,  41,  40,  40,  39,  39,  38,  38, 
     37,  36,  36,  35,  35,  34,  34,  33, 
     33,  32,  32,  31,  31,  30,  30,  29, 
     29,  28,  28,  27,  27,  26,  26,  25, 
     25,  24,  24,  23,  23,  22,  22,  21, 
     21,  20,  20,  19,  19,  18,  18,  17, 
     17,  17,  16,  16,  15,  15,  14,  14, 
     13,  13,  12,  12,  11,  11,  10,  10, 
      9,   9,   8,   8,   7,   7,   6,   6,
      6,   5,   4,   3,   3,   2,   1,   0,
      0,   0,  -1,  -2,  -3,  -3,  -4,  -5, 
     -6,  -6,  -7,  -8,  -9, -10, -10, -11, 
    -12, -13, -14, -14, -15, -16, -17, -18, 
    -19, -20, -21, -22, -24, -25, -26, -28, 
    -29, -30, -32, -33, -34, -36, -37, -38, 
};

const short ignition_time_values_map[] = {
     10, 108, 107, 106, 105, 105, 104, 103, 
    102, 102, 101, 100,  99,  99,  98,  97, 
     96,  96,  95,  94,  93,  93,  92,  91, 
     90,  90,  89,  88,  87,  87,  86,  85, 
     84,  84,  83,  82,  81,  81,  80,  79, 
     78,  78,  77,  76,  75,  75,  74,  73, 
     72,  72,  71,  70,  69,  69,  68,  67, 
     66,  66,  65,  64,  63,  63,  62,  61, 
     60,  60,  59,  58,  57,  57,  56,  55, 
     54,  54,  53,  52,  51,  51,  50,  49, 
     48,  48,  47,  46,  45,  45,  44,  43, 
     42,  42,  41,  40,  39,  39,  38,  37, 
     36,  36,  35,  34,  33,  33,  32,  31, 
     30,  30,  29,  28,  27,  27,  26,  25, 
     24,  24,  23,  22,  21,  21,  20,  19, 
     18,  18,  17,  16,  15,  15,  14,  13, 
     12,  12,  11,  10,   9,   9,   8,   7, 
      6,   6,   5,   4,   3,   3,   2,   1, 
      0,   0,  -1,  -2,  -3,  -3,  -4,  -5, 
     -6,  -6,  -7,  -8,  -9,  -9, -10, -11, 
    -12, -12, -13, -14, -15, -15, -16, -17, 
    -18, -18, -19, -20, -21, -21, -22, -23, 
    -24, -24, -25, -26, -27, -27, -28, -29, 
    -30, -30, -31, -32, -33, -33, -34, -35, 
    -36, -36, -37, -38, -39, -39, -40, -41, 
    -42, -42, -43, -44, -45, -45, -46, -47, 
    -48, -48, -49, -50, -51, -51, -52, -53, 
    -54, -54, -55, -56, -57, -57, -58, -59, 
    -60, -60, -61, -62, -63, -63, -64, -65, 
    -66, -66, -67, -68, -69, -69, -70, -71, 
    -72, -72, -73, -74, -75, -75, -76, -77, 
    -78, -78, -79, -80, -81, -81, -82, -83,
};
*/

#ifdef CONFIG_ML41_EMULATE_ECU

uint8_t find_request_packet_idx(const uint8_t *packet);

#endif

bool ml41_add_request(EcuRequestID request);

bool ml41_send_request(EcuRequestID request);

uint8_t ml41_recv_packet(uint8_t *buffer);

void ml41_send_slow_init_wakeup();

bool ml41_start_full_speed();

void ml41_set_connection_state(ECUConnectionState_t state);

bool ml41_recv_keywords();

void ml41_dump_packet(uint8_t *packet);

bool ml41_read_ecu_init_data();

bool ml41_start_connection(ml41_connection_t *connection);

ml41_connection_t *ml41_create_connection();

void ml41_set_connection_state_change_cb(void *cb);

#endif