#include "main.h"

#include <Arduino.h>
// #include <SimpleFOC.h>
// #include "pin_defs.h"
// #include <pin_defs.h>
#ifdef __cplusplus
extern "C" {
#endif

#define MAX_PACKET_SIZE 23

#define MBS_CMD_SET_ZERO           0x30
#define MBS_CMD_GET_POS            0x31
#define MBS_CMD_GET_POS_STATUS     0x64
#define MBS_CMD_GET_POS_RPM        0x73
#define MBS_CMD_GET_POS_TEMP       0x74

typedef union
{
    uint8_t RAW[MAX_PACKET_SIZE];
} pack;

// Структура для возврата результата разбора
typedef struct {
    uint8_t is_error;
    uint8_t is_warning;
    char    msg[64];      // Строка для вывода в консоль
} ENC_Diag_t;

typedef struct __attribute__((packed)) {
    // Порядок битов b0 -> b7
    uint8_t over_speed     : 1; // b0
    uint8_t temp_out       : 1; // b1
    uint8_t mag_weak       : 1; // b2
    uint8_t mag_large      : 1; // b3
    uint8_t battery_low    : 1; // b4
    uint8_t battery_lost   : 1; // b5
    uint8_t warning        : 1; // b6
    uint8_t error          : 1; // b7
} ENC_Status_t;

ENC_Diag_t Decode_Encoder_Status(uint8_t raw_byte);
uint8_t Get_CRC_position(uint8_t command);
uint8_t Get_Packet_CRC(uint8_t command, uint8_t *raw);
uint8_t CRC_is_OK(uint8_t command, uint8_t *data);
uint8_t ML_ENC_CalcCRC(volatile uint8_t *buffer, uint8_t length);
// void RS485_tx(uint8_t data);

static inline uint16_t Get_Multiturn(volatile uint8_t *raw) {
    return (uint16_t)((raw[0] << 8) | raw[1]);
}

static inline uint32_t Get_Singleturn(volatile uint8_t *raw) {
    // Используем индексы напрямую от начала массива, так нагляднее
    return ((uint32_t)raw[2] << 16) | 
           ((uint32_t)raw[3] << 8)  | 
            (uint32_t)raw[4];
}
inline float Get_Temperature(volatile uint8_t *raw) {
    int16_t temp = (int16_t)((raw[5] << 8) | raw[6]);
    return (float)temp / 10.0f;
}

static inline float Get_RPM_Raw(volatile uint8_t *raw) {
    int16_t rpm = (int16_t)((raw[5] << 8) | raw[6]);
    return (float)rpm / 10.0f;
}

// Просто вытаскиваем статус из нужного байта (RAW[5])
static inline uint8_t Get_Status_Raw(volatile uint8_t *raw) {
    return raw[5];
}

static inline float Get_angle_degree(volatile uint8_t *raw){
    return (float)(Get_Singleturn(raw) * 360.0f / 131072.0f);
}

static inline float Get_angle_radian(volatile uint8_t *raw) {
    // 0.00004793689f это (2 * PI) / 131072
    return (float)Get_Singleturn(raw) * 0.00004793689f;
}

static inline float Get_angle_shaft(float angle_degree, uint32_t multiturn, uint16_t ratio){
    return (((multiturn * 360.0f) + angle_degree )/ (float)ratio);
}


#ifdef __cplusplus
}
#endif