// Jetson <-> STM32 통신 규약 (초안 v1)
// 패킷 정의의 유일한 원본. STM32 펌웨어(C)와 turtle_stm32_bridge(C++)가 함께 include 한다.
// 수정 시 PROTOCOL.md 표와 PROTOCOL_VERSION 을 같이 바꿀 것.
#ifndef TURTLE_PROTOCOL_H
#define TURTLE_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PROTOCOL_VERSION      1

// 프레임: [0xAA][0x55][ID][LEN][PAYLOAD x LEN][CHECKSUM]
#define PKT_HEADER_1          0xAA
#define PKT_HEADER_2          0x55
#define PKT_OVERHEAD          5      // 헤더2 + ID + LEN + CHECKSUM
#define PKT_MAX_PAYLOAD       32

#define CMD_TIMEOUT_MS        200    // 이 시간 동안 MOTOR_CMD/HEARTBEAT 없으면 STM32가 모터 정지

// Jetson -> STM32
#define PKT_ID_MOTOR_CMD      0x01
#define PKT_ID_SET_PARAM      0x02
#define PKT_ID_HEARTBEAT      0x03   // payload 없음

// STM32 -> Jetson
#define PKT_ID_SENSOR         0x81
#define PKT_ID_MOTOR_STATE    0x82
#define PKT_ID_STATUS         0x83

// SET_PARAM 의 param_id
#define PARAM_ID_WHEEL_KP     1
#define PARAM_ID_WHEEL_KI     2
#define PARAM_ID_WHEEL_KD     3

// STATUS 의 state
#define MCU_STATE_IDLE        0
#define MCU_STATE_RUN         1
#define MCU_STATE_FAULT       2

// STATUS 의 error_flags (비트 OR)
#define ERR_DXL_LEFT          (1u << 0)
#define ERR_DXL_RIGHT         (1u << 1)
#define ERR_CMD_TIMEOUT       (1u << 2)
#define ERR_BAD_CHECKSUM      (1u << 3)

#pragma pack(push, 1)

typedef struct {
    int16_t left_mmps;       // 왼쪽 바퀴 목표 속도 [mm/s]
    int16_t right_mmps;      // 오른쪽 바퀴 목표 속도 [mm/s]
    uint8_t enable;          // 0이면 즉시 정지
} PktMotorCmd;

typedef struct {
    uint8_t param_id;        // PARAM_ID_*
    float   value;
} PktSetParam;

typedef struct {
    uint16_t psd_raw[3];     // ADC 원본값 (좌, 중앙, 우)
    uint16_t psd_mm[3];      // 환산 거리 [mm]
} PktSensor;

typedef struct {
    int16_t left_mmps;       // 현재 속도 [mm/s]
    int16_t right_mmps;
    uint8_t left_error;      // Dynamixel hardware error status
    uint8_t right_error;
} PktMotorState;

typedef struct {
    uint8_t  protocol_version;
    uint8_t  state;          // MCU_STATE_*
    uint16_t error_flags;    // ERR_*
} PktStatus;

#pragma pack(pop)

// ID + LEN + payload 바이트 합의 하위 8비트
static inline uint8_t pkt_checksum(uint8_t id, uint8_t len, const uint8_t *payload)
{
    uint8_t sum = (uint8_t)(id + len);
    for (uint8_t i = 0; i < len; i++) sum = (uint8_t)(sum + payload[i]);
    return sum;
}

#ifdef __cplusplus
}
#endif

#endif // TURTLE_PROTOCOL_H
