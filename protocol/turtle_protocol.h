// Jetson <-> STM32 통신 규약 (초안 v1)
// 패킷 정의의 유일한 원본. STM32 펌웨어(C)와 turtle_stm32_bridge(C++)가 함께 include 한다.
// 수정 시 PROTOCOL.md 표와 PROTOCOL_VERSION 을 같이 바꿀 것.
//
// [처음 보는 사람을 위한 요약]
//  - Jetson 과 STM32 는 UART 로 "패킷" 단위로 데이터를 주고받는다.
//  - 패킷 하나 = 시작 표시 2바이트 + 종류(ID) + 길이(LEN) + 내용(PAYLOAD) + 검사값(CHECKSUM)
//  - 내용(PAYLOAD)의 모양은 아래 Pkt* 구조체로 정해져 있고, 양쪽이 이 구조체를 그대로 memcpy 해서 쓴다.
//  - 그래서 이 파일이 양쪽에서 다르면 값이 엉뚱하게 해석된다 → 반드시 이 파일 하나만 고치고 양쪽 다시 빌드.
#ifndef TURTLE_PROTOCOL_H
#define TURTLE_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PROTOCOL_VERSION      1      // STATUS 패킷에 담겨 전송됨. 규약을 바꾸면 1씩 올려서 버전 불일치를 알아챌 수 있게 함

// 프레임: [0xAA][0x55][ID][LEN][PAYLOAD x LEN][CHECKSUM]
// 예) MOTOR_CMD: AA 55 01 05 <left 2B> <right 2B> <enable 1B> <checksum>  → 총 10바이트
// 수신 쪽은 0xAA 0x55 를 찾아 패킷 시작을 맞추고, CHECKSUM 이 틀리면 그 패킷은 버린다.
#define PKT_HEADER_1          0xAA
#define PKT_HEADER_2          0x55
#define PKT_OVERHEAD          5      // 헤더2 + ID + LEN + CHECKSUM
#define PKT_MAX_PAYLOAD       32     // 수신 버퍼 크기 기준. 이보다 긴 LEN 은 잘못된 패킷으로 처리

#define CMD_TIMEOUT_MS        200    // 이 시간 동안 MOTOR_CMD/HEARTBEAT 없으면 STM32가 모터 정지
                                     // (Jetson 이 죽거나 케이블이 빠졌을 때 로봇이 계속 달리는 것을 막는 안전장치)

// ---- 패킷 종류(ID) ----
// Jetson -> STM32 는 0x0?, STM32 -> Jetson 은 0x8? 로 방향을 구분

// Jetson -> STM32
#define PKT_ID_MOTOR_CMD      0x01   // payload: PktMotorCmd (바퀴 목표 속도)
#define PKT_ID_SET_PARAM      0x02   // payload: PktSetParam (GUI 에서 바꾼 PID 값 전달)
#define PKT_ID_HEARTBEAT      0x03   // payload 없음. 모터 명령이 없을 때도 "Jetson 살아 있음"을 알림

// STM32 -> Jetson
#define PKT_ID_SENSOR         0x81   // payload: PktSensor (PSD 3개)
#define PKT_ID_MOTOR_STATE    0x82   // payload: PktMotorState (바퀴 현재 속도)
#define PKT_ID_STATUS         0x83   // payload: PktStatus (STM32 상태, 에러)

// SET_PARAM 의 param_id (config/stm32_bridge.yaml 의 wheel_kp/ki/kd 와 대응)
#define PARAM_ID_WHEEL_KP     1
#define PARAM_ID_WHEEL_KI     2
#define PARAM_ID_WHEEL_KD     3

// STATUS 의 state
#define MCU_STATE_IDLE        0
#define MCU_STATE_RUN         1
#define MCU_STATE_FAULT       2

// STATUS 의 error_flags (비트 OR)
// 여러 에러가 동시에 있을 수 있음. 확인 예: if (flags & ERR_CMD_TIMEOUT) { ... }
#define ERR_DXL_LEFT          (1u << 0)
#define ERR_DXL_RIGHT         (1u << 1)
#define ERR_CMD_TIMEOUT       (1u << 2)
#define ERR_BAD_CHECKSUM      (1u << 3)

// ---- payload 구조체 ----
// pack(1): 구조체 사이에 빈 바이트(패딩)를 넣지 않게 해서, STM32 와 Jetson 의 메모리 모양을 똑같이 맞춤
// 두 CPU 모두 little-endian 이라 memcpy 로 바로 주고받을 수 있음
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
// 보내는 쪽: 계산해서 맨 끝에 붙임 / 받는 쪽: 다시 계산해서 받은 값과 다르면 그 패킷은 버림
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
