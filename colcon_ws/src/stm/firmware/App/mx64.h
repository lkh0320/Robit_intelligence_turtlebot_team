/*
 * mx64.h - MX-64(2.0) 모터 제어
 *
 * 다이나믹셀은 내부에 "Control Table" 이라는 메모리 표가 있다.
 * 정해진 주소에 값을 쓰면 모터가 그대로 동작한다.
 *   예) 64번지에 1 을 쓰면 토크 ON, 104번지에 50 을 쓰면 50 속도로 회전
 *
 * 주소는 ROBOTIS e-Manual 의 "MX-64(2.0) Control Table" 을 따른다.
 */
#ifndef MX64_H
#define MX64_H

#include <stdint.h>

/* ---- EEPROM 영역: 전원을 꺼도 저장됨. 토크가 꺼져 있어야 쓸 수 있다 ---- */
#define MX64_ADDR_ID             7
#define MX64_ADDR_BAUD_RATE      8
#define MX64_ADDR_OPERATING_MODE 11   /* 1 = 속도 모드, 3 = 위치 모드 */
#define MX64_ADDR_MAX_VOLTAGE    32   /* 2바이트, 전압 상한 (0.1V 단위) */
#define MX64_ADDR_MIN_VOLTAGE    34   /* 2바이트, 전압 하한 (0.1V 단위) */
#define MX64_ADDR_VELOCITY_LIMIT 44   /* 4바이트 */

/* ---- RAM 영역: 전원을 끄면 초기화됨 ---- */
#define MX64_ADDR_TORQUE_ENABLE    64   /* 1 = 힘 줌, 0 = 힘 뺌 */
#define MX64_ADDR_LED              65
#define MX64_ADDR_HARDWARE_ERROR   70   /* 하드웨어 에러 상태 (읽기 전용) */
#define MX64_ADDR_GOAL_VELOCITY    104  /* 목표 속도, 4바이트, 음수 = 반대 방향 */
#define MX64_ADDR_PRESENT_VELOCITY 128  /* 현재 속도, 4바이트 */
#define MX64_ADDR_PRESENT_VOLTAGE  144  /* 지금 들어오는 전압, 2바이트 (0.1V 단위) */

/* Operating Mode 값 */
#define MX64_MODE_VELOCITY 1
#define MX64_MODE_POSITION 3

/* Hardware Error Status (70번지) 의 각 비트 의미 */
#define MX64_HW_INPUT_VOLTAGE 0x01   /* 전압이 범위를 벗어남 */
#define MX64_HW_OVERHEATING   0x04   /* 과열 */
#define MX64_HW_ENCODER       0x08   /* 엔코더 이상 */
#define MX64_HW_SHOCK         0x10   /* 전기 충격 */
#define MX64_HW_OVERLOAD      0x20   /* 과부하 (바퀴가 막혀서 힘을 너무 씀) */

/* 속도 단위: 1 = 0.229 rpm.  예) 50 -> 약 11.5 rpm */

/*
 * 모터 함수의 반환값이 "실패"인지 판단한다.
 *   음수           -> 통신 실패
 *   에러 바이트의 아래 7비트가 0 이 아님 -> 명령 실패
 * 0x80(Alert) 비트만 켜진 경우는 "명령은 성공, 하드웨어 에러가 걸려 있음"
 * 이라서 여기서는 실패로 보지 않는다.
 */
int mx64_failed(int ret);

int mx64_torque(uint8_t id, uint8_t on);          /* on: 1 = ON, 0 = OFF */
/* 바퀴용 설정: 토크 OFF -> 속도 모드 -> 속도 0 -> 토크 ON */
int mx64_set_wheel_mode(uint8_t id);
int mx64_set_speed(uint8_t id, int32_t speed);    /* 모터 하나 속도 */
/* 여러 모터 속도를 패킷 하나로 동시에 보낸다 (응답 없음) */
int mx64_set_speeds(const uint8_t *ids, const int32_t *speeds, uint8_t count);

#endif /* MX64_H */
