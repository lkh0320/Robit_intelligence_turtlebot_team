/*
 * sw.h - 보드 슬라이드 스위치 S1~S4
 *
 *   S1 = PB12, S2 = PB13, S3 = PB14, S4 = PB15
 *   핀이 LOW(0) 이면 "켜짐" 으로 본다 (sw.c 의 SW_ON_LEVEL).
 *
 * sw_on() 은 켜진 스위치를 비트로 알려준다.
 *   bit0 = S1, bit1 = S2, bit2 = S3, bit3 = S4
 *   예) S1 과 S3 가 켜져 있으면 0b0101 = 5
 *
 * 특정 스위치가 켜졌는지 보려면:  if (on & SW3) { ... }
 */
#ifndef SW_H
#define SW_H

#include <stdint.h>

#define SW1 0x01
#define SW2 0x02
#define SW3 0x04
#define SW4 0x08

void sw_init(void);      /* 핀 설정 (한 번만 호출) */
uint8_t sw_on(void);     /* 켜진 스위치 비트 */

#endif /* SW_H */
