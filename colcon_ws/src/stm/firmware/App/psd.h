/*
 * psd.h - PSD 거리 센서 3개 (ADC1)
 *
 *   ADC1 채널 0 (PA0) = 왼쪽, 채널 1 (PA1) = 앞, 채널 2 (PA2) = 오른쪽
 *   (CubeMX: ADC1 4채널 스캔 + DMA2 Stream0 순환 모드. 채널 3 (PA3) 은 안 씀)
 *
 * psd_update() 를 10ms 마다 부르면
 *   1. DMA 가 받아 둔 ADC 값을 꺼내 부드럽게(필터) 만들고
 *   2. 전압 -> 거리 (mm) 로 바꾸고
 *   3. 다음 측정을 시작한다.
 *
 * 거리 변환은 Sharp GP2Y0A21 (10 ~ 80 cm) 기준이다. 다른 센서면 psd.c 의 식을 바꾼다.
 */
#ifndef PSD_H
#define PSD_H

#include <stdint.h>

#define PSD_COUNT 3
#define PSD_LEFT  0
#define PSD_FRONT 1
#define PSD_RIGHT 2

void psd_init(void);       /* ADC + DMA 시작 (한 번) */
void psd_update(void);     /* 10ms 마다 */

/* 거리 [mm] (100 ~ 800). [0] 왼쪽, [1] 앞, [2] 오른쪽 */
extern volatile uint16_t g_psd_mm[PSD_COUNT];
/* 디버깅용 */
extern volatile uint16_t g_psd_raw[PSD_COUNT];  /* ADC 값 0 ~ 4095 (필터 후) */
extern volatile uint16_t g_psd_mv[PSD_COUNT];   /* 센서 출력 전압 [mV] */

#endif /* PSD_H */
