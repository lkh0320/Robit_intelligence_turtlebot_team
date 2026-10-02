/*
 * psd.c - PSD 거리 센서 3개 읽기
 */
#include "psd.h"
#include "adc.h"
#include <math.h>

#define ADC_CHANNELS 4          /* CubeMX 에서 스캔하는 채널 수 (0, 1, 2, 3) */
#define ADC_MAX      4095.0f    /* 12비트 */
#define VREF         3.3f       /* ADC 기준 전압 [V] */

#define MIN_MM 100              /* 센서가 잴 수 있는 범위 (GP2Y0A21: 10 ~ 80 cm) */
#define MAX_MM 800

/* 새 값을 얼마나 믿을지 (0 ~ 1). 작을수록 부드럽지만 반응이 느리다 */
#define FILTER_ALPHA 0.3f

volatile uint16_t g_psd_mm[PSD_COUNT];
volatile uint16_t g_psd_raw[PSD_COUNT];
volatile uint16_t g_psd_mv[PSD_COUNT];

/* DMA 가 ADC 결과를 계속 써 넣는 곳 (CubeMX 에서 DMA 데이터 크기 = Word) */
static volatile uint32_t adc_buf[ADC_CHANNELS];
static float filtered[PSD_COUNT];
static int started;

/*
 * 전압 [V] -> 거리 [mm]
 * GP2Y0A21 데이터시트 그래프를 식으로 맞춘 것: 거리[cm] = 29.988 * V^(-1.173)
 * 가까울수록 전압이 높다 (10cm 약 2.3V, 80cm 약 0.4V)
 */
static uint16_t volt_to_mm(float volt)
{
  if (volt < 0.3f) {
    return MAX_MM;              /* 너무 낮음 = 아무것도 없음 (멀다) */
  }
  float mm = 299.88f * powf(volt, -1.173f);
  if (mm < MIN_MM) {
    mm = MIN_MM;
  }
  if (mm > MAX_MM) {
    mm = MAX_MM;
  }
  return (uint16_t)mm;
}

void psd_init(void)
{
  /* 4채널을 한 번 스캔하면 DMA 가 adc_buf[0..3] 에 넣고, 처음 자리로 돌아간다 (순환) */
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, ADC_CHANNELS) == HAL_OK) {
    started = 1;
  }
}

void psd_update(void)
{
  if (!started) {
    return;
  }

  for (int i = 0; i < PSD_COUNT; i++) {
    float raw = (float)(adc_buf[i] & 0x0FFF);
    filtered[i] += FILTER_ALPHA * (raw - filtered[i]);

    float volt = filtered[i] * VREF / ADC_MAX;
    g_psd_raw[i] = (uint16_t)filtered[i];
    g_psd_mv[i] = (uint16_t)(volt * 1000.0f);
    g_psd_mm[i] = volt_to_mm(volt);
  }

  /* 다음 스캔 시작 (CubeMX 설정이 "한 번 스캔하고 멈춤" 이라 매번 시작 신호를 준다) */
  hadc1.Instance->CR2 |= ADC_CR2_SWSTART;
}
