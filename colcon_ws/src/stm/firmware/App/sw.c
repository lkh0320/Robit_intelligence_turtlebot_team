/*
 * sw.c - 보드 슬라이드 스위치 S1~S4 (PB12~PB15)
 */
#include "sw.h"
#include "main.h"

/*
 * "켜짐" 으로 볼 핀 상태.
 * 실제 보드에서 확인: 아무것도 안 켜면 HIGH, 스위치를 켜면 LOW(0).
 * 스위치가 거꾸로 읽히면 GPIO_PIN_SET 으로 바꾸면 된다.
 */
#define SW_ON_LEVEL GPIO_PIN_RESET

/*
 * PB12~PB15 를 입력으로 설정한다.
 * CubeMX 설정과 똑같이 풀업/풀다운 없음 (보드에 저항이 달려 있음).
 */
void sw_init(void)
{
  GPIO_InitTypeDef init = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();         /* GPIOB 에 클럭 공급 */

  init.Pin  = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  init.Mode = GPIO_MODE_INPUT;
  init.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &init);
}

/* 켜진 스위치를 비트로 모아서 돌려준다 */
uint8_t sw_on(void)
{
  uint8_t on = 0;

  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == SW_ON_LEVEL) {
    on |= SW1;
  }
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == SW_ON_LEVEL) {
    on |= SW2;
  }
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == SW_ON_LEVEL) {
    on |= SW3;
  }
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == SW_ON_LEVEL) {
    on |= SW4;
  }
  return on;
}
