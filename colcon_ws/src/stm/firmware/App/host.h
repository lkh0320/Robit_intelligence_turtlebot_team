/*
 * host.h - 젯슨(상위 컴퓨터)과의 시리얼 통신 (USART6)
 *
 *   STM32 PC6 (USART6_TX) -> 젯슨 40핀 10번 (RXD)
 *   STM32 PC7 (USART6_RX) <- 젯슨 40핀  8번 (TXD)
 *   GND 끼리 연결. 115200bps, 8N1
 *
 * 받기: USART6 인터럽트가 바이트를 버퍼에 쌓아 두고, host_read() 로 꺼낸다.
 *       (다이나믹셀 통신이 응답을 기다리며 멈춰 있는 동안에도 놓치지 않는다)
 * 보내기: host_write() 는 다 보낼 때까지 기다린다 (115200bps 에서 1바이트 약 87us)
 *
 * 준비 (CubeMX 코드 생성 후 한 번):
 *   Core/Src/stm32f4xx_it.c 의
 *     USER CODE BEGIN Includes     에  #include "host.h"
 *     USER CODE BEGIN USART6_IRQn 0 에  host_irq();
 */
#ifndef HOST_H
#define HOST_H

#include <stdint.h>

void host_init(void);                          /* app_init 에서 한 번 */
void host_irq(void);                           /* USART6 인터럽트에서 호출 */

int host_read(uint8_t *byte);                  /* 받은 바이트가 있으면 1 */
void host_write(const uint8_t *data, uint16_t len);

/* 디버깅용 */
extern volatile uint32_t g_host_rx_bytes;      /* 젯슨에서 받은 바이트 수 */
extern volatile uint32_t g_host_overflow;      /* 버퍼가 넘쳐 버린 바이트 수 */
extern volatile uint32_t g_host_tx_bytes;      /* 젯슨으로 보낸 바이트 수 */

#endif /* HOST_H */
