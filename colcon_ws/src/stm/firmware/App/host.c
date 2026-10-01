/*
 * host.c - 젯슨과의 시리얼 통신 (USART6, 인터럽트 수신)
 */
#include "host.h"
#include "main.h"
#include "stm32f4xx_ll_usart.h"

#define HOST_USART  USART6
#define RX_BUF_SIZE 256

/*
 * 원형 수신 버퍼 (dxl.c 와 같은 방식)
 *   인터럽트가 rx_head 쪽에 넣고, 메인 루프가 rx_tail 쪽에서 꺼낸다.
 *   둘 다 인터럽트와 메인에서 같이 보므로 volatile.
 */
static volatile uint8_t  rx_buf[RX_BUF_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;

volatile uint32_t g_host_rx_bytes;
volatile uint32_t g_host_overflow;
volatile uint32_t g_host_tx_bytes;

void host_init(void)
{
  rx_head = 0;
  rx_tail = 0;

  if (!LL_USART_IsEnabled(HOST_USART)) {
    LL_USART_Enable(HOST_USART);
  }
  /* "바이트를 받으면 인터럽트를 걸어라" 켜기.
     (NVIC 쪽 USART6 인터럽트는 CubeMX 가 켜 준다) */
  LL_USART_EnableIT_RXNE(HOST_USART);

  /* 켜질 때 한 번 인사를 보낸다 (배선 확인용).
     - 젯슨에서 이 글자가 보이면 STM32 -> 젯슨 방향은 정상
     - J10 3번(TX)과 4번(RX)을 점퍼로 이으면 보낸 상태 패킷이 자기에게 돌아와
       g_host_rx_bytes 가 계속 늘어난다 (= STM32 쪽 UART 와 J10 커넥터는 정상) */
  const uint8_t hello[] = "STM32 ready\r\n";
  host_write(hello, sizeof(hello) - 1);
}

/* USART6 인터럽트가 걸릴 때마다 호출된다 (stm32f4xx_it.c) */
void host_irq(void)
{
  /* 오버런(너무 빨리 들어와 놓침)이 있으면 지운다. 안 지우면 수신이 멈춘다 */
  if (LL_USART_IsActiveFlag_ORE(HOST_USART)) {
    LL_USART_ClearFlag_ORE(HOST_USART);
  }

  if (LL_USART_IsActiveFlag_RXNE(HOST_USART)) {
    uint8_t b = LL_USART_ReceiveData8(HOST_USART);
    uint16_t next = (uint16_t)(rx_head + 1);

    if ((uint16_t)(next - rx_tail) > RX_BUF_SIZE) {
      g_host_overflow++;            /* 버퍼가 꽉 참: 이 바이트는 버린다 */
      return;
    }
    rx_buf[rx_head % RX_BUF_SIZE] = b;
    rx_head = next;
    g_host_rx_bytes++;
  }
}

int host_read(uint8_t *byte)
{
  if (rx_tail == rx_head) {
    return 0;                       /* 받은 게 없음 */
  }
  *byte = rx_buf[rx_tail % RX_BUF_SIZE];
  rx_tail++;
  return 1;
}

void host_write(const uint8_t *data, uint16_t len)
{
  for (uint16_t i = 0; i < len; i++) {
    while (!LL_USART_IsActiveFlag_TXE(HOST_USART)) {
    }
    LL_USART_TransmitData8(HOST_USART, data[i]);
    g_host_tx_bytes++;
  }
}
