/*
 * dxl.h - 다이나믹셀 통신 (Protocol 2.0)
 *
 * 이 파일은 "모터와 패킷을 주고받는 일"만 담당한다.
 * 어떤 주소에 무슨 값을 써야 모터가 도는지는 mx64.c 가 알고 있다.
 *
 * 하드웨어 구성
 *   STM32 USART3 (PC10 = TX, PC11 = RX)
 *     -> MAX13488E (RS-485 칩, 송신/수신 방향을 스스로 바꿈)
 *     -> 다이나믹셀 (A/B 두 선)
 *
 * 주의: 이 보드는 우리가 보낸 바이트가 RX 로 그대로 되돌아온다(에코).
 *       그래서 받은 패킷 중 "모터가 보낸 응답"만 골라서 쓴다.
 */
#ifndef DXL_H
#define DXL_H

#include <stdint.h>
#include "main.h"
#include "stm32f4xx_hal.h"

/* ID 0xFE 로 보내면 버스의 모든 모터가 받는다 (broadcast) */
#define DXL_BROADCAST_ID 0xFE

/*
 * 함수 반환값 규칙
 *   음수      : 통신 자체가 실패함 (아래 DXL_ERR_*)
 *   0 이상    : 모터가 보낸 응답의 "에러 바이트"
 *               0    = 정상
 *               0x80 = Alert. 모터에 하드웨어 에러(과부하 등)가 걸려 있다는 표시
 *               그 외 = 명령 자체에 문제가 있음 (잘못된 주소, 범위 초과 등)
 */
#define DXL_OK           0
#define DXL_ERR_TX      -1   /* 송신 실패 */
#define DXL_ERR_TIMEOUT -2   /* 응답이 안 옴 */
#define DXL_ERR_ARG     -3   /* 함수에 넣은 값이 너무 큼 */
#define DXL_ALERT        0x80

/* ---- 준비 ---- */
void dxl_init(USART_TypeDef *usart);     /* 사용할 UART 지정 (USART3) */
void dxl_set_baud(uint32_t baud);        /* 통신 속도 변경 (예: 1000000) */

/* 지금까지 받은 바이트 수 (에코 포함). 디버깅용 */
extern volatile uint32_t dxl_rx_count;

/* ---- 모터 찾기 ---- */
int dxl_ping(uint8_t id);                /* 해당 ID 모터가 있으면 0 이상 */
/* 버스의 모든 모터에게 ping. 찾은 ID 를 ids[] 에 넣고 개수를 돌려준다 */
int dxl_scan(uint8_t *ids, int max);
/* 모터 재부팅. 하드웨어 에러가 걸린 모터는 재부팅해야 다시 움직인다 */
int dxl_reboot(uint8_t id);

/* ---- 읽기 / 쓰기 ---- */
/* 모터의 addr 번지부터 len 바이트를 읽어 out[] 에 넣는다 */
int dxl_read(uint8_t id, uint16_t addr, uint16_t len, uint8_t *out);
/* 모터의 addr 번지부터 data[] 의 len 바이트를 쓴다 */
int dxl_write(uint8_t id, uint16_t addr, const uint8_t *data, uint16_t len);
int dxl_write_u8(uint8_t id, uint16_t addr, uint8_t value);    /* 1바이트 쓰기 */
int dxl_write_u32(uint8_t id, uint16_t addr, uint32_t value);  /* 4바이트 쓰기 */

/*
 * 여러 모터에 한 번에 쓰기 (sync write). 응답은 오지 않는다.
 *   data[] 에는 모터마다 len 바이트씩 순서대로 들어 있어야 한다.
 *   예) 모터 2개, len = 4 이면 data[0..3] = 첫 모터, data[4..7] = 둘째 모터
 */
int dxl_sync_write(uint16_t addr, uint16_t len, const uint8_t *ids,
                   const uint8_t *data, uint8_t count);

/* CRC 계산 (패킷 끝에 붙는 오류 검사 값) */
uint16_t dxl_crc(const uint8_t *data, uint16_t len);

#endif /* DXL_H */
