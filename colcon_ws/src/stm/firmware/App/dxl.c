/*
 * dxl.c - 다이나믹셀 통신 (Protocol 2.0)
 *
 * ================= 패킷 모양 =================
 *
 *  [FF][FF][FD][00] [ID] [LEN_L][LEN_H] [INST] [PARAM ...] [CRC_L][CRC_H]
 *   └── 헤더 4바이트 ─┘       └ 길이 ─┘   명령    데이터        오류 검사
 *
 *   LEN  = INST 1바이트 + PARAM 개수 + CRC 2바이트
 *   CRC  = 헤더부터 PARAM 끝까지 계산한 값 (dxl_crc)
 *   2바이트 이상 숫자는 "작은 자리 먼저" 보낸다 (little endian)
 *     예) 300 = 0x012C  ->  2C 01
 *
 * 모터가 보내는 응답 패킷은 INST 자리가 항상 0x55 이고,
 * 그 다음 바이트가 에러 바이트다.
 *
 * ================= 수신 방식 =================
 *
 * 인터럽트나 DMA 없이 "폴링"으로 받는다.
 * 즉, 응답을 기다리는 동안 계속 "새 바이트 왔니?" 하고 USART 를 확인한다.
 * 받은 바이트는 rx_buf 라는 원형 버퍼에 쌓아두고 하나씩 꺼내 쓴다.
 */
#include "dxl.h"
#include "stm32f4xx_ll_usart.h"
#include <string.h>

/* 명령 번호 (INST) */
#define INST_PING       0x01
#define INST_READ       0x02
#define INST_WRITE      0x03
#define INST_REBOOT     0x08
#define INST_SYNC_WRITE 0x83
#define INST_STATUS     0x55   /* 모터가 보내는 응답 패킷 */

#define TX_TIMEOUT_MS 20       /* 송신이 이 시간 안에 안 끝나면 실패 */
#define PKT_MAX       96       /* 패킷 최대 길이 (바이트) */

/* 수신 버퍼 크기. 256 이어야 아래 "% RX_BUF_SIZE" 계산이 맞다 */
#define RX_BUF_SIZE 256

static USART_TypeDef *dxl_usart;        /* 사용하는 UART (USART3) */
static uint32_t status_timeout_ms = 10; /* 응답을 기다리는 시간 */

/*
 * 원형 수신 버퍼
 *   rx_head : 다음에 "넣을" 위치 (계속 증가)
 *   rx_tail : 다음에 "꺼낼" 위치 (계속 증가)
 *   head == tail 이면 버퍼가 비어 있다.
 *   실제 배열 위치는 % RX_BUF_SIZE 로 0~255 로 맞춘다.
 */
static uint8_t rx_buf[RX_BUF_SIZE];
static uint16_t rx_head;
static uint16_t rx_tail;

volatile uint32_t dxl_rx_count;

static uint8_t tx_pkt[PKT_MAX];         /* 보낼 패킷을 만드는 곳 */


/* ======================================================================
 *  1. 작은 도우미 함수들
 * ====================================================================== */

/* 지금 시각이 deadline 을 지났으면 1 */
static int time_is_over(uint32_t deadline)
{
  /* 뺄셈 결과를 부호 있는 수로 보면 tick 값이 넘쳐 0 으로 돌아가도 안전하다 */
  if ((int32_t)(HAL_GetTick() - deadline) >= 0) {
    return 1;
  }
  return 0;
}

/* 16비트 숫자의 아래 8비트 / 위 8비트 */
static uint8_t low_byte(uint16_t value)  { return (uint8_t)(value & 0xFF); }
static uint8_t high_byte(uint16_t value) { return (uint8_t)(value >> 8); }


/* ======================================================================
 *  2. CRC (패킷 오류 검사 값)
 *     ROBOTIS 문서의 CRC-16 계산법 그대로. 다항식 0x8005.
 * ====================================================================== */

uint16_t dxl_crc(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0;

  for (uint16_t i = 0; i < len; i++) {
    crc = crc ^ ((uint16_t)data[i] << 8);

    for (int bit = 0; bit < 8; bit++) {
      if (crc & 0x8000) {
        crc = (uint16_t)((crc << 1) ^ 0x8005);
      } else {
        crc = (uint16_t)(crc << 1);
      }
    }
  }
  return crc;
}


/* ======================================================================
 *  3. 수신 (USART -> rx_buf -> 우리 코드)
 * ====================================================================== */

/* USART 에 새 바이트가 와 있으면 rx_buf 에 옮겨 담는다 */
static void rx_poll(void)
{
  /* 오버런 에러(너무 늦게 읽어서 바이트를 놓침)가 있으면 지운다.
     지우지 않으면 USART 가 더 이상 받지 않는다. */
  if (LL_USART_IsActiveFlag_ORE(dxl_usart)) {
    LL_USART_ClearFlag_ORE(dxl_usart);
  }

  /* RXNE = "받은 바이트가 있음" */
  if (LL_USART_IsActiveFlag_RXNE(dxl_usart)) {
    rx_buf[rx_head % RX_BUF_SIZE] = LL_USART_ReceiveData8(dxl_usart);
    rx_head++;
    dxl_rx_count++;
  }
}

/* 이전에 받아둔 바이트를 모두 버린다 (새 명령을 보내기 직전에 호출) */
static void rx_flush(void)
{
  while (LL_USART_IsActiveFlag_RXNE(dxl_usart)) {
    LL_USART_ReceiveData8(dxl_usart);
  }
  if (LL_USART_IsActiveFlag_ORE(dxl_usart)) {
    LL_USART_ClearFlag_ORE(dxl_usart);
  }
  rx_tail = rx_head;
}

/*
 * 받은 바이트 하나를 꺼낸다.
 * 아직 없으면 deadline 까지 기다린다.
 * 꺼냈으면 1, 시간이 지나도 안 오면 0.
 */
static int rx_get(uint8_t *byte, uint32_t deadline)
{
  while (rx_tail == rx_head) {          /* 버퍼가 비어 있는 동안 */
    rx_poll();
    if (rx_tail == rx_head && time_is_over(deadline)) {
      return 0;
    }
  }

  *byte = rx_buf[rx_tail % RX_BUF_SIZE];
  rx_tail++;
  return 1;
}


/* ======================================================================
 *  4. 송신
 * ====================================================================== */

/* data[] 의 len 바이트를 USART 로 보낸다 */
static int tx_bytes(const uint8_t *data, uint16_t len)
{
  uint32_t deadline = HAL_GetTick() + TX_TIMEOUT_MS + 1;

  for (uint16_t i = 0; i < len; i++) {
    /* TXE = "송신 레지스터가 비었음, 다음 바이트 넣어도 됨" */
    while (!LL_USART_IsActiveFlag_TXE(dxl_usart)) {
      rx_poll();                        /* 보내는 동안 돌아오는 에코도 받아둔다 */
      if (time_is_over(deadline)) {
        return DXL_ERR_TX;
      }
    }
    LL_USART_TransmitData8(dxl_usart, data[i]);
    rx_poll();
  }

  /* TC = "마지막 바이트까지 선으로 다 나갔음" */
  while (!LL_USART_IsActiveFlag_TC(dxl_usart)) {
    rx_poll();
    if (time_is_over(deadline)) {
      return DXL_ERR_TX;
    }
  }
  return DXL_OK;
}

/*
 * 패킷을 만들어서 보낸다.
 *   id      : 받을 모터 ID
 *   inst    : 명령 번호 (INST_PING 등)
 *   params  : 데이터
 *   nparams : 데이터 길이
 */
static int send_packet(uint8_t id, uint8_t inst, const uint8_t *params,
                       uint16_t nparams)
{
  uint16_t n;   /* tx_pkt 에서 다음에 쓸 위치 */

  /* 헤더, ID, 명령. 길이(5,6번)는 데이터를 다 넣은 뒤에 채운다 */
  tx_pkt[0] = 0xFF;
  tx_pkt[1] = 0xFF;
  tx_pkt[2] = 0xFD;
  tx_pkt[3] = 0x00;
  tx_pkt[4] = id;
  tx_pkt[7] = inst;
  n = 8;

  /* 데이터 복사 */
  for (uint16_t i = 0; i < nparams; i++) {
    if (n + 3 > PKT_MAX) {
      return DXL_ERR_ARG;
    }
    tx_pkt[n] = params[i];
    n++;

    /*
     * 바이트 스터핑: 데이터 중간에 FF FF FD 가 나오면 모터가 헤더로
     * 착각하므로 뒤에 FD 를 하나 더 넣는다. (받는 쪽에서 다시 뺀다)
     */
    if (tx_pkt[n - 3] == 0xFF && tx_pkt[n - 2] == 0xFF && tx_pkt[n - 1] == 0xFD) {
      tx_pkt[n] = 0xFD;
      n++;
    }
  }

  /* 길이 = 명령 1 + 데이터 + CRC 2  (= 지금까지 쓴 것 - 헤더/ID/길이 7 + CRC 2) */
  uint16_t len = n - 8 + 3;
  tx_pkt[5] = low_byte(len);
  tx_pkt[6] = high_byte(len);

  /* 맨 끝에 CRC */
  uint16_t crc = dxl_crc(tx_pkt, n);
  tx_pkt[n] = low_byte(crc);
  n++;
  tx_pkt[n] = high_byte(crc);
  n++;

  rx_flush();                   /* 이전 쓰레기 바이트 버리기 */
  return tx_bytes(tx_pkt, n);
}


/* ======================================================================
 *  5. 응답 패킷 받기
 * ====================================================================== */

/* 헤더 FF FF FD 00 이 나올 때까지 바이트를 버린다. 찾으면 1 */
static int wait_for_header(uint32_t deadline)
{
  const uint8_t header[4] = { 0xFF, 0xFF, 0xFD, 0x00 };
  uint8_t b;
  int matched = 0;              /* 헤더를 몇 바이트째까지 맞췄는지 */

  while (matched < 4) {
    if (!rx_get(&b, deadline)) {
      return 0;
    }
    if (b == header[matched]) {
      matched++;
    } else if (b == 0xFF && matched == 2) {
      /* FF FF FF: 마지막 두 개가 FF FF 이므로 그대로 2 */
    } else if (b == 0xFF) {
      matched = 1;              /* FF FF FD FF: 마지막 FF 부터 다시 시작 */
    } else {
      matched = 0;              /* 틀리면 처음부터 다시 */
    }
  }
  return 1;
}

/*
 * 응답 패킷 하나를 받는다. (우리가 보낸 패킷의 에코는 건너뛴다)
 *   deadline : 이 시각까지 기다림
 *   id       : 응답한 모터 ID 가 여기에 들어감
 *   params   : 응답 데이터가 여기에 들어감 (nparams 바이트)
 * 돌려주는 값: 에러 바이트 (0 이상) 또는 DXL_ERR_TIMEOUT
 */
static int recv_any(uint32_t deadline, uint8_t *id, uint8_t *params,
                    uint16_t nparams)
{
  uint8_t pkt[PKT_MAX];

  while (1) {
    /* (1) 헤더 찾기 */
    if (!wait_for_header(deadline)) {
      return DXL_ERR_TIMEOUT;
    }
    pkt[0] = 0xFF;
    pkt[1] = 0xFF;
    pkt[2] = 0xFD;
    pkt[3] = 0x00;

    /* (2) ID, 길이 2바이트 */
    for (int i = 4; i < 7; i++) {
      if (!rx_get(&pkt[i], deadline)) {
        return DXL_ERR_TIMEOUT;
      }
    }
    uint16_t len = pkt[5] | (pkt[6] << 8);
    if (len < 4 || len + 7 > PKT_MAX) {
      continue;                 /* 말이 안 되는 길이 -> 버리고 다음 패킷 */
    }

    /* (3) 나머지 (명령, 에러, 데이터, CRC) */
    for (uint16_t i = 0; i < len; i++) {
      if (!rx_get(&pkt[7 + i], deadline)) {
        return DXL_ERR_TIMEOUT;
      }
    }
    uint16_t total = len + 7;   /* 패킷 전체 길이 */

    /* (4) CRC 검사. 틀리면 깨진 패킷이니 버린다 */
    uint16_t crc = pkt[total - 2] | (pkt[total - 1] << 8);
    if (crc != dxl_crc(pkt, total - 2)) {
      continue;
    }

    /* (5) 응답(0x55)이 아니면 우리가 보낸 패킷의 에코다 -> 버린다 */
    if (pkt[7] != INST_STATUS) {
      continue;
    }

    /* (6) 데이터 꺼내기 (9번부터 CRC 앞까지). 스터핑된 FD 는 뺀다 */
    uint16_t out = 0;
    for (uint16_t i = 9; i < total - 2; i++) {
      int is_stuffed = (pkt[i] == 0xFD && pkt[i - 1] == 0xFD &&
                        pkt[i - 2] == 0xFF && pkt[i - 3] == 0xFF);
      if (is_stuffed) {
        continue;
      }
      if (out < nparams) {
        params[out] = pkt[i];
      }
      out++;
    }
    if (out < nparams) {
      continue;                 /* 원한 만큼 데이터가 없으면 버린다 */
    }

    *id = pkt[4];
    return pkt[8];              /* 에러 바이트 */
  }
}

/* 특정 ID 모터의 응답을 기다린다 */
static int recv_status(uint8_t id, uint8_t *params, uint16_t nparams)
{
  uint32_t deadline = HAL_GetTick() + status_timeout_ms + 1;
  uint8_t who;
  int ret;

  while (1) {
    ret = recv_any(deadline, &who, params, nparams);
    if (ret < 0) {
      return ret;               /* 시간 초과 */
    }
    if (who == id) {
      return ret;               /* 원하던 모터의 응답 */
    }
    /* 다른 모터의 응답이면 무시하고 계속 기다린다 */
  }
}


/* ======================================================================
 *  6. 밖에서 쓰는 함수들 (dxl.h 에 선언)
 * ====================================================================== */

void dxl_init(USART_TypeDef *usart)
{
  dxl_usart = usart;
  rx_head = 0;
  rx_tail = 0;
  if (!LL_USART_IsEnabled(usart)) {
    LL_USART_Enable(usart);
  }
  rx_flush();
}

void dxl_set_baud(uint32_t baud)
{
  /* 보내던 게 다 나갈 때까지 기다린 뒤 속도를 바꾼다 */
  while (!LL_USART_IsActiveFlag_TC(dxl_usart)) {
  }
  LL_USART_Disable(dxl_usart);
  LL_USART_SetBaudRate(dxl_usart, HAL_RCC_GetPCLK1Freq(),
                       LL_USART_OVERSAMPLING_16, baud);
  LL_USART_Enable(dxl_usart);

  /* 속도가 느릴수록 응답이 늦게 오므로 기다리는 시간을 늘린다
     (1Mbps -> 10ms, 57600bps -> 15ms, 9600bps -> 41ms) */
  status_timeout_ms = 10 + 300000 / baud;
  rx_flush();
}

int dxl_ping(uint8_t id)
{
  uint8_t info[3];              /* 응답: 모델 번호 2바이트 + 펌웨어 버전 1바이트 */
  int ret = send_packet(id, INST_PING, NULL, 0);
  if (ret != DXL_OK) {
    return ret;
  }
  return recv_status(id, info, 3);
}

int dxl_scan(uint8_t *ids, int max)
{
  uint8_t info[3];
  uint8_t id;
  int count = 0;

  /* broadcast ping: 모든 모터가 자기 ID 순서대로 조금씩 늦게 응답한다 */
  if (send_packet(DXL_BROADCAST_ID, INST_PING, NULL, 0) != DXL_OK) {
    return 0;
  }

  uint32_t deadline = HAL_GetTick() + 3 * status_timeout_ms + 10;
  while (recv_any(deadline, &id, info, 3) >= 0) {
    if (count < max) {
      ids[count] = id;
    }
    count++;
  }

  if (count > max) {
    return max;
  }
  return count;
}

int dxl_reboot(uint8_t id)
{
  int ret = send_packet(id, INST_REBOOT, NULL, 0);
  if (ret != DXL_OK) {
    return ret;
  }
  return recv_status(id, NULL, 0);
}

int dxl_read(uint8_t id, uint16_t addr, uint16_t len, uint8_t *out)
{
  /* 읽기 명령 데이터: 주소 2바이트 + 길이 2바이트 */
  uint8_t params[4];
  params[0] = low_byte(addr);
  params[1] = high_byte(addr);
  params[2] = low_byte(len);
  params[3] = high_byte(len);

  if (len + 11 > PKT_MAX) {
    return DXL_ERR_ARG;
  }
  int ret = send_packet(id, INST_READ, params, 4);
  if (ret != DXL_OK) {
    return ret;
  }
  return recv_status(id, out, len);
}

int dxl_write(uint8_t id, uint16_t addr, const uint8_t *data, uint16_t len)
{
  /* 쓰기 명령 데이터: 주소 2바이트 + 쓸 값들 */
  uint8_t params[PKT_MAX];

  if (len + 2 > PKT_MAX - 12) {
    return DXL_ERR_ARG;
  }
  params[0] = low_byte(addr);
  params[1] = high_byte(addr);
  memcpy(&params[2], data, len);

  int ret = send_packet(id, INST_WRITE, params, len + 2);
  if (ret != DXL_OK) {
    return ret;
  }
  if (id == DXL_BROADCAST_ID) {
    return ret;                 /* broadcast 는 응답이 없다 */
  }
  return recv_status(id, NULL, 0);
}

int dxl_write_u8(uint8_t id, uint16_t addr, uint8_t value)
{
  return dxl_write(id, addr, &value, 1);
}

int dxl_write_u32(uint8_t id, uint16_t addr, uint32_t value)
{
  /* 4바이트 숫자를 작은 자리부터 나눠 담는다 */
  uint8_t data[4];
  data[0] = (uint8_t)(value & 0xFF);
  data[1] = (uint8_t)((value >> 8) & 0xFF);
  data[2] = (uint8_t)((value >> 16) & 0xFF);
  data[3] = (uint8_t)(value >> 24);
  return dxl_write(id, addr, data, 4);
}

int dxl_sync_write(uint16_t addr, uint16_t len, const uint8_t *ids,
                   const uint8_t *data, uint8_t count)
{
  /*
   * sync write 데이터 모양
   *   [주소 2] [길이 2] [ID1][값 len개] [ID2][값 len개] ...
   */
  uint8_t params[PKT_MAX];
  uint16_t n = 0;

  if (4 + count * (len + 1) > PKT_MAX - 12) {
    return DXL_ERR_ARG;
  }
  params[0] = low_byte(addr);
  params[1] = high_byte(addr);
  params[2] = low_byte(len);
  params[3] = high_byte(len);
  n = 4;

  for (uint8_t i = 0; i < count; i++) {
    params[n] = ids[i];
    n++;
    memcpy(&params[n], &data[i * len], len);
    n += len;
  }
  return send_packet(DXL_BROADCAST_ID, INST_SYNC_WRITE, params, n);
}
