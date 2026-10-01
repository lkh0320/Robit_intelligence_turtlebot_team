/*
 * proto.c - 젯슨 <-> STM32 패킷 만들기 / 해석하기
 *
 * 받은 바이트를 한 개씩 proto_feed() 에 넣으면,
 * "지금 패킷의 어디까지 받았는지" (상태) 를 기억하면서 조립한다.
 */
#include "proto.h"

volatile uint32_t g_proto_ok;
volatile uint32_t g_proto_bad;

/* 지금 무엇을 기다리는 중인지 */
enum {
  WAIT_HEAD1,     /* 0xAA 를 기다림 */
  WAIT_HEAD2,     /* 0x55 를 기다림 */
  WAIT_CMD,
  WAIT_LEN,
  WAIT_DATA,
  WAIT_CHK
};

static int state = WAIT_HEAD1;
static proto_packet_t pkt;     /* 조립 중인 패킷 */
static uint8_t got;            /* DATA 를 몇 바이트 받았는지 */
static uint8_t sum;            /* 체크섬 계산 중인 값 */

int proto_feed(uint8_t byte, proto_packet_t *out)
{
  switch (state) {
  case WAIT_HEAD1:
    if (byte == PROTO_HEAD1) {
      state = WAIT_HEAD2;
    }
    break;

  case WAIT_HEAD2:
    if (byte == PROTO_HEAD2) {
      state = WAIT_CMD;
    } else if (byte == PROTO_HEAD1) {
      state = WAIT_HEAD2;      /* AA AA 55 처럼 와도 놓치지 않게 */
    } else {
      state = WAIT_HEAD1;
    }
    break;

  case WAIT_CMD:
    pkt.cmd = byte;
    sum = byte;
    state = WAIT_LEN;
    break;

  case WAIT_LEN:
    if (byte > PROTO_MAX_DATA) {
      state = WAIT_HEAD1;      /* 말이 안 되는 길이: 버리고 처음부터 */
      g_proto_bad++;
      break;
    }
    pkt.len = byte;
    sum += byte;
    got = 0;
    if (byte == 0) {
      state = WAIT_CHK;
    } else {
      state = WAIT_DATA;
    }
    break;

  case WAIT_DATA:
    pkt.data[got] = byte;
    got++;
    sum += byte;
    if (got >= pkt.len) {
      state = WAIT_CHK;
    }
    break;

  case WAIT_CHK:
    state = WAIT_HEAD1;
    if (byte == sum) {
      *out = pkt;
      g_proto_ok++;
      return 1;
    }
    g_proto_bad++;
    break;

  default:
    state = WAIT_HEAD1;
    break;
  }
  return 0;
}

uint16_t proto_build(uint8_t cmd, const uint8_t *data, uint8_t len, uint8_t *buf)
{
  uint8_t s = 0;
  uint16_t n = 0;

  buf[n] = PROTO_HEAD1;  n++;
  buf[n] = PROTO_HEAD2;  n++;
  buf[n] = cmd;          n++;  s += cmd;
  buf[n] = len;          n++;  s += len;
  for (uint8_t i = 0; i < len; i++) {
    buf[n] = data[i];
    n++;
    s += data[i];
  }
  buf[n] = s;            n++;
  return n;
}
