/*
 * mx64.c - MX-64(2.0) 모터 제어
 *
 * dxl.c 의 읽기/쓰기 함수로 Control Table 의 정해진 주소에 값을 쓴다.
 */
#include "mx64.h"
#include "dxl.h"

#define MAX_MOTORS 8   /* mx64_set_speeds 로 한 번에 보낼 수 있는 모터 수 */

int mx64_failed(int ret)
{
  if (ret < 0) {
    return 1;                   /* 통신 실패 */
  }
  if ((ret & 0x7F) != 0) {
    return 1;                   /* 모터가 명령을 거부함 */
  }
  return 0;
}

int mx64_torque(uint8_t id, uint8_t on)
{
  uint8_t value = 0;
  if (on) {
    value = 1;
  }
  return dxl_write_u8(id, MX64_ADDR_TORQUE_ENABLE, value);
}

int mx64_set_wheel_mode(uint8_t id)
{
  uint8_t mode;
  int ret;

  /* 1) 토크 OFF. Operating Mode 는 EEPROM 이라 토크가 켜져 있으면 못 바꾼다 */
  ret = mx64_torque(id, 0);
  if (mx64_failed(ret)) {
    return ret;
  }

  /* 2) 지금 모드를 읽어서, 속도 모드가 아닐 때만 바꾼다.
        EEPROM 은 쓸 수 있는 횟수에 한계가 있어서 매번 쓰지 않는다. */
  ret = dxl_read(id, MX64_ADDR_OPERATING_MODE, 1, &mode);
  if (mx64_failed(ret)) {
    return ret;
  }
  if (mode != MX64_MODE_VELOCITY) {
    ret = dxl_write_u8(id, MX64_ADDR_OPERATING_MODE, MX64_MODE_VELOCITY);
    if (mx64_failed(ret)) {
      return ret;
    }
  }

  /* 3) 토크를 켜자마자 튀어나가지 않도록 속도 0 */
  ret = mx64_set_speed(id, 0);
  if (mx64_failed(ret)) {
    return ret;
  }

  /* 4) 토크 ON */
  return mx64_torque(id, 1);
}

int mx64_set_speed(uint8_t id, int32_t speed)
{
  /* 음수도 4바이트 그대로 보내면 모터가 음수로 알아듣는다 */
  return dxl_write_u32(id, MX64_ADDR_GOAL_VELOCITY, (uint32_t)speed);
}

int mx64_set_speeds(const uint8_t *ids, const int32_t *speeds, uint8_t count)
{
  /* 모터마다 4바이트씩 이어 붙인다: [모터0 속도 4바이트][모터1 속도 4바이트] */
  uint8_t data[MAX_MOTORS * 4];

  if (count > MAX_MOTORS) {
    return DXL_ERR_ARG;
  }

  for (uint8_t i = 0; i < count; i++) {
    uint32_t v = (uint32_t)speeds[i];
    data[i * 4 + 0] = (uint8_t)(v & 0xFF);
    data[i * 4 + 1] = (uint8_t)((v >> 8) & 0xFF);
    data[i * 4 + 2] = (uint8_t)((v >> 16) & 0xFF);
    data[i * 4 + 3] = (uint8_t)(v >> 24);
  }
  return dxl_sync_write(MX64_ADDR_GOAL_VELOCITY, 4, ids, data, count);
}
