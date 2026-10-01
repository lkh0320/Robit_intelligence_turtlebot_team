/*
 * app.c - 스위치로 두 바퀴(MX-64) 를 굴린다
 *
 * ============ 전체 흐름 ============
 *
 *  app_init()  (전원 켜고 한 번)
 *    1. 스위치 핀 준비
 *    2. 모터 찾기   : 여러 통신 속도로 "누구 있어?" (broadcast ping)
 *    3. 모터 설정   : 에러 있으면 재부팅 -> 속도 모드 -> 토크 ON
 *
 *  app_loop()  (while(1) 안에서 계속)
 *    - 젯슨에서 온 패킷 처리 (왼쪽/오른쪽 바퀴 속도 명령 저장)
 *    - 0.1초마다 젯슨에 상태 보고
 *    - 준비 안 됐으면 : 0.5초마다 2~3 다시 시도 (12V 를 늦게 켜도 됨)
 *    - 준비 됐으면     : 0.05초마다 속도 정하기 -> 모터에 전송
 *
 * ============ 어떻게 움직일지 (위에서부터 우선) ============
 *
 *   S2 켜짐                          -> 비상 정지
 *   S1 꺼짐                          -> 정지
 *   젯슨 바퀴 명령이 0.3초 안에 옴   -> 그 명령대로 주행
 *   S3 또는 S4 켜짐                  -> 고정 속도 테스트 (S4 = 1단, S3 = 2단)
 *   그 외 (ROS 명령 끊김)            -> 정지
 *
 * ============ 바퀴 속도 -> MX-64 값 ============
 *
 *   젯슨(팀 stm 패키지)이 cmd_vel 을 왼쪽/오른쪽 바퀴 선속도(mm/s)로 바꿔서 보낸다.
 *   바퀴 각속도 (rad/s) = 바퀴 선속도 (m/s) / r      (r = 바퀴 반지름)
 *   MX-64 값 = rpm / 0.229,   rpm = rad/s * 60 / (2 * pi)
 */
#include "app.h"
#include "main.h"
#include "dxl.h"
#include "mx64.h"
#include "sw.h"
#include "host.h"
#include "proto.h"

/* ==================== 바꿔 쓰는 설정값 ==================== */

/* 속도 (1 = 0.229 rpm) */
#define SPEED_1 50               /* 약 11.5 rpm */
#define SPEED_2 100              /* 약 22.9 rpm */

/* 바퀴 방향. 오른쪽 모터는 반대로 달려 있어서 -1 을 곱해야 같이 앞으로 간다.
   바퀴가 반대로 돌면 부호를 바꾸면 된다. */
#define LEFT_DIR  (+1)
#define RIGHT_DIR (-1)

/* 어느 모터가 왼쪽인지. 1 = ID 가 작은 모터가 왼쪽, 0 = ID 가 큰 모터가 왼쪽.
   teleop 에서 j(왼쪽 회전)를 눌렀는데 오른쪽으로 돌면서 i 는 앞으로 가면
   이 값을 바꾸고 LEFT_DIR / RIGHT_DIR 부호도 서로 바꾼다. */
#define LEFT_IS_SMALLER_ID 1

/* ---- 로봇 치수 (임시값! 실제 로봇에 맞게 바꾸기) ---- */
#define WHEEL_RADIUS_M     0.05f   /* 바퀴 반지름 r (m) */
/* 좌우 바퀴 사이 거리는 젯슨 쪽 (stm_bridge.yaml 의 wheel_separation) 에서 쓴다 */

/* ---- 안전 제한 ---- */
#define MAX_WHEEL_MPS   0.30f      /* 바퀴 선속도 최대 (m/s) */
#define MAX_GOAL        250        /* MX-64 값 최대 (약 57 rpm) */
#define CMD_TIMEOUT_MS  300        /* 이 시간 동안 ROS 명령이 없으면 정지 */

#define STATUS_PERIOD_MS 100       /* 젯슨에 상태 보고하는 주기 */
#define UPDATE_PERIOD_MS 50        /* 속도 명령 보내는 주기 */
#define RETRY_PERIOD_MS  500     /* 모터를 못 찾았을 때 다시 찾는 주기 */
#define CHECK_PERIOD_MS  1000      /* 모터 전압 / 에러 / 토크를 다시 읽는 주기 */

/* 모터를 찾을 때 시도할 통신 속도 (앞에서부터 차례로) */
static const uint32_t bauds[] = { 1000000, 57600, 115200, 2000000, 9600 };
#define NUM_BAUDS (sizeof(bauds) / sizeof(bauds[0]))


/* ==================== 디버거로 보는 변수 ==================== */
/* volatile: 디버거(Live Expressions)에서 항상 최신 값이 보이게 한다 */

volatile int      g_dxl_ready;      /* 1 = 설정 끝, 달리는 중 */
volatile uint32_t g_dxl_baud;       /* 모터를 찾은 통신 속도 (0 = 못 찾음) */
volatile int      g_dxl_count;      /* 찾은 모터 수 */
volatile uint8_t  g_dxl_ids[2];     /* [0] 왼쪽, [1] 오른쪽 모터 ID */
volatile int      g_dxl_err;        /* 설정 실패 이유 (-2 = 응답 없음) */
volatile uint8_t  g_dxl_hw_err[2];  /* 모터에 걸려 있던 하드웨어 에러 */
volatile int      g_dxl_reboots;    /* 에러 때문에 재부팅한 횟수 */
/* 전압 (0.1V 단위, 예: 120 = 12.0V). 에러 1(입력 전압 이상) 확인용 */
volatile uint16_t g_dxl_volt[2];    /* 모터가 측정한 지금 전압 */
volatile uint16_t g_dxl_volt_min[2];/* 모터에 설정된 전압 하한 */
volatile uint16_t g_dxl_volt_max[2];/* 모터에 설정된 전압 상한 */
volatile uint8_t  g_dxl_torque[2];  /* 모터 토크 (1 = 켜짐, 0 = 모터가 스스로 끔) */
volatile uint32_t g_dxl_rx_bytes;   /* 받은 바이트 수 (0 = 배선 문제) */
volatile uint8_t  g_sw_on;          /* 켜진 스위치 (bit0 = S1 ... bit3 = S4) */
volatile int32_t  g_speed;          /* 스위치 테스트 모드 속도 */
volatile int16_t  g_cmd_left_mm;    /* 받은 왼쪽 바퀴 속도 (mm/s) */
volatile int16_t  g_cmd_right_mm;   /* 받은 오른쪽 바퀴 속도 (mm/s) */
volatile int32_t  g_goal[2];        /* 모터에 보내는 값 [0] 왼쪽, [1] 오른쪽 */
volatile uint8_t  g_state;          /* PROTO_STATE_* (proto.h) */


/* ==================== 이 파일 안에서만 쓰는 변수 ==================== */

static uint8_t  ids[2];             /* 찾은 모터 ID ([0] 왼쪽, [1] 오른쪽) */
static int      count;              /* 찾은 모터 수 */
static uint32_t last_tick;          /* 마지막으로 일한 시각 (ms) */
static uint32_t last_status_tick;   /* 마지막으로 상태를 보고한 시각 */
static uint32_t last_check_tick;    /* 마지막으로 모터 상태를 읽은 시각 */
static uint32_t last_cmd_tick;      /* 마지막으로 ROS 명령을 받은 시각 */
static int      has_cmd;            /* ROS 명령을 한 번이라도 받았는지 */


/*
 * 모터 찾기
 * bauds[] 의 속도를 하나씩 바꿔가며 broadcast ping 을 보낸다.
 * 한 대라도 응답하면 그 속도로 정하고 1 을 돌려준다.
 */
static int find_motors(void)
{
  for (unsigned i = 0; i < NUM_BAUDS; i++) {
    dxl_set_baud(bauds[i]);
    count = dxl_scan(ids, 2);

    if (count > 0) {
      /* 왼쪽 모터를 [0] 으로 (LEFT_IS_SMALLER_ID 참고) */
      if (count == 2) {
        int first_is_bigger = (ids[0] > ids[1]);
        if (first_is_bigger == LEFT_IS_SMALLER_ID) {
          uint8_t temp = ids[0];
          ids[0] = ids[1];
          ids[1] = temp;
        }
      }
      g_dxl_baud = bauds[i];
      return 1;
    }
  }
  return 0;
}

/*
 * 모터 토크가 실제로 켜져 있으면 1.
 * (과부하/과열 등으로 모터가 스스로 토크를 끈 상태인지 확인)
 */
static int torque_is_on(uint8_t id)
{
  uint8_t on = 0;
  if (dxl_read(id, MX64_ADDR_TORQUE_ENABLE, 1, &on) < 0) {
    return 0;
  }
  return on == 1;
}

/*
 * 모터 하나 설정
 * 성공하면 0, 실패하면 에러 값을 돌려준다.
 */
static int setup_one_motor(int index)
{
  uint8_t id = ids[index];
  uint8_t hw_error = 0;
  uint8_t buf[2];
  int ret;

  /* 0) 진단용: 전압과 전압 허용 범위를 읽어 둔다 (2바이트, 작은 자리 먼저) */
  if (dxl_read(id, MX64_ADDR_PRESENT_VOLTAGE, 2, buf) >= 0) {
    g_dxl_volt[index] = buf[0] | (buf[1] << 8);
  }
  if (dxl_read(id, MX64_ADDR_MIN_VOLTAGE, 2, buf) >= 0) {
    g_dxl_volt_min[index] = buf[0] | (buf[1] << 8);
  }
  if (dxl_read(id, MX64_ADDR_MAX_VOLTAGE, 2, buf) >= 0) {
    g_dxl_volt_max[index] = buf[0] | (buf[1] << 8);
  }

  /* 1) 바퀴용 설정 (속도 모드 + 토크 ON) */
  ret = mx64_set_wheel_mode(id);
  if (mx64_failed(ret)) {
    return ret;
  }

  /* 2) 에러 표시(Alert)가 있으면 어떤 에러인지 기록한다.
        전압 에러(0x01)처럼 토크를 끄지 않는 에러도 있으므로,
        실패 여부는 "토크가 실제로 켜졌는지"로 판단한다. */
  if (ret & DXL_ALERT) {
    if (dxl_read(id, MX64_ADDR_HARDWARE_ERROR, 1, &hw_error) >= 0) {
      g_dxl_hw_err[index] = hw_error;
    }
    if (torque_is_on(id)) {
      return 0;                  /* 경고는 있지만 움직일 수 있음 */
    }

    /* 3) 토크가 안 켜졌을 때만 재부팅해서 에러를 지우고 한 번 더 시도 */
    dxl_reboot(id);
    g_dxl_reboots++;
    HAL_Delay(1000);             /* 재부팅 끝날 때까지 기다림 */

    ret = mx64_set_wheel_mode(id);
    if (mx64_failed(ret)) {
      return ret;
    }
    if (!torque_is_on(id)) {
      return DXL_ALERT;          /* 재부팅 후에도 토크가 안 켜짐 */
    }
  }
  return 0;
}

/*
 * 모터 찾기 + 모두 설정
 * 다 되면 1, 하나라도 실패하면 0.
 */
static int motors_setup(void)
{
  g_dxl_count = 0;
  g_dxl_baud = 0;

  if (!find_motors()) {
    g_dxl_rx_bytes = dxl_rx_count;
    return 0;
  }
  g_dxl_count = count;

  for (int i = 0; i < count; i++) {
    g_dxl_ids[i] = ids[i];

    int ret = setup_one_motor(i);
    if (ret != 0) {
      g_dxl_err = ret;
      return 0;
    }
    g_dxl_torque[i] = (uint8_t)torque_is_on(ids[i]);
  }

  g_dxl_err = 0;
  return 1;
}

/*
 * 달리는 중에 모터 상태 다시 읽기 (1초마다)
 * 전압, 하드웨어 에러, 토크를 읽어 디버거 변수와 젯슨 상태 보고에 반영한다.
 * 모터가 과부하 등으로 토크를 스스로 껐으면 0 을 돌려준다 -> 다시 설정(재부팅)한다.
 */
static int check_motors(void)
{
  int ok = 1;

  for (int i = 0; i < count; i++) {
    uint8_t buf[2];
    uint8_t value;

    if (dxl_read(ids[i], MX64_ADDR_PRESENT_VOLTAGE, 2, buf) >= 0) {
      g_dxl_volt[i] = buf[0] | (buf[1] << 8);
    }
    if (dxl_read(ids[i], MX64_ADDR_HARDWARE_ERROR, 1, &value) >= 0) {
      g_dxl_hw_err[i] = value;
    }
    if (dxl_read(ids[i], MX64_ADDR_TORQUE_ENABLE, 1, &value) >= 0) {
      g_dxl_torque[i] = value;
      if (value == 0) {
        ok = 0;                  /* 토크가 꺼짐 -> 명령을 보내도 안 돈다 */
      }
    }
  }
  return ok;
}

/*
 * 스위치 상태로 속도를 정한다. 0 이면 정지.
 *   on & SW1  : S1 이 켜져 있으면 참
 *   !(...)    : 반대 (꺼져 있으면 참)
 */
static int32_t speed_from_switches(uint8_t on)
{
  /* S1 이 꺼져 있으면 정지 */
  if (!(on & SW1)) {
    return 0;
  }
  /* S2 가 켜져 있으면 정지 */
  if (on & SW2) {
    return 0;
  }
  /* S4 가 켜져 있으면 1단 (S3 보다 우선) */
  if (on & SW4) {
    return SPEED_1;
  }
  /* S3 가 켜져 있으면 2단 */
  if (on & SW3) {
    return SPEED_2;
  }
  /* 아무것도 없으면 1단 */
  return SPEED_1;
}


/* ==================== 젯슨 통신 ==================== */

/* 받은 바이트를 패킷으로 조립하고, 속도 명령이면 저장한다 */
static void handle_host(void)
{
  uint8_t b;
  proto_packet_t pkt;

  while (host_read(&b)) {
    if (!proto_feed(b, &pkt)) {
      continue;                  /* 아직 패킷이 다 안 모임 */
    }
    if (pkt.cmd == PROTO_CMD_WHEEL && pkt.len == 4) {
      /* 2바이트씩 작은 자리 먼저 -> 부호 있는 16비트 정수 */
      g_cmd_left_mm  = (int16_t)(pkt.data[0] | (pkt.data[1] << 8));
      g_cmd_right_mm = (int16_t)(pkt.data[2] | (pkt.data[3] << 8));
      last_cmd_tick = HAL_GetTick();
      has_cmd = 1;
    }
  }
}

/* 젯슨에 상태 패킷을 보낸다 */
static void send_status(void)
{
  uint8_t data[PROTO_STATUS_LEN];
  uint8_t buf[PROTO_STATUS_LEN + 5];

  data[0] = (uint8_t)g_dxl_ready;
  data[1] = g_state;
  data[2] = g_sw_on;
  data[3] = g_dxl_hw_err[0];
  data[4] = g_dxl_hw_err[1];
  data[5] = (uint8_t)(g_dxl_volt[0] & 0xFF);
  data[6] = (uint8_t)(g_dxl_volt[0] >> 8);
  data[7] = (uint8_t)g_dxl_count;            /* 찾은 모터 수 */
  data[8] = g_dxl_ids[0];                    /* 왼쪽 모터 ID */
  data[9] = g_dxl_ids[1];                    /* 오른쪽 모터 ID */
  data[10] = (uint8_t)(g_dxl_torque[0] | (g_dxl_torque[1] << 1));  /* bit0 왼쪽, bit1 오른쪽 */

  uint16_t n = proto_build(PROTO_CMD_STATUS, data, PROTO_STATUS_LEN, buf);
  host_write(buf, n);
}


/* ==================== 바퀴 속도 -> MX-64 값 ==================== */

/* 바퀴 각속도 (rad/s) -> MX-64 Goal Velocity 값 (소수) */
static float wheel_to_goal(float wheel_rad_s)
{
  const float TWO_PI = 6.2831853f;
  float rpm = wheel_rad_s * 60.0f / TWO_PI;
  return rpm / 0.229f;
}

/* 소수 -> 정수 (반올림) */
static int32_t round_to_int(float value)
{
  if (value >= 0.0f) {
    return (int32_t)(value + 0.5f);
  }
  return (int32_t)(value - 0.5f);
}

/* 절댓값 */
static float absf(float value)
{
  if (value < 0.0f) {
    return -value;
  }
  return value;
}

/* 왼쪽/오른쪽 바퀴 선속도(mm/s) -> 왼쪽/오른쪽 MX-64 값 */
static void wheels_to_goals(int16_t left_mm, int16_t right_mm,
                            int32_t *left, int32_t *right)
{
  float left_mps  = left_mm / 1000.0f;
  float right_mps = right_mm / 1000.0f;

  float left_rad_s  = left_mps / WHEEL_RADIUS_M;
  float right_rad_s = right_mps / WHEEL_RADIUS_M;

  float l = wheel_to_goal(left_rad_s);
  float r = wheel_to_goal(right_rad_s);

  /* 한쪽이라도 최대를 넘으면 두 바퀴를 같은 비율로 줄인다.
     (한쪽만 자르면 도는 모양이 바뀌기 때문) */
  float biggest = absf(l);
  if (absf(r) > biggest) {
    biggest = absf(r);
  }
  float limit = (float)MAX_GOAL;
  float limit_by_speed = wheel_to_goal(MAX_WHEEL_MPS / WHEEL_RADIUS_M);
  if (limit_by_speed < limit) {
    limit = limit_by_speed;
  }
  if (biggest > limit) {
    float scale = limit / biggest;
    l = l * scale;
    r = r * scale;
  }

  *left  = round_to_int(l);
  *right = round_to_int(r);
}

/*
 * 지금 어떻게 움직일지 정한다.
 * left, right 에 "앞으로 가는 방향 기준" 바퀴 값을 넣고 상태를 돌려준다.
 * (모터 방향 LEFT_DIR / RIGHT_DIR 은 보내기 직전에 곱한다)
 */
static uint8_t decide(uint8_t on, uint32_t now, int32_t *left, int32_t *right)
{
  *left = 0;
  *right = 0;

  if (on & SW2) {
    return PROTO_STATE_ESTOP;
  }
  if (!(on & SW1)) {
    return PROTO_STATE_STOP_SW;
  }
  if (has_cmd && (now - last_cmd_tick) < CMD_TIMEOUT_MS) {
    wheels_to_goals(g_cmd_left_mm, g_cmd_right_mm, left, right);
    return PROTO_STATE_ROS;
  }
  if (on & (SW3 | SW4)) {
    g_speed = speed_from_switches(on);
    *left = g_speed;
    *right = g_speed;
    return PROTO_STATE_MANUAL;
  }
  return PROTO_STATE_NO_CMD;
}


/* ==================== main.c 에서 부르는 함수 ==================== */

void app_init(void)
{
  sw_init();
  host_init();                   /* 젯슨 통신 (USART6) */
  dxl_init(USART3);
  HAL_Delay(300);                /* 모터가 전원 켜고 부팅할 시간 */

  g_dxl_ready = motors_setup();
  last_tick = HAL_GetTick();
}

void app_loop(void)
{
  uint32_t now = HAL_GetTick();

  /* ---- 젯슨에서 온 명령 처리 (모터 상태와 상관없이 항상) ---- */
  handle_host();

  /* ---- 0.1초마다 젯슨에 상태 보고 ---- */
  if (now - last_status_tick >= STATUS_PERIOD_MS) {
    last_status_tick = now;
    g_sw_on = sw_on();
    if (!g_dxl_ready) {
      g_state = PROTO_STATE_NOT_READY;
    }
    send_status();
  }

  /* ---- 아직 준비 안 됨: 0.5초마다 다시 시도 ---- */
  if (!g_dxl_ready) {
    if (now - last_tick >= RETRY_PERIOD_MS) {
      g_dxl_ready = motors_setup();
      last_tick = HAL_GetTick();
    }
    return;
  }

  /* ---- 준비 됨: 1초마다 모터 상태 확인. 토크가 꺼졌으면 다시 설정 ---- */
  if (now - last_check_tick >= CHECK_PERIOD_MS) {
    last_check_tick = now;
    if (!check_motors()) {
      g_dxl_ready = 0;           /* 다음 루프에서 motors_setup() 이 재부팅까지 해 준다 */
      return;
    }
  }

  /* ---- 준비 됨: 0.05초마다 속도 정해서 보내기 ---- */
  if (now - last_tick >= UPDATE_PERIOD_MS) {
    last_tick = now;

    int32_t left;
    int32_t right;
    g_sw_on = sw_on();
    g_state = decide(g_sw_on, now, &left, &right);
    g_goal[0] = left;
    g_goal[1] = right;

    int32_t speeds[2];
    speeds[0] = LEFT_DIR * left;       /* 왼쪽 */
    speeds[1] = RIGHT_DIR * right;     /* 오른쪽 */
    mx64_set_speeds(ids, speeds, (uint8_t)count);
  }
}
