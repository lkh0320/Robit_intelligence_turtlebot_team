/*
 * proto.h - 젯슨 <-> STM32 패킷 규칙
 *
 * ================= 패킷 모양 =================
 *
 *   [0xAA] [0x55] [CMD] [LEN] [DATA ... LEN 바이트] [CHK]
 *
 *   CHK = (CMD + LEN + DATA 모든 바이트) 의 합의 아래 8비트
 *   2바이트 숫자는 작은 자리 먼저 (little endian), 부호 있는 정수
 *
 * ================= 젯슨 -> STM32 =================
 *
 *   CMD 0x01  바퀴 속도 명령 (LEN = 4)   * 팀 stm 패키지 (protocol.hpp WHEEL_CMD) 와 같음
 *     DATA[0..1] : 왼쪽 바퀴 선속도   int16, 단위 mm/s  (앞 = +)
 *     DATA[2..3] : 오른쪽 바퀴 선속도 int16, 단위 mm/s  (앞 = +)
 *     예) 0.1 m/s 직진     -> 100, 100
 *         제자리 왼쪽 회전 -> -50, 50
 *   0.3초 안에 다음 명령이 안 오면 STM32 가 스스로 정지한다.
 *   그래서 젯슨은 멈춰 있을 때도 20Hz 정도로 계속 보내야 한다.
 *
 * ================= STM32 -> 젯슨 =================
 *
 *   CMD 0x81  상태 (LEN = 11), 0.1초마다
 *     DATA[0] : 모터 준비 (1 = 준비됨)
 *     DATA[1] : 상태 (PROTO_STATE_*)
 *     DATA[2] : 켜진 스위치 (bit0 = S1 ... bit3 = S4)
 *     DATA[3] : 왼쪽 모터 하드웨어 에러
 *     DATA[4] : 오른쪽 모터 하드웨어 에러
 *     DATA[5..6] : 모터 전압 (0.1V 단위, uint16)
 *     DATA[7] : 찾은 모터 수
 *     DATA[8] : 왼쪽 모터 ID
 *     DATA[9] : 오른쪽 모터 ID
 *     DATA[10] : 토크 켜짐 (bit0 = 왼쪽, bit1 = 오른쪽)
 */
#ifndef PROTO_H
#define PROTO_H

#include <stdint.h>

#define PROTO_HEAD1 0xAA
#define PROTO_HEAD2 0x55

#define PROTO_CMD_WHEEL  0x01
#define PROTO_CMD_STATUS 0x81

#define PROTO_MAX_DATA 16
#define PROTO_STATUS_LEN 11

/* 상태 값 */
#define PROTO_STATE_STOP_SW     0   /* S1 꺼짐 -> 정지 */
#define PROTO_STATE_ROS         1   /* 젯슨 바퀴 명령대로 주행 중 */
#define PROTO_STATE_ESTOP       2   /* S2 켜짐 -> 비상 정지 */
#define PROTO_STATE_NO_CMD      3   /* 젯슨 명령이 0.3초 넘게 없음 -> 정지 */
#define PROTO_STATE_NOT_READY   4   /* 모터 준비 안 됨 */
#define PROTO_STATE_MANUAL      5   /* 스위치 고정 속도 테스트 */

/* 받은 패킷 하나 */
typedef struct {
  uint8_t cmd;
  uint8_t len;
  uint8_t data[PROTO_MAX_DATA];
} proto_packet_t;

/*
 * 받은 바이트를 하나씩 넣는다.
 * 패킷 하나가 완성되고 체크섬이 맞으면 out 에 채우고 1 을 돌려준다.
 */
int proto_feed(uint8_t byte, proto_packet_t *out);

/* 패킷을 만들어 buf 에 넣고 전체 길이를 돌려준다 (buf 는 LEN + 5 바이트 이상) */
uint16_t proto_build(uint8_t cmd, const uint8_t *data, uint8_t len, uint8_t *buf);

/* 디버깅용 */
extern volatile uint32_t g_proto_ok;       /* 제대로 받은 패킷 수 */
extern volatile uint32_t g_proto_bad;      /* 체크섬 틀린 패킷 수 */

#endif /* PROTO_H */
