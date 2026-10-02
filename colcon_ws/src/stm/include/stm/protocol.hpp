// Jetson <-> STM32 시리얼 프레임 프로토콜
//
// 프레임: [0xAA][0x55][ID][LEN][PAYLOAD x LEN][CHK]
//   CHK = (ID + LEN + PAYLOAD 바이트 합) & 0xFF
//   멀티바이트 값은 little-endian
//
// Jetson -> STM32
//   ID 0x01 WHEEL_CMD  LEN 4  int16 left_mm_s, int16 right_mm_s  (바퀴 선속도 [mm/s])
//
// STM32 -> Jetson
//   ID 0x10 PSD        LEN 6  uint16 left_mm, uint16 front_mm, uint16 right_mm   (0.05초마다)
//   ID 0x81 STATUS     LEN 11 (0.1초마다, 펌웨어 App/proto.h)
//     [0] 모터 준비(1)  [1] 상태(STATE_*)  [2] 스위치 비트(bit0 = S1 ... bit3 = S4)
//     [3] 왼쪽 HW 에러  [4] 오른쪽 HW 에러  [5..6] uint16 전압 [0.1V]
//     [7] 찾은 모터 수  [8] 왼쪽 ID  [9] 오른쪽 ID  [10] 토크 비트 (bit0 왼쪽, bit1 오른쪽)
//
// STM32 펌웨어는 WHEEL_CMD 가 300ms 안 들어오면 스스로 모터를 정지한다.
//
// 예) 왼쪽 -100mm/s, 오른쪽 300mm/s 명령
//     AA 55 | 01 | 04 | 9C FF 2C 01 | CHK
//     sync    ID   LEN  left  right   (-100 = 0xFF9C, 300 = 0x012C 를 낮은 바이트부터)
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace stm::protocol
{

// 프레임 시작 표시 두 바이트. 이 두 바이트가 연속으로 와야 프레임 시작으로 본다.
constexpr uint8_t SYNC1 = 0xAA;
constexpr uint8_t SYNC2 = 0x55;
// LEN 이 이보다 크면 깨진 프레임으로 보고 버린다 (잡음 때문에 엄청 긴 길이를 기다리지 않도록)
constexpr uint8_t MAX_PAYLOAD = 32;

// 메시지 종류(ID)와 각 종류의 payload 길이
constexpr uint8_t ID_WHEEL_CMD = 0x01;
constexpr uint8_t ID_PSD = 0x10;
constexpr uint8_t ID_STATUS = 0x81;
constexpr uint8_t LEN_WHEEL_CMD = 4;
constexpr uint8_t LEN_PSD = 6;
constexpr uint8_t LEN_STATUS = 11;

// STATUS 의 상태 값 (interfaces/DxlState STATE_* 와 같다)
constexpr uint8_t STATE_STOP_SW = 0;     // S1 꺼짐 -> 정지
constexpr uint8_t STATE_ROS = 1;         // 젯슨 바퀴 명령대로 주행 중
constexpr uint8_t STATE_ESTOP = 2;       // S2 켜짐 -> 비상 정지
constexpr uint8_t STATE_NO_CMD = 3;      // 명령이 0.3초 넘게 없음 -> 정지
constexpr uint8_t STATE_NOT_READY = 4;   // 모터 준비 안 됨
constexpr uint8_t STATE_MANUAL = 5;      // 스위치 고정 속도 테스트

// 수신한 프레임 하나 (sync, LEN, CHK 는 검사 후 버리고 ID 와 내용만 남긴다)
struct Frame
{
  uint8_t id = 0;
  std::vector<uint8_t> payload;
};

// 체크섬: ID + LEN + payload 를 모두 더한 값의 하위 1바이트.
// uint8_t 로 더하므로 255 를 넘으면 자동으로 넘쳐서(& 0xFF 와 같은 효과) 하위 바이트만 남는다.
inline uint8_t checksum(uint8_t id, const uint8_t * payload, uint8_t len)
{
  uint8_t sum = id + len;
  for (uint8_t i = 0; i < len; ++i) {
    sum += payload[i];
  }
  return sum;
}

// ID + payload 를 보낼 수 있는 완성된 프레임 바이트열로 만든다.
inline std::vector<uint8_t> encode(uint8_t id, const std::vector<uint8_t> & payload)
{
  const auto len = static_cast<uint8_t>(payload.size());
  std::vector<uint8_t> out{SYNC1, SYNC2, id, len};          // 헤더
  out.insert(out.end(), payload.begin(), payload.end());   // 내용
  out.push_back(checksum(id, payload.data(), len));        // 마지막에 체크섬
  return out;
}

// int16 을 little-endian(낮은 바이트 먼저) 2바이트로 buf 뒤에 붙인다.
// 음수도 비트 그대로 uint16 으로 바꿔 보내고, 받는 쪽(STM32)이 다시 int16 으로 읽는다.
inline void put_i16(std::vector<uint8_t> & buf, int16_t v)
{
  const auto u = static_cast<uint16_t>(v);
  buf.push_back(static_cast<uint8_t>(u & 0xFF));   // 하위 바이트
  buf.push_back(static_cast<uint8_t>(u >> 8));     // 상위 바이트
}

// little-endian 2바이트(p[0] 하위, p[1] 상위)를 uint16 으로 읽는다.
inline uint16_t get_u16(const uint8_t * p)
{
  return static_cast<uint16_t>(p[0] | (p[1] << 8));
}

// 바퀴 속도 명령 프레임 만들기 (단위 mm/s)
inline std::vector<uint8_t> encode_wheel_cmd(int16_t left_mm_s, int16_t right_mm_s)
{
  std::vector<uint8_t> payload;
  put_i16(payload, left_mm_s);
  put_i16(payload, right_mm_s);
  return encode(ID_WHEEL_CMD, payload);
}

// STATUS 프레임 내용
struct Status
{
  bool ready = false;
  uint8_t state = STATE_NOT_READY;
  uint8_t switches = 0;
  uint8_t error[2] = {0, 0};      // [0] 왼쪽, [1] 오른쪽
  uint16_t voltage_dv = 0;        // [0.1V]
  uint8_t motor_count = 0;
  uint8_t id[2] = {0, 0};
  bool torque[2] = {false, false};
};

// STATUS payload -> Status. 길이가 다르면 false
inline bool decode_status(const std::vector<uint8_t> & p, Status & s)
{
  if (p.size() != LEN_STATUS) {
    return false;
  }
  s.ready = p[0] != 0;
  s.state = p[1];
  s.switches = p[2];
  s.error[0] = p[3];
  s.error[1] = p[4];
  s.voltage_dv = get_u16(&p[5]);
  s.motor_count = p[7];
  s.id[0] = p[8];
  s.id[1] = p[9];
  s.torque[0] = (p[10] & 0x01) != 0;
  s.torque[1] = (p[10] & 0x02) != 0;
  return true;
}

// 바이트 스트림에서 프레임을 복원하는 상태머신. 깨진 바이트는 버리고 다음 SYNC를 찾는다.
// 시리얼은 바이트가 끊겨서 들어올 수 있으므로, 받은 바이트를 하나씩 feed() 에 넣으면
// "지금 프레임의 어디까지 받았는지"를 state_ 로 기억하면서 조립한다.
//
//   Sync1 --AA--> Sync2 --55--> Id --> Len --> Payload(LEN개) --> Chk --> (완성) --> Sync1
class Parser
{
public:
  // 프레임이 완성되면 true를 반환하고 frame()으로 꺼낼 수 있다.
  bool feed(uint8_t b)
  {
    switch (state_) {
      case State::Sync1:
        // 0xAA 가 올 때까지 나머지 바이트는 전부 무시
        if (b == SYNC1) {state_ = State::Sync2;}
        break;
      case State::Sync2:
        if (b == SYNC2) {
          state_ = State::Id;
        } else if (b != SYNC1) {
          // AA 다음이 55 가 아니면 처음부터 다시.
          // 단, AA 가 또 오면(AA AA 55 ...) 그 AA 를 새 시작으로 보고 Sync2 상태를 유지한다.
          state_ = State::Sync1;
        }
        break;
      case State::Id:
        cur_.id = b;
        state_ = State::Len;
        break;
      case State::Len:
        if (b > MAX_PAYLOAD) {
          // 말이 안 되는 길이 -> 깨진 프레임, 다시 sync 찾기
          state_ = State::Sync1;
          ++errors_;
          break;
        }
        len_ = b;
        cur_.payload.clear();
        // payload 가 없는 프레임(LEN 0)은 바로 체크섬 단계로
        state_ = len_ == 0 ? State::Chk : State::Payload;
        break;
      case State::Payload:
        cur_.payload.push_back(b);
        if (cur_.payload.size() == len_) {state_ = State::Chk;}
        break;
      case State::Chk:
        // 체크섬이 맞든 틀리든 다음 바이트부터는 새 프레임을 찾는다
        state_ = State::Sync1;
        if (b == checksum(cur_.id, cur_.payload.data(), len_)) {
          done_ = cur_;   // 완성된 프레임을 따로 보관 (cur_ 는 다음 프레임 조립에 재사용)
          return true;
        }
        ++errors_;
        break;
    }
    return false;
  }

  // 마지막으로 완성된 프레임
  const Frame & frame() const {return done_;}
  // 지금까지 버린 깨진 프레임 수 (길이 초과 + 체크섬 불일치)
  size_t errors() const {return errors_;}

private:
  enum class State { Sync1, Sync2, Id, Len, Payload, Chk };
  State state_ = State::Sync1;
  Frame cur_;          // 조립 중인 프레임
  Frame done_;         // 마지막으로 완성된 프레임
  uint8_t len_ = 0;    // 조립 중인 프레임의 LEN
  size_t errors_ = 0;
};

}  // namespace stm::protocol
