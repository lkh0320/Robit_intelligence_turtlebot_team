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
//   ID 0x10 PSD        LEN 6  uint16 left_mm, uint16 front_mm, uint16 right_mm
//
// STM32 펌웨어는 WHEEL_CMD가 일정 시간(권장 300ms) 안 들어오면 스스로 모터를 정지해야 한다.
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace stm::protocol
{

constexpr uint8_t SYNC1 = 0xAA;
constexpr uint8_t SYNC2 = 0x55;
constexpr uint8_t MAX_PAYLOAD = 32;

constexpr uint8_t ID_WHEEL_CMD = 0x01;
constexpr uint8_t ID_PSD = 0x10;
constexpr uint8_t LEN_WHEEL_CMD = 4;
constexpr uint8_t LEN_PSD = 6;

struct Frame
{
  uint8_t id = 0;
  std::vector<uint8_t> payload;
};

inline uint8_t checksum(uint8_t id, const uint8_t * payload, uint8_t len)
{
  uint8_t sum = id + len;
  for (uint8_t i = 0; i < len; ++i) {
    sum += payload[i];
  }
  return sum;
}

inline std::vector<uint8_t> encode(uint8_t id, const std::vector<uint8_t> & payload)
{
  const auto len = static_cast<uint8_t>(payload.size());
  std::vector<uint8_t> out{SYNC1, SYNC2, id, len};
  out.insert(out.end(), payload.begin(), payload.end());
  out.push_back(checksum(id, payload.data(), len));
  return out;
}

inline void put_i16(std::vector<uint8_t> & buf, int16_t v)
{
  const auto u = static_cast<uint16_t>(v);
  buf.push_back(static_cast<uint8_t>(u & 0xFF));
  buf.push_back(static_cast<uint8_t>(u >> 8));
}

inline uint16_t get_u16(const uint8_t * p)
{
  return static_cast<uint16_t>(p[0] | (p[1] << 8));
}

inline std::vector<uint8_t> encode_wheel_cmd(int16_t left_mm_s, int16_t right_mm_s)
{
  std::vector<uint8_t> payload;
  put_i16(payload, left_mm_s);
  put_i16(payload, right_mm_s);
  return encode(ID_WHEEL_CMD, payload);
}

// 바이트 스트림에서 프레임을 복원하는 상태머신. 깨진 바이트는 버리고 다음 SYNC를 찾는다.
class Parser
{
public:
  // 프레임이 완성되면 true를 반환하고 frame()으로 꺼낼 수 있다.
  bool feed(uint8_t b)
  {
    switch (state_) {
      case State::Sync1:
        if (b == SYNC1) {state_ = State::Sync2;}
        break;
      case State::Sync2:
        if (b == SYNC2) {
          state_ = State::Id;
        } else if (b != SYNC1) {
          state_ = State::Sync1;
        }
        break;
      case State::Id:
        cur_.id = b;
        state_ = State::Len;
        break;
      case State::Len:
        if (b > MAX_PAYLOAD) {
          state_ = State::Sync1;
          ++errors_;
          break;
        }
        len_ = b;
        cur_.payload.clear();
        state_ = len_ == 0 ? State::Chk : State::Payload;
        break;
      case State::Payload:
        cur_.payload.push_back(b);
        if (cur_.payload.size() == len_) {state_ = State::Chk;}
        break;
      case State::Chk:
        state_ = State::Sync1;
        if (b == checksum(cur_.id, cur_.payload.data(), len_)) {
          done_ = cur_;
          return true;
        }
        ++errors_;
        break;
    }
    return false;
  }

  const Frame & frame() const {return done_;}
  size_t errors() const {return errors_;}

private:
  enum class State { Sync1, Sync2, Id, Len, Payload, Chk };
  State state_ = State::Sync1;
  Frame cur_;
  Frame done_;
  uint8_t len_ = 0;
  size_t errors_ = 0;
};

}  // namespace stm::protocol
