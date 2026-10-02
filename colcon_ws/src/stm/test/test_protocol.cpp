// protocol.hpp 단위 테스트 (gtest)
//   실행: colcon test --packages-select stm && colcon test-result --verbose
#include <gtest/gtest.h>

#include <vector>

#include "stm/protocol.hpp"

namespace proto = stm::protocol;

namespace
{
// 바이트열 전체를 파서에 넣고 완성된 프레임 개수를 돌려준다
int feed_all(proto::Parser & p, const std::vector<uint8_t> & bytes)
{
  int frames = 0;
  for (auto b : bytes) {
    frames += p.feed(b);
  }
  return frames;
}
}  // namespace

// 바퀴 명령 프레임이 문서대로 (little-endian, 체크섬 포함) 만들어지는지
TEST(Protocol, EncodeWheelCmd)
{
  // left = -100 (0xFF9C), right = 300 (0x012C)
  const auto f = proto::encode_wheel_cmd(-100, 300);
  const std::vector<uint8_t> expected{
    0xAA, 0x55, 0x01, 0x04, 0x9C, 0xFF, 0x2C, 0x01,
    static_cast<uint8_t>(0x01 + 0x04 + 0x9C + 0xFF + 0x2C + 0x01)};
  EXPECT_EQ(f, expected);
}

// encode 한 프레임을 다시 파싱하면 같은 값이 나오는지
TEST(Protocol, RoundTrip)
{
  proto::Parser p;
  EXPECT_EQ(feed_all(p, proto::encode(proto::ID_PSD, {0x10, 0x27, 0x00, 0x00, 0xFF, 0xFF})), 1);
  EXPECT_EQ(p.frame().id, proto::ID_PSD);
  EXPECT_EQ(proto::get_u16(&p.frame().payload[0]), 10000);
  EXPECT_EQ(proto::get_u16(&p.frame().payload[4]), 65535);
}

// 앞에 쓰레기 바이트가 있어도 그 뒤의 정상 프레임을 찾아내는지
TEST(Protocol, ResyncAfterGarbage)
{
  proto::Parser p;
  std::vector<uint8_t> bytes{0x00, 0xAA, 0xAA, 0x13};  // 잡음 + 끊긴 SYNC
  const auto good = proto::encode(proto::ID_PSD, {1, 2, 3, 4, 5, 6});
  bytes.insert(bytes.end(), good.begin(), good.end());
  EXPECT_EQ(feed_all(p, bytes), 1);
  EXPECT_EQ(p.frame().payload, (std::vector<uint8_t>{1, 2, 3, 4, 5, 6}));
}

// AA 가 두 번 연속 와도 (AA AA 55) 두 번째 AA 부터 프레임으로 인식하는지
TEST(Protocol, DoubleSyncByte)
{
  proto::Parser p;
  auto bytes = proto::encode(proto::ID_PSD, {1, 2, 3, 4, 5, 6});
  bytes.insert(bytes.begin(), 0xAA);  // AA AA 55 ...
  EXPECT_EQ(feed_all(p, bytes), 1);
}

// 체크섬이 틀린 프레임은 버리고 에러로 세는지
TEST(Protocol, BadChecksumRejected)
{
  proto::Parser p;
  auto bytes = proto::encode(proto::ID_PSD, {1, 2, 3, 4, 5, 6});
  bytes.back() ^= 0xFF;
  EXPECT_EQ(feed_all(p, bytes), 0);
  EXPECT_EQ(p.errors(), 1u);
}

// LEN 이 MAX_PAYLOAD 보다 크면 버리고, 그 뒤 정상 프레임은 받는지
TEST(Protocol, OversizedLengthRejected)
{
  proto::Parser p;
  EXPECT_EQ(feed_all(p, {0xAA, 0x55, 0x10, proto::MAX_PAYLOAD + 1}), 0);
  EXPECT_EQ(p.errors(), 1u);
  // 이후 정상 프레임은 받아야 함
  EXPECT_EQ(feed_all(p, proto::encode(proto::ID_PSD, {1, 2, 3, 4, 5, 6})), 1);
}

// 펌웨어 STATUS 프레임 (실제로 받은 바이트)을 풀었을 때 각 칸이 맞는지
TEST(Protocol, DecodeStatus)
{
  // 준비=1, 상태=ROS, 스위치=S1, 에러 0/0, 12.0V(120), 모터 1개, ID 1/0, 토크 왼쪽만
  const std::vector<uint8_t> bytes{
    0xAA, 0x55, 0x81, 0x0B, 0x01, 0x01, 0x01, 0x00, 0x00, 0x78, 0x00, 0x01, 0x01, 0x00, 0x01, 0x0A};
  proto::Parser p;
  ASSERT_EQ(feed_all(p, bytes), 1);
  ASSERT_EQ(p.frame().id, proto::ID_STATUS);
  proto::Status st;
  ASSERT_TRUE(proto::decode_status(p.frame().payload, st));
  EXPECT_TRUE(st.ready);
  EXPECT_EQ(st.state, proto::STATE_ROS);
  EXPECT_EQ(st.switches, 0x01);
  EXPECT_EQ(st.voltage_dv, 120);
  EXPECT_EQ(st.motor_count, 1);
  EXPECT_EQ(st.id[0], 1);
  EXPECT_EQ(st.id[1], 0);
  EXPECT_TRUE(st.torque[0]);
  EXPECT_FALSE(st.torque[1]);

  // 길이가 다르면 거부
  EXPECT_FALSE(proto::decode_status({0x01, 0x02}, st));
}
