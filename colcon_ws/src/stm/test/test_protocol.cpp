#include <gtest/gtest.h>

#include <vector>

#include "stm/protocol.hpp"

namespace proto = stm::protocol;

namespace
{
int feed_all(proto::Parser & p, const std::vector<uint8_t> & bytes)
{
  int frames = 0;
  for (auto b : bytes) {
    frames += p.feed(b);
  }
  return frames;
}
}  // namespace

TEST(Protocol, EncodeWheelCmd)
{
  // left = -100 (0xFF9C), right = 300 (0x012C)
  const auto f = proto::encode_wheel_cmd(-100, 300);
  const std::vector<uint8_t> expected{
    0xAA, 0x55, 0x01, 0x04, 0x9C, 0xFF, 0x2C, 0x01,
    static_cast<uint8_t>(0x01 + 0x04 + 0x9C + 0xFF + 0x2C + 0x01)};
  EXPECT_EQ(f, expected);
}

TEST(Protocol, RoundTrip)
{
  proto::Parser p;
  EXPECT_EQ(feed_all(p, proto::encode(proto::ID_PSD, {0x10, 0x27, 0x00, 0x00, 0xFF, 0xFF})), 1);
  EXPECT_EQ(p.frame().id, proto::ID_PSD);
  EXPECT_EQ(proto::get_u16(&p.frame().payload[0]), 10000);
  EXPECT_EQ(proto::get_u16(&p.frame().payload[4]), 65535);
}

TEST(Protocol, ResyncAfterGarbage)
{
  proto::Parser p;
  std::vector<uint8_t> bytes{0x00, 0xAA, 0xAA, 0x13};  // 잡음 + 끊긴 SYNC
  const auto good = proto::encode(proto::ID_PSD, {1, 2, 3, 4, 5, 6});
  bytes.insert(bytes.end(), good.begin(), good.end());
  EXPECT_EQ(feed_all(p, bytes), 1);
  EXPECT_EQ(p.frame().payload, (std::vector<uint8_t>{1, 2, 3, 4, 5, 6}));
}

TEST(Protocol, DoubleSyncByte)
{
  proto::Parser p;
  auto bytes = proto::encode(proto::ID_PSD, {1, 2, 3, 4, 5, 6});
  bytes.insert(bytes.begin(), 0xAA);  // AA AA 55 ...
  EXPECT_EQ(feed_all(p, bytes), 1);
}

TEST(Protocol, BadChecksumRejected)
{
  proto::Parser p;
  auto bytes = proto::encode(proto::ID_PSD, {1, 2, 3, 4, 5, 6});
  bytes.back() ^= 0xFF;
  EXPECT_EQ(feed_all(p, bytes), 0);
  EXPECT_EQ(p.errors(), 1u);
}

TEST(Protocol, OversizedLengthRejected)
{
  proto::Parser p;
  EXPECT_EQ(feed_all(p, {0xAA, 0x55, 0x10, proto::MAX_PAYLOAD + 1}), 0);
  EXPECT_EQ(p.errors(), 1u);
  // 이후 정상 프레임은 받아야 함
  EXPECT_EQ(feed_all(p, proto::encode(proto::ID_PSD, {1, 2, 3, 4, 5, 6})), 1);
}
