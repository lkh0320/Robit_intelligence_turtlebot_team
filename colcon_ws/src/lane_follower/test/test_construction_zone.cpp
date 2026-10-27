// construction_zone.hpp: 표지판 -> 박스 가까움 -> 빈 공간 조향 -> 옆 PSD 로 통과 3개 -> 차선 복귀
#include <gtest/gtest.h>

#include "lane_follower/construction_zone.hpp"

using lane_follower::ConstructionParams;
using lane_follower::ConstructionZone;
using Phase = ConstructionZone::Phase;

namespace
{
constexpr int SIGN = lane_follower::ZONE_SIGN_CONSTRUCTION;

// 시각 now 부터 dur 초 동안 20Hz 로 PSD (left, right) + 10Hz wall + 20Hz 제어 틱
struct Sim
{
  ConstructionZone z;
  ConstructionParams p;
  double now = 0;
  bool box = true;
  double bottom = 0.5, free_offset = 0.0, confidence = 1.0;
  double x_ratio = 0.0;
  char line_side = 0;   // wall_obstacle side (붙은 선)
  bool lane_ok = true;
  ConstructionZone::Cmd last;

  void run(double dur, double left = 0.8, double right = 0.8)
  {
    const double end = now + dur;
    int k = 0;
    while (now < end - 1e-9) {
      now += 0.05;
      z.onPsd(left, right, now, p);
      if (k++ % 2 == 0) {
        z.onWall(box, bottom, free_offset, confidence, now, p, x_ratio, line_side);
      }
      last = z.update(lane_ok, now, p);
    }
  }

  void arm()
  {
    z.onSign(SIGN, 0.9, 0.01, now, p);
  }
};
}  // namespace

TEST(ConstructionZone, BoxWithoutSignDoesNothing)
{
  Sim s;
  s.bottom = 0.8;
  s.run(2.0);
  EXPECT_EQ(s.z.phase(), Phase::IDLE);
  EXPECT_FALSE(s.last.active);
}

TEST(ConstructionZone, SignNeedsConfirmUnlessConfident)
{
  Sim s;
  s.z.onSign(SIGN, 0.7, 0.01, 0, s.p);
  EXPECT_EQ(s.z.phase(), Phase::IDLE);
  s.z.onSign(SIGN, 0.7, 0.01, 0.2, s.p);
  EXPECT_EQ(s.z.phase(), Phase::ARMED);

  Sim t;
  t.z.onSign(SIGN, 0.9, 0.01, 0, t.p);
  EXPECT_EQ(t.z.phase(), Phase::ARMED);

  Sim u;
  u.z.onSign(1, 0.95, 0.05, 0, u.p);   // LEFT 표지판
  u.z.onSign(SIGN, 0.95, 0.001, 0, u.p);   // 너무 작음
  EXPECT_EQ(u.z.phase(), Phase::IDLE);
}

TEST(ConstructionZone, FarBoxKeepsLaneFollowing)
{
  Sim s;
  s.arm();
  s.bottom = 0.2;
  s.run(2.0);
  EXPECT_EQ(s.z.phase(), Phase::ARMED);
  EXPECT_FALSE(s.last.active);
}

TEST(ConstructionZone, NearBoxSteersToFreeSpace)
{
  Sim s;
  s.p.line_mode = false;   // planner 방식
  s.arm();
  s.bottom = 0.35;
  s.free_offset = -0.4;   // 빈 공간이 왼쪽
  s.run(0.5);
  EXPECT_EQ(s.z.phase(), Phase::AVOID);
  ASSERT_TRUE(s.last.active);
  EXPECT_NEAR(s.last.v, s.p.linear, 1e-9);
  EXPECT_NEAR(s.last.w, 0.4, 1e-9);   // 왼쪽 = +
  s.free_offset = 0.9;
  s.run(0.2);
  EXPECT_NEAR(s.last.w, -s.p.max_angular, 1e-9);
}

TEST(ConstructionZone, BoxAndLaneLostStartsAvoid)
{
  Sim s;
  s.arm();
  s.bottom = 0.15;
  s.lane_ok = false;
  s.run(0.3);
  EXPECT_EQ(s.z.phase(), Phase::AVOID);
}

// 녹화 110713_manual 의 옆 PSD 순서 (RLR, 튀는 값 포함) -> 3개
TEST(ConstructionZone, RecordedPsdCountsThree)
{
  Sim s;
  s.arm();
  s.bottom = 0.2;
  s.run(1.0);
  s.run(0.8, 0.17, 0.8);   // 표지판 받침대가 왼쪽 (AVOID 전이라 안 셈)
  s.run(1.0);
  s.bottom = 0.35;
  s.run(1.0);
  ASSERT_EQ(s.z.phase(), Phase::AVOID);
  s.run(0.1, 0.8, 0.2);    // 0.1초 튀는 값
  s.run(2.9);
  EXPECT_EQ(s.z.passes(), 0);
  s.run(1.1, 0.8, 0.12);   // 첫 박스 오른쪽
  s.run(0.5, 0.12, 0.12);  // 두 번째 박스 왼쪽이 겹쳐 시작
  s.run(1.0, 0.10, 0.8);
  EXPECT_EQ(s.z.passes(), 1);
  s.run(0.35, 0.8, 0.8);   // 0.35초 열렸다 다시 막힘 -> 아직 통과 아님
  s.run(0.5, 0.15, 0.8);
  s.run(1.0, 0.4, 0.8);    // 사이 값
  EXPECT_EQ(s.z.passes(), 1);
  s.run(0.3, 0.4, 0.15);   // 세 번째 박스 오른쪽
  s.run(0.3, 0.8, 0.15);
  s.run(0.15, 0.8, 0.8);   // 0.15초 튐
  s.run(0.8, 0.8, 0.12);
  EXPECT_EQ(s.z.passes(), 2);
  s.box = false;
  s.lane_ok = false;
  s.run(0.6);
  EXPECT_EQ(s.z.passes(), 3);
  EXPECT_EQ(s.z.phase(), Phase::RETURN);
  EXPECT_TRUE(s.last.active);
  s.lane_ok = true;
  s.run(0.3);
  EXPECT_EQ(s.z.phase(), Phase::IDLE);
  EXPECT_FALSE(s.last.active);
}

TEST(ConstructionZone, SideBlockedAtStartIsIgnoredUntilOpen)
{
  Sim s;
  s.arm();
  s.bottom = 0.35;
  s.run(0.5, 0.1, 0.8);    // AVOID 시작 때 이미 왼쪽 막힘
  ASSERT_EQ(s.z.phase(), Phase::AVOID);
  s.run(1.0, 0.8, 0.8);
  EXPECT_EQ(s.z.passes(), 0);
  s.run(0.5, 0.1, 0.8);
  s.run(0.6, 0.8, 0.8);
  EXPECT_EQ(s.z.passes(), 1);
}

TEST(ConstructionZone, NoBoxAfterPassReturns)
{
  Sim s;
  s.arm();
  s.bottom = 0.35;
  s.run(0.5);
  s.run(1.0, 0.1, 0.8);
  s.run(0.6);
  ASSERT_EQ(s.z.passes(), 1);
  s.box = false;
  s.run(s.p.no_box_time + 0.2);
  EXPECT_NE(s.z.phase(), Phase::AVOID);
}

TEST(ConstructionZone, WallStaleStops)
{
  Sim s;
  s.p.line_mode = false;   // planner 방식
  s.arm();
  s.bottom = 0.35;
  s.free_offset = 0.5;
  s.run(0.5);
  ASSERT_EQ(s.z.phase(), Phase::AVOID);
  s.now += 1.0;   // wall_obstacle 안 옴
  const auto c = s.z.update(true, s.now, s.p);
  EXPECT_TRUE(c.active);
  EXPECT_EQ(c.v, 0.0);
  EXPECT_EQ(c.w, 0.0);
}

TEST(ConstructionZone, ArmTimesOutAndRearmWaits)
{
  Sim s;
  s.arm();
  s.bottom = 0.1;
  s.run(s.p.arm_max_time + 0.5);
  EXPECT_EQ(s.z.phase(), Phase::IDLE);
  s.arm();   // 끝난 직후 같은 표지판
  EXPECT_EQ(s.z.phase(), Phase::IDLE);
  s.run(s.p.rearm_time);
  s.arm();
  EXPECT_EQ(s.z.phase(), Phase::ARMED);
}

TEST(ConstructionZone, AvoidMaxTimeEnds)
{
  Sim s;
  s.arm();
  s.bottom = 0.35;
  s.run(s.p.avoid_max_time + 1.0);
  EXPECT_EQ(s.z.phase(), Phase::IDLE);
}

TEST(ConstructionZone, CancelForgets)
{
  Sim s;
  s.arm();
  s.bottom = 0.35;
  s.run(0.5);
  s.z.cancel();
  EXPECT_EQ(s.z.phase(), Phase::IDLE);
  s.run(0.2);
  EXPECT_FALSE(s.last.active);
}

TEST(ConstructionZone, SameSideTwiceQuicklyCountsOnce)
{
  // 10-14 실주행: 박스를 긁으며 왼쪽 PSD 가 멀어졌다 다시 가까워짐 -> 한 박스를 두 번 셈
  Sim s;
  s.arm();
  s.bottom = 0.35;
  s.run(0.5);
  ASSERT_EQ(s.z.phase(), Phase::AVOID);
  s.bottom = 0.2;
  s.run(1.0, 0.1, 0.8);
  s.run(0.6);
  ASSERT_EQ(s.z.passes(), 1);
  s.run(0.5, 0.1, 0.8);    // 1.1초 만에 또 왼쪽
  s.run(0.6);
  EXPECT_EQ(s.z.passes(), 1);
  s.run(1.0, 0.8, 0.1);    // 오른쪽 박스
  s.run(0.6);
  EXPECT_EQ(s.z.passes(), 2);
  s.run(2.0);
  s.run(1.0, 0.1, 0.8);    // 충분히 뒤 왼쪽 박스
  s.run(0.6);
  EXPECT_EQ(s.z.passes(), 3);
}

TEST(ConstructionZone, NearBoxKeepsAvoidingAfterCount)
{
  // 3개를 셌어도 가까운 박스가 보이면 차선으로 돌아가지 않는다 (잘못 센 경우 마지막 박스에 직진하던 문제)
  Sim s;
  s.arm();
  s.bottom = 0.35;
  s.run(0.5);
  s.p.same_side_gap = 0;
  for (int i = 0; i < 3; ++i) {
    s.run(1.0, 0.1, 0.8);
    s.run(0.6);
  }
  ASSERT_EQ(s.z.passes(), 3);
  EXPECT_EQ(s.z.phase(), Phase::AVOID);
  EXPECT_TRUE(s.last.active);
  s.lane_ok = false;       // 차선 아직 못 잡음 (잡으면 바로 끝남)
  s.bottom = 0.2;
  s.run(0.3);
  EXPECT_EQ(s.z.phase(), Phase::RETURN);
  s.bottom = 0.4;          // 차선 찾는 중 박스가 또 가까움
  s.run(0.3);
  EXPECT_EQ(s.z.phase(), Phase::AVOID);
}

TEST(ConstructionZone, LowConfidenceSlows)
{
  Sim s;
  s.p.line_mode = false;   // planner 방식
  s.arm();
  s.bottom = 0.35;
  s.free_offset = 0.2;
  s.run(0.5);
  ASSERT_TRUE(s.last.active);
  EXPECT_DOUBLE_EQ(s.last.v, s.p.linear);
  EXPECT_NEAR(s.last.w, -0.2 * s.p.kp, 1e-9);
  s.confidence = 0.3;
  s.run(0.2);
  EXPECT_DOUBLE_EQ(s.last.v, s.p.linear * s.p.slow_ratio);
}

// line 방식: 박스 쪽은 화면 위치가 아니라 붙은 선 (005115_line: 오른쪽 박스가 커브 때문에 화면 왼쪽에 보임)
TEST(ConstructionZone, LineModeUsesAttachedLineNotScreenSide)
{
  Sim s;
  s.arm();
  s.bottom = 0.35;
  s.x_ratio = -0.7;      // 화면 왼쪽
  s.line_side = 'R';     // 흰 선에 붙음
  s.run(1.0);
  ASSERT_EQ(s.z.phase(), Phase::AVOID);
  ASSERT_TRUE(s.last.lane_bias);
  EXPECT_NEAR(s.last.bias, s.p.line_shift, 1e-9);   // 왼쪽(노란 선) 으로
  // 붙은 선을 모르면 (선이 안 보임) 화면 위치로 바꾸지 않는다
  s.line_side = 0;
  s.x_ratio = 0.7;
  s.run(1.0);
  EXPECT_NEAR(s.last.bias, s.p.line_shift, 1e-9);
}

// line 방식에서 차선을 놓치면 직진 대신 경로 계획 방향 (011237_side: 직진으로 트랙 밖)
TEST(ConstructionZone, LineModeLaneLostFollowsPlanner)
{
  Sim s;
  s.p.offset_alpha = 1.0;   // yaml 값 (필터 끔)
  s.arm();
  s.bottom = 0.35;
  s.line_side = 'R';
  s.run(0.5);
  ASSERT_EQ(s.z.phase(), Phase::AVOID);
  s.lane_ok = false;
  s.free_offset = -1.0;   // 경로가 왼쪽
  s.run(0.3);
  EXPECT_FALSE(s.last.lane_bias);
  EXPECT_NEAR(s.last.v, s.p.linear * s.p.line_lost_ratio, 1e-9);
  EXPECT_NEAR(s.last.w, s.p.max_angular, 1e-9);   // 왼쪽 = +
  s.p.line_lost_steer = false;
  s.run(0.1);
  EXPECT_EQ(s.last.w, 0.0);
}
