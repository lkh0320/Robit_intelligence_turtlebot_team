#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "lane_follower/intersection_turn.hpp"

namespace lf = lane_follower;
using Phase = lf::IntersectionTurn::Phase;

namespace
{

// 명령대로 dt 동안 움직였다고 치고 한 틱
lf::IntersectionTurn::Cmd stepObs(lf::IntersectionTurn & s, const lf::IntersectionTurn::LaneObs & lane,
  const lf::TurnParams & p = {}, double dt = 0.05)
{
  const auto c = s.update(lane, p, dt);
  s.onMove(c.active ? c.v * dt : 0.0, c.active ? c.w * dt : 0.0, p);
  return c;
}

// ok 면 두 선 다 보이는 차선 (conf 0.9)
lf::IntersectionTurn::Cmd step(lf::IntersectionTurn & s, bool lost, bool ok, double angle,
  const lf::TurnParams & p = {}, double dt = 0.05)
{
  lf::IntersectionTurn::LaneObs l;
  l.lost = lost;
  l.ok = ok;
  l.confidence = ok ? 0.9 : 0.0;
  l.angle = angle;
  l.left = l.right = ok;
  return stepObs(s, l, p, dt);
}

// 한 선만 보이는 차선 (conf 0.6). yellow = 왼쪽(노란) 선, 아니면 오른쪽(흰) 선
lf::IntersectionTurn::LaneObs oneLine(bool yellow, double angle, double offset = 0.0)
{
  lf::IntersectionTurn::LaneObs l;
  l.ok = true;
  l.confidence = 0.6;
  l.angle = angle;
  l.offset = offset;
  l.left = yellow;
  l.right = !yellow;
  return l;
}

lf::IntersectionTurn::LaneObs lostLane()
{
  lf::IntersectionTurn::LaneObs l;
  l.lost = true;
  return l;
}

// 표지판 방향으로 회전을 시작해 yaw [rad] 만큼 (차선 없이) 돈다. 실제로 돈 양을 돌려줌
double turnTo(lf::IntersectionTurn & s, int sign, double yaw, const lf::TurnParams & p, double dt = 0.05)
{
  s.onSign(sign, 0.95, 0.03, p);
  double turned = 0;
  while (turned < yaw) {
    const auto c = stepObs(s, lostLane(), p, dt);
    if (!c.active) {
      break;
    }
    turned += std::abs(c.w) * dt;
  }
  return turned;
}

// 예전 동작 (바로 제자리 회전, 출구 없음, 가로 선 조건 없음): 회전 자체만 보는 테스트용
lf::TurnParams turnOnly()
{
  lf::TurnParams p;
  p.approach_time = 0;
  p.out_enabled = false;
  p.turn_pivot = false;
  p.require_cross_line = false;
  p.after_enabled = false;
  return p;
}

// 가로 선 조건 없이 기본값 (출구 / 직진 단계 테스트용)
lf::TurnParams noCross()
{
  lf::TurnParams p;
  p.require_cross_line = false;
  p.after_enabled = false;
  return p;
}

// 출구에서 예전처럼 제자리 회전 (out_follow false)
lf::TurnParams exitTurn()
{
  lf::TurnParams p = noCross();
  p.out_follow = false;
  return p;
}

}  // namespace

TEST(IntersectionTurn, NoSignNoManeuver)
{
  lf::IntersectionTurn s;
  for (int i = 0; i < 50; ++i) {
    EXPECT_FALSE(step(s, true, false, 0).active);
  }
  EXPECT_EQ(s.phase(), Phase::IDLE);
}

TEST(IntersectionTurn, LeftSignThenLostLaneTurnsLeftInPlace)
{
  const lf::TurnParams p = turnOnly();
  lf::IntersectionTurn s;
  s.onSign(lf::TURN_SIGN_LEFT, 0.8, 0.03, p);
  EXPECT_EQ(s.phase(), Phase::IDLE);   // 0.8 한 번은 부족
  s.onSign(lf::TURN_SIGN_LEFT, 0.8, 0.03, p);
  ASSERT_EQ(s.phase(), Phase::ARMED);
  EXPECT_EQ(s.direction(), 1);

  // 차선 잡는 동안은 관여 안 함
  EXPECT_FALSE(step(s, false, true, 0, p).active);
  // 차선 놓침 -> 바로 제자리에서 왼쪽으로 (직진 안 함)
  auto c = step(s, true, false, 0, p);
  EXPECT_TRUE(c.active);
  ASSERT_EQ(s.phase(), Phase::TURN);
  EXPECT_DOUBLE_EQ(c.v, 0.0);
  EXPECT_GT(c.w, 0.0);
  int n = 0;

  // 회전 초반에 차선이 보여도 turn_min_yaw 전엔 안 끝냄
  c = step(s, false, true, 0.1, p);
  EXPECT_TRUE(c.active);
  // 계속 돌다가 충분히 돈 뒤 차선을 3번 연속 잡으면 끝
  n = 0;
  while (s.phase() == Phase::TURN && n++ < 1000) {
    step(s, false, true, 0.1, p);
  }
  EXPECT_EQ(s.phase(), Phase::IDLE);
  EXPECT_LT(n, 100);
}

TEST(IntersectionTurn, RightSignTurnsRight)
{
  lf::IntersectionTurn s;
  s.onSign(lf::TURN_SIGN_RIGHT, 0.95, 0.03, turnOnly());
  ASSERT_EQ(s.phase(), Phase::ARMED);
  const auto c = step(s, true, false, 0, turnOnly());
  EXPECT_EQ(s.phase(), Phase::TURN);
  EXPECT_LT(c.w, 0.0);
}

TEST(IntersectionTurn, GivesUpAfterMaxYaw)
{
  const lf::TurnParams p = turnOnly();
  lf::IntersectionTurn s;
  s.onSign(lf::TURN_SIGN_LEFT, 0.95, 0.03, p);
  int n = 0;
  while (n++ < 2000) {
    if (!step(s, true, false, 0, p).active) {
      break;
    }
  }
  EXPECT_EQ(s.phase(), Phase::IDLE);
  // 0.35rad / 0.5 = 0.7s (14틱) + 1.85rad / 0.3 = 6.2s (124틱) = 약 139틱
  EXPECT_NEAR(n, 139, 3);
}

TEST(IntersectionTurn, TiltedLaneDoesNotEndTurn)
{
  lf::IntersectionTurn s;
  const auto p = turnOnly();
  s.onSign(lf::TURN_SIGN_LEFT, 0.95, 0.03, p);
  step(s, true, false, 0, p);
  ASSERT_EQ(s.phase(), Phase::TURN);
  // 차선은 보이지만 아직 비스듬하면 계속 돈다 -> 결국 max yaw 로 포기
  int n = 0;
  while (s.phase() == Phase::TURN && n++ < 1000) {
    step(s, false, true, 0.9, p);
  }
  EXPECT_EQ(s.phase(), Phase::IDLE);
  EXPECT_GT(n, 60);
}

TEST(IntersectionTurn, LaterSignWinsAndOldSignExpires)
{
  const lf::TurnParams p = turnOnly();
  lf::IntersectionTurn s;
  s.onSign(lf::TURN_SIGN_RIGHT, 0.92, 0.17, p);   // 멀티탭 오인식 (test_024032 27.7s)
  EXPECT_EQ(s.direction(), -1);
  s.onMove(0.5, 0, p);
  s.onSign(lf::TURN_SIGN_LEFT, 0.92, 0.033, p);   // 진짜 표지판
  EXPECT_EQ(s.direction(), 1);
  s.onMove(p.arm_max_dist + 0.1, 0, p);
  EXPECT_EQ(s.phase(), Phase::IDLE);
}

TEST(IntersectionTurn, OppositeSignEarlyInTurnSwitchesDirection)
{
  // test_030522 230s: 멀티탭 RIGHT 로 ARMED -> 차선 놓침 -> 0.06s 뒤 진짜 LEFT 확정
  const lf::TurnParams p = turnOnly();
  lf::IntersectionTurn s;
  s.onSign(lf::TURN_SIGN_RIGHT, 0.91, 0.187, p);
  auto c = step(s, true, false, 0, p);
  ASSERT_EQ(s.phase(), Phase::TURN);
  EXPECT_LT(c.w, 0.0);
  s.onSign(lf::TURN_SIGN_LEFT, 0.92, 0.029, p);
  s.onSign(lf::TURN_SIGN_LEFT, 0.94, 0.041, p);
  EXPECT_EQ(s.direction(), 1);
  c = step(s, true, false, 0, p);
  EXPECT_GT(c.w, 0.0);
}

TEST(IntersectionTurn, OppositeSignLateInTurnIgnored)
{
  const lf::TurnParams p = turnOnly();
  lf::IntersectionTurn s;
  s.onSign(lf::TURN_SIGN_LEFT, 0.95, 0.03, p);
  step(s, true, false, 0, p);
  for (int i = 0; i < 40; ++i) {   // 0.75 rad 돎 (turn_switch_max_yaw 0.6 넘음)
    step(s, true, false, 0, p);
  }
  s.onSign(lf::TURN_SIGN_RIGHT, 0.95, 0.1, p);
  EXPECT_EQ(s.direction(), 1);
  EXPECT_EQ(s.phase(), Phase::TURN);
}

TEST(IntersectionTurn, WeakOrSmallSignsIgnored)
{
  const lf::TurnParams p = turnOnly();
  lf::IntersectionTurn s;
  // test_024032 152~160s: 0.5~0.79 좌/우 섞인 검출 -> 확정 안 됨
  s.onSign(lf::TURN_SIGN_LEFT, 0.56, 0.049, p);
  s.onSign(lf::TURN_SIGN_LEFT, 0.52, 0.015, p);
  s.onSign(lf::TURN_SIGN_RIGHT, 0.61, 0.017, p);
  s.onSign(lf::TURN_SIGN_RIGHT, 0.79, 0.024, p);
  s.onSign(lf::TURN_SIGN_LEFT, 0.53, 0.011, p);
  s.onSign(lf::TURN_SIGN_LEFT, 0.95, 0.004, p);   // 너무 작음
  EXPECT_EQ(s.phase(), Phase::IDLE);
}

TEST(IntersectionTurn, SameSignAfterTurnIgnoredUntilRearm)
{
  const lf::TurnParams p = turnOnly();
  lf::IntersectionTurn s;
  s.onSign(lf::TURN_SIGN_LEFT, 0.95, 0.03, p);
  for (int i = 0; i < 2000 && (s.phase() != Phase::IDLE || i == 0); ++i) {
    step(s, s.phase() != Phase::TURN, s.phase() == Phase::TURN, 0.0, p);
  }
  ASSERT_EQ(s.phase(), Phase::IDLE);
  s.onSign(lf::TURN_SIGN_LEFT, 0.95, 0.1, p);
  EXPECT_EQ(s.phase(), Phase::IDLE);
  s.onMove(p.rearm_dist + 0.01, 0, p);
  s.onSign(lf::TURN_SIGN_LEFT, 0.95, 0.1, p);
  EXPECT_EQ(s.phase(), Phase::ARMED);
}

TEST(IntersectionTurn, BothLinesEndTurnEarly)
{
  // test_041619 22.9s: 왼쪽으로 44° 돌았을 때 노란 + 흰 선 (conf 0.9, angle -0.2) = 올바른 차선.
  // turn_min_yaw 75° 였을 땐 이걸 무시하고 93° 까지 돌아 흰 선 하나를 잡고 이탈
  const lf::TurnParams p = turnOnly();
  lf::IntersectionTurn s;
  const double turned = turnTo(s, lf::TURN_SIGN_LEFT, 0.77, p);
  ASSERT_EQ(s.phase(), Phase::TURN);
  for (int i = 0; i < 3; ++i) {
    step(s, false, true, -0.2, p);
  }
  EXPECT_EQ(s.phase(), Phase::IDLE);
  EXPECT_LT(turned, 0.85);
}

TEST(IntersectionTurn, LeadAngleEndsTurnBeforeFullyAligned)
{
  // test_054541 45.0s: 왼쪽으로 돌다 두 선이 보일 때 차선이 아직 왼쪽으로 0.72 rad
  lf::TurnParams p = turnOnly();
  lf::IntersectionTurn s;
  turnTo(s, lf::TURN_SIGN_LEFT, 0.5, p);
  for (int i = 0; i < 4; ++i) {
    step(s, false, true, -0.7, p);   // 기본 exit_lead_angle 0.5 -> 계속 돎
  }
  EXPECT_EQ(s.phase(), Phase::TURN);
  p.exit_lead_angle = 0.75;
  for (int i = 0; i < 3; ++i) {
    step(s, false, true, -0.7, p);
  }
  EXPECT_EQ(s.phase(), Phase::IDLE);
  // 지나친 쪽 (+0.7) 은 exit_max_angle 0.5 넘어서 안 끝냄
  lf::IntersectionTurn s2;
  turnTo(s2, lf::TURN_SIGN_LEFT, 0.5, p);
  for (int i = 0; i < 4; ++i) {
    step(s2, false, true, 0.7, p);
  }
  EXPECT_EQ(s2.phase(), Phase::TURN);
}

TEST(IntersectionTurn, NoExitBeforeMinYaw)
{
  // 돌기 시작할 땐 놓친 차선 / 교차로 가로 선이 아직 보인다
  const lf::TurnParams p = turnOnly();
  lf::IntersectionTurn s;
  turnTo(s, lf::TURN_SIGN_LEFT, 0.1, p);
  for (int i = 0; i < 5; ++i) {
    EXPECT_TRUE(step(s, false, true, 0.0, p).active);
  }
  EXPECT_EQ(s.phase(), Phase::TURN);
}

TEST(IntersectionTurn, RightTurnEndsOnWhiteLine)
{
  // test_041619 110.7s: 오른쪽으로 38° 돌았을 때 흰 선만 (conf 0.6, angle -0.35 ~ -0.40), 섬 노란 선은 화면 밖
  const lf::TurnParams p = turnOnly();
  lf::IntersectionTurn s;
  turnTo(s, lf::TURN_SIGN_RIGHT, 0.62, p);
  ASSERT_EQ(s.phase(), Phase::TURN);
  for (int i = 0; i < 3; ++i) {
    stepObs(s, oneLine(false, -0.35), p);
  }
  EXPECT_EQ(s.phase(), Phase::IDLE);
}

TEST(IntersectionTurn, RightTurnIgnoresYellowOnly)
{
  // test_041619 112.2s: 오른쪽으로 83° 돌아 노란 선만 -> 그걸로 끝내 섬 쪽으로 꺾임
  const lf::TurnParams p = turnOnly();
  lf::IntersectionTurn s;
  turnTo(s, lf::TURN_SIGN_RIGHT, 1.4, p);
  for (int i = 0; i < 10; ++i) {
    EXPECT_TRUE(stepObs(s, oneLine(true, 0.1), p).active);
  }
  EXPECT_EQ(s.phase(), Phase::TURN);
}

TEST(IntersectionTurn, LeftTurnIgnoresWhiteOnlyThenEndsOnYellow)
{
  // test_030522 30.3s: 왼쪽으로 56° 돌아 흰 선만 (angle +0.2) 보고 끝내 이탈
  const lf::TurnParams p = turnOnly();
  lf::IntersectionTurn s;
  turnTo(s, lf::TURN_SIGN_LEFT, 0.95, p);
  for (int i = 0; i < 10; ++i) {
    EXPECT_TRUE(stepObs(s, oneLine(false, 0.2), p).active);
  }
  ASSERT_EQ(s.phase(), Phase::TURN);
  for (int i = 0; i < 3; ++i) {
    stepObs(s, oneLine(true, 0.1), p);
  }
  EXPECT_EQ(s.phase(), Phase::IDLE);
}

TEST(IntersectionTurn, AnyLineEndsTurnWhenSideCheckOff)
{
  lf::TurnParams p = turnOnly();
  p.exit_turn_side_only = false;
  lf::IntersectionTurn s;
  turnTo(s, lf::TURN_SIGN_LEFT, 0.5, p);
  for (int i = 0; i < 3; ++i) {
    stepObs(s, oneLine(false, 0.2), p);
  }
  EXPECT_EQ(s.phase(), Phase::IDLE);
}

TEST(IntersectionTurn, SameLaneFrameCountedOnce)
{
  // 제어 틱(20Hz)이 lane_info(약 15Hz)보다 빨라 같은 프레임을 여러 번 받는다
  const lf::TurnParams p = turnOnly();
  lf::IntersectionTurn s;
  turnTo(s, lf::TURN_SIGN_LEFT, 0.5, p);
  auto l = oneLine(true, 0.0);
  stepObs(s, l, p);
  l.fresh = false;
  for (int i = 0; i < 5; ++i) {
    stepObs(s, l, p);
  }
  EXPECT_EQ(s.phase(), Phase::TURN);
  l.fresh = true;
  stepObs(s, l, p);
  stepObs(s, l, p);
  EXPECT_EQ(s.phase(), Phase::IDLE);
}

TEST(IntersectionTurn, LeavingDriveModeForgetsSign)
{
  // test_041619 66s: ARMED LEFT -> GUI 로 멈추고 손으로 옮김 -> 104s 재출발 뒤 그 LEFT 로 교차로 진입
  const lf::TurnParams p = turnOnly();
  lf::IntersectionTurn s;
  s.onSign(lf::TURN_SIGN_LEFT, 0.95, 0.03, p);
  ASSERT_EQ(s.phase(), Phase::ARMED);
  s.cancelManeuver();
  EXPECT_EQ(s.phase(), Phase::IDLE);
  EXPECT_FALSE(step(s, true, false, 0, p).active);
  // 멈추기 전에 본 표지판은 확정 횟수에도 안 셈
  s.onSign(lf::TURN_SIGN_LEFT, 0.8, 0.03, p);
  s.cancelManeuver();
  s.onSign(lf::TURN_SIGN_LEFT, 0.8, 0.03, p);
  EXPECT_EQ(s.phase(), Phase::IDLE);
}

TEST(IntersectionTurn, CancelManeuverKeepsNothingRunning)
{
  lf::IntersectionTurn s;
  s.onSign(lf::TURN_SIGN_LEFT, 0.95, 0.03, turnOnly());
  step(s, true, false, 0, turnOnly());
  ASSERT_EQ(s.phase(), Phase::TURN);
  s.cancelManeuver();
  EXPECT_EQ(s.phase(), Phase::IDLE);
}

TEST(IntersectionTurn, Disabled)
{
  lf::TurnParams p;
  p.enabled = false;
  lf::IntersectionTurn s;
  s.onSign(lf::TURN_SIGN_LEFT, 0.95, 0.03, p);
  EXPECT_FALSE(step(s, true, false, 0, p).active);
}

TEST(IntersectionTurn, ApproachStraightThenTurnSlowerWhileSearching)
{
  lf::TurnParams p;
  p.require_cross_line = false;
  p.out_enabled = false;
  lf::IntersectionTurn s;
  s.onSign(lf::TURN_SIGN_RIGHT, 0.95, 0.03, p);
  // 차선 놓침 -> 0.5s (10틱) 직진
  for (int i = 0; i < 10; ++i) {
    const auto c = step(s, true, false, 0, p);
    ASSERT_TRUE(c.active);
    EXPECT_EQ(s.phase(), Phase::APPROACH);
    EXPECT_DOUBLE_EQ(c.v, p.approach_linear);
    EXPECT_DOUBLE_EQ(c.w, 0.0);
  }
  // 그 다음 제자리 회전, turn_min_yaw 까지는 turn_angular
  auto c = step(s, true, false, 0, p);
  EXPECT_EQ(s.phase(), Phase::TURN);
  EXPECT_DOUBLE_EQ(c.v, 0.0);
  EXPECT_DOUBLE_EQ(c.w, -p.turn_angular);
  for (int i = 0; i < 60; ++i) {
    c = step(s, true, false, 0, p);
  }
  // turn_min_yaw 넘게 돈 뒤엔 천천히
  EXPECT_DOUBLE_EQ(c.w, -p.turn_search_angular);
  EXPECT_DOUBLE_EQ(c.v, 0.0);
}

TEST(IntersectionTurn, PivotTurnStopsInnerWheel)
{
  // 안쪽 바퀴 = v - |w| L / 2 = 0, 바깥 바퀴 = v + |w| L / 2 = |w| L
  for (const int sign : {lf::TURN_SIGN_LEFT, lf::TURN_SIGN_RIGHT}) {
    lf::TurnParams p;
    p.require_cross_line = false;
    p.approach_time = 0;
    p.turn_pivot = true;
    lf::IntersectionTurn s;
    s.onSign(sign, 0.95, 0.03, p);
    const auto c = step(s, true, false, 0, p);
    ASSERT_EQ(s.phase(), Phase::TURN);
    const double left = c.v - c.w * p.wheel_separation / 2;
    const double right = c.v + c.w * p.wheel_separation / 2;
    if (sign == lf::TURN_SIGN_LEFT) {
      EXPECT_NEAR(left, 0.0, 1e-9);
      EXPECT_NEAR(right, p.turn_angular * p.wheel_separation, 1e-9);
    } else {
      EXPECT_NEAR(right, 0.0, 1e-9);
      EXPECT_NEAR(left, p.turn_angular * p.wheel_separation, 1e-9);
    }
  }
}

TEST(IntersectionTurn, OppositeSignDuringApproachSwitchesDirection)
{
  lf::TurnParams p;
  p.require_cross_line = false;
  p.out_enabled = false;
  lf::IntersectionTurn s;
  s.onSign(lf::TURN_SIGN_RIGHT, 0.95, 0.187, p);
  step(s, true, false, 0, p);
  ASSERT_EQ(s.phase(), Phase::APPROACH);
  s.onSign(lf::TURN_SIGN_LEFT, 0.95, 0.03, p);
  EXPECT_EQ(s.direction(), 1);
  EXPECT_EQ(s.phase(), Phase::APPROACH);
}

namespace
{
// 왼쪽으로 돌고 차선을 잡을 때까지 (LOOP 로 들어갈 때까지) 진행
void turnLeftIntoLoop(lf::IntersectionTurn & s, const lf::TurnParams & p)
{
  s.onSign(lf::TURN_SIGN_LEFT, 0.95, 0.03, p);
  for (int i = 0; i < 2000 && s.phase() != Phase::LOOP && s.phase() != Phase::IDLE; ++i) {
    const bool turning = s.phase() == Phase::TURN;
    step(s, !turning, turning, 0.0, p);
  }
}
}  // namespace

TEST(IntersectionTurn, LeftTurnWaitsForLoopExitThenTurnsLeftAgain)
{
  const lf::TurnParams p = exitTurn();
  lf::IntersectionTurn s;
  turnLeftIntoLoop(s, p);
  ASSERT_EQ(s.phase(), Phase::LOOP);
  // 고리를 따라 차선 추종 (명령 없음). 섬 안 표지판이 다시 보여도 무시
  s.onMove(0.3, -0.5, p);
  s.onSign(lf::TURN_SIGN_RIGHT, 0.95, 0.05, p);
  s.onSign(lf::TURN_SIGN_RIGHT, 0.95, 0.05, p);
  EXPECT_EQ(s.phase(), Phase::LOOP);
  EXPECT_FALSE(step(s, false, true, 0.0, p).active);
  // 오른쪽으로 계속 꺾여 out_trigger_yaw (1.05) 넘으면 출구 -> 왼쪽으로 돎
  for (int i = 0; i < 2; ++i) {   // 합 1.0 rad
    s.onMove(0.12, -0.25, p);
    EXPECT_FALSE(step(s, false, true, 0.0, p).active);
  }
  s.onMove(0.12, -0.25, p);   // 합 1.25 rad
  auto c = step(s, false, true, 0.0, p);
  ASSERT_TRUE(c.active);
  EXPECT_EQ(s.phase(), Phase::TURN);   // out_approach_time 0 -> 바로 회전
  EXPECT_GT(c.w, 0.0);
  // out_turn_min_yaw (0.6) 돌고 차선 잡으면 끝, 다시 LOOP 로 가지 않음
  int n = 0;
  while (s.phase() == Phase::TURN && n++ < 1000) {
    step(s, false, true, 0.0, p);
  }
  EXPECT_EQ(s.phase(), Phase::IDLE);
  EXPECT_LT(n, 45);
}

TEST(IntersectionTurn, LoopLaneLostTriggersExitTurn)
{
  const lf::TurnParams p = exitTurn();
  lf::IntersectionTurn s;
  turnLeftIntoLoop(s, p);
  ASSERT_EQ(s.phase(), Phase::LOOP);
  // out_min_dist 전엔 차선을 놓쳐도 출구로 안 봄
  s.onMove(0.2, 0, p);
  EXPECT_FALSE(step(s, true, false, 0, p).active);
  EXPECT_EQ(s.phase(), Phase::LOOP);
  s.onMove(0.5, -1.0, p);
  const auto c = step(s, true, false, 0, p);
  EXPECT_TRUE(c.active);
  EXPECT_GT(c.w, 0.0);
}

TEST(IntersectionTurn, LoopIslandYellowOnRightTriggersExitTurn)
{
  // test_044608 128.9s / 91.3s: 고리 출구에서 섬 끝 노란 선만 보이고 offset +1 (노란 선이 로봇 오른쪽)
  const lf::TurnParams p = exitTurn();
  lf::IntersectionTurn s;
  turnLeftIntoLoop(s, p);
  ASSERT_EQ(s.phase(), Phase::LOOP);
  s.onMove(0.1, -0.2, p);
  EXPECT_FALSE(stepObs(s, oneLine(true, 0.0, 1.0), p).active);   // out_min_dist 전
  s.onMove(0.3, 0.0, p);
  // 고리를 덜 돌았으면 (test_044608 1번: 노란 선을 넘어 밖으로 나감) 섬으로 안 봄
  EXPECT_FALSE(stepObs(s, oneLine(true, 0.0, 1.0), p).active);
  s.onMove(0.05, -0.6, p);
  // 왼쪽에 있는 노란 선은 그냥 따라감
  EXPECT_FALSE(stepObs(s, oneLine(true, 0.2, 0.4), p).active);
  EXPECT_EQ(s.phase(), Phase::LOOP);
  const auto c = stepObs(s, oneLine(true, 0.0, 1.0), p);
  ASSERT_TRUE(c.active);
  EXPECT_EQ(s.phase(), Phase::TURN);
  EXPECT_GT(c.w, 0.0);
  EXPECT_NE(s.lastReason().find("섬 끝"), std::string::npos);   // 왜 돌았는지 로그에 남음
}

TEST(IntersectionTurn, ExitTurnSkipsLaneOutsideRobot)
{
  // 출구로 도는 중에도 섬 끝 노란 선 (offset +1) 은 차선으로 안 봄
  const lf::TurnParams p = exitTurn();
  lf::IntersectionTurn s;
  turnLeftIntoLoop(s, p);
  s.onMove(0.4, -1.2, p);
  step(s, false, true, 0.0, p);
  ASSERT_EQ(s.phase(), Phase::TURN);
  for (int i = 0; i < 40; ++i) {   // out_turn_min_yaw 넘게 돎
    stepObs(s, lostLane(), p);
  }
  for (int i = 0; i < 5; ++i) {
    EXPECT_TRUE(stepObs(s, oneLine(true, 0.0, 1.0), p).active);
  }
  EXPECT_EQ(s.phase(), Phase::TURN);
  for (int i = 0; i < 3; ++i) {
    step(s, false, true, 0.1, p);
  }
  EXPECT_EQ(s.phase(), Phase::IDLE);
}

TEST(IntersectionTurn, LaneLostWithoutCrossLineDoesNotTurn)
{
  // test_052007 45.7s: 진입 곡선에서 이미 돈 뒤 차선을 잠깐 놓침, 가로 선은 안 보임 -> 돌지 않음
  lf::TurnParams p;
  p.approach_time = 0;
  lf::IntersectionTurn s;
  s.onSign(lf::TURN_SIGN_LEFT, 0.95, 0.03, p);
  for (int i = 0; i < 10; ++i) {
    EXPECT_FALSE(step(s, true, false, 0, p).active);
  }
  EXPECT_EQ(s.phase(), Phase::ARMED);
  EXPECT_NE(s.lastReason().find("가로 선 없음"), std::string::npos);
  // 오래전에 본 가로 선도 안 셈
  s.onCrossLine(true);
  for (int i = 0; i < 25; ++i) {   // 1.25s 차선 잡고 달림
    step(s, false, true, 0, p);
  }
  EXPECT_FALSE(step(s, true, false, 0, p).active);
  EXPECT_EQ(s.phase(), Phase::ARMED);
}

TEST(IntersectionTurn, CrossLineThenLaneLostTurns)
{
  // test_052007 131.3s / test_044608: T자 가로 선이 보이고 0.3초 안에 차선 놓침 -> 돎
  lf::TurnParams p;
  p.approach_time = 0;
  lf::IntersectionTurn s;
  s.onSign(lf::TURN_SIGN_LEFT, 0.95, 0.03, p);
  s.onCrossLine(true);
  for (int i = 0; i < 6; ++i) {
    step(s, false, true, 0, p);
  }
  auto c = step(s, true, false, 0, p);
  EXPECT_TRUE(c.active);
  EXPECT_EQ(s.phase(), Phase::ALIGN);   // 이미 수평 (0°) -> 제자리에서 확인 프레임 기다림
  EXPECT_DOUBLE_EQ(c.w, 0.0);
  EXPECT_NE(s.lastReason().find("가로 선"), std::string::npos);
  s.onCrossLine(true, 0.01);
  c = step(s, true, false, 0, p);
  EXPECT_EQ(s.phase(), Phase::TURN);
  EXPECT_NE(s.lastReason().find("수평"), std::string::npos);
}

TEST(IntersectionTurn, AlignRotatesUntilCrossLineLevelThenPivots)
{
  // 가로 선 오른쪽 끝이 가까움 (+0.2 rad = 로봇이 왼쪽으로 틀어짐) -> 오른쪽으로 돌아 수평 맞춘 뒤 한쪽 바퀴로 왼쪽 회전
  lf::TurnParams p;
  p.approach_time = 0;
  p.turn_pivot = true;
  lf::IntersectionTurn s;
  s.onSign(lf::TURN_SIGN_LEFT, 0.95, 0.03, p);
  s.onCrossLine(true, 0.2);
  auto c = step(s, true, false, 0, p);
  ASSERT_EQ(s.phase(), Phase::ALIGN);
  EXPECT_DOUBLE_EQ(c.v, 0.0);
  EXPECT_DOUBLE_EQ(c.w, -p.align_max_angular);   // -1.5 * 0.2 = -0.3
  s.onCrossLine(true, -0.06);                      // 조금 지나침 -> 왼쪽으로 천천히 (최소 속도)
  c = step(s, true, false, 0, p);
  EXPECT_DOUBLE_EQ(c.w, p.align_min_angular);
  EXPECT_EQ(s.phase(), Phase::ALIGN);
  s.onCrossLine(true, 0.02);
  step(s, true, false, 0, p);
  s.onCrossLine(true, -0.01);
  c = step(s, true, false, 0, p);
  ASSERT_EQ(s.phase(), Phase::TURN);
  EXPECT_GT(c.w, 0.0);
  EXPECT_NEAR(c.v - c.w * p.wheel_separation / 2, 0.0, 1e-9);   // 왼쪽 바퀴 멈춤
}

TEST(IntersectionTurn, CrossAssistOnlyWhileArmedAndCrossLineFresh)
{
  lf::TurnParams p;
  lf::IntersectionTurn s;
  double w = 0, v = 0;
  s.onCrossLine(true, 0.1);
  EXPECT_FALSE(s.crossAssist(p, w, v));   // 표지판 전엔 안 함
  s.onSign(lf::TURN_SIGN_LEFT, 0.95, 0.03, p);
  ASSERT_TRUE(s.crossAssist(p, w, v));
  EXPECT_DOUBLE_EQ(w, -0.2);               // 오른쪽 끝이 가까움 (로봇이 왼쪽으로 틀어짐) -> 오른쪽으로
  EXPECT_DOUBLE_EQ(v, p.approach_slow_linear);
  s.onCrossLine(true, -0.5);
  ASSERT_TRUE(s.crossAssist(p, w, v));
  EXPECT_DOUBLE_EQ(w, p.approach_align_max);
  for (int i = 0; i < 8; ++i) {            // 0.4s 동안 가로 선 안 보임
    step(s, false, true, 0, p);
  }
  EXPECT_FALSE(s.crossAssist(p, w, v));
}

TEST(IntersectionTurn, AlignGivesUpWhenCrossLineGoneOrTimeout)
{
  lf::TurnParams p;
  p.approach_time = 0;
  lf::IntersectionTurn s;
  s.onSign(lf::TURN_SIGN_LEFT, 0.95, 0.03, p);
  s.onCrossLine(true, 0.1);
  step(s, true, false, 0, p);
  ASSERT_EQ(s.phase(), Phase::ALIGN);
  for (int i = 0; i < 12; ++i) {   // 0.6s 동안 가로 선 안 보임
    step(s, true, false, 0, p);
  }
  EXPECT_EQ(s.phase(), Phase::TURN);
  EXPECT_NE(s.lastReason().find("안 보임"), std::string::npos);

  lf::IntersectionTurn s2;
  s2.onSign(lf::TURN_SIGN_LEFT, 0.95, 0.03, p);
  s2.onCrossLine(true, 0.1);
  for (int i = 0; i < 70 && s2.phase() != Phase::TURN; ++i) {   // 계속 기울어 보임 -> align_max_time
    step(s2, true, false, 0, p);
    s2.onCrossLine(true, 0.1);
  }
  EXPECT_EQ(s2.phase(), Phase::TURN);
  EXPECT_NE(s2.lastReason().find("수평 아님"), std::string::npos);
}

TEST(IntersectionTurn, CrossLineSeenJustAfterLaneLostTurns)
{
  // test_044608 15.4s: 차선 놓친 0.1초 뒤에 가로 선 검출
  lf::TurnParams p;
  p.approach_time = 0;
  lf::IntersectionTurn s;
  s.onSign(lf::TURN_SIGN_LEFT, 0.95, 0.03, p);
  EXPECT_FALSE(step(s, true, false, 0, p).active);
  s.onCrossLine(true);
  EXPECT_TRUE(step(s, true, false, 0, p).active);
}

TEST(IntersectionTurn, ExitFollowsLaneButArcsLeftOnIslandLane)
{
  // test_054541 / test_044608: 출구(고리 60°)에서 제자리로 돌지 않고 차선을 따라간다.
  // 섬 끝 노란 선 (offset +1) 이나 차선 없음이면 왼쪽으로 완만한 호
  const lf::TurnParams p = noCross();
  lf::IntersectionTurn s;
  turnLeftIntoLoop(s, p);
  ASSERT_EQ(s.phase(), Phase::LOOP);
  s.onMove(0.4, -1.1, p);
  EXPECT_FALSE(stepObs(s, oneLine(false, 0.0, 0.2), p).active);   // 흰 ∩ 선 차선 -> 그대로 추종
  EXPECT_EQ(s.phase(), Phase::EXIT);
  EXPECT_NE(s.lastReason().find("차선 따라"), std::string::npos);
  auto c = stepObs(s, oneLine(true, 0.0, 1.0), p);                 // 섬 끝 노란 선 -> 호
  ASSERT_TRUE(c.active);
  EXPECT_DOUBLE_EQ(c.v, p.out_arc_linear);
  EXPECT_DOUBLE_EQ(c.w, p.out_arc_angular);
  c = stepObs(s, lostLane(), p);                                   // 차선 없음 -> 호
  EXPECT_TRUE(c.active);
  EXPECT_FALSE(stepObs(s, oneLine(true, 0.1, 0.1), p).active);     // 왼쪽 노란 선 -> 다시 추종
  // out_arc_max_dist 지나면 차선을 놓쳐도 호로 꺾지 않고 직전 차선 추종 명령 유지 (hold),
  // out_hold_time 넘게 못 찾으면 직진
  s.onMove(p.out_arc_max_dist, 0.0, p);
  c = stepObs(s, lostLane(), p);
  ASSERT_TRUE(c.active);
  EXPECT_TRUE(c.hold);
  EXPECT_NE(s.lastReason().find("직전 조향"), std::string::npos);
  for (int i = 0; i < 20; ++i) {   // 1s
    c = stepObs(s, lostLane(), p);
  }
  EXPECT_FALSE(c.hold);
  EXPECT_DOUBLE_EQ(c.w, 0.0);
  EXPECT_NE(s.lastReason().find("직진"), std::string::npos);
  EXPECT_FALSE(stepObs(s, oneLine(true, 0.1, 0.1), p).active);     // 다시 찾으면 추종
  // out_guard_dist 가면 끝, 표지판은 다시 rearm_dist 동안 무시
  s.onMove(p.out_guard_dist - p.out_arc_max_dist, 0.0, p);
  EXPECT_FALSE(stepObs(s, oneLine(true, 0.0, 1.0), p).active);
  EXPECT_EQ(s.phase(), Phase::IDLE);
  s.onSign(lf::TURN_SIGN_LEFT, 0.95, 0.05, p);
  EXPECT_EQ(s.phase(), Phase::IDLE);
}

TEST(IntersectionTurn, ExitKeepsArcUntilFollowedOnce)
{
  // 출구에서 차선을 한 번도 못 따라간 채 out_arc_max_dist 를 넘으면 직전 조향(고리 쪽) 대신 계속 호
  const lf::TurnParams p = noCross();
  lf::IntersectionTurn s;
  turnLeftIntoLoop(s, p);
  s.onMove(0.4, -1.1, p);
  auto c = stepObs(s, lostLane(), p);
  ASSERT_EQ(s.phase(), Phase::EXIT);
  s.onMove(p.out_arc_max_dist + 0.05, 0.0, p);
  c = stepObs(s, lostLane(), p);
  EXPECT_TRUE(c.active);
  EXPECT_FALSE(c.hold);
  EXPECT_DOUBLE_EQ(c.w, p.out_arc_angular);
}

TEST(IntersectionTurn, RightTurnRetriesWhenLaneLostSoonAfter)
{
  // 07:37: 오른쪽 30° 에서 회전 끝 -> 차선 추종 중 바로 놓침 -> 멈춤. 이제 같은 쪽으로 더 돌며 찾는다
  lf::TurnParams p = noCross();
  p.approach_time = 0;
  p.after_enabled = true;
  lf::IntersectionTurn s;
  turnTo(s, lf::TURN_SIGN_RIGHT, 0.5, p);
  for (int i = 0; i < p.exit_ok_count; ++i) {
    step(s, false, true, 0.4, p);   // 차선이 아직 오른쪽으로 0.4 rad (exit_lead_angle 안)
  }
  ASSERT_EQ(s.phase(), Phase::AFTER);
  EXPECT_FALSE(step(s, false, true, 0.3, p).active);   // 차선 추종
  s.onMove(0.1, 0.0, p);
  auto c = step(s, true, false, 0, p);                  // 놓침 -> 오른쪽으로 더 돎 (찾는 속도)
  ASSERT_EQ(s.phase(), Phase::TURN);
  EXPECT_DOUBLE_EQ(c.w, -p.turn_search_angular);
  EXPECT_NE(s.lastReason().find("더 돎"), std::string::npos);
  for (int i = 0; i < p.exit_ok_count; ++i) {
    step(s, false, true, 0.1, p);
  }
  EXPECT_EQ(s.phase(), Phase::AFTER);
  // 끝까지 잘 따라가면 IDLE
  s.onMove(p.after_dist, 0.0, p);
  EXPECT_FALSE(step(s, false, true, 0.0, p).active);
  EXPECT_EQ(s.phase(), Phase::IDLE);
}

TEST(IntersectionTurn, AfterRetryGivesUpAtMaxYawAndRetries)
{
  lf::TurnParams p = noCross();
  p.approach_time = 0;
  p.after_enabled = true;
  lf::IntersectionTurn s;
  turnTo(s, lf::TURN_SIGN_RIGHT, 0.5, p);
  for (int i = 0; i < p.exit_ok_count; ++i) {
    step(s, false, true, 0.2, p);
  }
  ASSERT_EQ(s.phase(), Phase::AFTER);
  for (int r = 0; r < p.after_max_retries; ++r) {
    step(s, true, false, 0, p);
    ASSERT_EQ(s.phase(), Phase::TURN);
    for (int i = 0; i < p.exit_ok_count; ++i) {
      step(s, false, true, 0.1, p);
    }
    ASSERT_EQ(s.phase(), Phase::AFTER);
  }
  EXPECT_FALSE(step(s, true, false, 0, p).active);   // 재시도 다 씀 -> lane_follower 가 멈춤
  EXPECT_EQ(s.phase(), Phase::AFTER);
}

TEST(IntersectionTurn, LoopForgetsAfterMaxDist)
{
  const lf::TurnParams p = noCross();
  lf::IntersectionTurn s;
  turnLeftIntoLoop(s, p);
  ASSERT_EQ(s.phase(), Phase::LOOP);
  s.onMove(p.out_max_dist + 0.1, 0, p);
  EXPECT_EQ(s.phase(), Phase::IDLE);
}

TEST(IntersectionTurn, RightTurnHasNoLoopExit)
{
  const lf::TurnParams p = noCross();
  lf::IntersectionTurn s;
  s.onSign(lf::TURN_SIGN_RIGHT, 0.95, 0.03, p);
  for (int i = 0; i < 2000 && (s.phase() != Phase::IDLE || i == 0); ++i) {
    const bool turning = s.phase() == Phase::TURN;
    step(s, !turning, turning, 0.0, p);
  }
  EXPECT_EQ(s.phase(), Phase::IDLE);
}
