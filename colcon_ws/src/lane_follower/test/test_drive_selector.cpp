#include <gtest/gtest.h>

#include <cmath>

#include "lane_follower/drive_selector.hpp"

namespace lf = lane_follower;

namespace
{

const lf::LaneStatus GOOD{true, 0.0, 0.9};
const lf::LaneStatus BAD{false, 0.0, 0.0};

// n 프레임 동안 같은 표지판
void signs(lf::DriveSelector & s, int type, int n, const lf::SelectorParams & p = {})
{
  for (int i = 0; i < n; ++i) {
    s.onSign(type, 0.9, 0.03, p);
  }
}

// 박스 하나를 dist0 에서 보기 시작해 BEV 아래로 사라질 때까지 앞으로 지나간다 (1cm 씩)
void passBox(lf::DriveSelector & s, double dist0, const lf::SelectorParams & p = {})
{
  const double near = 0.141;   // BEV 맨 아래 = 바퀴 축 앞 14.1cm
  for (double d = dist0; d > -0.4; d -= 0.01) {
    if (d > near) {
      s.onBox(true, d, 0.5, p);
    } else if (d > near - 0.12) {   // 박스 길이만큼은 아래에 걸쳐 보임
      s.onBox(true, near, 1.0, p);
    } else {
      s.onBox(false, 0, 0, p);
    }
    s.onMove(0.01, 0, p);
    s.update(BAD, true, p);
  }
}

}  // namespace

TEST(DriveSelector, NormalUsesSliding)
{
  lf::DriveSelector s;
  EXPECT_EQ(s.update(GOOD, true, {}), lf::Source::SLIDING);
  EXPECT_EQ(s.zone(), lf::Zone::NORMAL);
}

TEST(DriveSelector, PathFallbackWhenSlidingLost)
{
  lf::DriveSelector s;
  lf::SelectorParams p;
  EXPECT_EQ(s.update(BAD, true, p), lf::Source::PATH);
  EXPECT_EQ(s.update(BAD, false, p), lf::Source::SLIDING);
  p.path_fallback = false;
  EXPECT_EQ(s.update(BAD, true, p), lf::Source::SLIDING);
}

TEST(DriveSelector, ForkNeedsConfirmedSign)
{
  // 최근 5 프레임 중 3 번 (사이에 한 프레임씩 끊겨도 됨)
  lf::DriveSelector s;
  const lf::SelectorParams p;
  for (const int t : {1, 0, 0, 0, 1, 0, 1}) {
    s.onSign(t, 0.9, 0.03, p);
    EXPECT_EQ(s.zone(), lf::Zone::NORMAL);
  }
  s.onSign(lf::SIGN_LEFT, 0.9, 0.03, p);   // 최근 5: 0 1 0 1 1
  EXPECT_EQ(s.zone(), lf::Zone::FORK);
  EXPECT_EQ(s.forkDir(), -1);
  EXPECT_EQ(s.update(GOOD, true, p), lf::Source::PATH);
}

TEST(DriveSelector, SmallOrWeakSignIgnored)
{
  lf::DriveSelector s;
  lf::SelectorParams p;
  for (int i = 0; i < 10; ++i) {
    s.onSign(lf::SIGN_RIGHT, 0.3, 0.05, p);
    s.onSign(lf::SIGN_RIGHT, 0.9, 0.001, p);
  }
  EXPECT_EQ(s.zone(), lf::Zone::NORMAL);
}

TEST(DriveSelector, ForkDirectionByMajorityWhenSignsAlternate)
{
  lf::DriveSelector s;
  s.onSign(lf::SIGN_RIGHT, 0.9, 0.03, {});
  s.onSign(lf::SIGN_LEFT, 0.9, 0.03, {});
  s.onSign(lf::SIGN_RIGHT, 0.9, 0.03, {});
  EXPECT_EQ(s.zone(), lf::Zone::FORK);
  EXPECT_EQ(s.forkDir(), 1);
}

TEST(DriveSelector, ForkDirectionHeldDuringZone)
{
  // 구간에 들어간 뒤 반대 표지판을 봐도 방향을 안 바꾼다
  lf::DriveSelector s;
  signs(s, lf::SIGN_LEFT, 3);
  signs(s, lf::SIGN_RIGHT, 10);
  EXPECT_EQ(s.forkDir(), -1);
  EXPECT_EQ(s.hint(), lf::SIGN_LEFT);
}

TEST(DriveSelector, HintFollowsZone)
{
  lf::DriveSelector s;
  lf::SelectorParams p;
  EXPECT_EQ(s.hint(), lf::SIGN_NONE);
  signs(s, lf::SIGN_RIGHT, 3);
  EXPECT_EQ(s.hint(), lf::SIGN_RIGHT);
  s.onMove(0.01, -(p.fork_exit_yaw + 0.1), p);
  s.update(BAD, true, p);
  ASSERT_EQ(s.zone(), lf::Zone::RETURNING);
  EXPECT_EQ(s.hint(), lf::SIGN_NONE);   // 돌아 들어간 뒤에는 가점 없이 차선 따라가기
  lf::DriveSelector s2;
  signs(s2, lf::SIGN_CONSTRUCTION, 3);
  EXPECT_EQ(s2.hint(), lf::SIGN_CONSTRUCTION);
}

TEST(DriveSelector, ForkExitsAfterTurningThenReturnsToSliding)
{
  lf::DriveSelector s;
  lf::SelectorParams p;
  signs(s, lf::SIGN_RIGHT, 3);
  ASSERT_EQ(s.zone(), lf::Zone::FORK);
  // 반대(왼쪽)로 도는 건 탈출이 아님
  for (int i = 0; i < 20; ++i) {
    s.onMove(0.01, 0.05, p);
    EXPECT_EQ(s.update(GOOD, true, p), lf::Source::PATH);
  }
  EXPECT_EQ(s.zone(), lf::Zone::FORK);
  // 오른쪽(시계)으로 돌아 들어감: 처음 왼쪽으로 돈 1 rad 를 되돌린 뒤 fork_exit_yaw 더
  for (int i = 0; i < 60 && s.zone() == lf::Zone::FORK; ++i) {
    s.onMove(0.01, -0.05, p);
    s.update(BAD, true, p);
  }
  ASSERT_EQ(s.zone(), lf::Zone::RETURNING);
  // sliding 이 return_ok_count 번 연속 좋아야 복귀
  for (int i = 0; i < p.return_ok_count - 1; ++i) {
    EXPECT_EQ(s.update(GOOD, true, p), lf::Source::PATH);
  }
  EXPECT_EQ(s.update(GOOD, true, p), lf::Source::SLIDING);
  EXPECT_EQ(s.zone(), lf::Zone::NORMAL);
}

TEST(DriveSelector, ReturningWaitsForCenteredSliding)
{
  lf::DriveSelector s;
  lf::SelectorParams p;
  signs(s, lf::SIGN_LEFT, 3);
  s.onMove(0.01, p.fork_exit_yaw + 0.1, p);
  s.update(GOOD, true, p);
  ASSERT_EQ(s.zone(), lf::Zone::RETURNING);
  const lf::LaneStatus off_center{true, 0.8, 0.9};
  for (int i = 0; i < 20; ++i) {
    EXPECT_EQ(s.update(off_center, true, p), lf::Source::PATH);
  }
  // 그래도 return_max_dist 를 넘게 달리면 sliding 으로
  s.onMove(p.return_max_dist + 0.01, 0, p);
  EXPECT_EQ(s.update(off_center, true, p), lf::Source::SLIDING);
}

TEST(DriveSelector, ForkTimesOutByDistance)
{
  lf::DriveSelector s;
  lf::SelectorParams p;
  signs(s, lf::SIGN_LEFT, 3);
  s.onMove(p.fork_max_dist + 0.01, 0, p);
  s.update(GOOD, true, p);
  EXPECT_EQ(s.zone(), lf::Zone::RETURNING);
}

TEST(DriveSelector, NoReentryRightAfterLeaving)
{
  lf::DriveSelector s;
  lf::SelectorParams p;
  signs(s, lf::SIGN_LEFT, 3);
  s.onMove(0.01, p.fork_exit_yaw + 0.1, p);
  s.update(GOOD, true, p);
  for (int i = 0; i < p.return_ok_count; ++i) {
    s.update(GOOD, true, p);
  }
  ASSERT_EQ(s.zone(), lf::Zone::NORMAL);
  signs(s, lf::SIGN_LEFT, 5);
  EXPECT_EQ(s.zone(), lf::Zone::NORMAL);
  // 다른 구간 표지판은 바로 받는다
  signs(s, lf::SIGN_CONSTRUCTION, 3);
  EXPECT_EQ(s.zone(), lf::Zone::CONSTRUCTION);
}

TEST(DriveSelector, ForkRearmsAfterDistance)
{
  lf::DriveSelector s;
  lf::SelectorParams p;
  signs(s, lf::SIGN_LEFT, 3);
  s.onMove(0.01, p.fork_exit_yaw + 0.1, p);
  s.update(GOOD, true, p);
  for (int i = 0; i < p.return_ok_count; ++i) {
    s.update(GOOD, true, p);
  }
  s.onMove(p.rearm_dist + 0.01, 0, p);
  signs(s, lf::SIGN_RIGHT, 3);
  EXPECT_EQ(s.zone(), lf::Zone::FORK);
  EXPECT_EQ(s.forkDir(), 1);
}

TEST(DriveSelector, ConstructionCountsThreeBoxes)
{
  lf::DriveSelector s;
  lf::SelectorParams p;
  signs(s, lf::SIGN_CONSTRUCTION, 3);
  ASSERT_EQ(s.zone(), lf::Zone::CONSTRUCTION);
  passBox(s, 0.50, p);
  EXPECT_EQ(s.boxesPassed(), 1);
  EXPECT_EQ(s.zone(), lf::Zone::CONSTRUCTION);
  passBox(s, 0.52, p);
  EXPECT_EQ(s.boxesPassed(), 2);
  EXPECT_EQ(s.zone(), lf::Zone::CONSTRUCTION);
  passBox(s, 0.48, p);
  EXPECT_EQ(s.boxesPassed(), 3);
  EXPECT_EQ(s.zone(), lf::Zone::RETURNING);
}

TEST(DriveSelector, NextBoxSeenWhileFirstHiddenIsSeparate)
{
  // 첫 박스가 BEV 아래로 사라진 직후 다음 박스가 보이기 시작해도 따로 센다
  lf::DriveSelector s;
  lf::SelectorParams p;
  signs(s, lf::SIGN_CONSTRUCTION, 3);
  for (double d = 0.50; d > 0.15; d -= 0.01) {
    s.onBox(true, d, 0.5, p);
    s.onMove(0.01, 0, p);
  }
  // 첫 박스는 0.14m 에서 안 보임, 다음 박스가 0.53m 에 보인다
  double d2 = 0.53;
  for (int i = 0; i < 40; ++i, d2 -= 0.01) {
    s.onBox(true, d2, 0.5, p);
    s.onMove(0.01, 0, p);
  }
  EXPECT_EQ(s.boxesPassed(), 1);
  EXPECT_EQ(s.boxesTracked(), 1);
}

TEST(DriveSelector, ExitWaitsWhileBoxAhead)
{
  // 센 개수는 채웠어도 앞에 박스가 남아 있으면 (잘못 센 경우) 그것까지 지나가야 끝
  lf::DriveSelector s;
  lf::SelectorParams p;
  p.construction_boxes = 1;
  signs(s, lf::SIGN_CONSTRUCTION, 3, p);
  for (int i = 0; i < 3; ++i) {
    s.onBox(true, 0.30, 0.5, p);
    s.onBox(true, 0.52, 0.2, p);
  }
  ASSERT_EQ(s.boxesTracked(), 2);
  s.onMove(0.46, 0, p);
  EXPECT_EQ(s.boxesPassed(), 1);
  s.update(GOOD, true, p);
  EXPECT_EQ(s.zone(), lf::Zone::CONSTRUCTION);
  s.onMove(0.25, 0, p);
  s.update(GOOD, true, p);
  EXPECT_EQ(s.zone(), lf::Zone::RETURNING);
}

TEST(DriveSelector, BoxAheadReentersConstructionAfterExit)
{
  // 일찍 빠져나왔는데 박스가 눈앞에 보이면 (RETURNING / NORMAL 둘 다) 다시 공사 구간
  lf::SelectorParams p;
  for (const bool to_normal : {false, true}) {
    lf::DriveSelector s;
    signs(s, lf::SIGN_CONSTRUCTION, 3, p);
    s.onMove(p.construction_no_box_dist + 0.01, 0, p);
    s.update(GOOD, true, p);
    ASSERT_EQ(s.zone(), lf::Zone::RETURNING);
    if (to_normal) {
      for (int i = 0; i < p.return_ok_count; ++i) {
        s.update(GOOD, true, p);
      }
      ASSERT_EQ(s.zone(), lf::Zone::NORMAL);
      signs(s, lf::SIGN_CONSTRUCTION, 5, p);   // 표지판은 rearm_dist 동안 무시
      ASSERT_EQ(s.zone(), lf::Zone::NORMAL);
    }
    for (int i = 0; i < p.box_confirm_count; ++i) {
      s.onBox(true, 0.40, 0.5, p);
    }
    EXPECT_EQ(s.zone(), lf::Zone::CONSTRUCTION);
    EXPECT_EQ(s.update(GOOD, true, p), lf::Source::PATH);
  }
}

TEST(DriveSelector, SingleFrameBoxNoiseNotCounted)
{
  lf::DriveSelector s;
  lf::SelectorParams p;
  signs(s, lf::SIGN_CONSTRUCTION, 3);
  s.onBox(true, 0.3, 0.5, p);
  s.onMove(0.8, 0, p);
  EXPECT_EQ(s.boxesPassed(), 0);
}

TEST(DriveSelector, ConstructionByBoxesWithoutSign)
{
  lf::DriveSelector s;
  lf::SelectorParams p;
  for (int i = 0; i < p.box_confirm_count - 1; ++i) {
    s.onBox(true, 0.4, 0.5, p);
  }
  EXPECT_EQ(s.zone(), lf::Zone::NORMAL);
  s.onBox(true, 0.4, 0.5, p);
  EXPECT_EQ(s.zone(), lf::Zone::CONSTRUCTION);
  // 먼 박스는 진입 판단에 안 씀
  lf::DriveSelector s2;
  for (int i = 0; i < 20; ++i) {
    s2.onBox(true, 0.52, 0.2, p);
  }
  EXPECT_EQ(s2.zone(), lf::Zone::NORMAL);
}

TEST(DriveSelector, ConstructionEndsWhenBoxesStopAppearing)
{
  lf::DriveSelector s;
  lf::SelectorParams p;
  signs(s, lf::SIGN_CONSTRUCTION, 3);
  passBox(s, 0.5, p);
  passBox(s, 0.5, p);
  ASSERT_EQ(s.boxesPassed(), 2);   // 세 번째는 놓쳤다
  for (int i = 0; i < 200 && s.zone() == lf::Zone::CONSTRUCTION; ++i) {
    s.onBox(false, 0, 0, p);
    s.onMove(0.01, 0, p);
    s.update(BAD, true, p);
  }
  EXPECT_EQ(s.zone(), lf::Zone::RETURNING);
}

TEST(DriveSelector, ConstructionEndsWhenNoBoxEverSeen)
{
  lf::DriveSelector s;
  lf::SelectorParams p;
  signs(s, lf::SIGN_CONSTRUCTION, 3);
  s.onMove(p.construction_lost_dist + 0.1, 0, p);
  s.update(GOOD, true, p);
  EXPECT_EQ(s.zone(), lf::Zone::CONSTRUCTION);   // 박스를 하나도 안 지났으면 lost 로 안 끝남
  s.onMove(p.construction_no_box_dist - p.construction_lost_dist, 0, p);
  s.update(GOOD, true, p);
  EXPECT_EQ(s.zone(), lf::Zone::RETURNING);
}

TEST(DriveSelector, ConstructionTimesOutWhileStuckOnBox)
{
  // 박스를 보고 있는데 못 지나가는 경우 construction_max_dist 에서 끝
  lf::DriveSelector s;
  lf::SelectorParams p;
  signs(s, lf::SIGN_CONSTRUCTION, 3);
  for (int i = 0; i < 500 && s.zone() == lf::Zone::CONSTRUCTION; ++i) {
    s.onBox(true, 0.3, 0.5, p);
    s.onMove(0.01, 0, p);
    s.update(GOOD, true, p);
  }
  EXPECT_EQ(s.zone(), lf::Zone::RETURNING);
  EXPECT_EQ(s.boxesPassed(), 0);
}

TEST(DriveSelector, DisabledAlwaysSliding)
{
  lf::DriveSelector s;
  lf::SelectorParams p;
  p.enabled = false;
  signs(s, lf::SIGN_CONSTRUCTION, 5, p);
  EXPECT_EQ(s.update(BAD, true, p), lf::Source::SLIDING);
  EXPECT_EQ(s.zone(), lf::Zone::NORMAL);
}
