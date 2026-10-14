#include <gtest/gtest.h>

#include <cmath>

#include "lane_follower/controller.hpp"

namespace lf = lane_follower;

TEST(Controller, CenteredGoesStraight)
{
  lf::Controller c;
  const auto cmd = c.step(0, 0, 0.9, 0.05, {});
  EXPECT_DOUBLE_EQ(cmd.w, 0.0);
  EXPECT_DOUBLE_EQ(cmd.v, lf::Gains{}.linear_speed);
}

TEST(Controller, TargetRightTurnsRight)
{
  // offset/angle + = 목표가 오른쪽 -> ROS 각속도는 음수 (시계 방향)
  lf::Controller c;
  EXPECT_LT(c.step(0.3, 0.2, 0.9, 0.05, {}).w, 0.0);
  lf::Controller c2;
  EXPECT_GT(c2.step(-0.3, -0.2, 0.9, 0.05, {}).w, 0.0);
}

TEST(Controller, SlowsInCurveAndLowConfidence)
{
  const lf::Gains g;
  lf::Controller c;
  EXPECT_NEAR(c.step(0, g.slow_down_angle, 0.9, 0.05, g).v, g.min_linear_speed, 1e-9);
  lf::Controller c2;
  EXPECT_NEAR(c2.step(0, 0, 0.3, 0.05, g).v, g.min_linear_speed, 1e-9);
}

TEST(Controller, AngularClamped)
{
  const lf::Gains g;
  lf::Controller c;
  EXPECT_DOUBLE_EQ(c.step(-1, -1.5, 0.9, 0.05, g).w, g.max_angular);
}
