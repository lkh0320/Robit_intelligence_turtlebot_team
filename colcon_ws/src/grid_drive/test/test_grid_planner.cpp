// grid_planner.hpp: 바닥 지도 관측을 직접 그려 넣고 경로 / 조향이 맞는지
//   장면은 실제 축척 [mm] 으로 적고 그릴 때 앞뒤에 map_y_scale 을 곱한다 (지도 앞뒤는 실제의 map_y_scale 배)
#include <gtest/gtest.h>

#include <cmath>

#include <opencv2/imgproc.hpp>

#include "grid_drive/grid_planner.hpp"

namespace
{
using grid_drive::FloorObs;
using grid_drive::GridPlanner;
using grid_drive::Params;
using grid_drive::Turn;

// 실제 로봇 좌표 [mm] -> 바닥 지도 칸
cv::Point px(const Params & p, double x, double y)
{
  return {cvRound((x + p.half_width_mm) / p.cell_mm),
    cvRound((p.far_mm - y * p.map_y_scale) / p.cell_mm)};
}

// 실제 앞 거리 [mm] -> 지도 단위
double my(const Params & p, double y) {return y * p.map_y_scale;}
double farReal(const Params & p) {return p.far_mm / p.map_y_scale;}
double nearReal(const Params & p) {return p.bev_near_mm / p.map_y_scale;}

// 시야 = bev_near 보다 앞 전부. 선 / 박스 / 주행 불가는 비어 있음
FloorObs empty(const Params & p)
{
  const cv::Size fs = grid_drive::fineSize(p);
  FloorObs o;
  o.valid = cv::Mat::zeros(fs, CV_8U);
  o.valid(cv::Rect(cv::Point(0, 0), px(p, p.half_width_mm, nearReal(p)))).setTo(255);
  o.line = cv::Mat::zeros(fs, CV_8U);
  o.box = cv::Mat::zeros(fs, CV_8U);
  return o;
}

void line(const Params & p, FloorObs & o, double x0, double y0, double x1, double y1)
{
  cv::line(o.line, px(p, x0, y0), px(p, x1, y1), cv::Scalar(255), 5);   // 20mm 굵기
  o.line.setTo(0, ~o.valid);
}

void box(const Params & p, FloorObs & o, double x0, double y0, double x1, double y1)
{
  cv::rectangle(o.box, px(p, x0, y1), px(p, x1, y0), cv::Scalar(255), cv::FILLED);
}

// 경로 위 실제 앞 거리 y 에서 가장 가까운 점의 x
double xAt(const Params & p, const std::vector<cv::Point2d> & path, double y)
{
  double best = 1e9, x = 0;
  for (const auto & q : path) {
    if (std::abs(q.y - my(p, y)) < best) {
      best = std::abs(q.y - my(p, y));
      x = q.x;
    }
  }
  return x;
}

Params params()
{
  Params p;
  p.memory = false;
  // 장면은 보정 전 지도 기준으로 만들었다 (2026-10-28 보정값과 무관하게 계획 논리만 본다)
  p.map_y_scale = 2.4;
  p.robot_half_width_mm = 102.5;
  p.src_top_y = 0.360; p.src_top_left_x = 0.286; p.src_top_right_x = 0.714;
  p.src_bottom_y = 1.0; p.src_bottom_left_x = -0.023; p.src_bottom_right_x = 1.023;
  return p;
}
}  // namespace

// 곧은 차선 (선 사이 360mm): 가운데로 직진
TEST(GridPlanner, StraightLane)
{
  const Params p = params();
  FloorObs o = empty(p);
  line(p, o, -180, 0, -180, farReal(p));
  line(p, o, 180, 0, 180, farReal(p));
  GridPlanner g;
  const auto r = g.plan(o, p);
  ASSERT_TRUE(r.path_ok);
  EXPECT_FALSE(r.soft);
  EXPECT_TRUE(r.follow_ok);
  EXPECT_EQ(r.goal_kind, 0);
  EXPECT_LT(std::abs(r.goal.x), 30.0);
  EXPECT_LT(std::abs(r.curvature), 0.15 * p.max_curvature);
  EXPECT_GT(r.v, 0.0);
}

// 오른쪽으로 휜 차선: 오른쪽으로 조향 (w < 0)
TEST(GridPlanner, CurveRight)
{
  const Params p = params();
  FloorObs o = empty(p);
  // 반지름 450mm 로 오른쪽으로 휘는 차선 (폭 360): 중심 (450, 150)
  const cv::Point2d c(450, 150);
  for (double rad : {450.0 - 180, 450.0 + 180}) {
    line(p, o, c.x - rad, 0, c.x - rad, c.y);
    for (double a = 0; a < 1.2; a += 0.05) {
      line(p, o, c.x - rad * std::cos(a), c.y + rad * std::sin(a),
        c.x - rad * std::cos(a + 0.05), c.y + rad * std::sin(a + 0.05));
    }
  }
  GridPlanner g;
  const auto r = g.plan(o, p);
  ASSERT_TRUE(r.path_ok);
  EXPECT_FALSE(r.soft);
  EXPECT_GT(r.goal.x, 100.0);
  EXPECT_LT(r.w, 0.0);
}

// 넓은 길에 박스 두 개 (왼쪽 앞, 오른쪽 뒤): 첫 박스는 오른쪽, 두 번째는 왼쪽으로 지나가는 S자 경로
TEST(GridPlanner, TwoBoxesSShape)
{
  const Params p = params();
  FloorObs o = empty(p);
  Params q = p;
  q.map_y_scale = 1.0;   // 박스 사이를 S자로 지나갈 만큼 멀리 보이는 지도
  line(q, o, -450, 0, -450, farReal(q));
  line(q, o, 450, 0, 450, farReal(q));
  box(q, o, -450, 300, 60, 400);    // 왼쪽에서 가운데 넘어까지
  box(q, o, -60, 900, 450, 1000);   // 오른쪽에서 가운데 넘어까지
  GridPlanner g;
  const auto r = g.plan(o, q);
  ASSERT_TRUE(r.path_ok);
  EXPECT_FALSE(r.soft);
  EXPECT_GT(xAt(q, r.path, 350), 60.0 + q.robot_half_width_mm);
  EXPECT_LT(xAt(q, r.path, 950), -60.0 - q.robot_half_width_mm);
}

// T자 교차로 (앞이 가로 선으로 막힘, 양옆이 열림): 표지판 방향 옆 끝으로
TEST(GridPlanner, IntersectionFollowsTurn)
{
  const Params p = params();
  FloorObs o = empty(p);
  line(p, o, -180, 0, -180, 200);
  line(p, o, 180, 0, 180, 200);
  line(p, o, -p.half_width_mm, 420, p.half_width_mm, 420);   // 정면 가로 선
  GridPlanner left, right;
  const auto rl = left.plan(o, p, Turn::LEFT);
  const auto rr = right.plan(o, p, Turn::RIGHT);
  ASSERT_TRUE(rl.path_ok);
  ASSERT_TRUE(rr.path_ok);
  EXPECT_EQ(rl.goal_kind, 1);
  EXPECT_EQ(rr.goal_kind, 2);
  EXPECT_LT(rl.goal.x, 0.0);
  EXPECT_GT(rr.goal.x, 0.0);
}

// 주행 불가 (세그멘테이션 road 밖): 가운데 길로만
TEST(GridPlanner, OffroadIsForbidden)
{
  const Params p = params();
  FloorObs o = empty(p);
  o.offroad = o.valid.clone();
  cv::rectangle(o.offroad, px(p, -200, farReal(p)), px(p, 200, 0), cv::Scalar(0), cv::FILLED);
  o.offroad.setTo(0, ~o.valid);
  GridPlanner g;
  const auto r = g.plan(o, p);
  ASSERT_TRUE(r.path_ok);
  EXPECT_FALSE(r.soft);
  for (const auto & q : r.path) {
    EXPECT_LT(std::abs(q.x), 200.0 - p.robot_half_width_mm + 1e-6) << "y " << q.y;
  }
}

// 선 위에 서 있음 (진입 금지만으로는 길 없음): soft 경로로 천천히, 선을 덜 침범하는 쪽
TEST(GridPlanner, SoftWhenOnLine)
{
  const Params p = params();
  FloorObs o = empty(p);
  line(p, o, 10, 0, 10, farReal(p));      // 로봇 바로 오른쪽에 선 (hard_line_mm 안)
  line(p, o, -330, 0, -330, farReal(p));  // 왼쪽 선 (차선 폭 340, 로봇이 오른쪽 선 위)
  o.offroad = cv::Mat::zeros(o.valid.size(), CV_8U);   // 오른쪽 선 너머는 road 밖
  cv::rectangle(o.offroad, px(p, 25, farReal(p)), px(p, p.half_width_mm, 0), cv::Scalar(255), cv::FILLED);
  o.offroad.setTo(0, ~o.valid);
  Params q = p;
  q.start_escape_mm = 0;
  GridPlanner g;
  const auto r = g.plan(o, q);
  ASSERT_TRUE(r.path_ok);
  EXPECT_TRUE(r.soft);
  EXPECT_LT(xAt(p, r.path, 1000), 0.0);   // 왼쪽 (차선 가운데) 으로 빠져나감
  EXPECT_LE(r.v, p.linear * p.soft_speed_ratio + 1e-9);
}

// 선에 붙었지만 위는 아님: 복귀 구간 덕에 hard 경로로 차선 가운데로 돌아옴 (예전엔 soft)
TEST(GridPlanner, RecoverWhenNearLine)
{
  const Params p = params();
  FloorObs o = empty(p);
  line(p, o, 40, 0, 40, farReal(p));
  line(p, o, -300, 0, -300, farReal(p));
  GridPlanner g;
  const auto r = g.plan(o, p);
  ASSERT_TRUE(r.path_ok);
  EXPECT_FALSE(r.soft);
  EXPECT_LT(xAt(p, r.path, 1000), -100.0);   // 차선 가운데 (-130) 근처로
}

// 내 차선이 거의 안 보이면 (road 가 보인 곳의 road_min_ratio 미만) 주행 불가를 모름으로
TEST(FloorObs, TinyRoadIsUnknown)
{
  const Params p = params();
  grid_drive::SegMasks s;
  s.road = cv::Mat::zeros(480, 640, CV_8U);
  EXPECT_TRUE(grid_drive::observeSegmentation(s, p).offroad.empty());
  cv::rectangle(s.road, {150, 200}, {490, 479}, cv::Scalar(255), cv::FILLED);
  EXPECT_FALSE(grid_drive::observeSegmentation(s, p).offroad.empty());
}

// 커브: 차선이 보이는 사다리꼴의 비스듬한 옆변으로 빠져나가도 (위 / 좌우 끝에 안 닿음) hard 목표가 있다
TEST(GridPlanner, GoalOnVisibleEdge)
{
  Params p = params();
  p.robot_half_width_mm = 40.0;   // 차선 폭 조건과 섞이지 않게 (목표 후보만 본다)
  FloorObs o = empty(p);
  o.valid = grid_drive::visibleMask(p, cv::Size(640, 480));   // 실제 카메라 사다리꼴 (실제 앞 320mm 부터 좌우 끝)
  // 오른쪽으로 급히 꺾이는 차선: 실제 앞 150~250mm 에서 사다리꼴 오른쪽 옆변으로 빠져나감 (위 / 좌우 끝 열에 안 닿음)
  const std::vector<cv::Point> lane = {px(p, -150, 0), px(p, -150, 80), px(p, 300, 250), px(p, 700, 250),
    px(p, 700, 150), px(p, 150, 30), px(p, 150, 0)};
  o.offroad = o.valid.clone();
  cv::fillPoly(o.offroad, std::vector<std::vector<cv::Point>>{lane}, cv::Scalar(0));
  GridPlanner g;
  const auto r = g.plan(o, p);
  ASSERT_TRUE(r.path_ok);
  EXPECT_FALSE(r.soft);
  EXPECT_EQ(r.goal_kind, 2);
  EXPECT_GT(r.goal.x, 200.0);
}

// 안 보이는 띠 (바퀴 축 ~ bev_near) 로 옆으로 빠지지 않는다
TEST(GridPlanner, BlindBandKeepsForward)
{
  const Params p = params();
  FloorObs o = empty(p);
  line(p, o, -180, nearReal(p), -180, farReal(p));
  line(p, o, 180, nearReal(p), 180, farReal(p));
  box(p, o, -180, 250, 180, 280);   // 차선을 가로막는 박스: 가운데 길은 막힘
  GridPlanner g;
  const auto r = g.plan(o, p);
  for (const auto & q : r.raw_path) {
    if (q.y < p.bev_near_mm) {
      EXPECT_LT(std::abs(q.x), 60.0) << "y " << q.y;
    }
  }
}

// 바퀴 이동: 기억한 선이 안 보이는 띠로 옮겨진다
TEST(GridPlanner, MemoryMovesWithOdom)
{
  Params p;
  p.memory = true;
  FloorObs o = empty(p);
  line(p, o, -180, 0, -180, farReal(p));
  line(p, o, 180, 0, 180, farReal(p));
  GridPlanner g;
  g.plan(o, p);
  g.onMove(100.0, 0.0, p);   // 실제 100mm 앞으로
  const auto r = g.plan(empty(p), p);   // 이번에는 선이 하나도 안 보임
  EXPECT_GT(r.line_memory, 0);
  ASSERT_TRUE(r.path_ok);
  EXPECT_LT(std::abs(xAt(p, r.path, 60)), 40.0);
}

// 여유 띠보다 좁게 왼쪽으로 휘는 차선 (soft): soft_depth_cost 가 있으면 두 선 가운데 (안쪽 선에서 140mm) 로
TEST(GridPlanner, SoftDepthKeepsLaneCenter)
{
  Params p = params();
  p.line_margin_mm = 80.0;   // 띠 2 x 182.5 > 차선 폭 280 -> 여유를 지키는 경로 없음
  FloorObs o = empty(p);
  const cv::Point2d c(-450, 150);   // 반지름 450 으로 왼쪽으로 휨, 폭 280
  for (double rad : {450.0 - 140, 450.0 + 140}) {
    line(p, o, c.x + rad, 0, c.x + rad, c.y);
    for (double a = 0; a < 1.2; a += 0.05) {
      line(p, o, c.x + rad * std::cos(a), c.y + rad * std::sin(a),
        c.x + rad * std::cos(a + 0.05), c.y + rad * std::sin(a + 0.05));
    }
  }
  // 경로 점들이 차선 가운데 (반지름 450) 에서 벗어난 평균 [mm]
  auto off_center = [&](const grid_drive::Result & r) {
      double sum = 0;
      int n = 0;
      for (const auto & q : r.raw_path) {
        const double y = q.y / p.map_y_scale;
        if (y < c.y) {continue;}
        sum += std::abs(std::hypot(q.x - c.x, y - c.y) - 450.0);
        ++n;
      }
      return n ? sum / n : 0.0;
    };
  GridPlanner g0;
  const auto r0 = g0.plan(o, p);
  ASSERT_TRUE(r0.path_ok);
  ASSERT_TRUE(r0.soft);
  p.soft_depth_cost = 30.0;
  GridPlanner g1;
  const auto r1 = g1.plan(o, p);
  ASSERT_TRUE(r1.path_ok);
  ASSERT_TRUE(r1.soft);
  std::printf("off center: depth 0 -> %.0f mm, depth 30 -> %.0f mm\n", off_center(r0), off_center(r1));
  EXPECT_LT(off_center(r1), 25.0);
  EXPECT_LT(off_center(r1), off_center(r0) - 20.0);
}

// 지연 보정: 직선 차선에서 지금 왼쪽으로 돌고 있으면 (지연 동안 더 왼쪽으로 감) 오른쪽 원호를 고름
TEST(GridPlanner, LeadCountersCurrentTurn)
{
  Params p = params();
  FloorObs o = empty(p);
  line(p, o, -180, 0, -180, farReal(p));
  line(p, o, 180, 0, 180, farReal(p));
  GridPlanner g0;
  const auto r0 = g0.plan(o, p);
  ASSERT_TRUE(r0.follow_ok);
  p.lead_mm = 100.0;
  p.lead_k = -0.5 * p.max_curvature;   // 왼쪽으로 도는 중
  GridPlanner g1;
  const auto r1 = g1.plan(o, p);
  ASSERT_TRUE(r1.follow_ok);
  EXPECT_GT(r1.curvature, r0.curvature + 0.1 * p.max_curvature);   // 더 오른쪽
}

// 경로 추종 (track_len_mm): 앞 300mm 직진 뒤 오른쪽으로 휘는 차선. 원호 맞춤은 코너 전부터 꺾고 (안쪽 밟음),
//   추종은 로봇 자리 경로가 곧으니 거의 안 꺾음
TEST(GridPlanner, TrackWaitsForCorner)
{
  Params p = params();
  FloorObs o = empty(p);
  const cv::Point2d c(450, 300);
  for (double rad : {450.0 - 180, 450.0 + 180}) {
    line(p, o, c.x - rad, 0, c.x - rad, c.y);
    for (double a = 0; a < 1.2; a += 0.05) {
      line(p, o, c.x - rad * std::cos(a), c.y + rad * std::sin(a),
        c.x - rad * std::cos(a + 0.05), c.y + rad * std::sin(a + 0.05));
    }
  }
  GridPlanner g0;
  const auto r0 = g0.plan(o, p);
  ASSERT_TRUE(r0.follow_ok);
  p.track_len_mm = 200.0;
  GridPlanner g1;
  const auto r1 = g1.plan(o, p);
  ASSERT_TRUE(r1.follow_ok);
  EXPECT_LT(std::abs(r1.curvature), std::abs(r0.curvature));
  EXPECT_LT(std::abs(r1.curvature), 0.3 * p.max_curvature);
}

// 경로 추종: 차선 가운데가 오른쪽 (x 100) 이면 오른쪽으로 꺾음
TEST(GridPlanner, TrackTurnsTowardPath)
{
  Params p = params();
  p.track_len_mm = 200.0;
  FloorObs o = empty(p);
  line(p, o, -80, 0, -80, farReal(p));
  line(p, o, 280, 0, 280, farReal(p));
  GridPlanner g;
  const auto r = g.plan(o, p);
  ASSERT_TRUE(r.follow_ok);
  EXPECT_GT(r.curvature, 0.0);
  EXPECT_LT(r.w, 0.0);
}

// 경로 추종 바깥 비킴 (track_outer_mm): 오른쪽으로 휘는 차선에서 덜 오른쪽으로 꺾음
TEST(GridPlanner, TrackOuterTurnsLess)
{
  Params p = params();
  FloorObs o = empty(p);
  const cv::Point2d c(300, 0);
  for (double rad : {300.0 - 180, 300.0 + 180}) {
    for (double a = 0; a < 1.2; a += 0.05) {
      line(p, o, c.x - rad * std::cos(a), c.y + rad * std::sin(a),
        c.x - rad * std::cos(a + 0.05), c.y + rad * std::sin(a + 0.05));
    }
  }
  p.track_len_mm = 200.0;
  GridPlanner g0;
  const auto r0 = g0.plan(o, p);
  ASSERT_TRUE(r0.follow_ok);
  p.track_outer_mm = 40.0;
  GridPlanner g1;
  const auto r1 = g1.plan(o, p);
  ASSERT_TRUE(r1.follow_ok);
  EXPECT_LT(r1.curvature, r0.curvature);
}

// 차선 (선 사이 300mm) 이 앞에서 박스로 다 막힘, 왼쪽 선 너머는 빈 바닥: soft 경로가 선을 건너 돌아가면 실격
//   (2026-10-30 drive42). soft_line_block_mm 켜면 선을 건너는 경로를 안 냄
TEST(GridPlanner, SoftNeverCrossesLine)
{
  Params q = params();
  q.map_y_scale = 1.0;
  q.line_margin_mm = 35.0;
  q.robot_half_width_mm = 100.0;
  FloorObs o = empty(q);
  line(q, o, -150, 0, -150, farReal(q));
  line(q, o, 150, 0, 150, farReal(q));
  box(q, o, -150, 450, 150, 550);
  auto crosses = [&](const grid_drive::Result & r) {
      for (const auto & pt : r.path) {
        if (pt.y > 100.0 && (pt.x < -150.0 + 30.0 || pt.x > 150.0 - 30.0)) {
          return true;
        }
      }
      return false;
    };
  GridPlanner g0;
  const auto r0 = g0.plan(o, q);
  EXPECT_TRUE(r0.path_ok && r0.soft && crosses(r0)) << "예전 동작 (선 건넘) 이 재현돼야 시험이 의미 있음";
  q.soft_line_block_mm = 100.0;
  q.soft_line_keep_mm = 60.0;
  GridPlanner g1;
  const auto r1 = g1.plan(o, q);
  EXPECT_FALSE(r1.path_ok && crosses(r1));
}
