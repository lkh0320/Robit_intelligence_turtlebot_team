// box_planner.hpp: 바닥 지도에 그린 박스를 원본 화면으로 되돌려 넣고, 로봇 폭으로 피하는 경로를 고르는지
#include <gtest/gtest.h>

#include <string>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "obstacle_detection/box_planner.hpp"

namespace
{
using obstacle_detection::BoxPlanner;
using obstacle_detection::ColorParams;
using obstacle_detection::PlannerParams;

// 녹화의 색 (BGR): 도로 회색, 박스 갈색-빨강, 흰 선
const cv::Scalar ROAD(70, 70, 70), BOX(55, 60, 120), WHITE(240, 240, 240);

struct Scene
{
  PlannerParams p;
  cv::Mat ground;   // 바닥 지도 (칸)

  Scene()
  {
    ground = cv::Mat(BoxPlanner::mapSize(p), CV_8UC3, ROAD);
  }

  cv::Point cell(double x_mm, double y_mm) const
  {
    return {cvRound((x_mm + p.half_width_mm) / p.cell_mm), cvRound((p.far_mm - y_mm) / p.cell_mm)};
  }

  // 로봇 좌표 [mm] 의 박스 앞면 (가운데 x, 앞면 y), 폭 180, 지도 앞뒤로 depth
  void box(double x, double y, double depth = 200)
  {
    cv::rectangle(ground, cell(x - 90, y + depth), cell(x + 90, y), BOX, cv::FILLED);
  }

  void line(double x)
  {
    cv::line(ground, cell(x, p.bev_near_mm), cell(x, p.far_mm), WHITE, 3);
  }

  // 바닥 지도 -> 원본 640x480 (planner 가 원본을 줄여 펴는 것의 반대)
  cv::Mat raw() const
  {
    const cv::Size small(cvRound(640 * p.raw_scale), cvRound(480 * p.raw_scale));
    cv::Mat out, big;
    cv::warpPerspective(ground, out, BoxPlanner::mapToRaw(p, small), small, cv::INTER_NEAREST,
      cv::BORDER_CONSTANT, ROAD);
    cv::resize(out, big, cv::Size(640, 480), 0, 0, cv::INTER_NEAREST);
    return big;
  }
};

// 경로가 y 근처에서 지나는 가로 위치
double pathX(const obstacle_detection::PlanResult & r, double y)
{
  for (const auto & q : r.path) {
    if (q.y >= y) {
      return q.x;
    }
  }
  return r.path.empty() ? 0.0 : r.path.back().x;
}
}  // namespace

TEST(BoxPlanner, EmptyRoadGoesStraight)
{
  Scene s;
  BoxPlanner bp;
  const auto r = bp.plan(s.raw(), s.p, ColorParams(), false, false);
  EXPECT_TRUE(r.ok);
  EXPECT_FALSE(r.box_visible);
  EXPECT_NEAR(r.angle, 0.0, 0.05);
}

TEST(BoxPlanner, CloseBoxOnRightGoesLeft)
{
  Scene s;
  s.box(60, 450);
  BoxPlanner bp;
  const auto r = bp.plan(s.raw(), s.p, ColorParams(), false, false);
  EXPECT_TRUE(r.ok);
  EXPECT_TRUE(r.box_visible);
  EXPECT_LT(r.angle, -0.05);
  // 박스 앞면 줄에서 로봇 옆면이 박스 왼쪽 끝 (60 - 90) 보다 왼쪽
  EXPECT_LT(pathX(r, 450) + s.p.robot_half_width_mm, 60 - 90);
}

TEST(BoxPlanner, CloseBoxOnLeftGoesRight)
{
  Scene s;
  s.box(-60, 450);
  BoxPlanner bp;
  const auto r = bp.plan(s.raw(), s.p, ColorParams(), false, false);
  EXPECT_TRUE(r.ok);
  EXPECT_GT(r.angle, 0.05);
  EXPECT_GT(pathX(r, 450) - s.p.robot_half_width_mm, -60 + 90);
}

TEST(BoxPlanner, FarBoxIgnoredForNow)
{
  // react_mm 보다 먼 박스는 아직 안 피한다 (첫 박스에서 너무 일찍 꺾던 문제)
  Scene s;
  s.box(0, s.p.react_mm + 150);
  BoxPlanner bp;
  const auto r = bp.plan(s.raw(), s.p, ColorParams(), false, false);
  EXPECT_TRUE(r.box_visible);
  EXPECT_NEAR(r.angle, 0.0, 0.05);
}

TEST(BoxPlanner, StaysAwayFromLine)
{
  // 박스 오른쪽, 왼쪽에 흰 선: 선을 넘지 않고 박스와 선 사이로
  Scene s;
  s.box(80, 450);
  s.line(-300);
  BoxPlanner bp;
  const auto r = bp.plan(s.raw(), s.p, ColorParams(), false, false);
  ASSERT_TRUE(r.ok);
  for (const auto & q : r.path) {
    if (q.y > s.p.bev_near_mm && q.y < 700) {
      EXPECT_GT(q.x, -300 + s.p.cross_line_mm) << "y " << q.y;
    }
  }
}

TEST(BoxPlanner, RemembersBoxBelowCamera)
{
  // 왼쪽 박스를 본 뒤 앞으로 가서 카메라 아래로 사라져도 그쪽으로 돌아가지 않는다
  Scene s;
  s.box(-60, 420);
  BoxPlanner bp;
  auto r = bp.plan(s.raw(), s.p, ColorParams(), false, false);
  ASSERT_TRUE(r.ok);
  ASSERT_GT(r.memory_points, 0);
  // 엔코더 100mm = 지도 240mm -> 앞면이 y 180 (지도 맨 아래 179 근처, 카메라에 거의 안 보임)
  bp.onMove(100, 0, s.p);
  Scene empty;
  r = bp.plan(empty.raw(), empty.p, ColorParams(), false, false);
  ASSERT_TRUE(r.ok || r.fallback);
  EXPECT_TRUE(r.box_ahead);
  EXPECT_FALSE(r.box_visible);
  // 박스 앞면 ~ 뒷면 구간에서 로봇 왼쪽 옆면이 박스 오른쪽 끝 (-60 + 90) 보다 오른쪽
  for (const auto & q : r.path) {
    if (q.y > 180 && q.y < 380) {
      EXPECT_GT(q.x - s.p.robot_half_width_mm, -60 + 90 - 1) << "y " << q.y;
    }
  }
}

TEST(BoxPlanner, ForgetsBoxBehindRobot)
{
  Scene s;
  s.box(-60, 420);
  BoxPlanner bp;
  bp.plan(s.raw(), s.p, ColorParams(), false, false);
  bp.onMove(400, 0, s.p);   // 지도 960mm 앞으로 -> 앞면 y -540 (memory_back 250 넘음)
  Scene empty;
  const auto r = bp.plan(empty.raw(), empty.p, ColorParams(), false, false);
  EXPECT_EQ(r.memory_points, 0);
  EXPECT_FALSE(r.box_ahead);
}

TEST(BoxPlanner, SidePsdBlocksTurningIntoIt)
{
  // 오른쪽 PSD 가 가까우면 (박스 옆) 오른쪽으로 붙는 경로를 고르지 않는다
  Scene s;
  s.box(-150, 400);   // 왼쪽 박스 -> 원래는 오른쪽으로 비킴
  BoxPlanner bp;
  const auto r = bp.plan(s.raw(), s.p, ColorParams(), false, true);
  for (const auto & q : r.path) {
    if (q.y <= s.p.psd_len_mm) {
      EXPECT_LE(q.x, s.p.psd_keep_mm + 1e-6) << "y " << q.y;
    }
  }
}

TEST(BoxPlanner, BlockedEverywhereFallsBack)
{
  // 길 전체를 막은 가까운 박스 줄: 경로 없음 -> 가장 멀리 가는 후보로 버팀 (멈춰 갇히지 않게)
  Scene s;
  for (double x = -720; x <= 720; x += 180) {   // 지도 양 끝까지 (급한 커브 후보가 줄 끝으로 빠져나가지 않게)
    s.box(x, 400);
  }
  BoxPlanner bp;
  const auto r = bp.plan(s.raw(), s.p, ColorParams(), false, false);
  EXPECT_FALSE(r.ok);
  EXPECT_TRUE(r.fallback);
  EXPECT_FALSE(r.path.empty());
}

TEST(BoxPlanner, DoesNotSlipPastLineEndAtMapBottom)
{
  // 2026-10-14 230313_test 160.4s 원본 (planner 가 어차피 raw_scale 0.5 로 줄이므로 320x240 으로 저장).
  // 오른쪽 흰 선이 비스듬히 앞을 가로지르고 그 너머 왼쪽 앞에 박스. 선 마스크가 지도 맨 아래 몇 칸 위에서
  // 끊겨 보여, 예전에는 그 틈으로 선 오른쪽(트랙 밖)으로 나가는 경로를 "ok" 로 골라 트랙 밖으로 나갔다
  const cv::Mat half = cv::imread(std::string(TEST_DATA_DIR) + "/line_end_230313_160s.png");
  ASSERT_FALSE(half.empty());
  cv::Mat raw;
  cv::resize(half, raw, cv::Size(640, 480), 0, 0, cv::INTER_NEAREST);
  PlannerParams p;
  BoxPlanner bp;
  const auto r = bp.plan(raw, p, ColorParams(), false, false);
  ASSERT_FALSE(r.path.empty());
  EXPECT_LT(r.angle, 0.1);   // 예전 +0.65 (오른쪽 선 밖)
  // 고른 경로가 지나는 칸 (지도 안) 에 선이 없어야 한다
  const auto & line = bp.maps().line_wall;
  for (const auto & q : r.path) {
    if (q.y > p.line_horizon_mm) {
      break;
    }
    const cv::Point2d m = bp.toMap(q, p);
    const int u = cvRound(m.x), v = cvRound(m.y);
    if (u >= 0 && v >= 0 && u < line.cols && v < line.rows) {
      EXPECT_EQ(line.at<uint8_t>(v, u), 0) << "선 위: x " << q.x << " y " << q.y;
    }
  }
}
