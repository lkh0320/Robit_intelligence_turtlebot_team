// 합성 BEV 영상(400x400, 회색 바닥 + 노란/흰 선 + 박스)으로 경로 선택 확인
#include <gtest/gtest.h>

#include <opencv2/imgproc.hpp>

#include "path_planner/planner_core.hpp"

namespace pp = path_planner;

namespace
{
// BROWN = 박스 실측 색 (LAB 23, 145, 138)
const cv::Scalar FLOOR(80, 80, 80), YELLOW(0, 200, 230), WHITE(240, 240, 240), BROWN(10, 15, 49);
constexpr int W = 400, H = 400, LINE_W = 10;

cv::Mat floorImage() {return cv::Mat(H, W, CV_8UC3, FLOOR);}

// x = x_bottom + curve * (H - y)^2 인 곡선 (curve > 0 이면 위로 갈수록 오른쪽)
void drawLine(cv::Mat & img, double x_bottom, const cv::Scalar & color, double curve = 0,
  bool dashed = false)
{
  for (int y = 0; y < H; ++y) {
    if (dashed && ((H - y) / 30) % 2 == 1) {
      continue;   // 30px 선, 30px 빈틈
    }
    const double x = x_bottom + curve * (H - y) * (H - y);
    cv::line(img, {cvRound(x - LINE_W / 2.0), y}, {cvRound(x + LINE_W / 2.0), y}, color);
  }
}

// 노란 선 x=67, 흰 선 x=333 (차선 폭 267, 로봇 x=200 이 가운데)
cv::Mat lane(double shift = 0, double curve = 0, bool dashed_white = false)
{
  cv::Mat img = floorImage();
  drawLine(img, 67 + shift, YELLOW, curve);
  drawLine(img, 333 + shift, WHITE, curve, dashed_white);
  return img;
}

// 같은 영상을 여러 번 넣어 시간 필터가 수렴한 결과
pp::Result settle(const cv::Mat & img, const pp::Params & p = {}, const pp::Bias & b = {},
  pp::Maps * maps = nullptr)
{
  pp::Planner planner;
  pp::Result r;
  for (int i = 0; i < 15; ++i) {
    r = planner.step(img, p, b, {}, maps);
  }
  return r;
}
}  // namespace

TEST(PlannerCore, StraightCentered)
{
  const auto r = settle(lane());
  ASSERT_TRUE(r.detected);
  EXPECT_NEAR(r.offset, 0.0, 0.05);
  EXPECT_NEAR(r.angle, 0.0, 0.05);
  EXPECT_GT(r.confidence, 0.8f);
}

TEST(PlannerCore, LaneToTheRight)
{
  // 차선이 로봇보다 오른쪽 -> 오른쪽으로 가야 한다 (+)
  const auto r = settle(lane(50));
  ASSERT_TRUE(r.detected);
  EXPECT_GT(r.offset, 0.05);
  EXPECT_GT(r.angle, 0.0);
}

TEST(PlannerCore, CurveRight)
{
  const auto r = settle(lane(0, 0.0012));
  ASSERT_TRUE(r.detected);
  EXPECT_GT(r.k, 0.0);
  EXPECT_GT(r.angle, 0.05);
}

TEST(PlannerCore, DashedLineDoesNotLeak)
{
  // 흰 점선이어도 빈틈으로 빠져나가지 않고 직진
  const auto r = settle(lane(0, 0, true));
  ASSERT_TRUE(r.detected);
  EXPECT_NEAR(r.offset, 0.0, 0.1);
  ASSERT_GE(r.best, 0);
  EXPECT_DOUBLE_EQ(r.cands[r.best].line_hit, 0.0);
}

TEST(PlannerCore, BrownBoxIsNotYellowLine)
{
  cv::Mat img = lane();
  cv::rectangle(img, cv::Rect(180, 150, 60, 40), BROWN, cv::FILLED);
  pp::Maps m;
  settle(img, {}, {}, &m);
  EXPECT_EQ(cv::countNonZero(m.yellow(cv::Rect(170, 140, 80, 60))), 0);
  EXPECT_GT(m.box_pixels, 2000);
  // 박스 바닥(y=189)부터 위쪽 전부 막힘, 아래는 열림
  EXPECT_EQ(m.box_blocked.at<uchar>(10, 210), 255);
  EXPECT_EQ(m.box_blocked.at<uchar>(250, 210), 0);
}

TEST(PlannerCore, RedStopLineIsNotBox)
{
  cv::Mat img = lane();
  cv::rectangle(img, cv::Rect(72, 150, 256, 12), cv::Scalar(40, 30, 220), cv::FILLED);   // 빨간 정지선
  pp::Maps m;
  const auto r = settle(img, {}, {}, &m);
  EXPECT_EQ(m.box_pixels, 0);
  EXPECT_NEAR(r.offset, 0.0, 0.05);
}

TEST(PlannerCore, BlurredStopLineEdgesAreNotBox)
{
  // 실제 영상의 정지선(LAB 123, 167, 146)은 가장자리가 바닥과 섞여 박스 색이 된다
  cv::Mat img = lane();
  cv::rectangle(img, cv::Rect(72, 150, 256, 30), cv::Scalar(70, 80, 200), cv::FILLED);
  cv::GaussianBlur(img, img, cv::Size(15, 15), 0);
  pp::Maps m;
  const auto r = settle(img, {}, {}, &m);
  EXPECT_EQ(cv::countNonZero(m.box_blocked), 0);
  EXPECT_NEAR(r.offset, 0.0, 0.05);
}

TEST(PlannerCore, AvoidsBoxOnRight)
{
  // 차선 오른쪽 절반을 막은 박스 -> 공사 구간(선 벌점 낮음)에서 왼쪽으로 피한다
  cv::Mat img = lane();
  cv::rectangle(img, cv::Rect(200, 120, 130, 50), BROWN, cv::FILLED);
  pp::Params p;
  p.line_weight = 1.0;
  const auto r = settle(img, p);
  ASSERT_TRUE(r.detected);
  ASSERT_GE(r.best, 0);
  EXPECT_FALSE(r.cands[r.best].blocked);
  EXPECT_LT(r.k, 0.0);
  EXPECT_LT(r.offset, 0.0);
}

TEST(PlannerCore, ForkBias)
{
  // 갈라지는 길: 선이 멀리 있어 좌우 다 열려 있으면 표지판 방향을 따른다
  cv::Mat img = floorImage();
  drawLine(img, 20, YELLOW);
  drawLine(img, 380, WHITE);
  pp::Bias left, right;
  left.fork_dir = -1;
  right.fork_dir = 1;
  EXPECT_LT(settle(img, {}, left).k, 0.0);
  EXPECT_GT(settle(img, {}, right).k, 0.0);
}

TEST(PlannerCore, PsdPenalty)
{
  // 넓은 길에서 오른쪽 PSD 가 가까우면 오른쪽으로 도는 경로는 고르지 않는다
  cv::Mat img = floorImage();
  drawLine(img, 20, YELLOW);
  drawLine(img, 380, WHITE);
  pp::Bias b;
  b.fork_dir = 1;
  b.psd_right_close = true;
  EXPECT_LE(settle(img, {}, b).k, 1e-9);
}

TEST(PlannerCore, NothingVisible)
{
  EXPECT_FALSE(settle(floorImage()).detected);
}

TEST(PlannerCore, HoldsThenGivesUp)
{
  pp::Planner planner;
  const pp::Params p;
  ASSERT_TRUE(planner.step(lane(), p, {}).detected);
  for (int i = 0; i < p.hold_frames; ++i) {
    const auto r = planner.step(floorImage(), p, {});
    EXPECT_TRUE(r.detected);
    EXPECT_TRUE(r.holding);
  }
  EXPECT_FALSE(planner.step(floorImage(), p, {}).detected);
}

// 실제 로봇처럼 바퀴 축이 BEV 아래 335px 뒤에 있을 때
pp::Params farCamera()
{
  pp::Params p;
  p.robot_y_offset_px = 335;
  p.lookahead_px = 250;
  return p;
}

TEST(PlannerCore, FarCameraStraight)
{
  const auto r = settle(lane(), farCamera());
  ASSERT_TRUE(r.detected);
  EXPECT_NEAR(r.offset, 0.0, 0.05);
  EXPECT_NEAR(r.angle, 0.0, 0.05);
  // 목표점은 바퀴 축에서 250px 앞 -> BEV 아래 밖 (안 보이는 구간)
  EXPECT_GT(r.target.y, H);
}

TEST(PlannerCore, FarCameraCurveDoesNotTurnEarly)
{
  // 왼쪽 커브가 보이기 시작해도 바퀴 축 바로 앞(안 보이는 구간)은 아직 직선이므로
  // 목표점이 크게 왼쪽으로 가면 안 된다 (예전: BEV 아래에서 바로 돌기 시작해 안쪽을 파고듦)
  pp::Planner planner;
  const auto p = farCamera();
  pp::Result r;
  for (int i = 0; i < 3; ++i) {
    r = planner.step(lane(), p, {});
  }
  for (int i = 0; i < 5; ++i) {
    r = planner.step(lane(0, -0.0015), p, {});
  }
  ASSERT_TRUE(r.detected);
  EXPECT_LT(r.k, 0.0);                 // 보이는 곳에서는 왼쪽으로 도는 원호
  EXPECT_GT(r.angle, -0.1);            // 목표점(25cm 앞)은 아직 거의 정면
}

TEST(PlannerCore, MemoryMovesWithOdometry)
{
  // (기억 모드) 왼쪽으로 도는 경로를 기억한 뒤 그만큼 앞으로 가면, 출발 방향이 왼쪽으로 기운다
  pp::Planner planner;
  auto p = farCamera();
  p.n_hidden = 0;
  for (int i = 0; i < 5; ++i) {
    planner.step(lane(0, -0.0015), p, {});
  }
  pp::Odom od;
  od.ds_px = 200;
  const auto r = planner.step(lane(0, -0.0015), p, {}, od);
  ASSERT_TRUE(r.detected);
  EXPECT_LT(r.start_heading, -0.05);
  EXPECT_LT(r.angle, -0.05);           // 이제 목표점은 왼쪽 (커브 안으로 들어가는 중)
}
