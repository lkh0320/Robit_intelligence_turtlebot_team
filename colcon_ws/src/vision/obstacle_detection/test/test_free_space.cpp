// free_space.hpp: 그린 화면으로 박스 반대쪽 / 선 사이를 고르는지
#include <gtest/gtest.h>

#include <opencv2/imgproc.hpp>

#include "obstacle_detection/free_space.hpp"

namespace
{
using obstacle_detection::FreeSpaceParams;
using obstacle_detection::findFreeSpace;

// 녹화의 색 (BGR): 도로 회색, 박스 갈색-빨강, 흰 선, 노란 선
const cv::Scalar ROAD(70, 70, 70), BOX(55, 60, 120), WHITE(240, 240, 240), YELLOW(40, 200, 230);

cv::Mat road()
{
  return cv::Mat(480, 640, CV_8UC3, ROAD);
}

// 화면 아래에서 위로 비스듬히 좁아지는 차선 (왼쪽 노란, 오른쪽 흰)
void drawLines(cv::Mat & img)
{
  cv::line(img, {40, 479}, {250, 120}, YELLOW, 10);
  cv::line(img, {600, 479}, {390, 120}, WHITE, 10);
}
}  // namespace

TEST(FreeSpace, EmptyRoadGoesStraight)
{
  cv::Mat img = road();
  drawLines(img);
  const auto r = findFreeSpace(img, FreeSpaceParams());
  EXPECT_FALSE(r.box_found);
  EXPECT_NEAR(r.offset(), 0.0, 0.15);
}

TEST(FreeSpace, BoxOnRightSteersLeft)
{
  cv::Mat img = road();
  drawLines(img);
  cv::rectangle(img, cv::Rect(330, 220, 180, 120), BOX, cv::FILLED);
  const auto r = findFreeSpace(img, FreeSpaceParams());
  ASSERT_TRUE(r.box_found);
  EXPECT_LT(r.offset(), -0.1);
  EXPECT_GT(r.nearest.rect.x + r.nearest.rect.width / 2, r.blocked.cols / 2);
}

TEST(FreeSpace, BoxOnLeftSteersRight)
{
  cv::Mat img = road();
  drawLines(img);
  cv::rectangle(img, cv::Rect(130, 220, 180, 120), BOX, cv::FILLED);
  const auto r = findFreeSpace(img, FreeSpaceParams());
  ASSERT_TRUE(r.box_found);
  EXPECT_GT(r.offset(), 0.1);
}

TEST(FreeSpace, NearestBoxIsLowest)
{
  cv::Mat img = road();
  cv::rectangle(img, cv::Rect(100, 100, 100, 60), BOX, cv::FILLED);   // 멀리
  cv::rectangle(img, cv::Rect(400, 300, 120, 100), BOX, cv::FILLED);  // 가까이
  const auto r = findFreeSpace(img, FreeSpaceParams());
  ASSERT_TRUE(r.box_found);
  EXPECT_GT(r.nearest.rect.x, r.blocked.cols / 2);
}

TEST(FreeSpace, SmallSpeckIsNotBox)
{
  cv::Mat img = road();
  cv::rectangle(img, cv::Rect(300, 300, 10, 10), BOX, cv::FILLED);   // 줄이면 5x5 = 25px
  const auto r = findFreeSpace(img, FreeSpaceParams());
  EXPECT_FALSE(r.box_found);
}

TEST(FreeSpace, CorridorNeedsRobotWidth)
{
  // 열 하나만 깊게 빈 곳은 로봇 폭 통로가 아니다
  std::vector<float> free(100, 10.0f);
  free[20] = 90.0f;
  for (int i = 70; i < 90; ++i) {
    free[i] = 60.0f;
  }
  const auto corr = obstacle_detection::corridorHeights(free, 16);
  const int u = obstacle_detection::chooseColumn(corr, 0.0);
  EXPECT_GE(u, 70);
  EXPECT_LT(u, 90);
}
