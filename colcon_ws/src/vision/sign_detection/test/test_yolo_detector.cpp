// yolo_detector.hpp 단위 테스트 (gtest) - 실제 모델 없이 가짜 출력 텐서로 후처리를 검사
//   실행: colcon test --packages-select sign_detection && colcon test-result --verbose
#include <gtest/gtest.h>

#include "sign_detection/yolo_detector.hpp"

namespace sd = sign_detection;

namespace
{
constexpr int NC = sd::NUM_CLASSES;
constexpr int N = 8400;   // 640 입력 후보 수

// 1 x (4 + NC) x N 출력 텐서
cv::Mat makeOutput()
{
  const int sz[] = {1, 4 + NC, N};
  return cv::Mat(3, sz, CV_32F, cv::Scalar(0));
}

// 후보 i 에 박스(모델 입력 좌표, 중심/크기)와 한 클래스 점수를 넣는다
void setCandidate(cv::Mat & out, int i, float cx, float cy, float w, float h, int cls, float score)
{
  float * p = out.ptr<float>();
  p[0 * N + i] = cx;
  p[1 * N + i] = cy;
  p[2 * N + i] = w;
  p[3 * N + i] = h;
  p[(4 + cls) * N + i] = score;
}
}  // namespace

// 640x480 카메라 -> 640 정사각형: 크기 그대로, 위아래 80px 회색
TEST(YoloDetector, Letterbox640x480)
{
  cv::Mat img(480, 640, CV_8UC3, cv::Scalar(0, 0, 0)), dst;
  const auto lb = sd::letterbox(img, 640, dst);
  EXPECT_EQ(dst.size(), cv::Size(640, 640));
  EXPECT_DOUBLE_EQ(lb.scale, 1.0);
  EXPECT_EQ(lb.pad_x, 0);
  EXPECT_EQ(lb.pad_y, 80);
  EXPECT_EQ(dst.at<cv::Vec3b>(0, 0), cv::Vec3b(114, 114, 114));
  EXPECT_EQ(dst.at<cv::Vec3b>(320, 320), cv::Vec3b(0, 0, 0));
}

// 320 입력이면 반으로 줄이고 위아래 40px
TEST(YoloDetector, Letterbox320)
{
  cv::Mat img(480, 640, CV_8UC3), dst;
  const auto lb = sd::letterbox(img, 320, dst);
  EXPECT_EQ(dst.size(), cv::Size(320, 320));
  EXPECT_DOUBLE_EQ(lb.scale, 0.5);
  EXPECT_EQ(lb.pad_y, 40);
}

TEST(YoloDetector, ValidOutput)
{
  EXPECT_TRUE(sd::validOutput(makeOutput(), NC));
  EXPECT_FALSE(sd::validOutput(makeOutput(), NC - 1));
  const int nms_sz[] = {1, 300, 6};   // NMS 가 들어간 export 형식
  EXPECT_FALSE(sd::validOutput(cv::Mat(3, nms_sz, CV_32F), NC));
}

// 좌표를 원본으로 되돌리고, 같은 클래스 겹친 박스는 NMS 로 하나, 낮은 신뢰도는 버림
TEST(YoloDetector, DecodeAndNms)
{
  cv::Mat out = makeOutput();
  setCandidate(out, 0, 320, 320, 100, 50, sd::SIGN_LEFT, 0.9f);
  setCandidate(out, 1, 322, 321, 100, 50, sd::SIGN_LEFT, 0.6f);    // 0 과 거의 같은 박스 -> NMS
  setCandidate(out, 2, 100, 200, 40, 40, sd::SIGN_RIGHT, 0.7f);
  setCandidate(out, 3, 500, 300, 40, 40, sd::SIGN_PARKING, 0.3f);  // 신뢰도 낮음
  sd::Letterbox lb;
  lb.pad_y = 80;

  const auto dets = sd::decode(out, NC, lb, {640, 480}, 0.5f, 0.45f);
  ASSERT_EQ(dets.size(), 2u);
  EXPECT_EQ(dets[0].cls, sd::SIGN_LEFT);
  EXPECT_FLOAT_EQ(dets[0].conf, 0.9f);
  // 중심 (320, 320-80) 크기 100x50 -> 왼쪽 위 (270, 215)
  EXPECT_DOUBLE_EQ(dets[0].box.x, 270);
  EXPECT_DOUBLE_EQ(dets[0].box.y, 215);
  EXPECT_DOUBLE_EQ(dets[0].box.width, 100);
  EXPECT_DOUBLE_EQ(dets[0].box.height, 50);
  EXPECT_EQ(dets[1].cls, sd::SIGN_RIGHT);
}

// 화면 밖으로 나간 부분은 잘린다 (위쪽 회색 여백에 걸친 박스)
TEST(YoloDetector, DecodeClipsToImage)
{
  cv::Mat out = makeOutput();
  setCandidate(out, 0, 50, 90, 40, 40, sd::SIGN_PARKING, 0.8f);   // 원본 y = 10 - 20 = -10
  sd::Letterbox lb;
  lb.pad_y = 80;
  const auto dets = sd::decode(out, NC, lb, {640, 480}, 0.5f, 0.45f);
  ASSERT_EQ(dets.size(), 1u);
  EXPECT_DOUBLE_EQ(dets[0].box.y, 0);
  EXPECT_DOUBLE_EQ(dets[0].box.height, 30);
}

// 한 프레임에 박스가 여러 개면 표지판 중 신뢰도 최고 하나 (신호등/차단봉은 무시)
TEST(YoloDetector, BestSign)
{
  using D = sd::Detection;
  EXPECT_FALSE(sd::bestSign({}).has_value());
  EXPECT_FALSE(sd::bestSign({D{sd::LIGHT_RED, 0.9f, {}}}).has_value());

  const auto best = sd::bestSign({
    D{sd::LIGHT_GREEN, 0.95f, {}},
    D{sd::SIGN_LEFT, 0.7f, {}},
    D{sd::SIGN_RIGHT, 0.8f, {}},
  });
  ASSERT_TRUE(best.has_value());
  EXPECT_EQ(best->cls, sd::SIGN_RIGHT);
}
