// road_seg.hpp: 손으로 만든 출력 (박스 + 계수 + proto) 이 원본 화면 마스크로 맞게 풀리는지
#include <gtest/gtest.h>

#include <opencv2/core.hpp>

#include "grid_drive/road_seg.hpp"

namespace
{
constexpr int S = 320, N = 4, NM = 2, PH = 80, PW = 80;

struct Fake
{
  cv::Mat out0, proto;
  Fake()
  {
    const int s0[3] = {1, 5 + NM, N};
    out0 = cv::Mat(3, s0, CV_32F, cv::Scalar(0));
    const int s1[4] = {1, NM, PH, PW};
    proto = cv::Mat(4, s1, CV_32F, cv::Scalar(0));
    // proto 0 = 전부 +10 (sigmoid ~1), proto 1 = 전부 0
    cv::Mat(PH, PW, CV_32F, proto.ptr<float>(0, 0)).setTo(10);
  }
  // 박스 (입력 픽셀 cx cy w h), 점수, 계수 0
  void set(int i, float cx, float cy, float w, float h, float score, float c0)
  {
    float * d = out0.ptr<float>();
    d[i] = cx; d[N + i] = cy; d[2 * N + i] = w; d[3 * N + i] = h;
    d[4 * N + i] = score; d[5 * N + i] = c0;
  }
};

grid_drive::Letterbox lb640x480() {return {0.5, 0, 40};}
}  // namespace

TEST(RoadSeg, NoBoxIsEmptyMask)
{
  Fake f;
  const auto rs = grid_drive::decodeRoad(f.out0, f.proto, S, lb640x480(), {640, 480});
  EXPECT_EQ(rs.detections, 0);
  EXPECT_EQ(rs.mask.size(), cv::Size(640, 480));
  EXPECT_EQ(cv::countNonZero(rs.mask), 0);
}

TEST(RoadSeg, BoxCropAndLetterboxUndo)
{
  Fake f;
  // 입력 320 기준 아래 절반 (y 160~280 = 원본 y 240~480), 가로 x 80~240 (원본 160~480)
  f.set(0, 160, 220, 160, 120, 0.9f, 1.f);
  f.set(1, 160, 220, 150, 110, 0.3f, 1.f);   // 낮은 점수 -> 버림
  const auto rs = grid_drive::decodeRoad(f.out0, f.proto, S, lb640x480(), {640, 480});
  EXPECT_EQ(rs.detections, 1);
  EXPECT_FLOAT_EQ(rs.best_conf, 0.9f);
  EXPECT_EQ(rs.mask.at<uchar>(360, 320), 255);   // 박스 안
  EXPECT_EQ(rs.mask.at<uchar>(100, 320), 0);     // 박스 위
  EXPECT_EQ(rs.mask.at<uchar>(360, 60), 0);      // 박스 왼쪽
  EXPECT_EQ(rs.mask.at<uchar>(360, 600), 0);     // 박스 오른쪽
  const double area = cv::countNonZero(rs.mask);
  EXPECT_NEAR(area, 320.0 * 240.0, 320.0 * 240.0 * 0.05);
}

TEST(RoadSeg, NegativeCoefIsNotRoadAndUnionOfBoxes)
{
  Fake f;
  f.set(0, 80, 220, 120, 120, 0.8f, -1.f);    // 계수 - -> sigmoid ~0
  f.set(1, 260, 220, 100, 120, 0.7f, 1.f);    // 겹치지 않는 둘째 박스 (오른쪽)
  const auto rs = grid_drive::decodeRoad(f.out0, f.proto, S, lb640x480(), {640, 480});
  EXPECT_EQ(rs.detections, 2);
  EXPECT_EQ(rs.mask.at<uchar>(360, 160), 0);
  EXPECT_EQ(rs.mask.at<uchar>(360, 520), 255);
}

TEST(RoadSeg, BlobIsRgbChw)
{
  cv::Mat img(4, 4, CV_8UC3, cv::Scalar(255, 0, 51));   // BGR
  const cv::Mat b = grid_drive::toBlob(img);
  EXPECT_NEAR(b.ptr<float>(0, 0)[0], 0.2f, 1e-6);   // R
  EXPECT_NEAR(b.ptr<float>(0, 2)[5], 1.0f, 1e-6);   // B
}
