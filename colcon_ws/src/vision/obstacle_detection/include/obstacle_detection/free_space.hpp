// 공사 구간: 원본 카메라에서 박스 / 선이 없는 빈 공간 방향 찾기 (ROS 없이 테스트 가능)
//
//   2026-10-13 수동 주행 녹화 (~/construction_test_20261013/110713_manual) 로 정한 방법:
//     박스 위치를 미터로 구하는 건 카메라 모델 / odom 이 실측 보정 없이 안 맞아 접고 (NOTES.md),
//     픽셀만으로 "앞이 가장 멀리 비어 있는 방향" 을 고른다. 사람 조향과 뚜렷한 조향 39번 중 38번 같은 방향.
//   처리
//     1. work_width x work_height 로 줄이고 Lab 으로 막힘 마스크 = 박스(갈색) | 노란 선 | 흰 선
//        밝은 바닥(트랙 밖)은 넣지 않는다: 도로 위 조명 반사를 막힘으로 잡고, 선에서 이미 멈추므로 필요 없음
//     2. 열마다 아래에서 위로 처음 막힌 줄까지 높이 = 그 방향 빈 높이 (멀리 비어 있을수록 큼)
//     3. 로봇 폭(robot_width_px) 통로의 빈 높이 = 그 폭 안 열들의 최소값
//     4. 가운데 쪽에 가점 (center_bias) 을 곱해 가장 큰 열 = 진행 방향
//   박스: 박스 마스크 덩어리 중 아래 끝이 가장 아래(가장 가까운) 것
#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace obstacle_detection
{

struct FreeSpaceParams
{
  int work_width = 320;
  int work_height = 240;
  // 박스 (갈색 벽돌 더미): Lab
  int box_l_min = 10, box_l_max = 120;
  int box_a_min = 135, box_a_max = 170;
  int box_b_min = 130, box_b_max = 155;
  int box_min_area = 150;       // [px, work 크기 기준] 이보다 작은 덩어리는 잡음
  // 노란 선 / 흰 선 (lane_detection 과 비슷, 원본 카메라라 흰 선은 조금 낮춤)
  int yellow_l_min = 80, yellow_a_max = 135, yellow_b_min = 150;
  int white_l_min = 170, white_ab_dev = 15;
  int robot_width_px = 60;      // 빈 통로 최소 폭 [px, work 폭 기준]
  double center_bias = 0.3;     // 화면 끝 열은 빈 높이에 (1 - center_bias) 를 곱함
};

struct BoxBlob
{
  cv::Rect rect;
  int area = 0;
};

struct FreeSpaceResult
{
  cv::Mat small;                // 줄인 BGR (디버그 그리기용)
  cv::Mat blocked;              // 막힘 마스크
  cv::Mat box;                  // 박스 마스크
  std::vector<float> free;      // 열마다 빈 높이 [px]
  std::vector<float> corridor;  // 열마다 로봇 폭 통로 빈 높이 [px]
  int column = 0;               // 고른 열
  bool box_found = false;
  BoxBlob nearest;              // 가장 가까운 박스 (box_found 일 때)
  int box_pixels = 0;           // box_min_area 넘는 덩어리들 면적 합

  // 고른 방향 [-1, 1], + 오른쪽
  double offset() const
  {
    const double half = free.size() / 2.0;
    return half > 0 ? (column + 0.5 - half) / half : 0.0;
  }
  // 고른 통로가 얼마나 멀리 비었나 [0, 1] (1 = 화면 맨 위까지)
  double clearance() const
  {
    return blocked.rows > 0 ? corridor[column] / blocked.rows : 0.0;
  }
};

// 열마다 아래에서 위로 처음 막힌 줄까지의 높이 (막힌 게 없으면 rows)
inline std::vector<float> freeHeights(const cv::Mat & blocked)
{
  std::vector<float> free(blocked.cols, static_cast<float>(blocked.rows));
  for (int x = 0; x < blocked.cols; ++x) {
    for (int y = blocked.rows - 1; y >= 0; --y) {
      if (blocked.at<uint8_t>(y, x)) {
        free[x] = static_cast<float>(blocked.rows - 1 - y);
        break;
      }
    }
  }
  return free;
}

// 열 u 를 가운데로 한 robot_width 폭 안 최소 빈 높이 (화면 밖은 빼고)
inline std::vector<float> corridorHeights(const std::vector<float> & free, int robot_width)
{
  const int n = static_cast<int>(free.size());
  const int half = std::max(0, robot_width / 2);
  std::vector<float> corr(n);
  for (int u = 0; u < n; ++u) {
    const int a = std::max(0, u - half), b = std::min(n, u + half);
    corr[u] = b > a ? *std::min_element(free.begin() + a, free.begin() + b) : free[u];
  }
  return corr;
}

inline int chooseColumn(const std::vector<float> & corridor, double center_bias)
{
  const int n = static_cast<int>(corridor.size());
  const double half = n / 2.0;
  int best = n / 2;
  double best_score = -1;
  for (int u = 0; u < n; ++u) {
    const double score = corridor[u] * (1.0 - center_bias * std::abs(u + 0.5 - half) / half);
    if (score > best_score) {
      best_score = score;
      best = u;
    }
  }
  return best;
}

inline FreeSpaceResult findFreeSpace(const cv::Mat & bgr, const FreeSpaceParams & p)
{
  FreeSpaceResult r;
  cv::resize(bgr, r.small, cv::Size(p.work_width, p.work_height), 0, 0, cv::INTER_AREA);
  cv::Mat lab;
  cv::cvtColor(r.small, lab, cv::COLOR_BGR2Lab);
  const cv::Mat k2 = cv::Mat::ones(2, 2, CV_8U);

  cv::inRange(lab, cv::Scalar(p.box_l_min, p.box_a_min, p.box_b_min),
    cv::Scalar(p.box_l_max, p.box_a_max, p.box_b_max), r.box);
  cv::morphologyEx(r.box, r.box, cv::MORPH_OPEN, k2);
  // 작은 덩어리는 박스에서 빼고, 가장 가까운(아래 끝이 가장 아래) 덩어리를 찾는다
  cv::Mat labels, stats, centroids;
  const int n = cv::connectedComponentsWithStats(r.box, labels, stats, centroids, 8);
  cv::Mat keep = cv::Mat::zeros(r.box.size(), CV_8U);
  for (int i = 1; i < n; ++i) {
    const int area = stats.at<int>(i, cv::CC_STAT_AREA);
    if (area < p.box_min_area) {
      continue;
    }
    keep.setTo(255, labels == i);
    r.box_pixels += area;
    const cv::Rect rect(stats.at<int>(i, cv::CC_STAT_LEFT), stats.at<int>(i, cv::CC_STAT_TOP),
      stats.at<int>(i, cv::CC_STAT_WIDTH), stats.at<int>(i, cv::CC_STAT_HEIGHT));
    if (!r.box_found || rect.br().y > r.nearest.rect.br().y) {
      r.nearest = {rect, area};
      r.box_found = true;
    }
  }
  r.box = keep;

  cv::Mat yellow, white;
  cv::inRange(lab, cv::Scalar(p.yellow_l_min, 0, p.yellow_b_min),
    cv::Scalar(255, p.yellow_a_max, 255), yellow);
  cv::inRange(lab, cv::Scalar(p.white_l_min, 128 - p.white_ab_dev, 128 - p.white_ab_dev),
    cv::Scalar(255, 128 + p.white_ab_dev, 128 + p.white_ab_dev), white);
  r.blocked = r.box | yellow | white;
  cv::morphologyEx(r.blocked, r.blocked, cv::MORPH_OPEN, k2);

  r.free = freeHeights(r.blocked);
  r.corridor = corridorHeights(r.free, p.robot_width_px);
  r.column = chooseColumn(r.corridor, p.center_bias);
  return r;
}

}  // namespace obstacle_detection
