// 바닥 지도의 색 마스크 + 거리 변환 (box_planner 가 쓴다, ROS 없이 테스트 가능)
//   (2026-10-14 지운 path_planner 의 buildMaps 를 옮겨 온 것)
//   - LAB 색 마스크로 노란 선 / 흰 선 / 박스(어두운 적갈색)를 찾는다
//     박스는 어둡고 a 가 높다 (L ≈ 23, a ≈ 145, b ≈ 138). 빨간 정지선은 더 밝고 더 붉어서 a / b - a 로 빠진다
//   - 박스는 바닥 접점(덩어리의 열마다 가장 아래 픽셀)부터 위쪽 전부를 막힘으로 칠한다
//     (바닥 지도에서 박스가 위로 늘어나 보이는데, 박스 뒤는 어차피 안 보이는 곳이라 막힘이 맞다)
//   - 선 마스크는 line_gap_close_px > 1 이면 세로로 불려(dilate) 점선의 빈틈을 메운다
//   - 거리 변환으로 칸마다 "가장 가까운 선 / 박스까지 거리" 를 구한다
#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace obstacle_detection
{

struct ColorParams
{
  // OpenCV LAB: L 0~255, a/b 는 128 이 무채색. 선 색은 lane_detection.yaml 과 같게
  // (2026-10-01 실측: 노란 선 b≈188, 흰 선 L≈230~250, 바닥 L≈85)
  int yellow_l_min = 80;
  int yellow_b_min = 150;
  int yellow_a_max = 135;      // 이보다 a 가 크면(붉으면) 노란 선이 아니라 갈색 박스로 본다 (노란 선 a≈126, 박스 a≈145)
  int white_l_min = 190;
  int white_ab_dev = 15;
  int box_l_min = 10, box_l_max = 110;     // 박스 (2026-10-08 실측 L 20~47, a 143~147, b 136~140)
  int box_a_min = 137, box_a_max = 165;    // 바닥 a ≈ 127, 빨간 정지선 a ≈ 170
  int box_b_min = 128, box_b_max = 152;
  int box_min_b_minus_a = -15;  // 박스 b-a ≈ -7, 빨간 정지선 b-a ≈ -23
  int box_min_area = 300;      // 이보다 작은 갈색 덩어리는 잡음 [px]
  int line_gap_close_px = 1;   // 선 마스크를 세로로 불리는 높이 (1 = 안 불림, 공사 구간 선은 실선)
};

struct ColorMaps
{
  cv::Mat yellow, white, box;  // 색 마스크 (0/255)
  cv::Mat box_blocked;         // 박스 바닥 접점부터 위쪽 전부 (0/255)
  cv::Mat line_wall;           // 선 마스크 (line_gap_close_px > 1 이면 빈틈을 메움, 0/255)
  cv::Mat dist_line, dist_box; // 가장 가까운 선 / 박스까지 거리 [px] (CV_32F)
  int yellow_pixels = 0, white_pixels = 0, box_pixels = 0;
  // 가장 가까운 박스 (바닥 접점이 가장 아래인 덩어리의 bbox). near_box_area = 0 이면 없음
  cv::Rect near_box;
  int near_box_area = 0;
};

inline ColorMaps buildColorMaps(const cv::Mat & bgr, const ColorParams & p)
{
  ColorMaps m;
  cv::Mat lab;
  cv::cvtColor(bgr, lab, cv::COLOR_BGR2Lab);
  const int dev = p.white_ab_dev;
  cv::inRange(lab, cv::Scalar(p.yellow_l_min, 0, p.yellow_b_min),
    cv::Scalar(255, p.yellow_a_max, 255), m.yellow);
  cv::inRange(lab, cv::Scalar(p.white_l_min, 128 - dev, 128 - dev),
    cv::Scalar(255, 128 + dev, 128 + dev), m.white);
  cv::inRange(lab, cv::Scalar(p.box_l_min, p.box_a_min, p.box_b_min),
    cv::Scalar(p.box_l_max, p.box_a_max, p.box_b_max), m.box);
  const cv::Mat k3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  {
    // 빨간 정지선도 a 가 높아 위 범위에 걸리므로 b - a 로 한 번 더 거른다
    cv::Mat ch[3], b_minus_a;
    cv::split(lab, ch);
    cv::subtract(ch[2], ch[1], b_minus_a, cv::noArray(), CV_16S);
    const cv::Mat red = (b_minus_a < p.box_min_b_minus_a) & (ch[1] >= p.box_a_min);
    m.box.setTo(0, red);
    // 정지선 가장자리는 바닥과 섞여 어둡고 덜 붉어져 박스 색이 되므로 정지선 주변도 뺀다
    cv::Mat red_grown;
    cv::morphologyEx(red, red_grown, cv::MORPH_OPEN, k3);
    cv::dilate(red_grown, red_grown, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(11, 11)));
    m.box.setTo(0, red_grown);
  }
  cv::morphologyEx(m.yellow, m.yellow, cv::MORPH_OPEN, k3);
  cv::morphologyEx(m.white, m.white, cv::MORPH_OPEN, k3);
  cv::morphologyEx(m.box, m.box, cv::MORPH_OPEN, k3);
  // 박스 가장자리의 노란 기운 픽셀이 노란 선으로 남지 않게 박스 주변을 노란 마스크에서 뺀다
  cv::Mat box_grown;
  cv::dilate(m.box, box_grown, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7)));
  m.yellow.setTo(0, box_grown);

  // 박스: 큰 덩어리만, 열마다 가장 아래 픽셀(바닥 접점)부터 위쪽 전부 막힘
  m.box_blocked = cv::Mat::zeros(bgr.size(), CV_8U);
  cv::Mat labels, stats, centroids;
  const int n = cv::connectedComponentsWithStats(m.box, labels, stats, centroids, 8, CV_32S);
  for (int i = 1; i < n; ++i) {
    if (stats.at<int>(i, cv::CC_STAT_AREA) < p.box_min_area) {
      continue;
    }
    const int x0 = stats.at<int>(i, cv::CC_STAT_LEFT), y0 = stats.at<int>(i, cv::CC_STAT_TOP);
    const int bw = stats.at<int>(i, cv::CC_STAT_WIDTH), bh = stats.at<int>(i, cv::CC_STAT_HEIGHT);
    for (int x = x0; x < x0 + bw; ++x) {
      for (int y = y0 + bh - 1; y >= y0; --y) {
        if (labels.at<int>(y, x) == i) {
          m.box_blocked(cv::Rect(x, 0, 1, y + 1)).setTo(255);
          break;
        }
      }
    }
    m.box_pixels += stats.at<int>(i, cv::CC_STAT_AREA);
    if (m.near_box_area == 0 || y0 + bh > m.near_box.y + m.near_box.height) {
      m.near_box = cv::Rect(x0, y0, bw, bh);
      m.near_box_area = stats.at<int>(i, cv::CC_STAT_AREA);
    }
  }

  cv::bitwise_or(m.yellow, m.white, m.line_wall);
  if (p.line_gap_close_px > 1) {
    cv::dilate(m.line_wall, m.line_wall,
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, p.line_gap_close_px)));
  }
  m.yellow_pixels = cv::countNonZero(m.yellow);
  m.white_pixels = cv::countNonZero(m.white);

  // 거리 변환: 0 이 아닌 칸에서 가장 가까운 0 칸까지 거리 -> 벽을 0 으로 뒤집어서 넣는다
  // (벽이 하나도 없으면 결과가 아주 큰 값이 되는데, 점수에서 clearance_cap 으로 자르므로 괜찮다)
  cv::Mat free_line, free_box;
  cv::bitwise_not(m.line_wall, free_line);
  cv::bitwise_not(m.box_blocked, free_box);
  cv::distanceTransform(free_line, m.dist_line, cv::DIST_L2, 3);
  cv::distanceTransform(free_box, m.dist_box, cv::DIST_L2, 3);
  return m;
}

}  // namespace obstacle_detection
