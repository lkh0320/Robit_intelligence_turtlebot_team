// 바닥 지도 관측: 계획기 (grid_planner.hpp) 가 쓰는 마스크를 만드는 부분 (ROS 없이 테스트 가능)
//
//   계획기는 인식 방법을 모르고 바닥 지도 (로봇 좌표, cell_mm 칸) 위 마스크만 받는다:
//     valid   카메라에 보인 칸
//     line    넘으면 안 되는 선 (노란 / 흰)
//     box     장애물 (박스 바닥 네모)
//     offroad 주행 불가 영역 (트랙 밖 바닥 등). 비어 있으면 모름 (색 방식은 못 구함)
//   인식 방법
//     observeColor: 원본을 바닥 지도로 편 뒤 LAB 색 마스크 (obstacle_detection/color_maps.hpp 그대로)
//     observeSegmentation: 세그멘테이션 모델 (YOLO-seg 등) 의 원본 화면 클래스 마스크를 같은 변환으로 바닥 지도에 편다
//       road (주행 가능) 가 있으면 offroad = 보인 곳 중 road 아닌 곳
//     merge: 둘을 함께 쓸 때 (선 / 박스 / 주행 불가는 합침)
//
//   지도: 가로 ±half_width_mm, 세로 바퀴 축 (y = 0) ~ far_mm. 칸 (u, v) = (x + half, far - y) / cell
//   변환은 bird_eye_view.yaml 사다리꼴 (BEV 400x400, 1px = 1mm, 맨 아래 = 바퀴 축 앞 bev_near_mm) 을 넓힌 것
#pragma once

#include <algorithm>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "obstacle_detection/color_maps.hpp"

namespace grid_drive
{

struct FloorParams
{
  double src_top_y = 0.392, src_top_left_x = 0.274, src_top_right_x = 0.726;   // 2026-10-28 바닥 표시 보정 (f 399, 카메라 발 = 축 앞 85mm)
  double src_bottom_y = 1.109, src_bottom_left_x = -0.096, src_bottom_right_x = 1.096;
  double bev_size_mm = 400.0;
  double bev_near_mm = 179.0;          // BEV 맨 아래 = 바퀴 축 앞 [mm]
  double cell_mm = 4.0;
  double half_width_mm = 600.0;
  double far_mm = 1200.0;
  double raw_scale = 0.5;              // 색 방식: 원본을 이 배율로 줄여서 편다 (속도)
  double box_depth_mm = 200.0;         // 박스 앞면에서 뒤로 막힘 길이 (지도 단위)
  double box_gap_mm = 40.0;
  double road_min_ratio = 0.05;        // 보인 곳 중 road 가 이보다 적으면 (내 차선 안 보임, 회전 중 등) 주행 불가를 모름으로
};

struct FloorObs
{
  cv::Mat valid, line, box, offroad;   // 0/255, fineSize. offroad 는 비어 있을 수 있음
  cv::Mat ground;                      // 바닥 지도로 편 원본 (디버그 그림, 없을 수 있음)
  cv::Mat yellow, white;               // 선 색 구분 (디버그 그림, 없을 수 있음)
};

inline cv::Size fineSize(const FloorParams & p)
{
  return {static_cast<int>(std::lround(2 * p.half_width_mm / p.cell_mm)),
    static_cast<int>(std::lround(p.far_mm / p.cell_mm))};
}

// 바닥 지도 칸 -> 원본 (raw_size 크기 영상) 변환 행렬 (box_planner::mapToRaw 와 같은 식, 세로만 바퀴 축까지)
inline cv::Mat fineToRaw(const FloorParams & p, cv::Size raw_size)
{
  const float w = static_cast<float>(raw_size.width), h = static_cast<float>(raw_size.height);
  const float b = static_cast<float>(p.bev_size_mm);
  const cv::Point2f src[4] = {
    {static_cast<float>(p.src_top_left_x) * w, static_cast<float>(p.src_top_y) * h},
    {static_cast<float>(p.src_top_right_x) * w, static_cast<float>(p.src_top_y) * h},
    {static_cast<float>(p.src_bottom_right_x) * w, static_cast<float>(p.src_bottom_y) * h},
    {static_cast<float>(p.src_bottom_left_x) * w, static_cast<float>(p.src_bottom_y) * h},
  };
  const cv::Point2f dst[4] = {{0, 0}, {b, 0}, {b, b}, {0, b}};
  const cv::Mat bev_to_raw = cv::getPerspectiveTransform(dst, src);
  // 칸 (u, v) -> 로봇 (x, y) = (u c - half, far - v c) -> BEV (x + b/2, b - (y - near))
  const double c = p.cell_mm;
  const cv::Mat fine_to_bev = (cv::Mat_<double>(3, 3) <<
    c, 0, -p.half_width_mm + b / 2,
    0, c, b - p.far_mm + p.bev_near_mm,
    0, 0, 1);
  return bev_to_raw * fine_to_bev;
}

// 원본 화면 마스크 (raw_size 아무 크기) -> 바닥 지도 (가장 가까운 픽셀)
inline cv::Mat warpMask(const cv::Mat & raw_mask, const FloorParams & p)
{
  cv::Mat out;
  cv::warpPerspective(raw_mask, out, fineToRaw(p, raw_mask.size()), fineSize(p),
    cv::INTER_NEAREST | cv::WARP_INVERSE_MAP, cv::BORDER_CONSTANT);
  return out;
}

// 카메라에 보이는 칸 (가장자리 보간 섞인 칸은 뺌)
inline cv::Mat visibleMask(const FloorParams & p, cv::Size raw_size)
{
  cv::Mat v = warpMask(cv::Mat(raw_size, CV_8U, cv::Scalar(255)), p);
  cv::erode(v, v, cv::Mat::ones(3, 3, CV_8U));
  return v;
}

inline FloorObs observeColor(
  const cv::Mat & raw, const FloorParams & p, const obstacle_detection::ColorParams & colors)
{
  FloorObs o;
  cv::Mat small;
  cv::resize(raw, small, cv::Size(), p.raw_scale, p.raw_scale, cv::INTER_AREA);
  cv::warpPerspective(small, o.ground, fineToRaw(p, small.size()), fineSize(p),
    cv::INTER_LINEAR | cv::WARP_INVERSE_MAP, cv::BORDER_CONSTANT);
  o.valid = visibleMask(p, small.size());
  obstacle_detection::ColorParams c = colors;
  c.box_min_area = std::max(4, static_cast<int>(colors.box_min_area / (p.cell_mm * p.cell_mm)));
  c.box_depth_px = std::max(1, static_cast<int>(std::lround(p.box_depth_mm / p.cell_mm)));
  c.box_gap_px = std::max(1, static_cast<int>(std::lround(p.box_gap_mm / p.cell_mm)));
  const auto m = obstacle_detection::buildColorMaps(o.ground, c);
  const cv::Mat hidden = ~o.valid;
  o.line = m.line_wall.clone();
  o.box = m.box_blocked.clone();
  o.yellow = m.yellow.clone();
  o.white = m.white.clone();
  for (cv::Mat * k : {&o.line, &o.box, &o.yellow, &o.white}) {
    k->setTo(0, hidden);
  }
  return o;
}

// 세그멘테이션 결과 (원본 화면 크기 클래스 마스크, 0/255. 비어 있으면 그 클래스 없음)
struct SegMasks
{
  cv::Mat road;    // 주행 가능 바닥
  cv::Mat line;    // 선
  cv::Mat box;     // 장애물
};

// raw: 디버그 그림용 원본 (비어도 됨). 마스크는 모두 같은 크기 (원본과 비율이 같으면 크기는 상관없음)
inline FloorObs observeSegmentation(const SegMasks & s, const FloorParams & p, const cv::Mat & raw = {})
{
  FloorObs o;
  const cv::Mat & any = !s.road.empty() ? s.road : !s.line.empty() ? s.line : s.box;
  const cv::Size fs = fineSize(p);
  o.valid = any.empty() ? cv::Mat::zeros(fs, CV_8U) : visibleMask(p, any.size());
  auto warpOr0 = [&](const cv::Mat & m) {
      return m.empty() ? cv::Mat(cv::Mat::zeros(fs, CV_8U)) : warpMask(m, p);
    };
  o.line = warpOr0(s.line);
  o.box = warpOr0(s.box);
  if (!s.road.empty()) {
    const cv::Mat road = warpMask(s.road, p) & o.valid;
    const int nv = cv::countNonZero(o.valid);
    if (nv > 0 && cv::countNonZero(road) >= p.road_min_ratio * nv) {
      o.offroad = o.valid & ~road;
    }
  }
  if (!raw.empty()) {
    cv::warpPerspective(raw, o.ground, fineToRaw(p, raw.size()), fs,
      cv::INTER_LINEAR | cv::WARP_INVERSE_MAP, cv::BORDER_CONSTANT);
  }
  return o;
}

// add 를 base 에 합친다 (보인 곳 / 선 / 박스 / 주행 불가는 합집합). 디버그 그림은 base 것이 없으면 add 것
inline void merge(FloorObs & base, const FloorObs & add)
{
  auto orInto = [](cv::Mat & a, const cv::Mat & b) {
      if (b.empty()) {
        return;
      }
      if (a.empty()) {
        a = b.clone();
      } else {
        cv::bitwise_or(a, b, a);
      }
    };
  orInto(base.valid, add.valid);
  orInto(base.line, add.line);
  orInto(base.box, add.box);
  orInto(base.offroad, add.offroad);
  if (base.ground.empty()) {
    base.ground = add.ground;
  }
}

}  // namespace grid_drive
