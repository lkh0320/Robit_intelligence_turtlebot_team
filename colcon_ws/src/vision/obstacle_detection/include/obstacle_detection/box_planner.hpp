// 공사 구간 경로 계획: 박스 사이로 로봇 폭이 지나갈 경로를 후보 중에서 고른다 (ROS 없이 테스트 가능)
//
//   공사 구간 (2026-10-13 녹화 110713_manual): 가운데 선 없는 넓은 검은 길, 오른쪽 흰 선 밖은 트랙 밖(밝은 바닥).
//   박스(갈색 벽돌 더미, 앞면 18cm) 3개가 길 위 좌우에 놓인다.
//   2026-10-14 실주행 (빈 공간 방향 조향): 첫 박스는 너무 일찍 꺾고, 나머지는 지나가다 옆으로 긁음
//     -> 방향 하나만 고르면 로봇 몸이 지나갈 수 있는지 모르고, 박스가 카메라 아래로 사라지면 잊어서 일찍 돌아온다.
//   처리 순서 (매 영상)
//     1. 바닥 지도: 원본을 bird_eye_view 와 같은 변환(사다리꼴 -> 1px = 1mm)으로 넓게 편다
//        (BEV 는 좌우 ±20cm, 바퀴 축 앞 18~58cm 뿐이라 옆으로 비켜 설 자리가 안 보인다).
//        cell_mm 칸, 좌우 ±half_width_mm, 앞 far_mm 까지. 원본 밖은 "모름"
//        색 마스크 / 박스 바닥 접점부터 먼 쪽 막힘 / 거리 변환은 color_maps.hpp
//     2. 박스 기억: 박스 바닥 접점(앞면)을 로봇 좌표 점으로 기억해 바퀴 이동량만큼 옮긴다.
//        카메라 아래로 사라진 뒤 (바퀴 옆) 에도 막힘으로 남아, 지나가며 박스 쪽으로 돌아오지 않게 한다.
//        접점 바로 아래 바닥이 보일 때만 기억 (시야 가장자리에 잘린 박스의 끝은 접점이 아님)
//        지금 카메라에 보이는 곳의 기억은 새 관측으로 바꾼다. 바퀴 축 뒤로 memory_back_mm 지나면 잊는다
//        지도 앞뒤 축척은 엔코더와 다르다 (2026-10-14 녹화 분석: 바닥 무늬 / 박스가 지도에서 엔코더의 약 2.4배
//        빠르게 다가옴, 좌우 축척은 박스 폭 18.9cm 로 맞음) -> odom_scale 로 맞춘다
//     3. 옆 PSD: 옆 PSD 가 가까우면 (박스 옆) 그쪽으로 psd_keep_mm 넘게 붙는 경로는 탈락
//     4. 후보 경로: 바퀴 축에서 정면으로 출발해 옆으로 d 만큼 비켜 서는 S자 x = d s(y / L) (s = 3t^2 - 2t^3)
//        + 완만한 원호 (길이 휘는 곳). d 는 ±max_shift_mm, L 은 shift_lengths, 곡률은 curvatures
//     5. 점수: 박스(보이는 것 / 기억 / PSD)에 반폭 + box_margin_mm 안이면 탈락 (react_mm 보다 먼 박스는 아직 무시)
//              선(흰 = 트랙 끝, 노란 = 섬)에 반폭 안이면 벌점, 선을 넘으면 탈락 (line_horizon_mm 안에서만)
//              넘음 검사는 샘플 사이 선분 전체 칸으로 (샘플 간격이 선 굵기보다 넓으면 비스듬한 선을 건너뜀)
//              지도 맨 아래에 닿은 선은 안 보이는 띠 (바퀴 축 ~ 지도 맨 아래) 로 below_extend_mm 만 직선으로 늘여 검사
//              (선 마스크가 지도 맨 아래 1~3칸 위에서 끊겨, 그 틈으로 선 밖으로 빠지는 경로가 안 걸렸다.
//              바퀴 축까지 길게 늘이면 휜 선 끝의 방향이 틀어져 로봇 앞을 가로막는다)
//              (2026-10-14 230313_test: 두 번째 박스 앞에서 오른쪽 흰 선 밖으로 나가는 경로를 골라 트랙 밖으로 나감)
//              선 / 박스에서 멀수록 가점, 많이 비켜 설수록 / 많이 휠수록 조금 벌점, 직전 경로와 다를수록 벌점
//     6. 고른 경로 위 lookahead_mm 앞 목표점 방향 -> angle (lane_follower 가 pure pursuit 로 조향)
//   좌표: 로봇 좌표 x 오른쪽, y 앞 [mm, 지도 단위], 원점 = 바퀴 축 가운데. 각도 + = 오른쪽
#pragma once

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "obstacle_detection/color_maps.hpp"

namespace obstacle_detection
{

struct PlannerParams
{
  // 1. 바닥 지도 (bird_eye_view.yaml 사다리꼴과 같게: BEV 400x400, 1px = 1mm, 맨 아래 = 바퀴 축 앞 179mm)
  double src_top_y = 0.360, src_top_left_x = 0.286, src_top_right_x = 0.714;
  double src_bottom_y = 1.0, src_bottom_left_x = -0.023, src_bottom_right_x = 1.023;
  double bev_size_mm = 400.0;
  double bev_near_mm = 179.0;          // BEV 맨 아래 = 바퀴 축 앞 [mm]
  double cell_mm = 4.0;
  double half_width_mm = 600.0;
  double far_mm = 1200.0;
  double raw_scale = 0.5;              // 원본을 이 배율로 줄여서 편다 (속도)
  // 로봇 / 박스
  double robot_half_width_mm = 102.5;  // 실측 폭 20.5cm
  double box_margin_mm = 30.0;         // 박스와 로봇 옆면 사이 최소 여유
  double box_depth_mm = 200.0;         // 기억한 박스 앞면에서 뒤로 막힘 길이 (지도 단위, 두께 8.5cm x odom_scale)
  double react_mm = 800.0;             // 이보다 먼 박스는 아직 안 피함 (너무 일찍 꺾지 않게)
  // 2. 기억
  double odom_scale = 2.4;             // 지도 앞뒤 [mm] / 엔코더 이동 [mm]
  double memory_back_mm = 250.0;       // 바퀴 축 뒤로 이만큼 지나면 잊음 (지도 단위)
  double memory_step_mm = 12.0;        // 박스 접점을 이 간격으로 기억
  // 3. PSD
  double psd_keep_mm = 15.0;           // 옆 PSD 가 가까우면 그쪽으로 이보다 더 붙는 경로 탈락
  double psd_len_mm = 400.0;           // 그 검사를 하는 앞 구간 (지도 단위)
  // 4. 후보
  double max_shift_mm = 450.0;
  double shift_step_mm = 25.0;
  std::vector<double> shift_lengths{300.0, 550.0, 900.0};
  // [1/mm] + 오른쪽. 공사 구간 앞 왼쪽 커브(흰 선이 앞을 둥글게 가로지름)를 따라가려면 반지름 350 까지 필요
  std::vector<double> curvatures{-1.0 / 350, -1.0 / 500, -1.0 / 900, -1.0 / 1800, 0.0,
    1.0 / 1800, 1.0 / 900, 1.0 / 500, 1.0 / 350};
  double max_path_curvature = 1.0 / 120.0;   // S자 최대 곡률 (이보다 급하면 후보에서 뺌)
  double path_length_mm = 1100.0;
  double sample_step_mm = 20.0;
  // 5. 점수
  double line_weight = 4.0;            // 반폭 안에 선이 들어온 샘플 비율 x 이 값
  double cross_line_mm = 10.0;         // 경로 중심이 선에 이보다 가까우면 넘은 것
  double line_horizon_mm = 700.0;      // 선 벌점은 이보다 가까운 곳만 (먼 곳은 다가가며 다시 계획)
  double cross_weight = 20.0;          // cross_blocks 가 false 일 때 선을 넘는 경로 벌점
  bool cross_blocks = true;            // 2026-10-13 녹화: 옆 PSD 로 한쪽이 막히면 선 넘는 쪽 경로를 골랐다
  double below_band_mm = 120.0;        // 지도 맨 아래에서 이만큼 위까지의 선 조각을 본다
  double below_extend_mm = 60.0;       // 아래 끝에서 이만큼 아래로 늘임 (지도 단위)
  double below_fit_mm = 60.0;          // 그 조각의 아래 끝에서 이만큼 위까지로 방향을 잡는다 (휜 선은 끝 쪽 방향만)
  double below_touch_mm = 12.0;        // 아래 끝이 지도 맨 아래에서 이만큼 안이어야 늘임
                                       // (시야 옆 가장자리로 빠져나간 선은 띠로 안 들어감)
  int below_min_cells = 8;             // 이보다 작은 선 조각은 안 늘임
  double below_min_slope = 0.5;        // 선 방향의 앞뒤 성분이 이보다 작으면 (가로로 누운 선) 안 늘임
  double clearance_cap_mm = 150.0;
  double clearance_weight = 1.0;
  double shift_weight = 0.3;           // |비켜 선 거리| / max_shift_mm x 이 값
  double curve_weight = 0.1;           // |원호 곡률| / 가장 큰 곡률 x 이 값 (아무것도 없으면 직진)
  double smooth_weight = 1.0;          // 직전 경로와 lookahead 지점 가로 차이 / 200mm x 이 값
  // 6. 출력
  double lookahead_mm = 300.0;
};

struct PlanResult
{
  bool ok = false;                     // 지나갈 경로가 있음
  // ok 가 아니면: 가장 멀리까지 막히지 않는 후보 (blocked_at = 처음 막히는 곳, 지도 단위).
  // lane_follower 는 천천히 / 가까우면 제자리에서 그 방향으로
  bool fallback = false;
  double blocked_at_mm = 0;
  double angle = 0;                    // 목표점 방향 [rad] + 오른쪽
  cv::Point2d target;                  // 목표점 (로봇 좌표)
  std::vector<cv::Point2d> path;       // 고른 경로 (로봇 좌표)
  // 가장 가까운 박스 (보이는 것 + 기억, 바퀴 축 앞쪽만)
  bool box_ahead = false;
  double box_dist_mm = 0;              // 바퀴 축에서 앞면까지 (지도 단위)
  double box_x_mm = 0;                 // 앞면 가운데 가로 위치
  int box_pixels = 0;                  // 이번 영상에 보인 박스 픽셀 (칸)
  // 지금 보이는 가장 가까운 박스 (기억 제외): 바닥 접점 가운데를 원본 화면 비율로 [0, 1]
  bool box_visible = false;
  double box_raw_x = 0, box_raw_bottom = 0;
  int memory_points = 0;
  int candidates = 0, blocked = 0;
};

class BoxPlanner
{
public:
  struct Cand
  {
    double d = 0, L = 0, k = 0;
    std::vector<cv::Point2d> pts;
    bool blocked = false;
    double blocked_at = 1e9;   // 처음 막히는 샘플의 y
    double score = -1e18;
  };

  // 바퀴 이동 (엔코더): ds 앞 [mm], dyaw 왼쪽 + [rad]
  void onMove(double ds_mm, double dyaw, const PlannerParams & p)
  {
    auto move = [&](std::vector<cv::Point2d> & pts) {
        const double half = dyaw / 2, c = std::cos(dyaw), s = std::sin(dyaw);
        const double tx = -ds_mm * std::sin(half), ty = ds_mm * std::cos(half);
        std::vector<cv::Point2d> out;
        out.reserve(pts.size());
        for (const auto & q : pts) {
          const double x = q.x - tx, y = q.y / p.odom_scale - ty;   // 지도 앞뒤 -> 엔코더 단위
          const cv::Point2d n(x * c + y * s, (-x * s + y * c) * p.odom_scale);
          if (n.y > -p.memory_back_mm) {
            out.push_back(n);
          }
        }
        pts = std::move(out);
      };
    move(memory_);
    move(prev_path_);
  }

  void reset()
  {
    memory_.clear();
    prev_path_.clear();
  }

  const std::vector<cv::Point2d> & memory() const {return memory_;}
  const std::vector<Cand> & candidates() const {return cands_;}
  const ColorMaps & maps() const {return maps_;}

  // 바닥 지도 크기 [칸]
  static cv::Size mapSize(const PlannerParams & p)
  {
    return {static_cast<int>(std::lround(2 * p.half_width_mm / p.cell_mm)),
      static_cast<int>(std::lround((p.far_mm - p.bev_near_mm) / p.cell_mm))};
  }

  // 바닥 지도 칸 -> 원본 (raw_size 크기 영상) 변환 행렬
  static cv::Mat mapToRaw(const PlannerParams & p, cv::Size raw_size)
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
    cv::Mat bev_to_raw = cv::getPerspectiveTransform(dst, src);   // BEV [mm] -> 원본
    // 칸 (u, v) -> 로봇 (x, y) = (u c - half, far - v c) -> BEV (x + b/2, b - (y - near))
    const double c = p.cell_mm;
    cv::Mat map_to_bev = (cv::Mat_<double>(3, 3) <<
      c, 0, -p.half_width_mm + b / 2,
      0, c, b - p.far_mm + p.bev_near_mm,
      0, 0, 1);
    return bev_to_raw * map_to_bev;
  }

  cv::Point2d toMap(const cv::Point2d & q, const PlannerParams & p) const
  {
    return {(q.x + p.half_width_mm) / p.cell_mm, (p.far_mm - q.y) / p.cell_mm};
  }

  // raw: 원본 BGR. colors: 색 기준 (box_min_area 는 1mm 칸 기준, 여기서 cell_mm 칸으로 바꾼다)
  PlanResult plan(
    const cv::Mat & raw, const PlannerParams & p, const ColorParams & colors,
    bool psd_left_close, bool psd_right_close)
  {
    PlanResult r;
    // 1. 바닥 지도
    cv::Mat small;
    cv::resize(raw, small, cv::Size(), p.raw_scale, p.raw_scale, cv::INTER_AREA);
    const cv::Size ms = mapSize(p);
    const cv::Mat H = mapToRaw(p, small.size());
    cv::warpPerspective(small, ground_, H, ms, cv::INTER_LINEAR | cv::WARP_INVERSE_MAP,
      cv::BORDER_CONSTANT);
    cv::warpPerspective(cv::Mat(small.size(), CV_8U, cv::Scalar(255)), valid_, H, ms,
      cv::INTER_NEAREST | cv::WARP_INVERSE_MAP, cv::BORDER_CONSTANT);
    cv::erode(valid_, valid_, cv::Mat::ones(3, 3, CV_8U));   // 가장자리 보간 섞인 칸 빼기
    ColorParams c = colors;
    c.box_min_area = std::max(4, static_cast<int>(colors.box_min_area / (p.cell_mm * p.cell_mm)));
    maps_ = buildColorMaps(ground_, c);
    r.box_pixels = maps_.box_pixels;
    if (maps_.near_box_area > 0) {
      const auto & nb = maps_.near_box;
      std::vector<cv::Point2d> in{{nb.x + nb.width / 2.0, nb.y + nb.height - 0.5}}, out;
      cv::perspectiveTransform(in, out, H);
      r.box_visible = true;
      r.box_raw_x = std::clamp(out[0].x / small.cols, 0.0, 1.0);
      r.box_raw_bottom = std::clamp(out[0].y / small.rows, 0.0, 1.0);
    }

    // 2. 박스 기억: 보이는 곳 기억은 버리고, 이번 박스 접점을 넣는다
    std::vector<cv::Point2d> kept;
    for (const auto & q : memory_) {
      const cv::Point2d m = toMap(q, p);
      const int u = cvRound(m.x), v = cvRound(m.y);
      if (u >= 0 && v >= 0 && u < ms.width && v < ms.height && valid_.at<uint8_t>(v, u)) {
        continue;
      }
      kept.push_back(q);
    }
    memory_ = std::move(kept);
    const int col_step = std::max(1, static_cast<int>(std::lround(p.memory_step_mm / p.cell_mm)));
    for (int u = 0; u < ms.width; u += col_step) {
      for (int v = ms.height - 1; v >= 0; --v) {   // 아래(로봇 쪽)부터 처음 막힌 칸 = 바닥 접점
        if (maps_.box_blocked.at<uint8_t>(v, u)) {
          // 바로 아래 바닥이 보일 때만 진짜 접점. 시야 가장자리에 잘린 박스의 잘린 끝을 기억하면
          // 지나간 뒤 시야 경계를 따라 가짜 박스 줄이 남는다 (진짜 접점은 전에 보였을 때 기억해 둔 것이 남음)
          const int below = v + 2;
          if (below < ms.height && valid_.at<uint8_t>(below, u)) {
            memory_.emplace_back(u * p.cell_mm - p.half_width_mm, p.far_mm - v * p.cell_mm);
          }
          break;
        }
      }
    }
    r.memory_points = static_cast<int>(memory_.size());
    for (const auto & q : memory_) {
      if (q.y > -p.box_depth_mm && (!r.box_ahead || q.y < r.box_dist_mm)) {
        r.box_ahead = true;
        r.box_dist_mm = q.y;
        r.box_x_mm = q.x;
      }
    }

    // 3.5 안 보이는 띠 (바퀴 축 ~ 지도 맨 아래) 로 선 늘이기
    extendLinesBelow(ms, p);

    // 4~5. 후보
    const double reach = p.robot_half_width_mm + p.box_margin_mm;
    const double cap = std::max(1.0, p.clearance_cap_mm);
    const double prev_x = prevAt(p.lookahead_mm);
    // 샘플 간격이 선 굵기보다 넓으면 선을 건너뛰므로, 간격 절반 넘게 가까우면 넘은 것으로 본다
    const double cross_mm = std::max(p.cross_line_mm, 0.6 * p.sample_step_mm);
    double k_max = 1e-12;
    for (double k : p.curvatures) {
      k_max = std::max(k_max, std::abs(k));
    }
    cands_.clear();
    int best = -1;
    for (double L : p.shift_lengths) {
      for (double d = -p.max_shift_mm; d <= p.max_shift_mm + 1e-6; d += p.shift_step_mm) {
        if (std::abs(6 * d / (L * L)) > p.max_path_curvature) {
          continue;
        }
        for (double k : p.curvatures) {
          Cand cd;
          cd.d = d;
          cd.L = L;
          cd.k = k;
          double line_n = 0, clear_sum = 0, crossed = 0;
          int seen = 0;
          for (double y = 0; y <= p.path_length_mm + 1e-6; y += p.sample_step_mm) {
            const double t = std::clamp(y / L, 0.0, 1.0);
            const cv::Point2d q(d * t * t * (3 - 2 * t) + 0.5 * k * y * y, y);
            const cv::Point2d q_prev = cd.pts.empty() ? q : cd.pts.back();
            cd.pts.push_back(q);
            if (blockedByMemory(q, reach, p) || blockedByPsd(q, psd_left_close, psd_right_close, p)) {
              cd.blocked = true;
              cd.blocked_at = y;
              break;
            }
            if (p.cross_blocks && q.y <= p.line_horizon_mm && crossesLine(q_prev, q, p)) {
              cd.blocked = true;   // 선 넘음 = 트랙 밖
              cd.blocked_at = y;
              break;
            }
            const cv::Point2d m = toMap(q, p);
            const int u = cvRound(m.x), v = cvRound(m.y);
            if (v >= ms.height) {
              continue;  // 바퀴 축 ~ 지도 맨 아래 (카메라에 안 보이는 구간)
            }
            if (u < 0 || u >= ms.width) {
              if (q.y <= p.line_horizon_mm) {   // 가까이에서 지도 옆 (±half_width) 으로 나감 = 트랙 밖, 못 봄
                cd.blocked = true;
                cd.blocked_at = y;
              }
              break;
            }
            if (v < 0) {
              break;     // 지도 앞으로 나감
            }
            const double dl = maps_.dist_line.at<float>(v, u) * p.cell_mm;
            if (!valid_.at<uint8_t>(v, u)) {
              // 안 보이는 칸: 선이 시야 가장자리에서 끊겨 보여도 그 끝을 지나 시야 밖으로 빠지는 경로는 넘은 것
              if (p.cross_blocks && q.y <= p.line_horizon_mm && dl < cross_mm) {
                cd.blocked = true;
                cd.blocked_at = y;
                break;
              }
              continue;
            }
            const double db = maps_.dist_box.at<float>(v, u) * p.cell_mm;
            if (q.y <= p.react_mm && db < reach) {
              cd.blocked = true;
              cd.blocked_at = y;
              break;
            }
            if (p.cross_blocks && q.y <= p.line_horizon_mm && dl < cross_mm) {
              cd.blocked = true;   // 선 넘음 = 트랙 밖 (막힌 것과 같게: 전부 막히면 가장 멀리 가는 경로로 천천히)
              cd.blocked_at = y;
              break;
            }
            ++seen;
            if (q.y <= p.line_horizon_mm) {
              line_n += dl < p.robot_half_width_mm;
              crossed += dl < cross_mm;
            }
            clear_sum += std::min({dl, q.y <= p.react_mm ? db : cap, cap}) / cap;
          }
          if (!cd.blocked && seen > 0) {
            cd.score = p.clearance_weight * clear_sum / seen - p.line_weight * line_n / seen -
              (crossed > 0 ? p.cross_weight : 0.0) -
              p.shift_weight * std::abs(d) / std::max(1.0, p.max_shift_mm) -
              p.curve_weight * std::abs(k) / k_max;
            if (!prev_path_.empty()) {
              cd.score -= p.smooth_weight * std::abs(xAt(cd, p.lookahead_mm) - prev_x) / 200.0;
            }
            if (best < 0 || cd.score > cands_[best].score) {
              best = static_cast<int>(cands_.size());
            }
          } else if (!cd.blocked) {
            cd.score = -1e9;   // 하나도 안 보임 -> 다른 게 없을 때만
            if (best < 0) {
              best = static_cast<int>(cands_.size());
            }
          }
          r.blocked += cd.blocked;
          cands_.push_back(std::move(cd));
        }
      }
    }
    r.candidates = static_cast<int>(cands_.size());
    r.ok = best >= 0;
    if (best < 0) {
      // 전부 막힘: 가장 멀리까지 가는 후보 (같으면 덜 비켜 서는 것)
      for (size_t i = 0; i < cands_.size(); ++i) {
        const auto & c = cands_[i];
        if (best < 0 || c.blocked_at > cands_[best].blocked_at + 1e-6 ||
          (std::abs(c.blocked_at - cands_[best].blocked_at) < 1e-6 &&
          std::abs(c.d) < std::abs(cands_[best].d)))
        {
          best = static_cast<int>(i);
        }
      }
      if (best < 0) {
        return r;
      }
      r.fallback = true;
      r.blocked_at_mm = cands_[best].blocked_at;
    }
    best_ = best;
    const auto & b = cands_[best];
    r.path = b.pts;
    r.target = {xAt(b, p.lookahead_mm), p.lookahead_mm};
    r.angle = std::atan2(r.target.x, r.target.y);
    prev_path_ = b.pts;
    return r;
  }

  // 디버그 그림: 지도 (본 곳 회색, 선, 박스 빨강, 기억 보라, 후보, 고른 경로 초록)
  cv::Mat draw(const PlanResult & r, const PlannerParams & p, bool show_candidates = true) const
  {
    const cv::Size ms = mapSize(p);
    const int extra = static_cast<int>(std::lround((p.bev_near_mm + p.memory_back_mm) / p.cell_mm));
    cv::Mat vis(ms.height + extra, ms.width, CV_8UC3, cv::Scalar(40, 25, 25));
    cv::Mat top = vis(cv::Rect(0, 0, ms.width, ms.height));
    if (!ground_.empty()) {
      cv::Mat g;
      cv::cvtColor(ground_, g, cv::COLOR_BGR2GRAY);
      cv::cvtColor(g, g, cv::COLOR_GRAY2BGR);
      g.copyTo(top, valid_);
      top.setTo(cv::Scalar(255, 255, 255), maps_.white);
      top.setTo(cv::Scalar(0, 210, 255), maps_.yellow);
      top.setTo(cv::Scalar(0, 0, 160), maps_.box_blocked);
      top.setTo(cv::Scalar(0, 0, 255), maps_.box);
    }
    auto px = [&](const cv::Point2d & q) {
        const cv::Point2d m = toMap(q, p);
        return cv::Point(cvRound(m.x), cvRound(m.y));
      };
    // react 경계
    cv::line(vis, px({-p.half_width_mm, p.react_mm}), px({p.half_width_mm, p.react_mm}),
      cv::Scalar(90, 90, 0), 1);
    if (show_candidates) {
      for (size_t i = 0; i < cands_.size(); i += 3) {
        const auto & c = cands_[i];
        for (size_t j = 1; j < c.pts.size(); j += 2) {
          vis.at<cv::Vec3b>(std::clamp(px(c.pts[j]).y, 0, vis.rows - 1),
            std::clamp(px(c.pts[j]).x, 0, vis.cols - 1)) =
            c.blocked ? cv::Vec3b(0, 0, 110) : cv::Vec3b(150, 150, 150);
        }
      }
    }
    for (const auto & q : memory_) {
      cv::circle(vis, px(q), 2, cv::Scalar(255, 0, 255), cv::FILLED);
    }
    for (const auto & l : lines_below_) {
      cv::line(vis, px(l.first), px(l.second), cv::Scalar(200, 200, 200), 1);
    }
    for (size_t j = 1; j < r.path.size(); ++j) {
      cv::line(vis, px(r.path[j - 1]), px(r.path[j]), cv::Scalar(0, 255, 0), 2);
    }
    if (r.ok) {
      cv::circle(vis, px(r.target), 4, cv::Scalar(255, 0, 255), cv::FILLED);
    }
    // 로봇 (폭 x 바퀴 축 앞뒤 대략)
    cv::rectangle(vis, px({-p.robot_half_width_mm, 60}), px({p.robot_half_width_mm, -100}),
      cv::Scalar(255, 255, 0), 1);
    cv::line(vis, px({-p.half_width_mm, p.bev_near_mm}), px({p.half_width_mm, p.bev_near_mm}),
      cv::Scalar(255, 255, 0), 1);
    return vis;
  }

private:
  // 지도 맨 아래에 닿은 선 조각마다 아래 끝 방향으로 직선을 맞춰 below_extend_mm 만큼 아래로 늘인다
  void extendLinesBelow(const cv::Size & ms, const PlannerParams & p)
  {
    lines_below_.clear();
    const int band = std::clamp(static_cast<int>(std::lround(p.below_band_mm / p.cell_mm)), 2, ms.height);
    const int v0 = ms.height - band;
    cv::Mat labels, stats, centroids;
    const int n = cv::connectedComponentsWithStats(maps_.line_wall(cv::Rect(0, v0, ms.width, band)),
      labels, stats, centroids, 8, CV_32S);
    for (int i = 1; i < n; ++i) {
      if (stats.at<int>(i, cv::CC_STAT_AREA) < p.below_min_cells) {
        continue;
      }
      std::vector<cv::Point2f> all, pts;
      double y_low = 1e9;
      for (int v = 0; v < band; ++v) {
        for (int u = 0; u < ms.width; ++u) {
          if (labels.at<int>(v, u) == i) {
            const double y = p.far_mm - (v + v0) * p.cell_mm;
            all.emplace_back(static_cast<float>(u * p.cell_mm - p.half_width_mm), static_cast<float>(y));
            y_low = std::min(y_low, y);
          }
        }
      }
      if (y_low > p.bev_near_mm + p.below_touch_mm) {
        continue;
      }
      for (const auto & q : all) {
        if (q.y <= y_low + p.below_fit_mm) {
          pts.push_back(q);
        }
      }
      if (static_cast<int>(pts.size()) < p.below_min_cells) {
        continue;
      }
      cv::Vec4f l;
      cv::fitLine(pts, l, cv::DIST_L2, 0, 0.01, 0.01);
      double vx = l[0], vy = l[1];
      if (vy < 0) {
        vx = -vx;
        vy = -vy;
      }
      if (vy < p.below_min_slope || y_low <= 0) {
        continue;
      }
      auto at = [&](double y) {return cv::Point2d(l[2] + vx * (y - l[3]) / vy, y);};
      lines_below_.emplace_back(at(y_low), at(std::max(0.0, y_low - p.below_extend_mm)));
    }
  }

  // 경로 선분 a -> b 가 선을 넘는지: 지도 칸 (선분이 지나는 칸 전부) + 안 보이는 띠에 늘인 선
  bool crossesLine(const cv::Point2d & a, const cv::Point2d & b, const PlannerParams & p) const
  {
    for (const auto & l : lines_below_) {
      if (segmentsCross(a, b, l.first, l.second)) {
        return true;
      }
    }
    const cv::Point2d ma = toMap(a, p), mb = toMap(b, p);
    cv::LineIterator it(maps_.line_wall, cv::Point(cvRound(ma.x), cvRound(ma.y)),
      cv::Point(cvRound(mb.x), cvRound(mb.y)), 8);
    for (int i = 0; i < it.count; ++i, ++it) {
      if (**it) {
        return true;
      }
    }
    return false;
  }

  static bool segmentsCross(
    const cv::Point2d & a, const cv::Point2d & b, const cv::Point2d & c, const cv::Point2d & d)
  {
    auto cross = [](const cv::Point2d & o, const cv::Point2d & u, const cv::Point2d & w) {
        return (u.x - o.x) * (w.y - o.y) - (u.y - o.y) * (w.x - o.x);
      };
    const double d1 = cross(c, d, a), d2 = cross(c, d, b), d3 = cross(a, b, c), d4 = cross(a, b, d);
    return ((d1 > 0) != (d2 > 0)) && ((d3 > 0) != (d4 > 0));
  }

  bool blockedByMemory(const cv::Point2d & q, double reach, const PlannerParams & p) const
  {
    for (const auto & m : memory_) {
      // 기억한 박스: 앞면 점에서 뒤로 box_depth_mm 막힘. 아직 먼 박스(react 밖)는 무시
      if (m.y > p.react_mm) {
        continue;
      }
      // 이미 지금 로봇 자리에 겹친 점 (바퀴 축 옆 / 뒤, 반폭 안) 은 모든 후보가 막히므로 뺀다
      // (멈추면 못 빠져나온다. 옆 PSD 규칙이 그쪽으로 더 붙지 않게 막는다)
      if (m.y < p.sample_step_mm && std::abs(m.x) < reach) {
        continue;
      }
      const double dy = q.y < m.y ? m.y - q.y : std::max(0.0, q.y - (m.y + p.box_depth_mm));
      if (std::abs(q.x - m.x) < reach && dy < p.box_margin_mm) {
        return true;
      }
    }
    return false;
  }

  bool blockedByPsd(const cv::Point2d & q, bool left, bool right, const PlannerParams & p) const
  {
    if (q.y > p.psd_len_mm) {
      return false;
    }
    return (left && q.x < -p.psd_keep_mm) || (right && q.x > p.psd_keep_mm);
  }

  static double xAt(const Cand & c, double y)
  {
    const double t = std::clamp(y / c.L, 0.0, 1.0);
    return c.d * t * t * (3 - 2 * t) + 0.5 * c.k * y * y;
  }

  // 직전 경로 (이동량만큼 옮긴 것) 의 y 에서 가로 위치
  double prevAt(double y) const
  {
    for (size_t i = 1; i < prev_path_.size(); ++i) {
      const auto & a = prev_path_[i - 1];
      const auto & b = prev_path_[i];
      if (a.y <= y && b.y >= y && b.y > a.y) {
        return a.x + (b.x - a.x) * (y - a.y) / (b.y - a.y);
      }
    }
    return prev_path_.empty() ? 0.0 : prev_path_.back().x;
  }

  std::vector<cv::Point2d> memory_;      // 박스 앞면 점 (로봇 좌표, 지도 단위)
  std::vector<cv::Point2d> prev_path_;   // 직전에 고른 경로 (이동량만큼 옮김)
  std::vector<std::pair<cv::Point2d, cv::Point2d>> lines_below_;   // 안 보이는 띠로 늘인 선 (아래 끝 -> 더 아래)
  cv::Mat ground_, valid_;
  ColorMaps maps_;
  std::vector<Cand> cands_;
  int best_ = -1;
};

}  // namespace obstacle_detection
