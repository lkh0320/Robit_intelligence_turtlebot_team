// 비용 지도 위 최단 경로 + 곡선 후보로 따라가기 (ROS 없이 테스트 가능)
//
//   4번 (어디로 갈지): 바닥 지도 -> 거리 변환 비용 지도 -> Dijkstra 로 위쪽 / 옆쪽 끝 칸 중 가장 싼 곳까지 경로
//   3번 (어떻게 조향할지): 바퀴 축에서 출발하는 원호 후보 중 그 경로에 가장 가깝고 진입 금지에 안 닿는 것
//
//   처리 순서 (매 영상)
//     1. 바닥 지도 관측 (floor_obs.hpp): 보인 곳 / 선 / 박스 / 주행 불가 마스크. 색 방식이나 세그멘테이션 모델 어느 쪽이든
//        지도는 바퀴 축 (y = 0) 부터 far_mm 까지. 바퀴 축 ~ bev_near_mm 는 카메라에 안 보이는 띠
//     2. 기억: 선 / 박스 칸을 바퀴 이동량으로 옮기며 기억한다 (지금 보이는 곳은 새 관측으로 바꿈).
//        안 보이는 띠 / 시야 옆으로 빠진 선도 막힘으로 남아 코너를 깎지 않게 한다
//     3. 비용 지도 (실제 축척 plan_cell_mm 정사각 칸 = 지도 가로 plan_cell, 세로 plan_cell x map_y_scale):
//        선 / 박스 / 주행 불가까지 거리 d [실제 mm]
//        d < 반폭 + 여유 -> 진입 금지, 그 밖은 1 + clear_k / d (경계에 가까울수록 비쌈, clear_cap 넘으면 1)
//        안 보이는 칸 + unknown_cost, 직전 경로 근처는 x prev_discount (프레임마다 경로가 튀지 않게)
//     4. Dijkstra 한 번 (8방향): 바퀴 축 칸에서 모든 칸까지 비용
//        목표 = 위쪽 끝 띠 / 옆쪽 끝 띠 중 (도착 비용 + 목표 가중치) 가 가장 작은 칸
//          평소: 직전 목표와 가까울수록 (흔들림 방지), 옆쪽 끝은 벌점 (위쪽이 막힌 U자 커브에서만 감)
//          turn = LEFT / RIGHT (교차로 표지판): 그쪽 가장자리 쪽일수록 가점, 반대쪽 옆 끝은 큰 벌점
//     5. 다듬기: 격자 경로를 sample_mm 간격으로 다시 뽑아 이동 평균
//     6. 따라가기: 곡률이 다른 원호 (실제 축척) 중 경로와의 평균 거리 + 곡률 + 직전 곡률과 차이가 가장 작은 것.
//        check_mm 안에서 진입 금지 칸에 닿는 원호는 탈락. 전부 막히면 경로 방향으로 제자리 회전, 경로도 없으면 정지
//     4b. 진입 금지만으로는 목표가 없으면 (로봇이 이미 선에 붙음 / 선 위, 2026-10-27 녹화 재생 1667장 중 976장)
//        진입 금지 칸을 soft_forbid_cost (선 / 박스 칸 자체는 + soft_cross_cost) 로 바꿔 한 번 더 찾는다 (soft).
//        soft 면 원호도 탈락 대신 벌점, 속도는 soft_speed_ratio 배 -> 선을 가장 덜 침범하는 쪽으로 천천히 빠져나감
//
//   좌표: 로봇 좌표 x 오른쪽, y 앞 [mm, 지도 단위], 원점 = 바퀴 축 가운데. 곡률 + = 오른쪽으로 휨
//   지도 앞뒤는 실제의 map_y_scale 배 (box_planner odom_scale, 2026-10-14 녹화 분석) -> 원호 / 조향은 실제 축척으로 계산
#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "grid_drive/floor_obs.hpp"

namespace grid_drive
{

enum class Turn {NONE, LEFT, RIGHT};

// 1. 바닥 지도 크기 / 변환은 FloorParams (floor_obs.hpp)
struct Params : FloorParams
{
  double map_y_scale = 1.0;            // 지도 앞뒤 [mm] / 실제 [mm] (box_planner odom_scale)
  // 2. 기억
  bool memory = false;                 // 2026-10-28: 벨트 미끄럼 / 직진 중 틀어짐을 odom 이 못 봐 기억이 어긋남 (replay soft 31 -> 102)
  double memory_back_mm = 250.0;       // 바퀴 축 뒤로 이만큼 지나면 잊음 (지도 단위)
  // 3. 비용 지도
  double plan_cell_mm = 12.0;          // 탐색 칸 [실제 mm] (색 마스크보다 거칠게, 속도. 지도 세로로는 x map_y_scale)
  double robot_half_width_mm = 100.0;  // 바퀴 바깥 끝 폭 20cm (2026-10-28 사용자)
  double line_margin_mm = 10.0;        // 선 / 주행 불가와 로봇 옆면 사이 최소 여유 (선 밟으면 실격)
  // 복귀 구간: 로봇이 이미 가운데에서 벗어나 있으면 바로 앞은 위 여유를 못 지킴 (차선 320mm 면 가운데 +-47mm 뿐).
  // 실제 recover_mm 앞까지는 선 / 주행 불가에서 경로 중심이 hard_line_mm 만 떨어지면 되고 (대신 near_line_cost),
  // 그 너머는 원래 여유 (로봇 반폭 + line_margin) 를 지켜야 함 -> 차선 안이면 hard 경로로 가운데로 돌아옴
  double recover_mm = 200.0;           // [실제 mm]
  double hard_line_mm = 30.0;
  double near_line_cost = 6.0;
  double box_margin_mm = 30.0;         // 박스와 로봇 옆면 사이 최소 여유
  double clear_k_mm = 40.0;            // 비용 = 1 + clear_k / d (d = 진입 금지 경계까지 [mm])
  double clear_min_mm = 6.0;           // d 가 이보다 작으면 이 값으로 (비용 상한)
  double clear_cap_mm = 150.0;         // d 가 이보다 크면 비용 1
  double unknown_cost = 2.0;           // 안 보이는 칸 (기억도 없음) 에 더함
  // 안 보이는 띠 (바퀴 축 ~ bev_near) 는 로봇이 원호로 갈 수 있는 곳 (+ blind_half_mm) 만 지나감.
  // 2026-10-27 녹화 재생: 이 띠가 비어 있어 경로가 띠를 따라 옆으로 빠져 선 바깥으로 돌아감
  double blind_half_mm = 24.0;
  double prev_discount = 0.85;         // 직전 경로 prev_band_mm 안 칸 비용에 곱함
  double prev_band_mm = 30.0;
  double start_escape_mm = 60.0;       // 바퀴 축 이 안의 진입 금지 칸은 지나갈 수 있게 (선에 붙어 섰을 때 출발)
  double escape_cost = 8.0;            // 그 칸 비용
  double soft_forbid_cost = 15.0;      // soft: 진입 금지 칸 비용
  double soft_cross_cost = 60.0;       // soft: 선 / 박스 칸 자체에 더함
  double soft_arc_weight = 300.0;      // soft: 원호가 진입 금지 칸을 지나는 비율 x 이 값 [mm]
  double soft_speed_ratio = 0.5;
  // 4. 목표
  double goal_band_mm = 36.0;          // 위쪽 / 옆쪽 끝에서 이만큼 안 칸이 목표 후보
  double side_goal_min_y_mm = 300.0;   // 옆쪽 끝 목표는 이보다 앞 (지도 단위)
  double continuity_weight = 0.5;      // |목표 - 직전 목표| [mm] x 이 값
  double side_penalty_mm = 600.0;      // 평소 옆쪽 끝 목표 벌점 (위쪽이 막혔을 때만 옆으로)
  double turn_weight_mm = 900.0;       // turn 이면 (가로 위치 / half_width) x 이 값 만큼 그쪽 가점
  double turn_side_bonus_mm = 400.0;   // turn 쪽 옆쪽 끝 목표 가점
  double turn_wrong_side_mm = 3000.0;  // turn 반대쪽 옆쪽 끝 목표 벌점
  // 5. 다듬기
  double sample_mm = 20.0;
  int smooth_iters = 4;
  int smooth_half = 3;                 // 이동 평균 반창 [점]
  // 6. 따라가기 (실제 축척)
  int arc_count = 41;
  double max_curvature = 1.0 / 120.0;  // [1/mm 실제]
  double follow_mm = 250.0;            // 원호 길이 [실제 mm]
  double check_mm = 200.0;             // 이 안에서 진입 금지에 닿으면 탈락 [실제 mm]
  double arc_step_mm = 10.0;
  double curvature_weight = 10.0;      // |k| / max_curvature x 이 값 [mm]
  double change_weight = 15.0;         // |k - 직전 k| / max_curvature x 이 값 [mm]
  // 출력
  double linear = 0.10;                // [m/s]
  double min_linear = 0.04;
  double slow_curvature = 0.6;         // v = linear x (1 - 이 값 x |k| / max_curvature)
  double max_angular = 1.5;            // [rad/s]
  double pivot_angular = 0.6;          // 원호가 전부 막히면 경로 방향으로 제자리 회전 [rad/s]
  double pivot_lookahead_mm = 150.0;   // 그 방향을 보는 경로 위 거리 [실제 mm]
};

struct Result
{
  bool path_ok = false;                // Dijkstra 경로 있음
  bool follow_ok = false;              // 막히지 않은 원호 있음
  bool pivot = false;                  // 원호가 전부 막혀 제자리 회전
  bool soft = false;                   // 진입 금지를 비용으로 바꿔 찾은 경로 (선에 붙음)
  double v = 0, w = 0;                 // cmd_vel [m/s], [rad/s] (w + = 왼쪽, ROS)
  double curvature = 0;                // 고른 원호 [1/mm 실제], + 오른쪽
  int goal_kind = 0;                   // 0 위쪽, 1 왼쪽 끝, 2 오른쪽 끝
  cv::Point2d goal;                    // 목표 (로봇 좌표)
  std::vector<cv::Point2d> raw_path;   // 격자 경로 (로봇 좌표, 지도 단위)
  std::vector<cv::Point2d> path;       // 다듬은 경로
  std::string reason;                  // 정지 / 회전 이유 (로그용)
  int line_memory = 0, box_memory = 0;
  double ms_map = 0, ms_search = 0, ms_follow = 0;
};

class GridPlanner
{
public:
  // 바퀴 이동 (엔코더): ds 앞 [mm 실제], dyaw 왼쪽 + [rad]
  void onMove(double ds_mm, double dyaw, const Params & p)
  {
    const double half = dyaw / 2, c = std::cos(dyaw), s = std::sin(dyaw);
    const double tx = -ds_mm * std::sin(half), ty = ds_mm * std::cos(half);
    auto move = [&](cv::Point2d q) {
        const double x = q.x - tx, y = q.y / p.map_y_scale - ty;
        return cv::Point2d(x * c + y * s, (-x * s + y * c) * p.map_y_scale);
      };
    auto moveAll = [&](std::vector<cv::Point2d> & pts) {
        std::vector<cv::Point2d> out;
        out.reserve(pts.size());
        for (const auto & q : pts) {
          const cv::Point2d n = move(q);
          if (n.y > -p.memory_back_mm) {
            out.push_back(n);
          }
        }
        pts = std::move(out);
      };
    moveAll(line_mem_);
    moveAll(box_mem_);
    moveAll(prev_path_);
    if (have_prev_goal_) {
      prev_goal_ = move(prev_goal_);
    }
  }

  void reset()
  {
    line_mem_.clear();
    box_mem_.clear();
    prev_path_.clear();
    have_prev_goal_ = false;
    prev_k_ = 0;
  }

  // 탐색 칸 한 줄의 지도 세로 길이 (실제로는 plan_cell_mm)
  static double rowMm(const Params & p) {return p.plan_cell_mm * p.map_y_scale;}

  static cv::Size gridSize(const Params & p)
  {
    return {static_cast<int>(std::lround(2 * p.half_width_mm / p.plan_cell_mm)),
      static_cast<int>(std::lround(p.far_mm / rowMm(p)))};
  }

  // 로봇 좌표 <-> 탐색 칸
  static cv::Point toCell(const cv::Point2d & q, const Params & p)
  {
    return {static_cast<int>(std::floor((q.x + p.half_width_mm) / p.plan_cell_mm)),
      static_cast<int>(std::floor((p.far_mm - q.y) / rowMm(p)))};
  }

  static cv::Point2d toXY(int c, int r, const Params & p)
  {
    return {(c + 0.5) * p.plan_cell_mm - p.half_width_mm, p.far_mm - (r + 0.5) * rowMm(p)};
  }

  // 실제 축척 원호: 길이 s 지점 (로봇 좌표, 지도 단위). k + = 오른쪽
  static cv::Point2d arcPoint(double k, double s, const Params & p)
  {
    double x, y;
    if (std::abs(k) < 1e-9) {
      x = 0;
      y = s;
    } else {
      x = (1 - std::cos(k * s)) / k;
      y = std::sin(k * s) / k;
    }
    return {x, y * p.map_y_scale};
  }

  // 색 방식 관측으로 계획 (원본 BGR)
  Result plan(const cv::Mat & raw, const Params & p, const obstacle_detection::ColorParams & colors,
    Turn turn = Turn::NONE)
  {
    const auto t0 = std::chrono::steady_clock::now();
    FloorObs obs = observeColor(raw, p, colors);
    Result r = plan(std::move(obs), p, turn);
    r.ms_map += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() -
      (r.ms_map + r.ms_search + r.ms_follow);
    return r;
  }

  // 관측 (색 / 세그멘테이션 / 합친 것) 으로 계획
  Result plan(FloorObs obs, const Params & p, Turn turn = Turn::NONE)
  {
    using clk = std::chrono::steady_clock;
    const auto t0 = clk::now();
    Result r;
    obs_ = std::move(obs);
    shrinkObs(p);
    updateMemory(p);
    buildCost(p);
    r.line_memory = static_cast<int>(line_mem_.size());
    r.box_memory = static_cast<int>(box_mem_.size());
    const auto t1 = clk::now();
    search(r, p, turn, false);
    if (!r.path_ok) {
      search(r, p, turn, true);
      r.soft = r.path_ok;
      if (!r.path_ok) {
        prev_path_.clear();
        have_prev_goal_ = false;
      }
    }
    const auto t2 = clk::now();
    follow(r, p);
    const auto t3 = clk::now();
    r.ms_map = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.ms_search = std::chrono::duration<double, std::milli>(t2 - t1).count();
    r.ms_follow = std::chrono::duration<double, std::milli>(t3 - t2).count();
    return r;
  }

  // 디버그 그림 (색 마스크 지도 크기): 본 곳 회색, 흰 / 노란 선, 박스 빨강, 기억 (선 하늘 / 박스 보라),
  // 진입 금지 붉은 기운, 격자 경로 파랑, 다듬은 경로 하늘, 원호 회색, 고른 원호 초록 (회전이면 주황), 목표 원
  cv::Mat draw(const Result & r, const Params & p) const
  {
    const cv::Size fs = fineSize(p);
    cv::Mat vis(fs, CV_8UC3, cv::Scalar(40, 25, 25));
    if (!obs_.valid.empty()) {
      if (!obs_.ground.empty()) {
        cv::Mat g;
        cv::cvtColor(obs_.ground, g, cv::COLOR_BGR2GRAY);
        cv::cvtColor(g, g, cv::COLOR_GRAY2BGR);
        g.copyTo(vis, obs_.valid);
      } else {
        vis.setTo(cv::Scalar(90, 90, 90), obs_.valid);
      }
      if (!obs_.offroad.empty()) {
        vis.setTo(cv::Scalar(70, 40, 40), obs_.offroad);
      }
      cv::Mat forbid_big;
      cv::resize(forbid_, forbid_big, fs, 0, 0, cv::INTER_NEAREST);
      cv::Mat tint(fs, CV_8UC3, cv::Scalar(0, 0, 160));
      cv::Mat blended;
      cv::addWeighted(vis, 0.6, tint, 0.4, 0, blended);
      blended.copyTo(vis, forbid_big);
      vis.setTo(cv::Scalar(255, 255, 255), obs_.line);
      if (!obs_.yellow.empty()) {
        vis.setTo(cv::Scalar(0, 210, 255), obs_.yellow);
      }
      vis.setTo(cv::Scalar(0, 0, 255), obs_.box);
    }
    auto px = [&](const cv::Point2d & q) {
        return cv::Point(cvRound((q.x + p.half_width_mm) / p.cell_mm),
                 cvRound((p.far_mm - q.y) / p.cell_mm));
      };
    for (const auto & q : line_mem_) {
      cv::circle(vis, px(q), 1, cv::Scalar(255, 200, 0), cv::FILLED);
    }
    for (const auto & q : box_mem_) {
      cv::circle(vis, px(q), 1, cv::Scalar(200, 0, 200), cv::FILLED);
    }
    // 시야 아래 끝 (바퀴 축 ~ 여기는 안 보이는 띠)
    const int near_v = px({0, p.bev_near_mm}).y;
    cv::line(vis, {0, near_v}, {fs.width, near_v}, cv::Scalar(90, 90, 90), 1);
    for (const auto & a : arcs_) {
      for (size_t i = 1; i < a.size(); ++i) {
        cv::line(vis, px(a[i - 1]), px(a[i]), cv::Scalar(110, 110, 110), 1);
      }
    }
    for (size_t i = 1; i < r.raw_path.size(); ++i) {
      cv::line(vis, px(r.raw_path[i - 1]), px(r.raw_path[i]), cv::Scalar(255, 80, 0), 1);
    }
    for (size_t i = 1; i < r.path.size(); ++i) {
      cv::line(vis, px(r.path[i - 1]), px(r.path[i]), cv::Scalar(255, 255, 0), 2);
    }
    if (r.path_ok) {
      cv::circle(vis, px(r.goal), 6, cv::Scalar(255, 0, 255), 2);
    }
    if (r.follow_ok || r.pivot) {
      const cv::Scalar col = r.pivot ? cv::Scalar(0, 140, 255) : cv::Scalar(0, 255, 0);
      cv::Point prev = px(arcPoint(r.curvature, 0, p));
      for (double s = p.arc_step_mm; s <= p.follow_mm + 1e-6; s += p.arc_step_mm) {
        const cv::Point cur = px(arcPoint(r.curvature, s, p));
        cv::line(vis, prev, cur, col, 2);
        prev = cur;
      }
    }
    cv::circle(vis, px({0, 0}), 4, cv::Scalar(0, 255, 0), cv::FILLED);
    return vis;
  }

  const FloorObs & observation() const {return obs_;}
  const cv::Mat & forbidden() const {return forbid_;}
  const cv::Mat & cost() const {return cost_;}

private:
  // 1. 관측을 탐색 칸으로 줄이기: 칸 안에 하나라도 있으면 그 칸 (보인 곳은 절반 넘게)
  void shrinkObs(const Params & p)
  {
    const cv::Size gs = gridSize(p);
    auto shrink = [&](const cv::Mat & m, bool any) {
        if (m.empty()) {
          return cv::Mat(cv::Mat::zeros(gs, CV_8U));
        }
        cv::Mat f;
        m.convertTo(f, CV_32F, 1.0 / 255);
        cv::resize(f, f, gs, 0, 0, cv::INTER_AREA);
        return cv::Mat(any ? (f > 0.01) : (f > 0.5));
      };
    line_g_ = shrink(obs_.line, true);
    box_g_ = shrink(obs_.box, true);
    off_g_ = shrink(obs_.offroad, false);
    valid_g_ = shrink(obs_.valid, false);
  }

  // 2. 기억: 지금 보이는 곳의 기억은 버리고, 이번에 본 선 / 박스 칸을 넣는다
  void updateMemory(const Params & p)
  {
    if (!p.memory) {
      line_mem_.clear();
      box_mem_.clear();
      return;
    }
    const cv::Size gs = gridSize(p);
    auto keepUnseen = [&](std::vector<cv::Point2d> & pts) {
        std::vector<cv::Point2d> out;
        out.reserve(pts.size());
        for (const auto & q : pts) {
          const cv::Point c = toCell(q, p);
          const bool in = c.x >= 0 && c.y >= 0 && c.x < gs.width && c.y < gs.height;
          if (in && valid_g_.at<uint8_t>(c)) {
            continue;
          }
          if (q.x < -2 * p.half_width_mm || q.x > 2 * p.half_width_mm) {
            continue;
          }
          out.push_back(q);
        }
        pts = std::move(out);
      };
    keepUnseen(line_mem_);
    keepUnseen(box_mem_);
    for (int y = 0; y < gs.height; ++y) {
      for (int x = 0; x < gs.width; ++x) {
        if (line_g_.at<uint8_t>(y, x)) {
          line_mem_.push_back(toXY(x, y, p));
        }
        if (box_g_.at<uint8_t>(y, x)) {
          box_mem_.push_back(toXY(x, y, p));
        }
      }
    }
  }

  // 3. 비용 지도
  void buildCost(const Params & p)
  {
    const cv::Size gs = gridSize(p);
    cv::Mat lines = line_g_.clone(), boxes = box_g_.clone();
    known_ = valid_g_.clone();
    auto stamp = [&](const std::vector<cv::Point2d> & pts, cv::Mat & m) {
        for (const auto & q : pts) {
          const cv::Point c = toCell(q, p);
          if (c.x >= 0 && c.y >= 0 && c.x < gs.width && c.y < gs.height) {
            m.at<uint8_t>(c) = 255;
            known_.at<uint8_t>(c) = 255;
          }
        }
      };
    stamp(line_mem_, lines);
    stamp(box_mem_, boxes);

    cv::Mat dl, db, inv;
    cv::bitwise_not(lines | off_g_, inv);   // 주행 불가도 선과 같은 여유
    cv::distanceTransform(inv, dl, cv::DIST_L2, 3);
    cv::bitwise_not(boxes, inv);
    cv::distanceTransform(inv, db, cv::DIST_L2, 3);
    const double g = p.plan_cell_mm;
    const double line_lim = p.robot_half_width_mm + p.line_margin_mm;
    const double box_lim = p.robot_half_width_mm + p.box_margin_mm;

    cv::Mat prev_near = cv::Mat::zeros(gs, CV_8U);
    for (const auto & q : prev_path_) {
      const cv::Point c = toCell(q, p);
      cv::circle(prev_near, c, std::max(1, cvRound(p.prev_band_mm / g)), cv::Scalar(255), cv::FILLED);
    }
    const cv::Point start = toCell({0, 0}, p);
    const double esc = p.start_escape_mm / g;

    cost_.create(gs, CV_32F);
    soft_cost_.create(gs, CV_32F);
    forbid_ = cv::Mat::zeros(gs, CV_8U);
    for (int y = 0; y < gs.height; ++y) {
      // 안 보이는 띠: 최대 곡률 원호가 이 앞 거리에서 옆으로 가는 만큼만
      const double fy = p.far_mm - (y + 0.5) * rowMm(p);
      double blind_lim = -1;
      if (fy < p.bev_near_mm) {
        const double s = std::max(0.0, fy) / p.map_y_scale;   // 실제 [mm]
        const double k = p.max_curvature;
        blind_lim = p.blind_half_mm + (k * s < M_PI / 2 ? (1 - std::cos(k * s)) / k : 1 / k);
      }
      for (int x = 0; x < gs.width; ++x) {
        if (blind_lim >= 0 && std::abs((x + 0.5) * g - p.half_width_mm) > blind_lim) {
          cost_.at<float>(y, x) = -1.0f;
          soft_cost_.at<float>(y, x) = -1.0f;
          continue;
        }
        const double dline = dl.at<float>(y, x) * g;
        const bool recover = fy / p.map_y_scale < p.recover_mm;
        const double d = std::min(dline - line_lim, db.at<float>(y, x) * g - box_lim);
        const bool forbid = recover ?
          (dline < p.hard_line_mm || db.at<float>(y, x) * g < box_lim) : d < 0;
        float c;
        if (forbid) {
          forbid_.at<uint8_t>(y, x) = 255;
          const double ds = std::hypot(x - start.x, y - start.y);
          c = ds <= esc ? static_cast<float>(p.escape_cost) : -1.0f;   // -1 = 못 감
          float sc = static_cast<float>(p.soft_forbid_cost);
          if (lines.at<uint8_t>(y, x) || boxes.at<uint8_t>(y, x) || off_g_.at<uint8_t>(y, x)) {
            sc += static_cast<float>(p.soft_cross_cost);
          }
          soft_cost_.at<float>(y, x) = sc;
        } else {
          c = 1.0f;
          if (d < p.clear_cap_mm) {
            c += static_cast<float>(p.clear_k_mm / std::max(d, p.clear_min_mm));
          }
          if (dline < line_lim) {   // 복귀 구간에서 여유를 못 지키는 칸
            c += static_cast<float>(p.near_line_cost * (line_lim - dline) / (line_lim - p.hard_line_mm));
          }
          if (!known_.at<uint8_t>(y, x)) {
            c += static_cast<float>(p.unknown_cost);
          }
          if (prev_near.at<uint8_t>(y, x)) {
            c *= static_cast<float>(p.prev_discount);
          }
          soft_cost_.at<float>(y, x) = c;
        }
        cost_.at<float>(y, x) = c;
      }
    }
  }

  // 보이는 칸인데 옆 (8방향) 에 안 보이는 칸이 있음
  bool onVisibleEdge(int x, int y) const
  {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        const int nx = x + dx, ny = y + dy;
        if (nx >= 0 && ny >= 0 && nx < valid_g_.cols && ny < valid_g_.rows && !valid_g_.at<uint8_t>(ny, nx)) {
          return true;
        }
      }
    }
    return false;
  }

  // 4. Dijkstra + 목표 + 5. 다듬기
  void search(Result & r, const Params & p, Turn turn, bool soft)
  {
    const cv::Size gs = gridSize(p);
    const int W = gs.width, H = gs.height, N = W * H;
    const float INF = std::numeric_limits<float>::infinity();
    dist_.assign(N, INF);
    parent_.assign(N, -1);
    const cv::Point s = toCell({0, 0}, p);
    const int sx = std::clamp(s.x, 0, W - 1), sy = std::clamp(s.y, 0, H - 1);
    using Item = std::pair<float, int>;
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> pq;
    const int start = sy * W + sx;
    dist_[start] = 0;
    pq.push({0.0f, start});
    static const int DX[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    static const int DY[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    const float g = static_cast<float>(p.plan_cell_mm);
    const float * cost = (soft ? soft_cost_ : cost_).ptr<float>();
    while (!pq.empty()) {
      const auto [d, i] = pq.top();
      pq.pop();
      if (d > dist_[i]) {
        continue;
      }
      const int x = i % W, y = i / W;
      const float ci = std::max(cost[i], 0.0f);
      for (int k = 0; k < 8; ++k) {
        const int nx = x + DX[k], ny = y + DY[k];
        if (nx < 0 || ny < 0 || nx >= W || ny >= H) {
          continue;
        }
        const int j = ny * W + nx;
        if (cost[j] < 0) {
          continue;
        }
        const float len = k < 4 ? g : g * 1.41421356f;
        const float nd = d + 0.5f * (ci + cost[j]) * len;
        if (nd < dist_[j]) {
          dist_[j] = nd;
          parent_[j] = i;
          pq.push({nd, j});
        }
      }
    }

    // 목표 고르기
    const int band = std::max(1, static_cast<int>(std::lround(p.goal_band_mm / p.plan_cell_mm)));
    const int band_rows = std::max(1, static_cast<int>(std::lround(p.goal_band_mm / rowMm(p))));
    const int side_max_row = static_cast<int>((p.far_mm - p.side_goal_min_y_mm) / rowMm(p));
    double best = INF;
    int best_i = -1, best_kind = 0;
    for (int y = 0; y < H; ++y) {
      for (int x = 0; x < W; ++x) {
        const int i = y * W + x;
        if (!std::isfinite(dist_[i]) || !valid_g_.at<uint8_t>(y, x) ||
          (!soft && forbid_.at<uint8_t>(y, x)))
        {
          continue;
        }
        int kind = -1;
        if (y < band_rows) {
          kind = 0;
        } else if (y <= side_max_row && x < band) {
          kind = 1;
        } else if (y <= side_max_row && x >= W - band) {
          kind = 2;
        } else if (y <= side_max_row && onVisibleEdge(x, y)) {
          // 보이는 사다리꼴의 비스듬한 옆변: 커브에서 차선이 여기로 빠져나감 (위 / 좌우 끝에 안 닿음)
          kind = x < W / 2 ? 1 : 2;
        }
        if (kind < 0) {
          continue;
        }
        const cv::Point2d q = toXY(x, y, p);
        double score = dist_[i];
        if (have_prev_goal_) {
          score += p.continuity_weight * cv::norm(q - prev_goal_);
        }
        if (turn == Turn::NONE) {
          if (kind != 0) {
            score += p.side_penalty_mm;
          }
        } else {
          const double dir = turn == Turn::LEFT ? -1.0 : 1.0;   // 가고 싶은 쪽 x 부호
          score -= p.turn_weight_mm * dir * q.x / p.half_width_mm;
          if ((kind == 1 && dir < 0) || (kind == 2 && dir > 0)) {
            score -= p.turn_side_bonus_mm;
          } else if (kind != 0) {
            score += p.turn_wrong_side_mm;
          }
        }
        if (score < best) {
          best = score;
          best_i = i;
          best_kind = kind;
        }
      }
    }
    if (best_i < 0) {
      r.reason = "목표 없음 (갈 수 있는 끝 칸이 없음)";
      return;
    }
    r.path_ok = true;
    r.goal_kind = best_kind;
    r.goal = toXY(best_i % W, best_i / W, p);
    prev_goal_ = r.goal;
    have_prev_goal_ = true;
    for (int i = best_i; i >= 0; i = parent_[i]) {
      r.raw_path.push_back(toXY(i % W, i / W, p));
    }
    std::reverse(r.raw_path.begin(), r.raw_path.end());
    r.raw_path.front() = {0, 0};
    r.path = smooth(r.raw_path, p);
    prev_path_ = r.path;
  }

  static std::vector<cv::Point2d> resample(const std::vector<cv::Point2d> & in, double step)
  {
    std::vector<cv::Point2d> out{in.front()};
    double carry = 0;
    for (size_t i = 1; i < in.size(); ++i) {
      const cv::Point2d a = in[i - 1], b = in[i];
      const double len = cv::norm(b - a);
      double t = step - carry;
      while (t <= len) {
        out.push_back(a + (b - a) * (t / len));
        t += step;
      }
      carry = len - (t - step);
    }
    if (cv::norm(out.back() - in.back()) > step * 0.3) {
      out.push_back(in.back());
    }
    return out;
  }

  static std::vector<cv::Point2d> smooth(const std::vector<cv::Point2d> & raw, const Params & p)
  {
    if (raw.size() < 2) {
      return raw;
    }
    std::vector<cv::Point2d> pts = resample(raw, p.sample_mm);
    const int n = static_cast<int>(pts.size()), h = p.smooth_half;
    for (int it = 0; it < p.smooth_iters; ++it) {
      std::vector<cv::Point2d> out = pts;
      for (int i = 1; i < n - 1; ++i) {   // 처음 (바퀴 축) / 끝 (목표) 은 그대로
        cv::Point2d sum(0, 0);
        int cnt = 0;
        for (int j = std::max(0, i - h); j <= std::min(n - 1, i + h); ++j) {
          sum += pts[j];
          ++cnt;
        }
        out[i] = sum / cnt;
      }
      pts = std::move(out);
    }
    return pts;
  }

  // 6. 원호 후보로 경로 따라가기
  void follow(Result & r, const Params & p)
  {
    arcs_.clear();
    if (!r.path_ok) {
      r.reason = r.reason.empty() ? "경로 없음" : r.reason;
      return;
    }
    // 경로를 실제 축척으로
    std::vector<cv::Point2d> ref;
    ref.reserve(r.path.size());
    for (const auto & q : r.path) {
      ref.emplace_back(q.x, q.y / p.map_y_scale);
    }
    auto refDist = [&](const cv::Point2d & q) {
        double best = std::numeric_limits<double>::infinity();
        for (size_t i = 1; i < ref.size(); ++i) {
          const cv::Point2d a = ref[i - 1], ab = ref[i] - a;
          const double l2 = ab.dot(ab);
          const double t = l2 > 0 ? std::clamp((q - a).dot(ab) / l2, 0.0, 1.0) : 0.0;
          best = std::min(best, cv::norm(q - (a + ab * t)));
        }
        return best;
      };
    const cv::Size gs = gridSize(p);
    const cv::Point start = toCell({0, 0}, p);
    const double esc = p.start_escape_mm / p.plan_cell_mm;
    const int n = std::max(1, p.arc_count);
    double best = std::numeric_limits<double>::infinity();
    double best_k = 0;
    for (int a = 0; a < n; ++a) {
      const double k = n == 1 ? 0.0 : p.max_curvature * (2.0 * a / (n - 1) - 1.0);
      std::vector<cv::Point2d> pts;
      bool blocked = false;
      double err = 0;
      int cnt = 0, inside = 0;
      for (double s = 0; s <= p.follow_mm + 1e-6; s += p.arc_step_mm) {
        const cv::Point2d q = arcPoint(k, s, p);
        pts.push_back(q);
        const cv::Point c = toCell(q, p);
        if (s <= p.check_mm && c.x >= 0 && c.y >= 0 && c.x < gs.width && c.y < gs.height &&
          forbid_.at<uint8_t>(c) && std::hypot(c.x - start.x, c.y - start.y) > esc)
        {
          if (!r.soft) {
            blocked = true;
            break;
          }
          ++inside;
        }
        err += refDist({q.x, q.y / p.map_y_scale});
        ++cnt;
      }
      arcs_.push_back(pts);
      if (blocked || cnt == 0) {
        continue;
      }
      const double score = err / cnt + p.soft_arc_weight * inside / cnt +
        p.curvature_weight * std::abs(k) / p.max_curvature +
        p.change_weight * std::abs(k - prev_k_) / p.max_curvature;
      if (score < best) {
        best = score;
        best_k = k;
      }
    }
    if (std::isfinite(best)) {
      r.follow_ok = true;
      r.curvature = best_k;
      prev_k_ = best_k;
      const double kr = std::abs(best_k) / p.max_curvature;
      double v = std::max(p.min_linear, p.linear * (1.0 - p.slow_curvature * kr));
      if (r.soft) {
        v *= p.soft_speed_ratio;
      }
      double w = -v * best_k * 1000.0;   // [1/mm] -> [1/m], 오른쪽으로 휨 = 시계 방향 = w 음수
      if (std::abs(w) > p.max_angular) {
        v *= p.max_angular / std::abs(w);
        w = std::copysign(p.max_angular, w);
      }
      r.v = v;
      r.w = w;
      return;
    }
    // 원호가 전부 막힘: 경로 위 pivot_lookahead 지점 방향으로 제자리 회전
    double acc = 0;
    cv::Point2d target = ref.back();
    for (size_t i = 1; i < ref.size(); ++i) {
      acc += cv::norm(ref[i] - ref[i - 1]);
      if (acc >= p.pivot_lookahead_mm) {
        target = ref[i];
        break;
      }
    }
    const double ang = std::atan2(target.x, target.y);   // + 오른쪽
    r.pivot = true;
    r.curvature = 0;
    r.v = 0;
    r.w = ang > 0 ? -p.pivot_angular : p.pivot_angular;
    r.reason = "원호가 전부 막힘 -> 경로 방향으로 제자리 회전";
    prev_k_ = 0;
  }

  FloorObs obs_;                           // 이번 관측 (cell_mm)
  cv::Mat line_g_, box_g_, off_g_, valid_g_;   // 탐색 칸 (plan_cell_mm)
  cv::Mat known_, cost_, soft_cost_, forbid_;
  std::vector<float> dist_;
  std::vector<int> parent_;
  std::vector<cv::Point2d> line_mem_, box_mem_, prev_path_;
  std::vector<std::vector<cv::Point2d>> arcs_;
  cv::Point2d prev_goal_;
  bool have_prev_goal_ = false;
  double prev_k_ = 0;
};

}  // namespace grid_drive
