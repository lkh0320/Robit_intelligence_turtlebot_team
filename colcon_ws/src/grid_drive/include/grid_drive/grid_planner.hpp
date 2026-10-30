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
//   지도 앞뒤는 실제의 map_y_scale 배 (예전 box_planner odom_scale, 2026-10-14 녹화 분석) -> 원호 / 조향은 실제 축척으로 계산
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
  double map_y_scale = 1.0;            // 지도 앞뒤 [mm] / 실제 [mm] (예전 box_planner odom_scale)
  // 2. 기억
  bool memory = false;                 // 2026-10-28: 벨트 미끄럼 / 직진 중 틀어짐을 odom 이 못 봐 기억이 어긋남 (replay soft 31 -> 102)
  double memory_back_mm = 250.0;       // 바퀴 축 뒤로 이만큼 지나면 잊음 (지도 단위)
  // 기억은 이 프레임 수 (30fps 에서 15 = 0.5초) 넘게 화면 밖에 있으면 잊음. odom 이 영상과 어긋나는 만큼 (지연, 벨트
  //   미끄럼) 오래된 기억일수록 더 틀어져 선이 두 겹으로 남고 경로가 좌우로 뒤집혔다 (2026-10-28 drive16). 0 이하 = 제한 없음
  int memory_max_frames = 15;
  bool memory_lines = true;            // false = 박스만 기억 (공사 구간)
  double box_memory_back_mm = 250.0;   // 박스 기억만 따로: 바퀴 축 뒤로 이만큼 지나면 잊음 (음수 = 축 앞에서 잊음)
  // 박스 기억만 따로 프레임 수 제한 (음수 = memory_max_frames 와 같게). 박스는 화면 아래로 빠진 뒤에도 바퀴 옆에
  //   남아 있어서 0.5초 만에 잊으면 지나간 박스 쪽으로 다시 꺾어 바퀴가 걸렸다 (2026-10-28)
  int box_memory_max_frames = -1;
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
  // 차선 폭 지키기: 보이는 선에서 lane_half_mm 보다 멀어지면 칸마다 비용 (멀수록 더, 100mm 에서 최대).
  // 코너에서 안쪽 선이 화면 밖이면 "안쪽이 비었다" 고 보고 안쪽으로 붙는 것을 막음 (바깥 선에서 차선 반폭 근처 유지)
  double lane_half_mm = 150.0;
  double far_line_cost = 0.0;
  // 가운데 따라가기: 선 / 주행 불가까지 거리가 주변 (center_radius_mm 안) 최댓값 (= 두 선 사이 가운데) 보다
  //   작을수록 비용. 최단 경로가 코너 안쪽을 파고드는 것 막음. 0 = 끔 (교차로 U턴용, 2026-10-28)
  double center_cost = 0.0;
  double center_radius_mm = 250.0;
  // 주행 가능 구역 (도로 모델) 이 있으면 그 안에서만 계획: 주행 불가 칸과 화면 밖 칸은 soft 에서도 못 감
  // (보이는 칸이 없는 줄 = 로봇 바로 앞 안 보이는 띠, 와 출발 탈출 반경은 예외)
  bool road_only = false;
  double box_margin_mm = 30.0;         // 박스와 로봇 옆면 사이 최소 여유
  double clear_k_mm = 120.0;            // 비용 = 1 + clear_k / d (d = 진입 금지 경계까지 [mm])
  double clear_min_mm = 6.0;           // d 가 이보다 작으면 이 값으로 (비용 상한)
  double clear_cap_mm = 200.0;         // d 가 이보다 크면 비용 1
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
  // soft 에서도 로봇 앞 soft_line_block_mm [실제 mm] 보다 먼 곳은 선 중심에서 soft_line_keep_mm 안을 못 지나감 (선을 건너는 경로 금지).
  //   공사 표지판 앞 코너에서 앞이 박스 / 띠로 막히자 soft 경로가 왼쪽 노란선을 건너 옆 끝 목표로 가 선을 넘었다 (실격,
  //   2026-10-30 drive41 283.8s / drive42 39.6s, 76.2s). 0 = 예전 (벌점만)
  double soft_line_block_mm = 0.0;
  double soft_line_keep_mm = 60.0;
  double soft_arc_weight = 300.0;      // soft: 원호가 진입 금지 칸을 지나는 비율 x 이 값 [mm]
  // soft: 띠 안으로 들어간 깊이 (0 = 띠 가장자리 ~ 1 = 선 위) x 이 값을 더함. 0 이면 띠 안은 다 같은 값이라
  //   최단 경로 = 안쪽 선에 붙음 (2026-10-29 left1: 교차로 진입 / U턴 내내 soft 로 안쪽 선 밟음)
  double soft_depth_cost = 0.0;
  double soft_speed_ratio = 0.5;
  // 4. 목표
  double goal_band_mm = 36.0;          // 위쪽 / 옆쪽 끝에서 이만큼 안 칸이 목표 후보
  double side_goal_min_y_mm = 300.0;   // 옆쪽 끝 목표는 이보다 앞 (지도 단위)
  double continuity_weight = 0.5;      // |목표 - 직전 목표| [mm] x 이 값
  // 막다른 길 (차단바 / 박스가 앞을 다 막아 위 / 옆 끝 목표가 없음): 갈 수 있는 가장 앞 칸까지 가서 멈춤.
  // 차단바 / 박스가 막음: 그 갈 수 있는 끝 칸이 바퀴 축에서 dead_end_stop_mm [실제] 보다 가까우면 정지 (예전 방식.
  //   차단바는 공중에 뜬 막대라 바닥 지도에서 거리가 실제보다 멀게 재져서 막는 것 기준은 안 맞음)
  // 코스 끝 (선 / 주행 불가가 막음): 그 막는 것이 바퀴 축에서 dead_end_line_mm 이하면 정지
  // 둘 다 정지 거리보다 dead_end_slow_mm 안쪽으로 오면 감속
  bool dead_end_goal = true;
  bool dead_end_force_box = false;     // 비전이 닫힌 차단바를 봄: 막는 것을 색과 상관없이 차단바로 (흰 줄무늬가 선으로 잡힘)
  double dead_end_stop_mm = 220.0;
  double dead_end_line_mm = 350.0;
  double dead_end_slow_mm = 250.0;
  double side_penalty_mm = 600.0;      // 평소 옆쪽 끝 목표 벌점 (위쪽이 막혔을 때만 옆으로)
  double turn_weight_mm = 900.0;       // turn 이면 (가로 위치 / half_width) x 이 값 만큼 그쪽 가점
  double turn_side_bonus_mm = 400.0;   // turn 쪽 옆쪽 끝 목표 가점
  double turn_wrong_side_mm = 3000.0;  // turn 반대쪽 옆쪽 끝 목표 벌점
  bool turn_block_wrong = false;       // true: turn 반대쪽 목표는 아예 안 고름 (옆 끝, 또는 가운데에서 turn_block_x_mm 넘게 반대쪽)
  double turn_block_x_mm = 100.0;
  // 5. 다듬기
  double sample_mm = 20.0;
  int smooth_iters = 4;
  int smooth_half = 3;                 // 이동 평균 반창 [점]
  // 6. 따라가기 (실제 축척)
  int arc_count = 41;
  double max_curvature = 1.0 / 150.0;  // [1/mm 실제]
  double follow_mm = 250.0;            // 원호 길이 [실제 mm]
  double check_mm = 200.0;             // 이 안에서 진입 금지에 닿으면 탈락 [실제 mm]
  double arc_step_mm = 10.0;
  double curvature_weight = 10.0;      // |k| / max_curvature x 이 값 [mm]
  double change_weight = 15.0;         // |k - 직전 k| / max_curvature x 이 값 [mm]
  // 지연 보정: 새 명령이 바퀴에 먹기 전 lead_mm 만큼 지금 곡률 lead_k [1/mm, 오른쪽 +] 로 더 간다고 보고
  //   그 자리에서 원호를 고름 (0 = 끔). 속도를 올리자 지연 (~0.25s) 때문에 직선에서 출렁이고 코너 안쪽을 밟았다
  double lead_mm = 0.0;
  double lead_k = 0.0;
  // 경로 추종 (track_len_mm > 0 이면 원호 맞춤 대신): 로봇 (lead 자리) 에서 가장 가까운 경로 점의 옆 거리 e,
  //   경로 방향 차 th, 경로 곡률 k_path 로 k = k_path + 2 th / L + e / L^2 (L = track_len_mm, 임계 감쇠).
  //   원호 하나를 앞 follow_mm 에 맞추면 코너 전부터 꺾어 안쪽을 밟았다 (2026-10-29 drive32/33 안쪽 40~90mm).
  //   막히면 (진입 금지) 원호 맞춤으로 돌아감
  double track_len_mm = 0.0;
  double track_preview_mm = 80.0;      // 경로 곡률을 재는 구간 [실제 mm] (가장 가까운 점부터 앞으로)
  double track_outer_mm = 0.0;         // 굽은 길에서 바깥으로 이만큼 [실제 mm] 비켜 따라감 (반지름 300mm 이하 곡선에서 다)
  double track_bend_mm = 300.0;        // 길이 어느 쪽으로 휘는지 재는 앞 구간 [실제 mm]. 계획 경로는 처음에 안쪽으로 먼저
                                       //   붙어서 가까운 곡률은 부호가 뒤집힐 수 있음
  // 바깥 비킴 맞추기 (1 = 켬): 비킨 원호가 선 띠 (빨간 칸) 에 들어가면 비킴을 반 / 0 으로 줄여 덜 들어가는 것을 고름.
  //   SOFT (거의 항상) 에서는 띠에 들어가도 안 막혀서 비킴 90mm 가 그대로 바깥 선을 넘었다 (2026-10-30 drive39:
  //   첫 왼쪽 코너 / 교차로 U턴 / 지그재그 끝). 0 = 예전
  double track_outer_fit = 0.0;
  // 코너 앞 감속 (0 = 끔, 1 = 앞 곡률을 지금 곡률처럼): track_bend_mm 앞까지 경로가 꺾이는 정도로 미리 감속.
  //   지금 원호 곡률로만 감속하면 0.5 m/s 로 코너에 들어가 늦게 꺾고 바깥 선에 닿아 제자리 회전 (drive39 39.2s)
  double track_slow_ahead = 0.0;
  // 따라가는 원호가 이 거리 [실제 mm] 보다 멀리서 막히면 막힌 것으로 안 봄 (경로는 막힌 곳을 피해 있고 다음 프레임에 다시 고름).
  //   좁은 박스 사이 길에서 원호 하나가 길 안에 다 안 들어가 제자리 회전 / 좌우 끝 원호로 튀어 움찔댔다 (2026-10-30 drive41
  //   195~202s, 경로는 깨끗했음). 0 = 예전 (check_mm 안 어디서 막혀도 탈락)
  double track_block_ok_mm = 0.0;
  // 출력
  double linear = 0.10;                // [m/s]
  double min_linear = 0.04;
  double slow_curvature = 0.0;         // v = linear x (1 - 이 값 x |k| / max_curvature)
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
  bool dead_end = false;               // 앞이 막혀 갈 수 있는 가장 앞 칸을 목표로 함
  double dead_end_mm = 0;              // 막는 것까지 앞 거리 [실제 mm, 바퀴 축 기준]
  double dead_end_target_mm = 0;       // 그 막는 것 앞에서 멈출 거리 (dead_end_box_mm / dead_end_line_mm)
  bool dead_end_box = false;           // 막는 것이 박스 (차단바)
  double v = 0, w = 0;                 // cmd_vel [m/s], [rad/s] (w + = 왼쪽, ROS)
  double curvature = 0;                // 고른 원호 [1/mm 실제], + 오른쪽
  double track_outer_used = 0;         // 이번에 쓴 바깥 비킴 [mm] (track_outer_fit 로 줄었는지 디버그)
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
    auto moveAll = [&](std::vector<cv::Point2d> & pts, std::vector<int> * age, double back) {
        std::vector<cv::Point2d> out;
        std::vector<int> out_age;
        out.reserve(pts.size());
        for (size_t i = 0; i < pts.size(); ++i) {
          const cv::Point2d n = move(pts[i]);
          if (n.y > -back) {
            out.push_back(n);
            if (age) {
              out_age.push_back((*age)[i]);
            }
          }
        }
        pts = std::move(out);
        if (age) {
          *age = std::move(out_age);
        }
      };
    moveAll(line_mem_, &line_age_, p.memory_back_mm);
    moveAll(box_mem_, &box_age_, p.box_memory_back_mm);
    moveAll(prev_path_, nullptr, p.memory_back_mm);
    if (have_prev_goal_) {
      prev_goal_ = move(prev_goal_);
    }
  }

  void reset()
  {
    line_mem_.clear();
    box_mem_.clear();
    line_age_.clear();
    box_age_.clear();
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
  Result plan(const cv::Mat & raw, const Params & p, const grid_drive::ColorParams & colors,
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
    search(r, p, turn, false, false);
    if (!r.path_ok) {
      // soft: 선을 최소로 밟고 빠져나가는 경로. 단 박스 / 차단바 여유 안을 지나면 쓰지 않고
      // 막다른 길 (갈 수 있는 가장 앞 칸까지 가서 멈춤) 로
      const cv::Point2d keep_goal = prev_goal_;
      const bool keep_have = have_prev_goal_;
      Result rs = r;
      search(rs, p, turn, true, false);
      if (rs.path_ok && !crossesBox(rs.raw_path, p)) {
        r = rs;
        r.soft = true;
      } else {
        prev_goal_ = keep_goal;
        have_prev_goal_ = keep_have;
        if (p.dead_end_goal) {
          search(r, p, turn, false, true);
        }
      }
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
      line_age_.clear();
      box_age_.clear();
      return;
    }
    if (!p.memory_lines) {
      line_mem_.clear();
      line_age_.clear();
    }
    const cv::Size gs = gridSize(p);
    auto keepUnseen = [&](std::vector<cv::Point2d> & pts, std::vector<int> & age, int max_frames) {
        std::vector<cv::Point2d> out;
        std::vector<int> out_age;
        out.reserve(pts.size());
        for (size_t i = 0; i < pts.size(); ++i) {
          const cv::Point2d q = pts[i];
          if (max_frames > 0 && age[i] + 1 > max_frames) {
            continue;
          }
          const cv::Point c = toCell(q, p);
          const bool in = c.x >= 0 && c.y >= 0 && c.x < gs.width && c.y < gs.height;
          if (in && valid_g_.at<uint8_t>(c)) {
            continue;
          }
          if (q.x < -2 * p.half_width_mm || q.x > 2 * p.half_width_mm) {
            continue;
          }
          out.push_back(q);
          out_age.push_back(age[i] + 1);
        }
        pts = std::move(out);
        age = std::move(out_age);
      };
    keepUnseen(line_mem_, line_age_, p.memory_max_frames);
    keepUnseen(box_mem_, box_age_, p.box_memory_max_frames < 0 ? p.memory_max_frames : p.box_memory_max_frames);
    for (int y = 0; y < gs.height; ++y) {
      for (int x = 0; x < gs.width; ++x) {
        if (p.memory_lines && line_g_.at<uint8_t>(y, x)) {
          line_mem_.push_back(toXY(x, y, p));
          line_age_.push_back(0);
        }
        if (box_g_.at<uint8_t>(y, x)) {
          box_mem_.push_back(toXY(x, y, p));
          box_age_.push_back(0);
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
    cv::Mat dmax;   // 주변 최대 선 거리 (가운데 능선 높이)
    if (p.center_cost > 0) {
      const int r = std::max(1, static_cast<int>(std::lround(p.center_radius_mm / g)));
      cv::dilate(dl, dmax, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * r + 1, 2 * r + 1)));
    }
    const double box_lim = p.robot_half_width_mm + p.box_margin_mm;
    box_near_ = db * g < box_lim;

    cv::Mat prev_near = cv::Mat::zeros(gs, CV_8U);
    for (const auto & q : prev_path_) {
      const cv::Point c = toCell(q, p);
      cv::circle(prev_near, c, std::max(1, cvRound(p.prev_band_mm / g)), cv::Scalar(255), cv::FILLED);
    }
    const cv::Point start = toCell({0, 0}, p);
    const double esc = p.start_escape_mm / g;
    const bool road_only = p.road_only && !obs_.offroad.empty() && !off_g_.empty();
    // 화면 밖 칸은 보이는 칸이 있는 줄에서만 막음 (사다리꼴 옆). 사다리꼴 아래 ~ 로봇 사이 줄은 막지 않음
    std::vector<uint8_t> row_seen(gs.height, 0);
    for (int y = 0; y < gs.height; ++y) {
      row_seen[y] = cv::countNonZero(valid_g_.row(y)) > 0;
    }

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
        if (road_only && row_seen[y] &&
          (off_g_.at<uint8_t>(y, x) || (!valid_g_.at<uint8_t>(y, x) && !known_.at<uint8_t>(y, x))) &&
          std::hypot(x - start.x, y - start.y) > esc)
        {
          forbid_.at<uint8_t>(y, x) = 255;
          cost_.at<float>(y, x) = -1.0f;
          soft_cost_.at<float>(y, x) = -1.0f;
          continue;
        }
        if (forbid) {
          forbid_.at<uint8_t>(y, x) = 255;
          const double ds = std::hypot(x - start.x, y - start.y);
          c = ds <= esc ? static_cast<float>(p.escape_cost) : -1.0f;   // -1 = 못 감
          float sc = static_cast<float>(p.soft_forbid_cost);
          if (p.soft_depth_cost > 0) {
            sc += static_cast<float>(p.soft_depth_cost * std::clamp(-d / line_lim, 0.0, 1.0));
          }
          if (lines.at<uint8_t>(y, x) || boxes.at<uint8_t>(y, x) || off_g_.at<uint8_t>(y, x)) {
            sc += static_cast<float>(p.soft_cross_cost);
          }
          if (p.soft_line_block_mm > 0 && fy / p.map_y_scale > p.soft_line_block_mm && dline < p.soft_line_keep_mm && ds > esc) {
            sc = -1.0f;
          }
          soft_cost_.at<float>(y, x) = sc;
        } else {
          c = 1.0f;
          if (d < p.clear_cap_mm) {
            c += static_cast<float>(p.clear_k_mm / std::max(d, p.clear_min_mm));
          }
          if (p.far_line_cost > 0 && dline > p.lane_half_mm) {   // 보이는 선에서 차선 반폭보다 먼 칸
            c += static_cast<float>(p.far_line_cost * std::min(1.0, (dline - p.lane_half_mm) / 100.0));
          }
          if (p.center_cost > 0) {   // 가운데 능선에서 벗어난 만큼 (0 ~ 1)
            const float m = dmax.at<float>(y, x);
            if (m > 0) {
              c += static_cast<float>(p.center_cost * (m - dl.at<float>(y, x)) / m);
            }
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
  // 경로가 박스 여유 (로봇 반폭 + box_margin) 안 칸을 지나는지 (출발 탈출 반경 밖)
  bool crossesBox(const std::vector<cv::Point2d> & path, const Params & p) const
  {
    const cv::Point start = toCell({0, 0}, p);
    const double esc = p.start_escape_mm / p.plan_cell_mm;
    for (const auto & q : path) {
      const cv::Point c = toCell(q, p);
      if (c.x >= 0 && c.y >= 0 && c.x < box_near_.cols && c.y < box_near_.rows &&
        box_near_.at<uint8_t>(c) && std::hypot(c.x - start.x, c.y - start.y) > esc)
      {
        return true;
      }
    }
    return false;
  }

  void search(Result & r, const Params & p, Turn turn, bool soft, bool dead_end)
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
          if (p.turn_block_wrong &&
            ((kind == 1 && dir > 0) || (kind == 2 && dir < 0) || dir * q.x < -p.turn_block_x_mm))
          {
            continue;
          }
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
    if (best_i < 0 && !soft && dead_end) {
      // 막다른 길: 갈 수 있는 (진입 금지 아닌, 보이는) 칸 중 가장 앞, 같은 줄이면 가까운 경로
      for (int y = 0; y < H && best_i < 0; ++y) {
        float bd = INF;
        for (int x = 0; x < W; ++x) {
          const int i = y * W + x;
          if (std::isfinite(dist_[i]) && valid_g_.at<uint8_t>(y, x) && !forbid_.at<uint8_t>(y, x) &&
            dist_[i] < bd)
          {
            bd = dist_[i];
            best_i = i;
          }
        }
      }
      if (best_i >= 0) {
        // 목표 칸에서 앞으로 (같은 열) 처음 만나는 박스 / 선 / 주행 불가 = 막는 것
        const int gx = best_i % W, gy = best_i / W;
        double obst = -1;
        bool obst_box = false;
        for (int y = gy - 1; y >= 0 && obst < 0; --y) {
          const bool b = box_g_.at<uint8_t>(y, gx) != 0;
          if (b || line_g_.at<uint8_t>(y, gx) || off_g_.at<uint8_t>(y, gx)) {
            obst = toXY(gx, y, p).y / p.map_y_scale;
            obst_box = b;
          }
        }
        // 도로 모델이 차단바 바로 앞에서 끊겨 주행 불가를 먼저 만나도, 그 뒤 200mm 안에 박스가 있으면 차단바로 봄
        if (obst >= 0 && !obst_box) {
          for (int y = gy - 1; y >= 0; --y) {
            const double yy = toXY(gx, y, p).y / p.map_y_scale;
            if (yy > obst + 200.0) {
              break;
            }
            if (yy >= obst && box_g_.at<uint8_t>(y, gx)) {
              obst_box = true;
              break;
            }
          }
        }
        obst_box = obst_box || p.dead_end_force_box;
        if (obst < 0) {   // 열에서 못 찾음 (옆에서 막힘 등): 진입 금지 띠만큼 앞으로 봄
          obst = toXY(gx, gy, p).y / p.map_y_scale + p.robot_half_width_mm + p.line_margin_mm;
        }
        r.dead_end = true;   // 정지일 때도 soft 로 막힌 곳을 뚫고 가지 않게
        r.dead_end_box = obst_box;
        // 차단바 / 박스: 갈 수 있는 끝 칸까지 거리 기준 (예전 방식), 코스 끝: 막는 것까지 거리 기준
        r.dead_end_mm = obst_box ? toXY(gx, gy, p).y / p.map_y_scale : obst;
        r.dead_end_target_mm = obst_box ? p.dead_end_stop_mm : p.dead_end_line_mm;
        if (obst <= r.dead_end_target_mm) {
          r.reason = obst_box ? "차단바 / 박스 앞 -> 정지" : "코스 끝 -> 정지";
          return;
        }
        best_kind = 0;
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
    // 지연 보정: 원호 시작 자리 / 방향 (실제 축척, 오른쪽 + 방향각)
    const double lead_k = std::clamp(p.lead_k, -p.max_curvature, p.max_curvature);
    const cv::Point2d lead0 = p.lead_mm > 0 ? arcPoint(lead_k, p.lead_mm, p) : cv::Point2d(0, 0);
    const cv::Point2d lead(lead0.x, lead0.y / p.map_y_scale);
    const double lead_th = p.lead_mm > 0 ? lead_k * p.lead_mm : 0.0;
    const double lc = std::cos(lead_th), ls = std::sin(lead_th);
    double block_s = 0;   // arcOf 가 막힌 곳 (원호 위 거리)
    auto arcOf = [&](double k, bool & blocked, int & inside, double & err, int & cnt) {
        std::vector<cv::Point2d> pts;
        blocked = false;
        block_s = 0;
        inside = cnt = 0;
        err = 0;
        for (double s = 0; s <= p.follow_mm + 1e-6; s += p.arc_step_mm) {
          const cv::Point2d q0 = arcPoint(k, s, p);
          const double lx = q0.x, ly = q0.y / p.map_y_scale;
          const cv::Point2d q(lead.x + lx * lc + ly * ls, (lead.y - lx * ls + ly * lc) * p.map_y_scale);
          pts.push_back(q);
          const cv::Point c = toCell(q, p);
          if (s <= p.check_mm && c.x >= 0 && c.y >= 0 && c.x < gs.width && c.y < gs.height &&
            forbid_.at<uint8_t>(c) && std::hypot(c.x - start.x, c.y - start.y) > esc)
          {
            if (!r.soft) {
              blocked = true;
              block_s = s;
              break;
            }
            ++inside;
          }
          err += refDist({q.x, q.y / p.map_y_scale});
          ++cnt;
        }
        return pts;
      };
    bool tracked = false;
    double k_ahead = 0;   // 앞 꺾임 [1/mm] (track 방식에서만)
    if (p.track_len_mm > 0 && ref.size() >= 2) {
      // 경로를 lead 자리 / 방향 기준으로 (x 오른쪽, y 앞)
      std::vector<cv::Point2d> loc;
      loc.reserve(ref.size());
      for (const auto & q : ref) {
        const double dx = q.x - lead.x, dy = q.y - lead.y;
        loc.emplace_back(dx * lc - dy * ls, dx * ls + dy * lc);
      }
      // 원점에서 가장 가까운 선분 위 점
      double bd = std::numeric_limits<double>::infinity();
      size_t bi = 0;
      double bt = 0;
      for (size_t i = 1; i < loc.size(); ++i) {
        const cv::Point2d a = loc[i - 1], ab = loc[i] - a;
        const double l2 = ab.dot(ab);
        const double t = l2 > 0 ? std::clamp(-a.dot(ab) / l2, 0.0, 1.0) : 0.0;
        const double d = cv::norm(a + ab * t);
        if (d < bd) {
          bd = d;
          bi = i;
          bt = t;
        }
      }
      const cv::Point2d a = loc[bi - 1], ab = loc[bi] - a, near = a + ab * bt;
      const double th = std::atan2(ab.x, ab.y);   // 경로 방향 (+ 오른쪽)
      // 경로 곡률: 가장 가까운 점부터 track_preview_mm 앞까지 방향 변화 / 길이
      double acc = cv::norm(loc[bi] - near), th2 = th, len = acc;
      for (size_t i = bi + 1; i < loc.size() && acc < p.track_preview_mm; ++i) {
        const cv::Point2d d = loc[i] - loc[i - 1];
        const double l = cv::norm(d);
        if (l < 1e-6) {
          continue;
        }
        th2 = std::atan2(d.x, d.y);
        acc += l;
        len = acc;
      }
      const double k_path = len > 1.0 ? std::remainder(th2 - th, 2 * M_PI) / len : 0.0;
      const double L = p.track_len_mm;
      // 바깥으로 비킴: track_bend_mm 앞까지 오른쪽으로 휘면 경로보다 왼쪽을 목표로
      double outer = 0;
      if (p.track_outer_mm > 0) {
        double acc2 = cv::norm(loc[bi] - near), th3 = th, len2 = acc2;
        for (size_t i = bi + 1; i < loc.size() && acc2 < p.track_bend_mm; ++i) {
          const cv::Point2d d = loc[i] - loc[i - 1];
          const double l = cv::norm(d);
          if (l < 1e-6) {
            continue;
          }
          th3 = std::atan2(d.x, d.y);
          acc2 += l;
          len2 = acc2;
        }
        const double k_bend = len2 > 1.0 ? std::remainder(th3 - th, 2 * M_PI) / len2 : 0.0;
        outer = p.track_outer_mm * std::clamp(k_bend * 300.0, -1.0, 1.0);
      }
      // 앞 꺾임 (코너 앞 감속): track_bend_mm 과 그 반 구간 중 큰 방향 변화 / 길이
      {
        double acc3 = cv::norm(loc[bi] - near), th4 = th;
        for (size_t i = bi + 1; i < loc.size() && acc3 < p.track_bend_mm; ++i) {
          const cv::Point2d d = loc[i] - loc[i - 1];
          const double l = cv::norm(d);
          if (l < 1e-6) {
            continue;
          }
          th4 = std::atan2(d.x, d.y);
          acc3 += l;
          if (acc3 > 30.0) {
            k_ahead = std::max(k_ahead, std::abs(std::remainder(th4 - th, 2 * M_PI)) / std::max(acc3, p.track_bend_mm * 0.5));
          }
        }
      }
      // 비킴 1 -> 0.5 -> 0: 선 띠에 덜 들어가는 것 (같으면 큰 비킴)
      const double scales[3] = {1.0, 0.5, 0.0};
      const int ns = (p.track_outer_fit > 0 && outer != 0) ? 3 : 1;
      int best_in = std::numeric_limits<int>::max();
      double k_sel = 0;
      std::vector<cv::Point2d> pts_sel;
      for (int si = 0; si < ns; ++si) {
        const double o = outer * scales[si];
        const double kk = std::clamp(k_path + 2.0 * th / L + (near.x - o) / (L * L), -p.max_curvature, p.max_curvature);
        bool blocked;
        int inside, cnt;
        double err;
        std::vector<cv::Point2d> pts = arcOf(kk, blocked, inside, err, cnt);
        if (blocked) {
          if (p.track_block_ok_mm <= 0 || block_s < p.track_block_ok_mm) {
            continue;
          }
          inside = 100000 - static_cast<int>(block_s);   // 막히지 않은 것보다 뒤, 멀리서 막힐수록 앞
        }
        if (inside < best_in) {
          best_in = inside;
          k_sel = kk;
          pts_sel = std::move(pts);
          r.track_outer_used = o;
        }
        if (inside == 0) {
          break;
        }
      }
      if (best_in != std::numeric_limits<int>::max()) {
        arcs_.push_back(pts_sel);
        tracked = true;
        best = 0;
        best_k = k_sel;
      }
    }
    for (int a = 0; a < n && !tracked; ++a) {
      const double k = n == 1 ? 0.0 : p.max_curvature * (2.0 * a / (n - 1) - 1.0);
      bool blocked;
      double err;
      int cnt, inside;
      const std::vector<cv::Point2d> pts = arcOf(k, blocked, inside, err, cnt);
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
      const double kr = std::min(1.0, std::max(std::abs(best_k), p.track_slow_ahead * k_ahead) / p.max_curvature);
      double v = std::max(p.min_linear, p.linear * (1.0 - p.slow_curvature * kr));
      if (r.soft) {
        v *= p.soft_speed_ratio;
      }
      if (r.dead_end && p.dead_end_slow_mm > 0) {   // 막힌 곳에 가까울수록 감속
        v *= std::clamp((r.dead_end_mm - r.dead_end_target_mm) / p.dead_end_slow_mm, 0.3, 1.0);
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
  cv::Mat box_near_;                       // 박스 여유 (로봇 반폭 + box_margin) 안 칸
  std::vector<float> dist_;
  std::vector<int> parent_;
  std::vector<cv::Point2d> line_mem_, box_mem_, prev_path_;
  std::vector<int> line_age_, box_age_;   // 기억 칸이 화면 밖에 있은 프레임 수 (line_mem_ / box_mem_ 와 같은 순서)
  std::vector<std::vector<cv::Point2d>> arcs_;
  cv::Point2d prev_goal_;
  bool have_prev_goal_ = false;
  double prev_k_ = 0;
};

}  // namespace grid_drive
