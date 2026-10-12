// 2D 지역 지도 기반 경로 계획 (ROS 없이 OpenCV 만 쓰는 알고리즘 부분)
//
// 카메라는 바퀴 축보다 robot_y_offset_px 앞부터 보인다 (BEV 맨 아래 = 축에서 약 14cm 앞).
// 그 사이 안 보이는 바닥도 1~2초 전에는 카메라에 보였으므로, 본 것을 로봇 중심 지도에 쌓아 두고 쓴다.
//
// 처리 순서 (매 영상)
//   1. 로봇 자세 갱신: 바닥에 고정된 지도 위에서 로봇 자세를 직전 영상 이후 이동(Odom)만큼 옮긴다
//   2. 덮어쓰기: 지금 카메라에 보이는 영역은 이번 BEV 의 선 / 박스 마스크로 덮어쓴다
//      (선 / 박스 마스크는 path_planner::buildMaps 와 같다: 점선 빈틈 메우기, 박스 바닥 접점부터 위쪽 막힘)
//      선은 노란 선 비율도 따로 기억한다 (색 규칙에 씀)
//   3. 교차로 빈틈 메우기 (lane_rules): 로봇 좌우의 지금 차선 선을 앞으로 따라가다 끊긴 뒤
//      gap_fill_px 안에서 같은 색 선이 같은 방향으로 다시 이어지면 그 빈틈을 가상 선으로 잇는다.
//      (교차로에서 끊긴 틈으로 옆 길 / 건너편 차선으로 빠지지 않게. 갈림길 표지판 방향 쪽은 잇지 않는다)
//   4. 거리 변환: 칸마다 가장 가까운 선(가상 선 포함) / 박스까지 거리
//   5. 후보 경로: 바퀴 축에서 정면으로 출발해 곡률이 거리에 따라 변하는 곡선 k(t) = k0 + dk t.
//      지도 전체(안 보이는 구간 포함)에서 점수를 매긴다. 아직 본 적 없는 칸은 점수에서 뺀다
//   6. 점수: 박스에 닿으면 탈락, 선에 닿으면 큰 벌점, 가장 가까운 선에서 반 차선 폭일수록 가점,
//            직전 경로와 비슷할수록 가점, 갈림길 방향 가점, 가까운 PSD 쪽 벌점
//      lane_rules 일 때 (공사 구간 밖):
//        - 색 순서: 코스는 노란 선 = 왼쪽, 흰 선 = 오른쪽 (주차 구간은 양쪽 흰색).
//          오른쪽 첫 선이 노란색(노란 선을 넘음)이거나, 왼쪽 첫 선이 흰색인데 그 너머에 노란 선이 있으면
//          (흰 선을 넘음) 잘못된 쪽 -> side_weight 벌점
//        - 차선 폭: 차선 폭보다 먼 선은 기준으로 안 쓰고, 양쪽 선 간격이 차선 폭보다 많이 넓으면
//          (교차로 / 합류) 두 선의 가운데가 아니라 한쪽 선에서 반 차선 폭을 따른다
//   7. 고른 경로 위, 바퀴 축에서 lookahead_px 떨어진 목표점 -> offset / angle (LaneInfo 규약)
//
// 좌표: 로봇 좌표 (x 오른쪽, y 앞, 원점 = 바퀴 축 가운데) [BEV px], 지도 칸 (u 오른쪽, v 아래 = 뒤)
// 곡률 k > 0, 방향 > 0 = 오른쪽. Odom 회전은 ROS 규약대로 + = 왼쪽
#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "path_planner/planner_core.hpp"

namespace map_planner
{

using path_planner::Bias;
using path_planner::Odom;

struct Params
{
  // 0. 원본 영상을 바로 지도에 펼칠 때 (input = raw): bird_eye_view.yaml 의 사다리꼴과 같게.
  //    BEV 는 이 사다리꼴(로봇 좌우 약 ±20cm)만 잘라 펴지만, 같은 변환은 바닥 전체에 맞으므로
  //    원본의 양옆 / 먼 곳도 지도에 넣을 수 있다 (넓은 공사 구간의 양쪽 선)
  double src_top_y = 0.266, src_top_left_x = 0.264, src_top_right_x = 0.736;
  double src_bottom_y = 1.0, src_bottom_left_x = -0.061, src_bottom_right_x = 1.061;
  int bev_width = 400, bev_height = 400;
  double raw_max_range_px = 1000.0;   // 원본에서 지도에 넣을 가장 먼 거리 (바퀴 축 기준, 멀수록 흐림)
  double raw_max_lateral_px = 450.0;  // 원본에서 지도에 넣을 좌우 범위
  double raw_scale = 0.5;             // 원본 마스크는 이 배율로 줄여 만든다 (옆 / 먼 곳 보충용이라 충분, 속도)
  // 1. 지도 (단위는 BEV px, 1px ≈ 1mm)
  double bev_center_x = 0.5;          // 로봇 가로 위치 (BEV 폭 비율)
  double robot_y_offset_px = 141.0;   // 바퀴 축이 BEV 맨 아래보다 얼마나 뒤에 있는지
  double cell_px = 4.0;               // 지도 한 칸 크기
  double map_size_px = 2000.0;        // 지도 한 변 (로봇이 가운데)
  double map_recenter_px = 300.0;     // 로봇이 지도 가운데에서 이만큼 벗어나면 지도를 옮긴다
  double memory_px = 700.0;           // 마지막으로 본 뒤 이만큼 넘게 달리면 그 칸은 잊는다
                                      // (바퀴 이동량 오차가 쌓인 오래된 관측 버리기, 안 보이는 구간 141 보다 커야 함)
  double raw_memory_px = 100.0;       // 원본으로만 본 칸 (BEV 밖 옆 / 먼 곳) 은 이만큼만 기억
                                      // (시야 밖으로 나간 옆쪽 선이 이동량 오차로 어긋나 이중선이 되는 것 방지)
  double view_half_width_px = 600.0;  // 화면용 창: 로봇 좌우
  double view_front_px = 750.0;       // 화면용 창: 로봇 앞 (BEV 맨 위 = 141 + 400, 그 앞은 원본 보충)
  double view_back_px = 200.0;        // 화면용 창: 로봇 뒤
  // 2. 후보 경로
  int n_paths = 21;                   // 시작 곡률 k0 개수
  double max_curvature = 1.0 / 150.0; // [1/px] (반지름 150px)
  int n_curv_rates = 9;               // 곡률 변화율 dk 개수
  double max_curvature_rate = 1.0 / 150.0 / 150.0;   // [1/px^2] (150px 동안 max_curvature 만큼)
  double path_length_px = 500.0;      // 길면 먼 끝(곡률 일정 가정이 안 맞는 곳)의 선 벌점이 작은 보정을 막는다
  double sample_step_px = 5.0;
  double min_known_ratio = 0.15;      // 경로 샘플 중 본 적 있는 칸이 이보다 적으면 탈락 (처음엔 안 보이는 구간이 비어 약 0.33)
  // 3. 점수
  double robot_half_width_px = 102.5; // 경로 중심에서 이 거리 안에 선/박스가 있으면 닿은 것
  // 차선 가운데 점수: 샘플마다 경로에 수직으로 좌우 max_lane_width_px 안에서 선을 찾는다.
  //   양쪽 다 있으면 두 선의 가운데일수록 만점 (공사 구간처럼 넓은 차선도 폭을 몰라도 가운데로)
  //   한쪽만 있으면 그 선에서 clearance_cap_px (보통 차선 폭 / 2) 일 때 만점
  //   (커브 안쪽 선이 시야 밖으로 나가 선이 하나만 보일 때)
  // 박스는 이 거리 이상이면 만점
  double clearance_cap_px = 148.0;
  double max_lane_width_px = 600.0;
  double line_weight = 10.0;          // 선에 닿은 샘플 비율 x 이 값 = 벌점 (공사 구간에서는 노드가 낮춤)
  double clearance_weight = 1.0;
  double align_weight = 1.0;
  double box_margin_px = 80.0;        // 박스까지 반폭 + 이 거리 안이면 가까울수록 벌점 (일찍 비켜 가게)
  double box_weight = 3.0;          // 선과 나란하지 않은 경로 벌점 (비스듬히 가운데를 가로지르는 경로 방지)
  double smooth_weight = 0.5;         // 직전 경로와 곡률 차이 벌점
  double straight_weight = 0.1;       // 급커브 벌점 (평균 곡률 기준)
  double fork_weight = 2.0;           // 갈림길 방향 가점
  // 선 넘기: 경로 중심이 선(교차로 가상 선 포함) 위를 지나가면 벌점 (lane_rules 일 때만, 공사 구간은 끔).
  //   line_weight 는 "선 근처를 지난 비율"이라 급커브에서 선을 스치며 도는 경로와 선을 가로질러
  //   직진하는 경로를 구분 못 한다 (2026-10-12 실주행: 좌회전 중 바깥 흰 선을 넘는 직진 경로가 이김)
  double cross_line_px = 8.0;         // 경로 중심에서 선까지 이보다 가까운 샘플이 있으면 넘은 것
  double cross_weight = 5.0;
  double psd_weight = 2.0;            // 가까운 PSD 쪽으로 도는 경로 벌점
  // 차선 규칙 (위 6. 설명). false 면 예전처럼 색 / 차선 폭을 안 보고 좌우 가장 가까운 선의 가운데로
  // (공사 구간: 차선이 넓고 박스를 피해 선을 넘어야 해서 노드가 끈다)
  bool lane_rules = true;
  double lane_width_tolerance = 0.35; // 양쪽 선 간격이 lane_width_px x (1 + 이 값) 보다 넓으면 한 차선이 아님
  double side_weight = 5.0;           // 색 순서가 틀린 샘플 비율 x 이 값 = 벌점
  // true: 왼쪽 첫 선이 흰색이면 무조건 틀린 자리 (흰 선 = 오른쪽 선). false: 그 너머 노란 선이 보일 때만
  //   (주차 구간은 양쪽이 흰 선이라 노드가 주차 표지판을 본 뒤 잠시 false 로 바꾼다)
  bool strict_colors = true;
  // 복귀: 로봇이 지금 노란 선의 왼쪽(트랙 밖 / 코너를 잘라 노란 선을 넘음)에 있으면 그 노란 선은 넘어도
  //   선 벌점을 안 준다. 안 그러면 노란 선을 다시 넘어 차선으로 가는 경로가 선 벌점에 져서, 색 규칙 벌점을
  //   받으면서도 노란 선에서 멀어지는 쪽(트랙 밖)으로 간다 (2026-10-12 시험)
  bool recover_wrong_side = true;
  double recover_inlane_px = 75.0;    // 복귀 중 이만큼 노란 선 오른쪽(차선)을 지나지 않는 경로는 side_weight 벌점
  double gap_fill_px = 600.0;         // 지금 차선 선이 끊겼다 다시 이어지는 빈틈을 이 길이까지 메움 (0 = 끔)
  // 4. 출력
  double lane_width_px = 295.0;       // offset 정규화용 (반 차선 폭 = 1)
  double lookahead_px = 200.0;        // 바퀴 축에서 목표점까지 거리 (짧을수록 커브 안쪽으로 덜 붙는다)
  // lane_follower 는 곡률 = 2 sin(angle) / pp_lookahead 로 조향하므로, 목표점 거리가 달라도 같은 곡률이 되게
  // angle 을 환산해 내보낸다 (lane_follower pp_lookahead 0.25m x 1000px/m)
  double follower_lookahead_px = 250.0;
  double smooth_alpha = 0.6;          // 고른 곡률(k0, dk)의 새 값 비중 (1 = 필터 없음)
  int hold_frames = 5;
  int min_wall_pixels = 200;          // 이번 BEV 에 선 + 박스 픽셀이 이보다 적으면 "아무것도 안 보임"
};

struct Candidate
{
  double k0 = 0, dk = 0;              // 시작 곡률 [1/px], 곡률 변화율 [1/px^2]
  double k = 0;                       // 평균 곡률 [1/px] (도는 방향)
  std::vector<cv::Point2d> pts;       // 샘플점 (로봇 좌표, 가까운 쪽부터, 지도 밖에서 끊김)
  bool blocked = false;               // 박스에 닿았거나 본 적 있는 칸이 너무 적어 탈락
  bool touches_line = false;
  double line_hit = 0;                // 본 칸 중 선에 닿은 샘플 비율 [0, 1]
  double min_line_dist = 1e9;         // 경로 중심에서 가장 가까운 선까지 [px] (본 칸만)
  bool crosses_line = false;          // 경로 중심이 선 위를 지나감 (선을 넘음)
  double clearance = 0;               // 차선 가운데 / 박스에서 먼 정도 평균 [0, 1] (본 적 없는 샘플은 0)
  double misalign = 0;                // 선과 나란하지 않은 정도 평균 [0, 1]
  double box_near = 0;                // 박스에 가까운 정도 평균 [0, 1]
  double wrong_side = 0;              // 색 순서가 틀린 샘플 비율 (원래 길이 기준) [0, 1]
  double known = 0;                   // 본 적 있는 칸 비율 (원래 길이 기준)
  double score = 0;
};

struct Result
{
  bool detected = false;
  float offset = 0, angle = 0, confidence = 0;
  double k0 = 0, dk = 0, k = 0;       // 필터를 거친 경로
  int best = -1;                      // 이번 영상에 고른 후보 (없으면 -1)
  bool holding = false;               // 이번에 못 골라 직전 값으로 버티는 중
  bool recovering = false;            // 로봇이 노란 선 왼쪽에 있어 노란 선 넘기를 허용 중
  std::vector<cv::Point2d> path;      // 따라갈 경로 (로봇 좌표)
  cv::Point2d target;                 // 목표점 (로봇 좌표)
  std::vector<Candidate> cands;
  // 교차로 빈틈을 메운 가상 선 (로봇 좌표, 시작 / 끝), 노란 선이면 true
  struct Fill
  {
    cv::Point2d a, b;
    bool yellow;
  };
  std::vector<Fill> fills;
};

// BEV -> 원본 영상 투시 변환 (bird_eye_view 노드와 같은 식)
inline cv::Matx33d bevToRaw(const Params & p, cv::Size raw)
{
  const float w = static_cast<float>(raw.width), h = static_cast<float>(raw.height);
  const cv::Point2f src[4] = {
    {static_cast<float>(p.src_top_left_x) * w, static_cast<float>(p.src_top_y) * h},
    {static_cast<float>(p.src_top_right_x) * w, static_cast<float>(p.src_top_y) * h},
    {static_cast<float>(p.src_bottom_right_x) * w, static_cast<float>(p.src_bottom_y) * h},
    {static_cast<float>(p.src_bottom_left_x) * w, static_cast<float>(p.src_bottom_y) * h},
  };
  const float bw = static_cast<float>(p.bev_width), bh = static_cast<float>(p.bev_height);
  const cv::Point2f dst[4] = {{0, 0}, {bw, 0}, {bw, bh}, {0, bh}};
  return cv::Matx33d(cv::getPerspectiveTransform(dst, src));
}

// 원본 -> BEV (bird_eye_view 노드와 같은 변환). valid: 원본 안에 들어오는 BEV 칸 (0/255)
inline cv::Mat makeBev(const cv::Mat & raw, const Params & p, cv::Mat * valid = nullptr)
{
  const cv::Mat G(bevToRaw(p, raw.size()));
  const cv::Size sz(p.bev_width, p.bev_height);
  cv::Mat bev;
  cv::warpPerspective(raw, bev, G, sz, cv::INTER_LINEAR | cv::WARP_INVERSE_MAP,
    cv::BORDER_CONSTANT);
  if (valid) {
    cv::warpPerspective(cv::Mat(raw.size(), CV_8U, cv::Scalar(255)), *valid, G, sz,
      cv::INTER_NEAREST | cv::WARP_INVERSE_MAP, cv::BORDER_CONSTANT, 0);
    cv::erode(*valid, *valid, cv::getStructuringElement(cv::MORPH_RECT, {5, 5}));
  }
  return bev;
}

// 영상 한 장에서 지도에 넣을 것: BEV 마스크(가까운 가운데, 정확) + 원본 마스크(넓은 범위 보충)
struct Observation
{
  cv::Mat bev, bev_valid;          // 원본으로 만든 BEV, 원본 안에 들어오는 BEV 칸
  path_planner::Maps bev_maps;     // BEV 색 마스크 (점선 메우기 포함)
  path_planner::Maps raw_maps;     // 줄인 원본 색 마스크 (점선 메우기 없음: 먼 곳에서 선이 길게 늘어남)
};

inline Observation observe(const cv::Mat & raw, const path_planner::Params & mask, const Params & p)
{
  Observation o;
  o.bev = makeBev(raw, p, &o.bev_valid);
  o.bev_maps = path_planner::buildMaps(o.bev, mask, false);
  const double sc = std::clamp(p.raw_scale, 0.1, 1.0);
  cv::Mat small;
  if (sc < 1.0) {
    cv::resize(raw, small, {}, sc, sc, cv::INTER_AREA);
  } else {
    small = raw;
  }
  path_planner::Params rm = mask;
  rm.line_gap_close_px = 1;
  rm.box_min_area = std::max(1, cvRound(mask.box_min_area * sc * sc));
  o.raw_maps = path_planner::buildMaps(small, rm, false);
  return o;
}

// 바닥에 고정된 지도: 선 / 그중 노란 선 / 박스 / 본 적 있음 (CV_32F, 0~1) + 지도 위 로봇 자세
//   지도를 매번 옮기면(보간) 얇은 선이 흐려져 사라지므로, 지도는 그대로 두고 로봇 자세만 Odom 으로 갱신한다.
//   로봇이 지도 가운데에서 recenter_px 넘게 벗어나면 지도를 정수 칸만큼만 옮긴다 (보간 없음).
//   세계 좌표: 처음 로봇 좌표와 같은 방향 (x 오른쪽, y 앞) [BEV px], 칸 (u, v) 의 중심
//     wx = origin_x + (u + 0.5) cell, wy = origin_y - (v + 0.5) cell
class LocalMap
{
public:
  void configure(const Params & p)
  {
    const int n = std::max(16, cvRound(p.map_size_px / p.cell_px));
    if (cell_ != p.cell_px || line.rows != n) {
      cell_ = p.cell_px;
      line = cv::Mat::zeros(n, n, CV_32F);
      yellow = cv::Mat::zeros(n, n, CV_32F);
      box = cv::Mat::zeros(n, n, CV_32F);
      known = cv::Mat::zeros(n, n, CV_32F);
      stamp_ = cv::Mat(n, n, CV_32F, cv::Scalar(kNever));
      resetPose();
    }
    recenter_px_ = p.map_recenter_px;
    memory_px_ = p.memory_px;
  }

  void clear()
  {
    line.setTo(0);
    yellow.setTo(0);
    box.setTo(0);
    known.setTo(0);
    stamp_.setTo(kNever);
    resetPose();
  }

  // 로봇 좌표 <-> 세계 좌표 <-> 칸 (실수)
  cv::Point2d robotToWorld(const cv::Point2d & q) const
  {
    const double c = std::cos(psi_), s = std::sin(psi_);
    return {px_ + q.x * c + q.y * s, py_ - q.x * s + q.y * c};
  }
  cv::Point2d worldToRobot(const cv::Point2d & wq) const
  {
    const double c = std::cos(psi_), s = std::sin(psi_);
    const double dx = wq.x - px_, dy = wq.y - py_;
    return {c * dx - s * dy, s * dx + c * dy};
  }
  cv::Point2d toCell(const cv::Point2d & q) const
  {
    const auto wq = robotToWorld(q);
    return {(wq.x - ox_) / cell_ - 0.5, (oy_ - wq.y) / cell_ - 0.5};
  }
  cv::Point2d cellToRobot(const cv::Point2d & c) const
  {
    return worldToRobot({ox_ + (c.x + 0.5) * cell_, oy_ - (c.y + 0.5) * cell_});
  }
  // 로봇 좌표 q 에 있는 칸의 값 (지도 밖이면 0)
  float at(const cv::Mat & m, const cv::Point2d & q) const
  {
    const auto c = toCell(q);
    const int u = cvRound(c.x), v = cvRound(c.y);
    if (u < 0 || v < 0 || u >= m.cols || v >= m.rows) {
      return 0.0f;
    }
    return m.at<float>(v, u);
  }

  // 1. 로봇이 ds 앞으로 가며 dtheta(왼쪽 +) 돌았다
  void move(const Odom & od)
  {
    const double dpsi = -od.dtheta;   // 지도 쪽 방향은 + = 오른쪽
    const double mid = psi_ + dpsi / 2;
    px_ += od.ds_px * std::sin(mid);
    py_ += od.ds_px * std::cos(mid);
    psi_ += dpsi;
    odo_ += std::abs(od.ds_px);
    recenter();
    // 오래전에 본 칸은 잊는다
    known.setTo(0.0f, stamp_ < odo_ - memory_px_);
  }

  // 2. 이번 BEV 에서 보이는 영역을 덮어쓴다
  //    line_wall / yellow_wall / box_blocked: BEV 크기 0/255 마스크 (yellow_wall 은 비어 있어도 됨),
  //    valid: 실제로 찍힌 BEV 칸 (0/255)
  void integrate(
    const cv::Mat & line_wall, const cv::Mat & yellow_wall, const cv::Mat & box_blocked,
    const cv::Mat & valid, double bev_center_x, double robot_y_offset_px)
  {
    const int w = line_wall.cols, h = line_wall.rows;
    const double cx = bev_center_x * w, off = robot_y_offset_px;
    // BEV 네 모서리가 들어가는 칸 범위만 계산한다
    cv::Rect roi;
    {
      std::vector<cv::Point2f> pts;
      for (const auto & q : {cv::Point2d(-cx, off), cv::Point2d(w - cx, off),
          cv::Point2d(-cx, off + h), cv::Point2d(w - cx, off + h)})
      {
        const auto c = toCell(q);
        pts.emplace_back(static_cast<float>(c.x), static_cast<float>(c.y));
      }
      roi = cv::boundingRect(pts) & cv::Rect(0, 0, line.cols, line.rows);
    }
    if (roi.area() == 0) {
      return;
    }
    // 칸 (u, v) -> 세계 -> 로봇 -> BEV (bx, by) 는 선형 변환
    //   dx = cell u + ax, dy = -cell v + ay, x = c dx - s dy, y = s dx + c dy
    //   bx = cx + x, by = (h - 1) + off - y
    const double c = std::cos(psi_), s = std::sin(psi_);
    const double ax = ox_ + (roi.x + 0.5) * cell_ - px_, ay = oy_ - (roi.y + 0.5) * cell_ - py_;
    cv::Mat M = (cv::Mat_<double>(2, 3) <<
      c * cell_, s * cell_, cx + c * ax - s * ay,
      -s * cell_, c * cell_, (h - 1) + off - s * ax - c * ay);
    cv::Mat vis, l, y, b;
    cv::warpAffine(valid, vis, M, roi.size(), cv::INTER_NEAREST | cv::WARP_INVERSE_MAP,
      cv::BORDER_CONSTANT, 0);
    cv::warpAffine(line_wall, l, M, roi.size(), cv::INTER_LINEAR | cv::WARP_INVERSE_MAP,
      cv::BORDER_CONSTANT, 0);
    if (yellow_wall.empty()) {
      y = cv::Mat::zeros(roi.size(), CV_8U);
    } else {
      cv::warpAffine(yellow_wall, y, M, roi.size(), cv::INTER_LINEAR | cv::WARP_INVERSE_MAP,
        cv::BORDER_CONSTANT, 0);
    }
    cv::warpAffine(box_blocked, b, M, roi.size(), cv::INTER_LINEAR | cv::WARP_INVERSE_MAP,
      cv::BORDER_CONSTANT, 0);
    l.convertTo(l, CV_32F, 1.0 / 255);
    y.convertTo(y, CV_32F, 1.0 / 255);
    b.convertTo(b, CV_32F, 1.0 / 255);
    l.copyTo(line(roi), vis);
    y.copyTo(yellow(roi), vis);
    b.copyTo(box(roi), vis);
    known(roi).setTo(1.0f, vis);
    stamp_(roi).setTo(odo_, vis);
  }

  // 2'. 원본 영상 마스크를 바로 펼쳐 덮어쓴다 (BEV 와 같은 투시 변환, 범위만 넓게)
  //    line_wall / yellow_wall / box_blocked: 원본 크기 0/255 마스크 (yellow_wall 은 비어 있어도 됨)
  void integrateRaw(
    const cv::Mat & line_wall, const cv::Mat & yellow_wall, const cv::Mat & box_blocked,
    const Params & p)
  {
    // 원본으로만 본 칸은 memory_px 대신 raw_memory_px 만 기억되도록 본 시각을 앞당겨 적는다
    const double stamp = odo_ - std::max(0.0, memory_px_ - p.raw_memory_px);
    const double cx = p.bev_center_x * p.bev_width, off = p.robot_y_offset_px;
    const double hb = p.bev_height;
    // 넣을 범위: 로봇 좌표 x in [-lat, lat], y in [off, max_range] (원본 맨 아래 줄 = BEV 맨 아래 = off)
    const double lat = p.raw_max_lateral_px, far = std::max(p.raw_max_range_px, off + 1);
    std::vector<cv::Point2f> corners;
    for (const auto & q : {cv::Point2d(-lat, off), cv::Point2d(lat, off), cv::Point2d(lat, far),
        cv::Point2d(-lat, far)})
    {
      const auto c = toCell(q);
      corners.emplace_back(static_cast<float>(c.x), static_cast<float>(c.y));
    }
    const cv::Rect roi = cv::boundingRect(corners) & cv::Rect(0, 0, line.cols, line.rows);
    if (roi.area() == 0) {
      return;
    }
    // 칸 -> BEV (선형, integrate 와 같은 식) -> 원본 (투시)
    const double c = std::cos(psi_), s = std::sin(psi_);
    const double ax = ox_ + (roi.x + 0.5) * cell_ - px_, ay = oy_ - (roi.y + 0.5) * cell_ - py_;
    const cv::Matx33d A(
      c * cell_, s * cell_, cx + c * ax - s * ay,
      -s * cell_, c * cell_, (hb - 1) + off - s * ax - c * ay,
      0, 0, 1);
    const cv::Matx33d P = bevToRaw(p, line_wall.size()) * A;
    cv::Mat vis(roi.size(), CV_8U, cv::Scalar(0)), l, y, b, inside;
    {
      // 범위 사각형 (로봇 좌표) 을 ROI 칸 좌표로
      std::vector<cv::Point> poly;
      for (const auto & q : corners) {
        poly.emplace_back(cvRound(q.x - roi.x), cvRound(q.y - roi.y));
      }
      cv::fillConvexPoly(vis, poly, cv::Scalar(255));
    }
    const cv::Mat full(line_wall.size(), CV_8U, cv::Scalar(255));
    cv::warpPerspective(full, inside, cv::Mat(P), roi.size(),
      cv::INTER_NEAREST | cv::WARP_INVERSE_MAP, cv::BORDER_CONSTANT, 0);
    vis &= inside;   // 원본 영상 안에 들어오는 칸만
    cv::warpPerspective(line_wall, l, cv::Mat(P), roi.size(),
      cv::INTER_LINEAR | cv::WARP_INVERSE_MAP, cv::BORDER_CONSTANT, 0);
    if (yellow_wall.empty()) {
      y = cv::Mat::zeros(roi.size(), CV_8U);
    } else {
      cv::warpPerspective(yellow_wall, y, cv::Mat(P), roi.size(),
        cv::INTER_LINEAR | cv::WARP_INVERSE_MAP, cv::BORDER_CONSTANT, 0);
    }
    cv::warpPerspective(box_blocked, b, cv::Mat(P), roi.size(),
      cv::INTER_LINEAR | cv::WARP_INVERSE_MAP, cv::BORDER_CONSTANT, 0);
    l.convertTo(l, CV_32F, 1.0 / 255);
    y.convertTo(y, CV_32F, 1.0 / 255);
    b.convertTo(b, CV_32F, 1.0 / 255);
    l.copyTo(line(roi), vis);
    y.copyTo(yellow(roi), vis);
    b.copyTo(box(roi), vis);
    known(roi).setTo(1.0f, vis);
    stamp_(roi).setTo(stamp, vis);
  }

  // 4. 칸마다 가장 가까운 선 / 박스까지 거리 [px]. line_layer 를 주면 line 대신 그것으로 (가상 선 포함)
  void distances(cv::Mat & dist_line, cv::Mat & dist_box, const cv::Mat & line_layer = {}) const
  {
    auto dist = [this](const cv::Mat & ev, cv::Mat & out) {
        cv::Mat free = (ev < 0.5f);   // 벽이 아닌 칸 = 255
        cv::distanceTransform(free, out, cv::DIST_L2, 3);
        out *= cell_;
      };
    dist(line_layer.empty() ? line : line_layer, dist_line);
    dist(box, dist_box);
  }

  // 화면용: 로봇 중심 창 (좌우 half, 앞 front, 뒤 back [px]) 으로 잘라 돌린 그림 (칸 크기 그대로)
  cv::Mat robotView(const cv::Mat & m, double half, double front, double back) const
  {
    const int cols = cvRound(2 * half / cell_), rows = cvRound((front + back) / cell_);
    // 창 칸 (i, j) -> 로봇 (x = (i + 0.5) cell - half, y = front - (j + 0.5) cell) -> 지도 칸
    const double c = std::cos(psi_), s = std::sin(psi_);
    const double x0 = 0.5 * cell_ - half, y0 = front - 0.5 * cell_;
    // 세계: wx = px + x c + y s, wy = py - x s + y c, 칸: u = (wx - ox)/cell - 0.5, v = (oy - wy)/cell - 0.5
    cv::Mat M = (cv::Mat_<double>(2, 3) <<
      c, -s, (px_ + x0 * c + y0 * s - ox_) / cell_ - 0.5,
      s, c, (oy_ - py_ + x0 * s - y0 * c) / cell_ - 0.5);
    cv::Mat out;
    cv::warpAffine(m, out, M, {cols, rows}, cv::INTER_NEAREST | cv::WARP_INVERSE_MAP,
      cv::BORDER_CONSTANT, 0);
    return out;
  }

  double cell() const {return cell_;}

  cv::Mat line, yellow, box, known;   // yellow: line 중 노란 선 (yellow <= line)

private:
  void resetPose()
  {
    px_ = py_ = psi_ = 0;
    // 로봇이 지도 가운데에 오게
    ox_ = -0.5 * line.cols * cell_;
    oy_ = 0.5 * line.rows * cell_;
  }

  void recenter()
  {
    const double cxw = ox_ + 0.5 * line.cols * cell_, cyw = oy_ - 0.5 * line.rows * cell_;
    if (std::abs(px_ - cxw) <= recenter_px_ && std::abs(py_ - cyw) <= recenter_px_) {
      return;
    }
    // 로봇이 가운데 오도록 정수 칸만큼 옮긴다
    const int du = cvRound((px_ - cxw) / cell_), dv = cvRound((cyw - py_) / cell_);
    for (cv::Mat * m : {&line, &yellow, &box, &known, &stamp_}) {
      cv::Mat out(m->size(), m->type(), cv::Scalar(m == &stamp_ ? kNever : 0.0));
      const cv::Rect src = cv::Rect(du, dv, m->cols, m->rows) & cv::Rect(0, 0, m->cols, m->rows);
      if (src.area() > 0) {
        (*m)(src).copyTo(out(cv::Rect(src.x - du, src.y - dv, src.width, src.height)));
      }
      *m = out;
    }
    ox_ += du * cell_;
    oy_ -= dv * cell_;
  }

  static constexpr double kNever = -1e9;
  cv::Mat stamp_;                      // 칸을 마지막으로 본 때의 누적 주행 거리 [px]
  double odo_ = 0, memory_px_ = 700;
  double cell_ = 0, recenter_px_ = 300;
  double px_ = 0, py_ = 0, psi_ = 0;   // 세계 좌표 로봇 자세 (psi + = 오른쪽)
  double ox_ = 0, oy_ = 0;             // 칸 (0, 0) 의 왼쪽 위 모서리 세계 좌표
};

// 곡선 k0 + dk t 의 길이 length 동안 평균 곡률 (k_max 로 자른 것)
inline double meanCurvature(double k0, double dk, double k_max, double length, double step)
{
  double sum = 0;
  int n = 0;
  for (double t = step / 2; t < length; t += step, ++n) {
    sum += std::clamp(k0 + dk * t, -k_max, k_max);
  }
  return n > 0 ? sum / n : k0;
}

// 바퀴 축에서 정면으로 출발하는 곡선 k(t) = k0 + dk t (|k| <= k_max), 90도 넘게 돌면 멈춘다
inline std::vector<cv::Point2d> curveFromRobot(
  double k0, double dk, double k_max, double length, double step)
{
  std::vector<cv::Point2d> pts{{0, 0}};
  cv::Point2d q(0, 0);
  double hd = 0;
  for (double t = 0; t + step <= length + 1e-9; t += step) {
    const double k = std::clamp(k0 + dk * (t + step / 2), -k_max, k_max);
    const double dh = k * step;
    q.x += step * std::sin(hd + dh / 2);
    q.y += step * std::cos(hd + dh / 2);
    hd += dh;
    if (std::abs(hd) > CV_PI / 2) {
      break;
    }
    pts.push_back(q);
  }
  return pts;
}

class MapPlanner
{
public:
  // bev_maps: path_planner::buildMaps 결과, valid: 실제로 찍힌 BEV 칸 (0/255, 비어 있으면 전부)
  Result step(
    const path_planner::Maps & bev_maps, const cv::Mat & valid, const Params & p,
    const Bias & bias, const Odom & od = {})
  {
    map_.configure(p);
    map_.move(od);
    if (has_prev_) {
      prev_k0_ = std::clamp(prev_k0_ + prev_dk_ * od.ds_px, -p.max_curvature, p.max_curvature);
    }
    cv::Mat v = valid;
    if (v.empty()) {
      v = cv::Mat(bev_maps.line_wall.size(), CV_8U, cv::Scalar(255));
    }
    map_.integrate(bev_maps.line_wall, bev_maps.yellow_wall, bev_maps.box_blocked, v,
      p.bev_center_x, p.robot_y_offset_px);
    const bool sees_walls =
      bev_maps.yellow_pixels + bev_maps.white_pixels + bev_maps.box_pixels >= p.min_wall_pixels;
    return plan(p, bias, sees_walls);
  }

  // 원본 + BEV 섞기: 원본(넓은 범위)을 먼저 펼치고, BEV 사다리꼴 안은 BEV 마스크로 덮어쓴다
  //   raw_maps: 원본 영상의 buildMaps 결과, bev_maps / bev_valid: 같은 영상으로 만든 BEV 의 결과
  //   (BEV 쪽 마스크가 형태 처리(점선 메우기 등)가 거리마다 일정해 더 정확하다)
  Result step(const Observation & o, const Params & p, const Bias & bias, const Odom & od = {})
  {
    return stepRaw(o.raw_maps, o.bev_maps, o.bev_valid, p, bias, od);
  }

  Result stepRaw(
    const path_planner::Maps & raw_maps, const path_planner::Maps & bev_maps,
    const cv::Mat & bev_valid, const Params & p, const Bias & bias, const Odom & od = {})
  {
    map_.configure(p);
    map_.move(od);
    if (has_prev_) {
      prev_k0_ = std::clamp(prev_k0_ + prev_dk_ * od.ds_px, -p.max_curvature, p.max_curvature);
    }
    map_.integrateRaw(raw_maps.line_wall, raw_maps.yellow_wall, raw_maps.box_blocked, p);
    map_.integrate(bev_maps.line_wall, bev_maps.yellow_wall, bev_maps.box_blocked, bev_valid,
      p.bev_center_x, p.robot_y_offset_px);
    // 선 / 박스가 보이는지: BEV 또는 원본 (원본은 줄인 배율만큼 픽셀 수를 되돌려 비교)
    const double sc = std::clamp(p.raw_scale, 0.1, 1.0);
    const bool sees_walls =
      bev_maps.yellow_pixels + bev_maps.white_pixels + bev_maps.box_pixels >= p.min_wall_pixels ||
      (raw_maps.yellow_pixels + raw_maps.white_pixels + raw_maps.box_pixels) / (sc * sc) >=
      p.min_wall_pixels;
    return plan(p, bias, sees_walls);
  }

  const LocalMap & map() const {return map_;}
  void reset() {has_prev_ = false; miss_ = 0; map_.clear();}

private:
  Result plan(const Params & p, const Bias & bias, bool sees_walls)
  {
    Result r;
    // 3. 교차로 빈틈을 가상 선으로 메운 선 지도 (메울 게 없으면 지도 그대로) -> 4. 거리 변환
    line_ = map_.line;
    yellow_ = map_.yellow;
    if (p.lane_rules && p.gap_fill_px > 0) {
      line_ = map_.line.clone();
      yellow_ = map_.yellow.clone();
      fillGaps(p, bias, r);
    }
    map_.distances(dist_line_, dist_box_, line_);
    // 3-1. 복귀: 로봇이 노란 선 왼쪽에 있으면 노란 선을 뺀 선 지도로 거리를 다시 구한다
    //      (노란 선 닿기 / 넘기 벌점 없음). 대신 노란 선을 넘어 그 오른쪽(차선)에 들어가지 않는 경로는
    //      side_weight 벌점: 노란 선에서 멀어지는 경로는 곧 본 적 없는 칸으로 빠져 색 규칙 벌점이 줄어서 이긴다
    if (p.lane_rules && p.recover_wrong_side && robotLeftOfYellow(p)) {
      r.recovering = true;
      cv::Mat white_only = line_.clone();
      white_only.setTo(0, yellow_ * 2 >= line_);
      map_.distances(dist_line_, dist_box_, white_only);
    }
    const double k_max = std::max(p.max_curvature, 1e-6);
    const double dk_max = std::max(p.max_curvature_rate, 0.0);
    const double cap = std::max(p.clearance_cap_px, 1.0);
    const double step = std::max(1.0, p.sample_step_px);
    const int n = std::max(1, p.n_paths), nr = std::max(1, p.n_curv_rates);
    // -1 ~ 1 을 제곱 간격으로: 0 근처(작은 보정)를 촘촘하게, 끝(급커브)은 듬성듬성
    auto spread = [](int i, int cnt) {
        const double u = cnt == 1 ? 0.0 : -1.0 + 2.0 * i / (cnt - 1);
        return u * std::abs(u);
      };

    double best_score = -1e18;
    for (int i = 0; i < n; ++i) {
      for (int j = 0; j < nr; ++j) {
        Candidate c;
        c.k0 = k_max * spread(i, n);
        c.dk = dk_max * spread(j, nr);
        c.k = meanCurvature(c.k0, c.dk, k_max, p.path_length_px, step);
        const auto curve = curveFromRobot(c.k0, c.dk, k_max, p.path_length_px, step);
        int seen = 0, hits = 0, wrong_n = 0, inlane_n = 0;
        double clear_sum = 0, align_sum = 0, box_sum = 0;
        cv::Point2d prev_cell = map_.toCell(curve[0]);
        for (size_t u = 1; u < curve.size(); ++u) {
          const cv::Point2d cc = map_.toCell(curve[u]);
          const int qx = cvRound(cc.x), qy = cvRound(cc.y);
          if (qx < 0 || qx >= dist_line_.cols || qy < 0 || qy >= dist_line_.rows) {
            break;   // 지도 밖
          }
          c.pts.push_back(curve[u]);
          const cv::Point2d prev_c = prev_cell;
          prev_cell = cc;
          if (map_.known.at<float>(qy, qx) < 0.5f) {
            continue;   // 아직 본 적 없는 칸
          }
          ++seen;
          const float dl = dist_line_.at<float>(qy, qx), db = dist_box_.at<float>(qy, qx);
          if (db < p.robot_half_width_px) {
            c.blocked = true;
          } else if (p.box_margin_px > 0) {
            box_sum += std::max(0.0, 1 - (db - p.robot_half_width_px) / p.box_margin_px);
          }
          c.min_line_dist = std::min<double>(c.min_line_dist, dl);
          c.crosses_line = c.crosses_line || dl < p.cross_line_px;
          if (dl < p.robot_half_width_px) {
            ++hits;
            c.touches_line = true;
          }
          int wrong = 0;
          bool yellow_left = false;
          clear_sum += std::min(centerTerm(qx, qy, cc - prev_c, p, bias.fork_dir, wrong, &yellow_left),
            std::min<double>(db, cap) / cap);
          wrong_n += wrong != 0;
          inlane_n += wrong == 0 && yellow_left;
          // 가까운 선과 나란한지: 선까지 거리의 기울기(선에서 멀어지는 방향)와 진행 방향의 내적
          if (dl < 2 * cap && qx > 0 && qy > 0 && qx < dist_line_.cols - 1 &&
            qy < dist_line_.rows - 1)
          {
            const double gx = dist_line_.at<float>(qy, qx + 1) - dist_line_.at<float>(qy, qx - 1);
            const double gy = dist_line_.at<float>(qy + 1, qx) - dist_line_.at<float>(qy - 1, qx);
            const cv::Point2d t = cc - prev_c;
            const double gn = std::hypot(gx, gy), tn = std::hypot(t.x, t.y);
            if (gn > 1e-6 && tn > 1e-6) {
              align_sum += std::abs(gx * t.x + gy * t.y) / (gn * tn);
            }
          }
        }
        // 평균은 원래 길이의 샘플 수로 나눈다: 90도 넘게 돌아 끊긴 부분, 지도 밖, 본 적 없는 칸은
        // 가점 0 으로 친다 (짧은 경로나 모르는 곳으로 가는 경로가 유리해지지 않게)
        const double expect = std::max(1.0, std::floor(p.path_length_px / step));
        c.known = seen / expect;
        if (seen == 0 || c.known < p.min_known_ratio) {
          c.blocked = true;
        } else {
          c.line_hit = static_cast<double>(hits) / seen;
          c.clearance = clear_sum / expect;
          c.misalign = align_sum / expect;
          c.box_near = box_sum / expect;
          c.wrong_side = wrong_n / expect;
        }
        const double kn = c.k / k_max;   // [-1, 1]
        c.score = p.clearance_weight * c.clearance - p.align_weight * c.misalign -
          p.line_weight * c.line_hit - p.box_weight * c.box_near -
          p.straight_weight * std::abs(kn) + p.fork_weight * bias.fork_dir * kn;
        if (p.lane_rules) {
          c.score -= p.side_weight * c.wrong_side;
          // 복귀 중: 노란 선이 왼쪽인 자리를 recover_inlane_px 넘게 지나야 노란 선을 넘어 차선에 들어간 것
          if (r.recovering && inlane_n * step < p.recover_inlane_px) {
            c.score -= p.side_weight;
          }
          if (c.crosses_line) {
            c.score -= p.cross_weight;
          }
        }
        if (has_prev_) {
          c.score -= p.smooth_weight * (std::abs(c.k0 - prev_k0_) / k_max +
            (dk_max > 0 ? 0.5 * std::abs(c.dk - prev_dk_) / dk_max : 0.0));
        }
        if (bias.psd_left_close && kn < 0) {
          c.score -= p.psd_weight * -kn;
        }
        if (bias.psd_right_close && kn > 0) {
          c.score -= p.psd_weight * kn;
        }
        if (!c.blocked && c.score > best_score) {
          best_score = c.score;
          r.best = static_cast<int>(r.cands.size());
        }
        r.cands.push_back(std::move(c));
      }
    }

    // 시간 필터 / 버티기
    if (sees_walls && r.best >= 0) {
      const auto & b = r.cands[r.best];
      const double a = has_prev_ ? p.smooth_alpha : 1.0;
      r.k0 = a * b.k0 + (1 - a) * prev_k0_;
      r.dk = a * b.dk + (1 - a) * prev_dk_;
      prev_k0_ = r.k0;
      prev_dk_ = r.dk;
      has_prev_ = true;
      miss_ = 0;
      r.confidence = b.touches_line ? 0.5f : 0.9f;
    } else if (has_prev_ && ++miss_ <= p.hold_frames) {
      r.k0 = prev_k0_;
      r.dk = prev_dk_;
      r.holding = true;
      r.confidence = 0.3f;
    } else {
      has_prev_ = false;
      miss_ = 0;
      return r;   // detected = false
    }
    r.k = meanCurvature(r.k0, r.dk, k_max, p.path_length_px, step);
    r.path = curveFromRobot(r.k0, r.dk, k_max, p.path_length_px, step);

    // 6. 목표점: 경로 위에서 바퀴 축으로부터 lookahead_px 떨어진 첫 점
    cv::Point2d tgt = r.path.back();
    for (const auto & q : r.path) {
      if (q.y > 0 && std::hypot(q.x, q.y) >= p.lookahead_px) {
        tgt = q;
        break;
      }
    }
    r.detected = true;
    r.target = tgt;
    r.offset = static_cast<float>(std::clamp(tgt.x / (p.lane_width_px / 2), -1.0, 1.0));
    const double a = std::atan2(tgt.x, std::max(tgt.y, 1.0));
    const double dist = std::max(std::hypot(tgt.x, tgt.y), 1.0);
    const double kappa = 2 * std::sin(a) / dist;   // 목표점을 지나는 원호 곡률 (pure pursuit)
    r.angle = static_cast<float>(p.follower_lookahead_px > 0 ?
      std::asin(std::clamp(kappa * p.follower_lookahead_px / 2, -1.0, 1.0)) : a);
    return r;
  }

  // 칸 (qx, qy) 에서 진행 방향 t (칸 단위) 에 수직으로 좌우 선을 찾아 차선 가운데 점수 [0, 1]
  //   wrong: 색 순서가 틀린 자리 (lane_rules 일 때만). 1 = 오른쪽 첫 선이 노란색, 2 = 왼쪽 첫 선이 흰색 (둘 다면 3)
  //   yellow_left: 왼쪽 첫 선(차선 폭 안)이 노란색 (lane_rules 일 때만)
  //   fork_dir: 갈림길 표지판 방향 (-1 왼쪽, +1 오른쪽, 0 없음). 차선이 한 차선보다 넓어지면 그쪽 선을 따라간다
  double centerTerm(
    int qx, int qy, const cv::Point2d & t, const Params & p, int fork_dir, int & wrong,
    bool * yellow_left = nullptr) const
  {
    wrong = 0;
    const double cap = std::max(p.clearance_cap_px, 1.0);
    const double tn = std::hypot(t.x, t.y);
    // n: 진행 방향의 왼쪽 (칸 좌표는 u 오른쪽, v 아래)
    const cv::Point2d n = tn > 1e-6 ? cv::Point2d(t.y / tn, -t.x / tn) : cv::Point2d(1, 0);
    const double cell = map_.cell();
    // 한쪽으로 선을 찾는다 (본 적 없는 칸이나 지도 밖에 닿으면 거기서 멈춤)
    //   d1, y1: 첫 선까지 거리 [px] (first_max 안, 없으면 -1), 노란 선인지
    //   d2, y2: 첫 선이 흰 선이고 after_white > 0 이면, 그 너머 after_white 안의 다음 선
    struct Ray
    {
      double d1 = -1, d2 = -1;
      bool y1 = false, y2 = false;
    };
    auto cast = [&](double sx, double first_max, double after_white) {
        Ray r;
        bool in_line = false;
        const int max_steps = cvRound((first_max + std::max(after_white, 0.0)) / cell);
        for (int k = 1; k <= max_steps; k += 2) {
          const int u = cvRound(qx + sx * n.x * k), v = cvRound(qy + sx * n.y * k);
          if (u < 0 || v < 0 || u >= line_.cols || v >= line_.rows ||
            map_.known.at<float>(v, u) < 0.5f)
          {
            break;
          }
          const float l = line_.at<float>(v, u);
          const bool is_line = l >= 0.5f;
          if (is_line && !in_line) {
            const bool yel = yellow_.at<float>(v, u) * 2 >= l;   // 섞인 칸은 많은 쪽 색
            if (r.d1 < 0) {
              if (k * cell > first_max) {
                break;
              }
              r.d1 = k * cell;
              r.y1 = yel;
              if (yel || after_white <= 0) {
                break;
              }
            } else {
              if (k * cell <= r.d1 + after_white) {
                r.d2 = k * cell;
                r.y2 = yel;
              }
              break;
            }
          }
          in_line = is_line;
        }
        return r;
      };
    // 선에서 반 차선 폭(cap) 떨어졌을 때 만점
    auto tent = [cap](double d) {return d <= cap ? d / cap : std::max(0.0, 1 - (d - cap) / cap);};

    if (!p.lane_rules) {
      // 예전 방식: 색 / 차선 폭을 안 보고 max_lane_width_px 안 좌우 가장 가까운 선의 가운데
      const double d1 = cast(1, p.max_lane_width_px, 0).d1, d2 = cast(-1, p.max_lane_width_px, 0).d1;
      if (d1 >= 0 && d2 >= 0) {
        return 1 - std::abs(d1 - d2) / std::max(d1 + d2, 1.0);
      }
      const double d = std::max(d1, d2);
      return d < 0 ? 0.5 : tent(d);   // 양쪽 다 선이 없으면 (넓은 곳) 모든 후보 같은 점수
    }

    // 지금 차선의 선은 차선 폭 안에 있다. 그보다 먼 선은 건너편 / 옆 차선이라 기준으로 안 쓴다
    const double lw = p.lane_width_px, wide = lw * (1 + p.lane_width_tolerance);
    const Ray L = cast(1, lw, wide), R = cast(-1, lw, 0);
    // 색 순서: 오른쪽 첫 선이 노란색 -> 노란 선을 넘어 그 왼쪽에 있음
    //          왼쪽 첫 선이 흰색 -> 흰 선을 넘어 그 오른쪽에 있음 (코너에서 바깥 흰 선을 넘어 직진하는 경로)
    //          strict_colors = false (주차 구간) 이면 그 너머에 노란 선이 있을 때만
    wrong = (R.d1 >= 0 && R.y1 ? 1 : 0) |
      (L.d1 >= 0 && !L.y1 && (p.strict_colors || (L.d2 >= 0 && L.y2)) ? 2 : 0);
    if (yellow_left) {
      *yellow_left = L.d1 >= 0 && L.y1;
    }
    if (wrong) {
      return 0.0;
    }
    if (L.d1 >= 0 && R.d1 >= 0) {
      if (L.d1 + R.d1 <= wide) {
        return 1 - std::abs(L.d1 - R.d1) / std::max(L.d1 + R.d1, 1.0);
      }
      // 한 차선보다 넓음 (교차로 / 합류 / 갈림길): 가운데가 아니라 어느 한쪽 선에서 반 차선 폭.
      // 갈림길 표지판이 있으면 그쪽 선만 따라간다 (반대쪽 갈래로 가는 경로는 가점 없음)
      if (fork_dir < 0) {
        return tent(L.d1);
      }
      if (fork_dir > 0) {
        return tent(R.d1);
      }
      return std::max(tent(L.d1), tent(R.d1));
    }
    if (L.d1 >= 0 || R.d1 >= 0) {
      return tent(std::max(L.d1, R.d1));
    }
    return 0.5;
  }

  // 로봇이 노란 선 왼쪽에 있는지: 로봇 앞 60 / 120 / 180px 중 본 적 있는 곳 모두에서 정면 기준으로
  // 오른쪽 첫 선이 노란색이고 왼쪽 첫 선은 흰색이 아니면 (둘 다면 역주행이라 선을 넘어서 고칠 수 없다) true.
  //   바퀴 축 옆은 안 본다: 제자리 회전 등으로 축 밑 기억에 노란 선 자국이 남기도 한다.
  //   더 멀리는 안 본다: 왼쪽 커브 안쪽에서는 정면 먼 곳이 이미 차선 안이고,
  //   오른쪽 급커브에서는 정면 먼 곳이 바깥 노란 선 너머다
  //   (60 / 120 은 카메라 시야(141~) 아래라 처음 출발할 때는 아직 모른다 -> 건너뜀)
  bool robotLeftOfYellow(const Params & p) const
  {
    int n = 0;
    for (const double y : {60.0, 120.0, 180.0}) {
      const cv::Point2d c = map_.toCell({0, y});
      if (!knownAt(c)) {
        continue;
      }
      int wrong = 0;
      centerTerm(cvRound(c.x), cvRound(c.y), map_.toCell({0, y + map_.cell()}) - c, p, 0, wrong);
      if (wrong != 1) {
        return false;
      }
      ++n;
    }
    return n > 0;
  }

  // 3. 교차로 빈틈 메우기: 로봇 좌우 지금 차선의 선을 앞으로 따라가다 끊긴 뒤, gap_fill_px 안에서
  //    같은 색 선이 같은 방향으로 다시 이어지면 그 사이를 가상 선으로 line_ / yellow_ 에 그린다.
  //    다시 안 이어지면(커브 / T자로 꺾여 나감) 안 그린다 -> 회전은 방해하지 않는다
  //    갈림길 표지판이 가리키는 쪽은 그 틈으로 빠져나가야 하므로 메우지 않는다
  void fillGaps(const Params & p, const Bias & bias, Result & r)
  {
    const double cell = map_.cell();
    for (const int side : {-1, 1}) {   // -1 왼쪽 (노란 선, 주차 구간은 흰 선), +1 오른쪽 (흰 선)
      if (bias.fork_dir == side) {
        continue;
      }
      // 지금 차선의 선 찾기: 로봇 뒤(지도 기억) ~ 앞 여러 높이에서 옆으로 쏴서 차선 폭 안에 처음 걸리는 선
      for (const double y0 : {-150.0, -75.0, 0.0, 75.0, 150.0, 225.0, 300.0}) {
        double in = -1, out = -1;
        for (double x = cell; x <= p.lane_width_px; x += cell) {
          const cv::Point2d c = map_.toCell({side * x, y0});
          const int u = cvRound(c.x), v = cvRound(c.y);
          if (u < 0 || v < 0 || u >= line_.cols || v >= line_.rows ||
            map_.known.at<float>(v, u) < 0.5f)
          {
            break;
          }
          const bool is_line = line_.at<float>(v, u) >= 0.5f;
          if (in < 0 && is_line) {
            in = x;
          } else if (in >= 0 && !is_line) {
            out = x;
            break;
          }
        }
        if (in < 0) {
          continue;
        }
        if (out < 0) {
          out = in + cell;
        }
        const cv::Point2d start = map_.toCell({side * (in + out) / 2, y0});
        const bool yel = isYellow(start);
        if (side > 0 && yel) {
          break;   // 오른쪽 선이 노란색: 로봇이 차선 밖 -> 이쪽은 메우지 않음
        }
        const cv::Point2d fwd = map_.toCell({side * (in + out) / 2, y0 + cell}) - start;
        trackLine(start, fwd, yel, p, r);
        break;
      }
    }
  }

  // 칸 좌표 c 가 지도 안 본 적 있는 칸인지 / 선인지 / 노란 선인지
  bool knownAt(const cv::Point2d & c) const
  {
    const int u = cvRound(c.x), v = cvRound(c.y);
    return u >= 0 && v >= 0 && u < line_.cols && v < line_.rows && map_.known.at<float>(v, u) >= 0.5f;
  }
  bool lineAt(const cv::Point2d & c) const
  {
    return knownAt(c) && line_.at<float>(cvRound(c.y), cvRound(c.x)) >= 0.5f;
  }
  bool isYellow(const cv::Point2d & c) const
  {
    const int u = cvRound(c.x), v = cvRound(c.y);
    return knownAt(c) && yellow_.at<float>(v, u) * 2 >= line_.at<float>(v, u);
  }

  // start (칸 좌표, 선 가운데) 에서 방향 dir 로 선(색 yel)을 따라가며 빈틈을 메운다
  void trackLine(cv::Point2d pos, cv::Point2d d, bool yel, const Params & p, Result & r)
  {
    const double cell = map_.cell();
    const double S = 20.0 / cell;         // 한 걸음 (20px)
    const double win = 24.0 / cell;       // 따라갈 때 좌우로 선을 찾는 폭
    const double gap_win = 40.0 / cell;   // 빈틈 건너편에서 다시 찾을 때 좌우 폭
    const double band = 16.0 / cell;      // 빈틈을 가로지르는 다른 색 선을 보는 폭
    const double max_turn = 0.2;          // 한 걸음에 꺾는 최대 각 [rad] (반지름 100px 커브까지 따라감)
    const double corner_px = 100.0;       // 끊긴 직후 이 거리는 모퉁이(꺾여 나가는 선)라 보지 않음
    const int trim = 3;                   // 끊기기 직전 이 걸음 수(60px)는 모퉁이에 끌렸을 수 있어 버림
    const int dir_steps = 6;              // 그 앞 이 걸음 수(120px)로 늘일 방향을 잡음
    const int confirm = 3;              // 다시 찾은 선이 이 걸음 수만큼 나란히 이어져야 진짜 이어지는 선
    const double max_len_px = 1500.0;     // 따라가는 최대 길이
    d /= std::max(cv::norm(d), 1e-9);

    // q 에서 n 방향으로 [-half, half] 칸을 훑는다: 같은 색 선 칸의 평균 위치, 다른 색 선이 가운데를 가로지르는지
    auto scan = [&](const cv::Point2d & q, const cv::Point2d & n, double half, double & mean,
      bool & other_cross) {
        int cnt = 0;
        double sum = 0;
        bool other_l = false, other_r = false;
        for (double o = -half; o <= half; o += 1.0) {
          const cv::Point2d c = q + n * o;
          if (!lineAt(c)) {
            continue;
          }
          if (isYellow(c) == yel) {
            ++cnt;
            sum += o;
          } else if (std::abs(o) <= band) {
            (o < 0 ? other_l : other_r) = true;
          }
        }
        mean = cnt ? sum / cnt : 0.0;
        other_cross = other_l && other_r;
        return cnt > 0;
      };

    std::vector<cv::Point2d> pts{pos};
    size_t seg = 0;   // 이번 구간(마지막 빈틈 뒤)이 pts 에서 시작하는 위치
    double len = 0;
    while (len < max_len_px) {
      // 따라가기: 한 걸음 앞에서 좌우로 같은 색 선을 찾아 그쪽으로 (최대 max_turn 만큼) 꺾는다
      const cv::Point2d q = pos + d * S;
      if (!knownAt(q)) {
        return;
      }
      const cv::Point2d n(-d.y, d.x);
      double m;
      bool other;
      if (scan(q, n, win, m, other)) {
        const cv::Point2d v = q + n * m - pos;
        const double a = std::clamp(std::atan2(d.x * v.y - d.y * v.x, d.x * v.x + d.y * v.y),
          -max_turn, max_turn);
        d = cv::Point2d(d.x * std::cos(a) - d.y * std::sin(a), d.x * std::sin(a) + d.y * std::cos(a));
        pos += d * S;
        pts.push_back(pos);
        len += S * cell;
        continue;
      }
      // 끊김: 끊기기 직전 trim 걸음은 모퉁이(꺾여 나가는 선)에 끌렸을 수 있어서 버리고,
      // 그 앞 dir_steps 걸음 구간의 방향으로 그 끝에서부터 곧게 늘인다.
      // 실제 선을 그만큼 따라가지 못했으면(가로지르는 선 위에서 시작 등) 방향을 모르니 메우지 않는다
      if (pts.size() < seg + trim + dir_steps + 1) {
        return;
      }
      const size_t a = pts.size() - 1 - trim;
      const cv::Point2d v = pts[a] - pts[a - dir_steps];
      if (cv::norm(v) < 0.5 * dir_steps * S) {
        return;
      }
      const cv::Point2d gd = v / cv::norm(v);
      const double back_px = cv::norm(pos - pts[a]) * cell;
      pos = pts[a];
      len -= back_px;
      const cv::Point2d gn(-gd.y, gd.x);
      int run = 0, first = 0;
      double first_m = 0, last_m = 0;
      for (int j = 1; ; ++j) {
        const double dist = j * S * cell - back_px;   // 끊긴 곳부터 잰 거리
        const cv::Point2d g = pos + gd * (S * j);
        if (!knownAt(g) || (run == 0 && dist > p.gap_fill_px)) {
          return;   // 안 보이는 곳까지 갔거나 너무 김 -> 메우지 않음
        }
        if (dist < corner_px) {
          continue;
        }
        const bool same = scan(g, gn, gap_win, m, other);
        if (!same) {
          if (run > 0 || other) {
            return;   // 잠깐 걸렸다 끊김(가로지르는 선) / 다른 색 선을 가로지름 -> 빈틈이 아님
          }
          continue;
        }
        if (run == 0) {
          first = j;
          first_m = m;
        }
        last_m = m;
        if (++run >= confirm) {
          break;
        }
      }
      // 다시 찾은 선이 늘인 방향과 나란해야 한다 (비스듬히 지나가는 다른 선이 아님)
      if (std::abs(last_m - first_m) > 0.3 * (confirm - 1) * S + 1.0) {
        return;
      }
      const cv::Point2d g1 = pos + gd * (S * first) + gn * first_m;
      const int th = std::max(1, cvRound(10.0 / cell));   // 실제 선 굵기 정도
      cv::line(line_, {cvRound(pos.x), cvRound(pos.y)}, {cvRound(g1.x), cvRound(g1.y)},
        cv::Scalar(1.0), th);
      cv::line(yellow_, {cvRound(pos.x), cvRound(pos.y)}, {cvRound(g1.x), cvRound(g1.y)},
        cv::Scalar(yel ? 1.0 : 0.0), th);
      r.fills.push_back({map_.cellToRobot(pos), map_.cellToRobot(g1), yel});
      len += cv::norm(g1 - pos) * cell;
      pos = g1;
      d = gd;
      seg = pts.size();
      pts.push_back(pos);
    }
  }

  LocalMap map_;
  cv::Mat line_, yellow_;          // 이번 영상의 선 지도 (map_.line / yellow + 교차로 빈틈 가상 선)
  cv::Mat dist_line_, dist_box_;
  bool has_prev_ = false;
  double prev_k0_ = 0, prev_dk_ = 0;
  int miss_ = 0;
};

// GUI / 디버그용 지도 그림 (지도 한 칸 = scale 픽셀, 위 = 로봇 앞)
//   어두운 남색: 본 적 없는 칸, 회색: 바닥, 흰색 / 노란색: 선, 빨강: 박스로 막힌 곳
//   주황 선: 교차로 빈틈을 메운 가상 선
//   회색 가는 선: 막히지 않은 후보, 어두운 빨강: 탈락 후보, 초록 굵은 선: 고른 경로, 분홍 점: 목표점
//   하늘색: 로봇 (바퀴 축 + 몸통 폭), 가는 하늘색 선: 카메라가 보기 시작하는 곳 (BEV 맨 아래)
inline cv::Mat drawMap(const LocalMap & m, const Result & r, const Params & p, int scale = 2,
  bool draw_cands = true)
{
  const double half = p.view_half_width_px, front = p.view_front_px, back = p.view_back_px;
  const cv::Mat line = m.robotView(m.line, half, front, back);
  const cv::Mat yellow = m.robotView(m.yellow, half, front, back);
  const cv::Mat box = m.robotView(m.box, half, front, back);
  const cv::Mat known = m.robotView(m.known, half, front, back);
  cv::Mat img(line.size(), CV_8UC3, cv::Scalar(60, 30, 30));
  img.setTo(cv::Scalar(70, 70, 70), known >= 0.5f);
  img.setTo(cv::Scalar(240, 240, 240), (line >= 0.5f) & (known >= 0.5f));
  img.setTo(cv::Scalar(0, 210, 240), (line >= 0.5f) & (yellow * 2 >= line) & (known >= 0.5f));
  img.setTo(cv::Scalar(0, 0, 170), (box >= 0.5f) & (known >= 0.5f));
  cv::resize(img, img, {}, scale, scale, cv::INTER_NEAREST);
  const double cell = m.cell();
  auto px = [&](const cv::Point2d & q) {
      return cv::Point(cvRound((q.x + half) / cell * scale), cvRound((front - q.y) / cell * scale));
    };
  auto poly = [&](const std::vector<cv::Point2d> & pts, const cv::Scalar & col, int th) {
      std::vector<cv::Point> v;
      for (const auto & q : pts) {
        v.push_back(px(q));
      }
      cv::polylines(img, v, false, col, th, cv::LINE_AA);
    };
  if (draw_cands) {
    for (const auto & c : r.cands) {
      poly(c.pts, c.blocked ? cv::Scalar(0, 0, 110) : cv::Scalar(130, 130, 130), 1);
    }
  }
  for (const auto & f : r.fills) {
    cv::line(img, px(f.a), px(f.b), cv::Scalar(0, 140, 255), std::max(2, scale + 1), cv::LINE_AA);
  }
  if (r.detected) {
    poly(r.path, r.holding ? cv::Scalar(0, 120, 0) : cv::Scalar(0, 255, 0), 2);
    cv::circle(img, px(r.target), 5, cv::Scalar(255, 0, 255), -1);
  }
  const auto o = px({0, 0});
  const int hw = cvRound(p.robot_half_width_px / cell * scale);
  cv::line(img, o - cv::Point(hw, 0), o + cv::Point(hw, 0), cv::Scalar(255, 255, 0), 2);
  cv::line(img, o, px({0, 60}), cv::Scalar(255, 255, 0), 2);
  cv::line(img, px({-half, p.robot_y_offset_px}), px({half, p.robot_y_offset_px}),
    cv::Scalar(200, 200, 0), 1);
  return img;
}

}  // namespace map_planner
