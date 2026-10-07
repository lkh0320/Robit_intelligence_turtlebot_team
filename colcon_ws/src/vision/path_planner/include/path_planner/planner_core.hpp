// 주행 가능 영역 + 곡선 후보 경로 계획 (ROS 없이 OpenCV 만 쓰는 알고리즘 부분)
//
// 처리 순서
//   1. 막힌 곳 지도: LAB 색 마스크로 노란 선 / 흰 선 / 박스(어두운 적갈색)를 찾는다
//      - 박스는 어둡고 a 가 높다 (L ≈ 23, a ≈ 145, b ≈ 138). 빨간 정지선은 더 밝고 더 붉어서 a / b - a 로 빠진다
//      - 박스는 바닥 접점(덩어리의 열마다 가장 아래 픽셀)부터 위쪽 전부를 막힘으로 칠한다
//        (BEV 에서 박스가 위로 늘어나 보이는데, 박스 뒤는 어차피 안 보이는 곳이라 막힘이 맞다)
//      - 선 마스크는 세로로 불려(dilate) 점선의 빈틈을 메운다 (틈으로 경로가 새지 않게)
//      - 거리 변환으로 칸마다 "가장 가까운 선 / 박스까지 거리" 를 구한다
//   2. 경로 기억: 카메라는 바퀴 축보다 robot_y_offset_px 앞부터 보인다 (BEV 맨 아래 = 축에서 약 33cm 앞).
//      그 사이 안 보이는 구간은 지난 프레임에 고른 경로를 기억해 두고, 바퀴 속도(Odom)만큼 옮겨 쓴다.
//   3. 후보 경로: 기억한 경로가 BEV 맨 아래 줄을 지나는 점과 방향(= 로봇이 거기 도착했을 때의 자세)에서
//      곡률이 거리에 따라 일정하게 변하는 곡선 k(t) = k0 + dk t (클로소이드).
//      시작 곡률 k0 n_paths 개 x 변화율 dk n_curv_rates 개. 기억이 없으면 BEV 맨 아래 가운데에서 직진 방향
//      (원호만 쓰면 비스듬히 선 로봇이 "가운데로 돌아왔다가 나란히 펴지는" 경로를 못 만들어 좌우로 흔들린다)
//   4. 점수: 박스에 닿으면 탈락, 선에 가까울수록 벌점, 벽에서 멀수록 가점,
//            직전 경로와 비슷할수록 가점, 갈림길 방향 가점, 가까운 PSD 쪽 벌점
//   5. 기억한 경로 위, 바퀴 축에서 lookahead_px 떨어진 목표점 -> offset / angle (LaneInfo 규약)
//
// 좌표: BEV 이미지 좌표 (x 오른쪽, y 아래), 로봇 좌표 (x 오른쪽, y 앞, 원점 = 바퀴 축 가운데) [px]
// 곡률 k > 0, 방향 phi > 0 = 오른쪽
#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace path_planner
{

struct Params
{
  // 1. 색 (OpenCV LAB: L 0~255, a/b 는 128 이 무채색)
  int yellow_l_min = 80;
  int yellow_b_min = 150;
  int yellow_a_max = 135;      // 이보다 a 가 크면(붉으면) 노란 선이 아니라 갈색 박스로 본다
  int white_l_min = 190;
  int white_ab_dev = 15;
  int box_l_min = 10, box_l_max = 110;     // 박스 (2026-10-08 실측 L 20~47, a 143~147, b 136~140)
  int box_a_min = 137, box_a_max = 165;    // 바닥 a ≈ 127, 빨간 정지선 a ≈ 170
  int box_b_min = 128, box_b_max = 152;
  int box_min_b_minus_a = -15;  // 박스 b-a ≈ -7, 빨간 정지선 b-a ≈ -23
  int box_min_area = 300;      // 이보다 작은 갈색 덩어리는 잡음
  int line_gap_close_px = 31;  // 선 마스크를 세로로 불리는 높이 (점선 빈틈 메우기)
  // 2. 후보 경로
  double bev_center_x = 0.5;   // 로봇 가로 위치 (폭 비율)
  double robot_y_offset_px = 0.0;   // 바퀴 축이 BEV 맨 아래보다 얼마나 더 뒤에 있는지 [px]
  int n_paths = 21;
  double max_curvature = 1.0 / 150.0;   // 가장 급한 후보의 곡률 [1/px] (반지름 150px)
  int n_curv_rates = 1;                 // 1 = 원호만 (실험: 9 면 곡률이 변하는 곡선도)
  double max_curvature_rate = 1.0 / 150.0 / 150.0;   // 곡률 변화율 [1/px^2] (150px 동안 max_curvature 만큼)
  // 안 보이는 구간: n_hidden > 0 이면 매 프레임 바퀴 축에서 다시 계획한다. BEV 맨 아래에서의
  // 가로 위치 n_hidden 개 x 방향 n_hidden 개를 끝점으로 하는 3차 곡선 (바퀴 축에서는 정면 방향).
  // 0 이면 지난 프레임 경로를 기억해 Odom 으로 옮겨 쓴다
  int n_hidden = 0;                     // 기본: 기억 모드 (실험한 3차 곡선 모드는 > 0)
  double max_hidden_offset_px = 200.0;   // BEV 맨 아래에서 가로 위치 범위 [px] (반지름 45cm 커브 ≈ 150)
  double max_hidden_heading = 1.1;       // BEV 맨 아래에서 방향 범위 [rad] (반지름 45cm 커브 ≈ 0.84)
  bool continuous_curvature = true;      // 보이는 구간 시작 곡률을 3차 곡선 끝 곡률에 맞춤 (k0 후보 대신)
  double path_length_px = 300.0;
  double sample_step_px = 5.0;
  // 3. 점수
  double robot_half_width_px = 103.0;  // 경로 중심에서 이 거리 안에 선/박스가 있으면 닿은 것 (10.25cm)
  double line_soft_px = 0.0;           // > 0 이면 선 벌점이 반폭 + 이 거리부터 반폭까지 0 -> 1 로 커진다 (0 = 반폭 안만)
  double clearance_cap_px = 133.0;     // 벽까지 거리가 이 이상이면 똑같이 만점 (보통 차선 폭 / 2)
  double line_weight = 10.0;           // 샘플 평균 선 벌점 x 이 값 (공사 구간에서는 노드가 낮춤)
  double clearance_weight = 1.0;
  double smooth_weight = 0.5;          // 직전 경로와 곡률 차이 벌점
  double straight_weight = 0.1;        // 급커브 벌점 (아무것도 안 보일 때 직진, 평균 곡률 기준)
  double fork_weight = 1.0;            // 갈림길 방향 가점
  double psd_weight = 2.0;             // 가까운 PSD 쪽으로 도는 경로 벌점
  // 4. 출력
  double lane_width_px = 267.0;        // offset 정규화용 (반 차선 폭 = 1)
  double lookahead_px = 120.0;         // 바퀴 축에서 목표점까지 거리
  double smooth_alpha = 0.6;           // 고른 곡률(k0, dk)의 새 값 비중 (1 = 필터 없음)
  int hold_frames = 5;
  int min_wall_pixels = 200;           // 선 + 박스 픽셀이 이보다 적으면 "아무것도 안 보임"
};

// 노드가 매 프레임 넘겨주는 외부 상황
struct Bias
{
  int fork_dir = 0;            // -1 왼쪽, +1 오른쪽, 0 없음 (갈림길 표지판)
  bool psd_left_close = false;
  bool psd_right_close = false;
};

// 직전 프레임 이후 로봇 이동 (바퀴 속도 적분)
struct Odom
{
  double ds_px = 0;    // 앞으로 간 거리 [BEV px]
  double dtheta = 0;   // 회전 [rad], + = 왼쪽 (ROS 규약)
};

struct Maps
{
  cv::Mat yellow, white, box;  // 색 마스크 (0/255)
  cv::Mat box_blocked;         // 박스 바닥 접점부터 위쪽 전부 (0/255)
  cv::Mat line_wall;           // 빈틈을 메운 선 마스크 (0/255)
  cv::Mat dist_line, dist_box; // 가장 가까운 선 / 박스까지 거리 [px] (CV_32F)
  int yellow_pixels = 0, white_pixels = 0, box_pixels = 0;
};

struct Candidate
{
  double he = 0, phe = 0;          // 안 보이는 구간 끝(BEV 맨 아래) 가로 위치 [px], 방향 [rad]
  double k0 = 0, dk = 0;           // 시작 곡률 [1/px], 곡률 변화율 [1/px^2], + 오른쪽
  double k = 0;                    // 평균 곡률 [1/px] (도는 방향)
  std::vector<cv::Point2d> pts;    // 이미지 안에 있는 샘플점 (가까운 쪽부터)
  bool blocked = false;            // 박스에 닿아 탈락
  bool touches_line = false;       // 반폭 안에 선이 들어온 샘플이 있음
  double line_hit = 0;             // 샘플 평균 선 벌점 [0, 1]
  double clearance = 0;            // 벽까지 평균 거리 / clearance_cap [0, 1]
  double score = 0;
};

struct Result
{
  bool detected = false;
  float offset = 0, angle = 0, confidence = 0;
  double k = 0;                    // 필터를 거친 경로의 평균 곡률 (+ 오른쪽)
  double he = 0, phe = 0;          // 필터를 거친 안 보이는 구간 끝 가로 위치 / 방향 (n_hidden > 0 일 때)
  double k0 = 0, dk = 0;           // 필터를 거친 시작 곡률 / 곡률 변화율
  int best = -1;                   // 이번 프레임에 고른 후보 (없으면 -1)
  bool holding = false;            // 이번에 못 골라 직전 값으로 버티는 중
  cv::Point2d robot, target;      // BEV 좌표 (로봇은 BEV 아래 밖에 있을 수 있다)
  cv::Point2d start;              // 후보 경로 시작점 (BEV)
  double start_heading = 0;       // [rad], + 오른쪽
  std::vector<cv::Point2d> memory;   // 기억한 경로 (BEV 좌표, 로봇 쪽부터)
  std::vector<Candidate> cands;
};

// 로봇 좌표에서 (start, 방향 phi) 로 출발해 곡률 k(t) = k0 + dk t (|k| <= k_max) 로 가는 곡선의
// 거리 0, step, 2 step ... length 인 점들. 방향이 출발보다 90도 넘게 돌면 멈춘다
inline std::vector<cv::Point2d> curveFrom(
  const cv::Point2d & start, double phi, double k0, double dk, double k_max, double length,
  double step)
{
  std::vector<cv::Point2d> pts{start};
  cv::Point2d q = start;
  double h = phi;
  for (double t = 0; t + step <= length + 1e-9; t += step) {
    const double k = std::clamp(k0 + dk * (t + step / 2), -k_max, k_max);
    const double dh = k * step;
    q.x += step * std::sin(h + dh / 2);
    q.y += step * std::cos(h + dh / 2);
    h += dh;
    if (std::abs(h - phi) > CV_PI / 2) {
      break;
    }
    pts.push_back(q);
  }
  return pts;
}

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

// with_distances = false 면 거리 변환(dist_line, dist_box)을 건너뛴다 (map_planner 는 지도에서 따로 구함)
inline Maps buildMaps(const cv::Mat & bev, const Params & p, bool with_distances = true)
{
  Maps m;
  cv::Mat lab;
  cv::cvtColor(bev, lab, cv::COLOR_BGR2Lab);
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
  m.box_blocked = cv::Mat::zeros(bev.size(), CV_8U);
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
  }

  // 선: 세로로 불려 점선 빈틈 메우기
  cv::bitwise_or(m.yellow, m.white, m.line_wall);
  if (p.line_gap_close_px > 1) {
    cv::dilate(m.line_wall, m.line_wall,
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, p.line_gap_close_px)));
  }
  m.yellow_pixels = cv::countNonZero(m.yellow);
  m.white_pixels = cv::countNonZero(m.white);

  // 거리 변환: 0 이 아닌 칸에서 가장 가까운 0 칸까지 거리 -> 벽을 0 으로 뒤집어서 넣는다
  // (벽이 하나도 없으면 결과가 아주 큰 값이 되는데, 점수에서 clearance_cap 으로 자르므로 괜찮다)
  if (!with_distances) {
    return m;
  }
  cv::Mat free_line, free_box;
  cv::bitwise_not(m.line_wall, free_line);
  cv::bitwise_not(m.box_blocked, free_box);
  cv::distanceTransform(free_line, m.dist_line, cv::DIST_L2, 3);
  cv::distanceTransform(free_box, m.dist_box, cv::DIST_L2, 3);
  return m;
}

// 매 프레임 지도를 만들어 경로를 고르고, 고른 경로를 기억해 안 보이는 구간을 메운다
class Planner
{
public:
  Result step(
    const cv::Mat & bev, const Params & p, const Bias & bias, const Odom & od = {},
    Maps * maps_out = nullptr)
  {
    Maps m = buildMaps(bev, p);
    Result r = plan(m, p, bias, od);
    if (maps_out) {
      *maps_out = std::move(m);
    }
    return r;
  }

  Result plan(const Maps & m, const Params & p, const Bias & bias, const Odom & od = {})
  {
    Result r;
    const int w = m.dist_line.cols, h = m.dist_line.rows;
    const double cx = p.bev_center_x * w, off = std::max(0.0, p.robot_y_offset_px);
    // 로봇 좌표 (x 오른쪽, y 앞) <-> BEV 좌표
    auto toBev = [&](const cv::Point2d & q) {return cv::Point2d(cx + q.x, (h - 1) - (q.y - off));};
    r.robot = toBev({0, 0});
    const double k_max = std::max(p.max_curvature, 1e-6);
    const double cap = std::max(p.clearance_cap_px, 1.0);
    const double step = std::max(1.0, p.sample_step_px);
    const bool sees_walls =
      m.yellow_pixels + m.white_pixels + m.box_pixels >= p.min_wall_pixels;
    // 안 보이는 구간이 없으면(카메라가 바퀴 축부터 보이면) 끝점 후보는 정면 하나뿐
    const bool from_robot = p.n_hidden > 0;
    const int nh = off < step ? 1 : std::max(1, p.n_hidden);

    // 1. 기억한 경로를 로봇 이동만큼 옮긴다 (직전 곡선의 시작 곡률도 그만큼 진행)
    moveMemory(od);
    if (has_prev_) {
      prev_k0_ = std::clamp(prev_k0_ + prev_dk_ * od.ds_px, -k_max, k_max);
    }

    // 2. 출발 자세 (BEV 맨 아래 줄, y = off) 와 그 앞의 안 보이는 구간(prefix)
    struct Start
    {
      double he, phe;            // 끝 가로 위치, 방향 (바퀴 축에서 계획할 때)
      cv::Point2d s;
      double phi, k_end;         // 출발 방향, 출발 곡률
      std::vector<cv::Point2d> prefix;
    };
    // 바퀴 축 (0, 0, 정면) 에서 (he, off, 방향 phe) 까지 x = a y^2 + b y^3
    auto hiddenStart = [&](double he, double phe) {
        Start st{he, phe, {he, off}, phe, 0, {}};
        const double L = std::max(off, 1.0), tn = std::tan(phe);
        const double a2 = (3 * he - L * tn) / (L * L), b3 = (L * tn - 2 * he) / (L * L * L);
        for (double y = 0; y < off; y += step) {
          st.prefix.emplace_back(a2 * y * y + b3 * y * y * y, y);
        }
        const double d1 = tn, d2 = 2 * a2 + 6 * b3 * L;
        st.k_end = d2 / std::pow(1 + d1 * d1, 1.5);
        return st;
      };
    std::vector<Start> starts;
    if (from_robot) {
      for (int i = 0; i < nh; ++i) {
        for (int j = 0; j < nh; ++j) {
          // 가운데(직진 근처)를 촘촘하게: -1 ~ 1 을 1.5 제곱으로 펼친다
          auto spread = [nh](int k) {
              const double u = nh == 1 ? 0.0 : -1.0 + 2.0 * k / (nh - 1);
              return std::copysign(std::pow(std::abs(u), 1.5), u);
            };
          starts.push_back(hiddenStart(spread(i) * p.max_hidden_offset_px,
            spread(j) * p.max_hidden_heading));
        }
      }
    } else {
      // 기억한 경로가 BEV 맨 아래 줄을 지나는 점과 방향. 기억이 없으면 직진
      Start st{0, 0, {0, off}, 0, 0, {}};
      if (!findCrossing(off, st.s, st.phi, st.prefix)) {
        st = {0, 0, {0, off}, 0, 0, {}};
        for (double t = 0; t < off; t += step) {
          st.prefix.emplace_back(0, t);
        }
      } else {
        st.prefix.pop_back();   // 교차점은 새 곡선의 첫 점으로 들어간다
      }
      // 기억이 잘못 쌓여 이상한 곳에서 출발하지 않도록 제한
      st.s.x = std::clamp(st.s.x, -0.5 * w + 10, 0.5 * w - 10);
      st.phi = std::clamp(st.phi, -1.0, 1.0);
      starts.push_back(std::move(st));
    }

    // 3~4. 후보마다 샘플점을 찍고 점수 매기기 (출발 자세마다 가장 좋은 후보 하나만 r.cands 에 남긴다)
    const int n = std::max(1, p.n_paths), nr = std::max(1, p.n_curv_rates);
    const double dk_max = std::max(p.max_curvature_rate, 0.0);
    const double soft = std::max(p.line_soft_px, 0.0);
    const bool cont = from_robot && p.continuous_curvature && off >= step;
    double best_score = -1e18;
    Start best_st{};
    auto evalStart = [&](const Start & st) {
        Candidate keep;
        keep.score = -1e18;
        bool have = false;
        for (int i = 0; i < (cont ? 1 : n); ++i) {
          for (int j = 0; j < nr; ++j) {
            Candidate c;
            c.he = st.he;
            c.phe = st.phe;
            c.k0 = cont ? std::clamp(st.k_end, -k_max, k_max) :
              n == 1 ? 0.0 : -k_max + 2 * k_max * i / (n - 1);
            c.dk = nr == 1 ? 0.0 : -dk_max + 2 * dk_max * j / (nr - 1);
            c.k = meanCurvature(c.k0, c.dk, k_max, p.path_length_px, step);
            double clear_sum = 0, line_sum = 0;
            const auto curve = curveFrom(st.s, st.phi, c.k0, c.dk, k_max, p.path_length_px, step);
            for (size_t u = 1; u < curve.size(); ++u) {
              const cv::Point2d q = toBev(curve[u]);
              const int qx = cvRound(q.x), qy = cvRound(q.y);
              if (qy >= h) {
                continue;   // BEV 아래 (안 보이는 곳)
              }
              if (qx < 0 || qx >= w || qy < 0) {
                break;
              }
              c.pts.push_back(q);
              const float dl = m.dist_line.at<float>(qy, qx), db = m.dist_box.at<float>(qy, qx);
              if (db < p.robot_half_width_px) {
                c.blocked = true;
              }
              if (dl < p.robot_half_width_px) {
                c.touches_line = true;
                line_sum += 1;
              } else if (soft > 0) {
                line_sum += std::max(0.0, 1 - (dl - p.robot_half_width_px) / soft);
              }
              clear_sum += std::min<double>(std::min(dl, db), cap) / cap;
            }
            if (c.pts.empty()) {
              c.blocked = true;
            } else {
              const double ns = static_cast<double>(c.pts.size());
              c.line_hit = line_sum / ns;
              c.clearance = clear_sum / ns;
            }
            const double kn = c.k / k_max;   // [-1, 1]
            c.score = p.clearance_weight * c.clearance - p.line_weight * c.line_hit -
              p.straight_weight * std::abs(kn) + p.fork_weight * bias.fork_dir * kn;
            if (has_prev_) {
              c.score -= p.smooth_weight * (std::abs(c.k0 - prev_k0_) / k_max +
                (dk_max > 0 ? 0.5 * std::abs(c.dk - prev_dk_) / dk_max : 0.0) +
                (from_robot ? std::abs(c.he - prev_he_) / std::max(p.max_hidden_offset_px, 1.0) +
                std::abs(c.phe - prev_phe_) / std::max(p.max_hidden_heading, 1e-3) : 0.0));
            }
            if (bias.psd_left_close && kn < 0) {
              c.score -= p.psd_weight * -kn;
            }
            if (bias.psd_right_close && kn > 0) {
              c.score -= p.psd_weight * kn;
            }
            // 막힌 후보는 막히지 않은 후보가 하나도 없을 때만 화면용으로 남긴다
            if (!have || (keep.blocked && !c.blocked) ||
              (keep.blocked == c.blocked && c.score > keep.score))
            {
              keep = std::move(c);
              have = true;
            }
          }
        }
        if (!keep.blocked && keep.score > best_score) {
          best_score = keep.score;
          r.best = static_cast<int>(r.cands.size());
          best_st = st;
        }
        r.cands.push_back(std::move(keep));
      };
    for (const auto & st : starts) {
      evalStart(st);
    }
    // 바퀴 축에서 계획할 때: 격자 사이 값이 필요하므로 가장 좋은 끝점 주변을 두 번 더 촘촘하게 찾는다
    if (from_robot && r.best >= 0 && nh > 1) {
      double dh = p.max_hidden_offset_px / (nh - 1);
      double dp = p.max_hidden_heading / (nh - 1);
      for (int it = 0; it < 2; ++it, dh /= 2, dp /= 2) {
        const double he0 = best_st.he, phe0 = best_st.phe;
        for (int i = -1; i <= 1; ++i) {
          for (int j = -1; j <= 1; ++j) {
            if (i != 0 || j != 0) {
              evalStart(hiddenStart(he0 + i * dh, phe0 + j * dp));
            }
          }
        }
      }
    }

    // 시간 필터 / 버티기
    if (sees_walls && r.best >= 0) {
      const auto & b = r.cands[r.best];
      const double a = has_prev_ ? p.smooth_alpha : 1.0;
      r.he = a * b.he + (1 - a) * prev_he_;
      r.phe = a * b.phe + (1 - a) * prev_phe_;
      r.k0 = a * b.k0 + (1 - a) * prev_k0_;
      r.dk = a * b.dk + (1 - a) * prev_dk_;
      prev_he_ = r.he;
      prev_phe_ = r.phe;
      prev_k0_ = r.k0;
      prev_dk_ = r.dk;
      has_prev_ = true;
      miss_ = 0;
      const bool both = m.yellow_pixels > 0 && m.white_pixels > 0;
      r.confidence = b.touches_line ? 0.5f : (both ? 0.9f : 0.7f);
      // 새 기억 = 안 보이는 구간 + 이번에 고른 곡선 (필터를 거친 값으로 다시 그린다)
      Start st = std::move(best_st);
      if (from_robot) {
        st = hiddenStart(r.he, r.phe);
        if (p.continuous_curvature) {
          r.k0 = std::clamp(st.k_end, -k_max, k_max);
          prev_k0_ = r.k0;
        }
      }
      r.start = toBev(st.s);
      r.start_heading = st.phi;
      mem_ = std::move(st.prefix);
      const auto curve = curveFrom(st.s, st.phi, r.k0, r.dk, k_max, p.path_length_px, step);
      mem_.insert(mem_.end(), curve.begin(), curve.end());
    } else if (has_prev_ && !mem_.empty() && ++miss_ <= p.hold_frames) {
      r.he = prev_he_;
      r.phe = prev_phe_;
      r.k0 = prev_k0_;
      r.dk = prev_dk_;
      r.holding = true;
      r.confidence = 0.3f;
    } else {
      reset();
      return r;   // detected = false
    }
    r.k = meanCurvature(r.k0, r.dk, k_max, p.path_length_px, step);

    // 5. 목표점: 기억한 경로 위에서 바퀴 축으로부터 lookahead_px 떨어진 첫 점
    cv::Point2d tgt = mem_.back();
    for (const auto & q : mem_) {
      if (q.y > 0 && std::hypot(q.x, q.y) >= p.lookahead_px) {
        tgt = q;
        break;
      }
    }
    r.detected = true;
    r.target = toBev(tgt);
    r.offset = static_cast<float>(std::clamp(tgt.x / (p.lane_width_px / 2), -1.0, 1.0));
    r.angle = static_cast<float>(std::atan2(tgt.x, std::max(tgt.y, 1.0)));
    for (const auto & q : mem_) {
      r.memory.push_back(toBev(q));
    }
    return r;
  }

  void reset() {has_prev_ = false; miss_ = 0; mem_.clear();}

private:
  // 로봇이 ds 앞으로 가며 dtheta(왼쪽 +) 돌았을 때 기억한 점들을 새 로봇 좌표로
  void moveMemory(const Odom & od)
  {
    if (mem_.empty() || (od.ds_px == 0 && od.dtheta == 0)) {
      return;
    }
    // 원호로 움직였다고 보고 중간 방향으로 이동, 그 뒤 회전
    const double half = od.dtheta / 2;
    const double tx = -od.ds_px * std::sin(half), ty = od.ds_px * std::cos(half);
    const double c = std::cos(od.dtheta), sn = std::sin(od.dtheta);
    std::vector<cv::Point2d> moved;
    for (const auto & q : mem_) {
      const double x = q.x - tx, y = q.y - ty;
      const cv::Point2d n(x * c + y * sn, -x * sn + y * c);
      if (n.y > -50) {   // 로봇 뒤로 많이 지나간 점은 버린다
        moved.push_back(n);
      }
    }
    mem_ = std::move(moved);
  }

  // 기억한 경로가 y = line 을 처음 넘는 점과 그 방향. prefix 에는 그 앞 점들 + 교차점
  bool findCrossing(double line, cv::Point2d & s, double & phi, std::vector<cv::Point2d> & prefix)
  {
    for (size_t i = 1; i < mem_.size(); ++i) {
      const auto & a = mem_[i - 1];
      const auto & b = mem_[i];
      prefix.push_back(a);
      if (a.y < line && b.y >= line) {
        const double u = (line - a.y) / (b.y - a.y);
        s = a + u * (b - a);
        phi = std::atan2(b.x - a.x, b.y - a.y);
        prefix.push_back(s);
        return true;
      }
    }
    return false;
  }

  bool has_prev_ = false;
  double prev_he_ = 0, prev_phe_ = 0, prev_k0_ = 0, prev_dk_ = 0;
  int miss_ = 0;
  std::vector<cv::Point2d> mem_;   // 기억한 경로 (로봇 좌표, 로봇 쪽부터)
};

}  // namespace path_planner
