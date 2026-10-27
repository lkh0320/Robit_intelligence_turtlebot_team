// 주행용 선 검출 (Bird's Eye View + 슬라이딩 윈도우 + 2차 곡선)
//   구독: image_bev (sensor_msgs/Image, bird_eye_view 노드)
//   발행: lane_info (interfaces/LaneInfo)
//         cross_line (geometry_msgs/Vector3Stamped, 교차로 T자 가로 선: 로봇 앞을 가로지르는 긴 흰/노란 직선)
//           x = 세로 위치 (BEV 높이 비율, 1 = 맨 아래 = 로봇 쪽), y = 기울기 [rad] (+ = 오른쪽 끝이 더 가까움
//           = 로봇이 왼쪽으로 틀어져 있음 -> 오른쪽으로 돌면 수평), z = 길이 / BEV 폭 (0 = 못 찾음)
//           (interfaces 에 기울기 칸이 없어서 표준 메시지를 씀. 메시지를 바꾸면 turtle_gui 까지 다시 빌드해야 함)
//         vision/lane_debug/compressed (GUI 선·벡터 검출 화면, 구독자가 있을 때만)
//   TODO: stop_line (interfaces/StopLine, 빨간 정지선)
//
// 처리 순서
//   1. LAB 색공간으로 노란 선(b 채널 높음) / 흰 선(밝고 b 중립) 마스크
//      코스 규칙: 노란 선 = 왼쪽 차선, 흰 선 = 오른쪽 차선 (위치가 아니라 색으로 좌우 구분)
//   2. 차선 시작점: 화면 아래 절반의 열 히스토그램 최고점
//      흰 선은 노란 선에서 차선 폭만큼 오른쪽 근처에서만 찾는다 (옆 차선의 흰 선 제외)
//   3. 슬라이딩 윈도우로 아래에서 위로 차선 픽셀을 따라간다
//      직전 프레임 곡선이 있으면 그 곡선 근처만 탐색
//   4. 차선 픽셀에 2차 곡선 x = a*y^2 + b*y + c 를 맞춘다
//   5. 검사: 두 차선 폭이 lane_width_px 와 크게 다르면 픽셀이 적은 쪽을 버린다
//      한쪽만 있으면 차선 폭의 절반만큼 옆으로 옮겨 차선 중심을 추정한다 (single_line_normal_shift: 곡선의 수직 방향으로)
//   6. 곡선 계수를 직전 값과 섞어(smooth_alpha) 흔들림을 줄이고, 놓치면 hold_frames 동안 유지
//   7. 로봇 위치(BEV 아래, bev_center_x)에서 차선 중심까지 offset, 중심 곡선의 방향 angle
//      선 픽셀이 있는 가장 아래 줄보다 아래는 곡선 대신 접선으로 늘린다 (linear_below_pixels)
//   +. 가로 선: 노란 | 흰 마스크에서 HoughLinesP 로 cross_min_length_px 넘는 직선 중 로봇 진행 방향과 거의
//      직각(가로에서 cross_max_angle_deg 안)인 것. lane_follower 가 교차로 회전을 시작하는 조건으로 쓴다
//      (test_052007: 진입 곡선에서 이미 돈 뒤 차선을 잠깐 놓친 것을 교차로로 보고 또 돌던 문제)
//      녹화 재생: T자 앞에서 차선 놓친 7번 모두 놓치기 0.3초 전 ~ 직후에 검출, 진입 곡선 쪽은 검출 없음
//
// offset: (차선 중심 - 로봇) / (차선 폭 / 2)  0 = 가운데, +1 = 차선 중심이 로봇보다 반 차선 오른쪽
// angle : 로봇 위치에서 차선 중심 곡선이 향하는 방향 [rad], + 는 오른쪽
//         (BEV 가로/세로 축척이 같다고 가정한 값. 실측 축척을 넣기 전까지는 제어 게인으로 보정)
#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "cv_bridge/cv_bridge.hpp"
#include "interfaces/msg/lane_info.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace
{
// x = a*y^2 + b*y + c (BEV 이미지 좌표, y 는 아래로 증가)
// 차선은 대체로 세로로 길게 뻗으므로 y 를 넣으면 x 가 나오는 형태로 둔다.
// (x 로 y 를 구하는 형태면 거의 수직인 차선에서 기울기가 무한대가 되어 맞출 수 없다)
//   a: 휘어진 정도 (0 이면 직선), b: 기울기, c: y = 0(화면 맨 위)에서의 x
struct Poly
{
  double a = 0, b = 0, c = 0;
  double x(double y) const {return (a * y + b) * y + c;}   // 높이 y 에서 곡선의 가로 위치
  double dxdy(double y) const {return 2 * a * y + b;}       // 높이 y 에서 곡선의 기울기
  Poly shifted(double dx) const {return {a, b, c + dx};}    // 같은 모양으로 옆으로 dx 만큼 이동
  Poly blend(const Poly & o, double alpha) const   // alpha*this + (1-alpha)*o
  {
    return {alpha * a + (1 - alpha) * o.a, alpha * b + (1 - alpha) * o.b,
      alpha * c + (1 - alpha) * o.c};
  }
};

// 최소제곱 2차 곡선 맞춤 (점이 min_points 보다 적으면 nullopt)
// 픽셀의 세로 범위가 min_span 보다 짧으면 휘어짐 추정이 발산하므로 직선으로 맞춘다
std::optional<Poly> fitPoly(const std::vector<cv::Point> & pts, int min_points, int min_span)
{
  if (static_cast<int>(pts.size()) < min_points) {
    return std::nullopt;
  }
  // 점마다 한 줄씩: [y^2, y, 1] * [a, b, c]^T = x
  // 점이 3개보다 훨씬 많으므로 정확한 해는 없고, 오차 제곱합이 가장 작은 a, b, c 를 구한다
  int y_min = pts[0].y, y_max = pts[0].y;
  cv::Mat A(static_cast<int>(pts.size()), 3, CV_64F), X(static_cast<int>(pts.size()), 1, CV_64F);
  for (int i = 0; i < A.rows; ++i) {
    const double y = pts[i].y;
    A.at<double>(i, 0) = y * y;
    A.at<double>(i, 1) = y;
    A.at<double>(i, 2) = 1.0;
    X.at<double>(i, 0) = pts[i].x;
    y_min = std::min(y_min, pts[i].y);
    y_max = std::max(y_max, pts[i].y);
  }
  cv::Mat coef;
  // 픽셀이 세로로 충분히 길게 퍼져 있으면 2차 곡선
  if (y_max - y_min >= min_span) {
    if (!cv::solve(A, X, coef, cv::DECOMP_QR)) {
      return std::nullopt;
    }
    return Poly{coef.at<double>(0), coef.at<double>(1), coef.at<double>(2)};
  }
  // 짧으면 y^2 열을 빼고 직선 x = b*y + c 로 맞춘다 (a = 0)
  if (!cv::solve(A.colRange(1, 3), X, coef, cv::DECOMP_QR)) {
    return std::nullopt;
  }
  return Poly{0.0, coef.at<double>(0), coef.at<double>(1)};
}

// 곡선을 진행 방향의 수직(법선)으로 d 만큼 옮긴 곡선 (d > 0 = 진행 방향 왼쪽, 위로 갈 때 기준)
// 가로로만 옮기면(shifted) 곡선이 많이 누운 곳에서 실제 간격이 d*cos(기울기) 로 줄어든다.
//   2026-10-26 022124_uturn: U자 직전 흰 선만 보일 때 중심을 흰 선에 너무 붙여 offset +1.0 -> 오른쪽으로 틀다 놓침
std::optional<Poly> normalShifted(const Poly & p, double d, int h)
{
  std::vector<cv::Point> pts;
  for (int y = 0; y < h; y += 4) {
    const double k = p.dxdy(y);
    const double n = std::sqrt(1 + k * k);
    // 위로 가는 진행 방향 (-k, -1) / n 의 왼쪽 = (-1, k) / n
    pts.emplace_back(static_cast<int>(std::lround(p.x(y) - d / n)),
      static_cast<int>(std::lround(y + d * k / n)));
  }
  return fitPoly(pts, 3, h / 2);
}

// mask 의 [x0, x1) 열 중 아래 절반에서 픽셀이 가장 많은 열 (min_count 미만이면 -1)
int histogramPeak(const cv::Mat & mask, int x0, int x1, int min_count)
{
  x0 = std::clamp(x0, 0, mask.cols);
  x1 = std::clamp(x1, 0, mask.cols);
  if (x1 - x0 < 1) {
    return -1;
  }
  // 아래 절반만 보는 이유: 로봇에 가까운 쪽이라 차선이 거의 세로로 서 있어서 한 열에 픽셀이 몰린다
  // reduce(..., 0, SUM): 세로 방향으로 다 더해 열마다 합 하나 -> 1 x (x1-x0) 히스토그램
  cv::Mat hist;
  cv::reduce(mask(cv::Rect(x0, mask.rows / 2, x1 - x0, mask.rows - mask.rows / 2)), hist, 0,
    cv::REDUCE_SUM, CV_32S);
  double max_val;
  cv::Point loc;
  cv::minMaxLoc(hist, nullptr, &max_val, nullptr, &loc);
  // 마스크 픽셀 값은 255 라서 합 / 255 = 픽셀 개수
  return max_val / 255 >= min_count ? x0 + loc.x : -1;
}
}  // namespace

class LaneDetectionNode : public rclcpp::Node
{
public:
  LaneDetectionNode()
  : Node("lane_detection")
  {
    const auto image_topic = declare_parameter("image_topic", std::string("image_bev"));
    // 1. 색 (OpenCV LAB: L 0~255, a/b 는 128 이 무채색)
    declare_parameter("yellow_b_min", 150);        // 노란 선 b ≈ 188
    declare_parameter("yellow_l_min", 80);
    declare_parameter("white_l_min", 190);         // 흰 선 L ≈ 230~250, 바닥 ≈ 85
    declare_parameter("white_ab_dev", 15);         // 흰 선은 a, b 가 128 ± 이 값 안
    // 노란 선 시작점을 화면 왼쪽 이 비율 안에서만 찾는다 (1.0 = 화면 전체).
    // 양쪽이 다 노란선인 주차 구역에서 오른쪽 노란선을 왼쪽 선으로 잘못 잡지 않게 한다.
    declare_parameter("yellow_search_max_ratio", 1.0);
    // 2~5. 차선 탐색
    declare_parameter("lane_width_px", 295.0);     // BEV 에서 두 차선 중심 사이 폭 (bird_eye_view 보정값)
    declare_parameter("bev_center_x", 0.5);        // BEV 에서 로봇(카메라 중심)의 가로 위치 (폭 비율)
    // offset / angle 을 재는 높이: BEV 맨 아래에서 이만큼 더 아래(로봇 쪽)로 차선 곡선을 늘려서 잰다 [px]
    // 재는 곳이 멀수록 그 점을 차선 가운데 두려고 일찍 꺾어 코너 안쪽을 파고든다 (반지름 R 커브에서 약 d^2 / 2R)
    declare_parameter("eval_below_px", 0.0);
    // 이번 프레임 선 픽셀의 가장 아래 줄보다 아래는 2차 곡선 대신 그 줄의 접선(직선)으로 늘려서 잰다
    declare_parameter("linear_below_pixels", true);
    declare_parameter("base_min_pixels", 30);      // 시작점 열에 필요한 최소 픽셀 수
    declare_parameter("n_windows", 10);
    declare_parameter("window_margin", 30);        // 윈도우 반폭 / 직전 곡선 주변 탐색 폭 [px]
    declare_parameter("window_min_pixels", 20);    // 이보다 많으면 다음 윈도우 중심을 옮긴다
    declare_parameter("min_lane_pixels", 150);     // 곡선 맞춤에 필요한 최소 픽셀 수
    declare_parameter("min_curve_span", 0.4);      // 픽셀 세로 범위가 이 비율보다 짧으면 직선으로 맞춤
    declare_parameter("width_tolerance", 0.35);    // 차선 폭이 lane_width_px 에서 이 비율 넘게 다르면 이상
    declare_parameter("single_line_normal_shift", false);   // 한쪽 선만 있을 때 곡선의 수직 방향으로 옮겨 중심 추정
    // 6. 시간 필터
    declare_parameter("smooth_alpha", 0.5);        // 새 곡선 비중 (1 = 필터 없음)
    declare_parameter("hold_frames", 5);
    // +. 교차로 가로 선
    declare_parameter("cross_enabled", true);
    declare_parameter("cross_min_length_px", 250);    // 이보다 긴 직선만 (곡선 차선 / 섬 끝 U 는 짧다)
    declare_parameter("cross_max_angle_deg", 15.0);   // 가로에서 이 각도 안 (진입 곡선에서 비스듬히 보면 제외)
    declare_parameter("cross_hough_threshold", 80);   // HoughLinesP 투표 수
    declare_parameter("cross_max_gap_px", 20);        // 흰 / 노란 이음매 같은 빈틈 허용
    // 디버그: final (최종) / mask (색 마스크 + 윈도우)
    declare_parameter("debug_view", std::string("final"));
    declare_parameter("debug_jpeg_quality", 70);

    const auto qos = rclcpp::SensorDataQoS();
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, qos, [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {onImage(msg);});
    lane_pub_ = create_publisher<interfaces::msg::LaneInfo>("lane_info", qos);
    cross_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("cross_line", qos);
    debug_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      "vision/lane_debug/compressed", qos);

    RCLCPP_INFO(get_logger(), "lane_detection 시작 (구독: %s)", image_topic.c_str());
  }

private:
  // 차선 하나의 추적 상태
  // 왼쪽(노란 선) / 오른쪽(흰 선) 각각 하나씩 있고, 프레임이 바뀌어도 계속 유지된다
  struct Track
  {
    std::optional<Poly> fit;   // 필터를 거친 곡선 (없으면 지금 이 차선을 모르는 상태)
    int miss = 0;              // 연속으로 놓친 프레임 수
    // 이번 프레임 결과 (디버그용)
    std::vector<cv::Point> pixels;
    std::vector<cv::Rect> windows;
  };

  int intParam(const std::string & name) {return static_cast<int>(get_parameter(name).as_int());}

  // +. 교차로 가로 선: 가장 긴 "거의 가로" 직선. 없으면 길이 0
  struct Cross
  {
    cv::Vec4i seg{0, 0, 0, 0};
    double length = 0, angle_deg = 0;
  };
  Cross detectCross(const cv::Mat & lines)
  {
    Cross best;
    if (!get_parameter("cross_enabled").as_bool()) {
      return best;
    }
    std::vector<cv::Vec4i> segs;
    cv::HoughLinesP(lines, segs, 2, CV_PI / 90, intParam("cross_hough_threshold"),
      intParam("cross_min_length_px"), intParam("cross_max_gap_px"));
    const double max_angle = dblParam("cross_max_angle_deg");
    for (const auto & l : segs) {
      double angle = std::atan2(l[3] - l[1], l[2] - l[0]) * 180.0 / CV_PI;   // 가로 = 0
      angle = std::fmod(angle + 270.0, 180.0) - 90.0;                         // -90 ~ 90
      const double length = std::hypot(l[2] - l[0], l[3] - l[1]);
      if (std::abs(angle) <= max_angle && length > best.length) {
        best = {l, length, angle};
      }
    }
    if (best.length <= 0) {
      return best;
    }
    // 각도 다듬기: HoughLinesP 는 2° 단위라 정렬(lane_follower ALIGN)에 거칠다.
    // 찾은 직선에서 12px 안의 선 픽셀로 직선을 다시 맞춘다
    const cv::Point2d a(best.seg[0], best.seg[1]), b(best.seg[2], best.seg[3]);
    const cv::Point2d d = (b - a) / best.length;
    std::vector<cv::Point> nz, near;
    cv::findNonZero(lines, nz);
    for (const auto & q : nz) {
      const cv::Point2d v = cv::Point2d(q) - a;
      const double along = v.dot(d), across = v.x * d.y - v.y * d.x;
      if (std::abs(across) <= 12 && along >= 0 && along <= best.length) {
        near.push_back(q);
      }
    }
    if (near.size() >= 50) {
      cv::Vec4f fit;
      cv::fitLine(near, fit, cv::DIST_L2, 0, 0.01, 0.01);
      double angle = std::atan2(fit[1], fit[0]) * 180.0 / CV_PI;
      best.angle_deg = std::fmod(angle + 270.0, 180.0) - 90.0;
    }
    return best;
  }
  double dblParam(const std::string & name) {return get_parameter(name).as_double();}

  // 3. 차선 픽셀 모으기: 직전 곡선이 있으면 그 주변, 없으면 base_x 에서 슬라이딩 윈도우
  void collect(const std::vector<cv::Point> & nonzero, const cv::Mat & mask, int base_x,
    Track & t)
  {
    t.pixels.clear();
    t.windows.clear();
    const int margin = intParam("window_margin");
    // (가) 직전 프레임에서 이 차선을 찾았으면: 그 곡선 좌우 margin 안의 픽셀만 모은다
    //     차선은 프레임 사이에 조금밖에 안 움직이므로 빠르고, 멀리 있는 잡음도 무시된다
    if (t.fit && t.miss == 0) {
      for (const auto & p : nonzero) {
        if (std::abs(p.x - t.fit->x(p.y)) <= margin) {
          t.pixels.push_back(p);
        }
      }
      return;
    }
    // (나) 아니면: 시작점 base_x 에서 슬라이딩 윈도우로 새로 찾는다
    if (base_x < 0) {
      return;   // 시작점도 없으면 이번 프레임은 이 차선 없음
    }
    // 화면을 아래에서 위로 n 칸으로 나누고, 칸마다 x 를 중심으로 폭 2*margin 인 상자(윈도우)를 놓는다.
    // 상자 안 픽셀을 차선으로 모으고, 충분히 많으면 그 평균 x 로 다음(위) 상자 중심을 옮긴다
    // -> 차선이 휘어도 상자가 따라간다
    const int n = std::max(1, intParam("n_windows"));
    const int win_h = mask.rows / n;
    const int min_pix = intParam("window_min_pixels");
    int x = base_x;
    for (int i = 0; i < n; ++i) {
      const int y1 = mask.rows - i * win_h, y0 = std::max(0, y1 - win_h);
      // & 연산: 화면 밖으로 나간 부분을 잘라낸다
      const cv::Rect win = cv::Rect(x - margin, y0, 2 * margin, y1 - y0) &
        cv::Rect(0, 0, mask.cols, mask.rows);
      if (win.area() == 0) {
        break;
      }
      t.windows.push_back(win);
      std::vector<cv::Point> found;
      cv::findNonZero(mask(win), found);
      long sum_x = 0;
      for (auto & p : found) {
        p += win.tl();   // findNonZero 는 상자 기준 좌표를 주므로 전체 영상 좌표로 바꾼다
        sum_x += p.x;
        t.pixels.push_back(p);
      }
      if (static_cast<int>(found.size()) >= min_pix) {
        x = static_cast<int>(sum_x / static_cast<long>(found.size()));
      }
    }
  }

  // 6. 시간 필터: 새 곡선을 직전 값과 섞고, 놓치면 hold_frames 동안 유지
  void update(Track & t, const std::optional<Poly> & found)
  {
    if (found) {
      // 처음 찾았으면 그대로, 이미 있으면 새 곡선 alpha + 이전 곡선 (1 - alpha) 로 섞는다
      t.fit = t.fit ? found->blend(*t.fit, dblParam("smooth_alpha")) : *found;
      t.miss = 0;
    } else if (t.fit && ++t.miss > intParam("hold_frames")) {
      // 잠깐 가려진 건 이전 곡선으로 버티고, hold_frames 넘게 계속 못 찾으면 포기
      t.fit.reset();
    }
  }

  // BEV 프레임 하나가 들어올 때마다 위 1~7 단계를 수행
  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    cv::Mat bev;
    try {
      bev = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "cv_bridge: %s", e.what());
      return;
    }
    const int w = bev.cols, h = bev.rows;
    const double lane_w = dblParam("lane_width_px");
    const double robot_x = dblParam("bev_center_x") * w;   // 로봇 위치 [px]

    // 1. 색 마스크
    //   LAB: L = 밝기, a = 초록(-)~빨강(+), b = 파랑(-)~노랑(+). 128 이 중간(무채색)
    //   BGR 보다 조명이 바뀌어도 "노란 정도"(b), "밝기"(L) 가 따로 움직여서 임계값 잡기가 쉽다
    //   inRange 결과는 조건을 만족하면 255, 아니면 0 인 흑백 마스크
    cv::Mat lab, yellow, white;
    cv::cvtColor(bev, lab, cv::COLOR_BGR2Lab);
    const int dev = intParam("white_ab_dev");
    // 노란 선: 너무 어둡지 않고(L) b 가 큰(노란) 픽셀
    cv::inRange(lab, cv::Scalar(intParam("yellow_l_min"), 0, intParam("yellow_b_min")),
      cv::Scalar(255, 255, 255), yellow);
    // 흰 선: 아주 밝고(L) 색이 거의 없는(a, b 가 128 근처) 픽셀
    cv::inRange(lab, cv::Scalar(intParam("white_l_min"), 128 - dev, 128 - dev),
      cv::Scalar(255, 128 + dev, 128 + dev), white);
    // 열기(open) = 깎았다가 다시 불리기: 3x3 보다 작은 점 잡음을 지운다
    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(yellow, yellow, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(white, white, cv::MORPH_OPEN, kernel);

    // +. 교차로 가로 선 (T자는 흰 선과 노란 선이 이어져 있어 두 마스크를 합쳐서 본다)
    cross_ = detectCross(yellow | white);
    geometry_msgs::msg::Vector3Stamped cross;
    cross.header = msg->header;
    if (cross_.length > 0) {
      cross.vector.x = (cross_.seg[1] + cross_.seg[3]) / 2.0 / h;   // 1 = 로봇 바로 앞
      cross.vector.y = cross_.angle_deg * CV_PI / 180.0;
      cross.vector.z = std::max(1e-3, cross_.length / w);
    }
    cross_pub_->publish(cross);

    // 2. 시작점 (직전 곡선이 없을 때만 쓰인다)
    const int base_min = intParam("base_min_pixels");
    // 노란 선은 화면 전체에서, 흰 선은 노란 선 오른쪽 0.5~1.5 차선 폭 범위에서 찾는다.
    // 노란 선이 안 보이면 흰 선은 로봇 위치보다 오른쪽에서만 찾는다
    const int yellow_x1 = static_cast<int>(dblParam("yellow_search_max_ratio") * w);
    const int left_base = histogramPeak(yellow, 0, yellow_x1, base_min);
    const int right_base = left_base >= 0 ?
      histogramPeak(white, left_base + static_cast<int>(0.5 * lane_w),
      left_base + static_cast<int>(1.5 * lane_w), base_min) :
      histogramPeak(white, static_cast<int>(robot_x), w, base_min);

    // 3~4. 픽셀 모으기 + 곡선 맞춤
    std::vector<cv::Point> yellow_px, white_px;
    cv::findNonZero(yellow, yellow_px);
    cv::findNonZero(white, white_px);
    collect(yellow_px, yellow, left_base, left_);
    collect(white_px, white, right_base, right_);
    const int min_pix = intParam("min_lane_pixels");
    const int min_span = static_cast<int>(dblParam("min_curve_span") * h);   // 비율 -> px
    auto left_fit = fitPoly(left_.pixels, min_pix, min_span);
    auto right_fit = fitPoly(right_.pixels, min_pix, min_span);

    // 5. 두 차선 폭 검사 (아래 / 위), 이상하면 픽셀이 적은 쪽을 버린다
    if (left_fit && right_fit) {
      const double tol = dblParam("width_tolerance") * lane_w;
      const double wb = right_fit->x(h - 1) - left_fit->x(h - 1);   // 맨 아래에서 폭
      const double wt = right_fit->x(0) - left_fit->x(0);           // 맨 위에서 폭
      // 진짜 두 차선이면 폭이 일정해야 한다. 아니면 둘 중 하나는 잡음/다른 선을 잡은 것
      if (std::abs(wb - lane_w) > tol || std::abs(wt - lane_w) > tol) {
        (left_.pixels.size() < right_.pixels.size() ? left_fit : right_fit).reset();
      }
    }

    // 6. 시간 필터
    update(left_, left_fit);
    update(right_, right_fit);

    // 7. 차선 중심 -> offset, angle
    std::optional<Poly> center;
    const bool normal_shift = get_parameter("single_line_normal_shift").as_bool();
    interfaces::msg::LaneInfo lane;
    lane.header = msg->header;
    //   두 차선 다 있음 -> 두 곡선의 평균이 중심
    //   한쪽만 있음     -> 그 곡선을 차선 폭 절반만큼 안쪽으로 옮겨 중심으로 추정
    //   confidence: 이번 프레임에 직접 찾았는지(miss == 0), 이전 값으로 버티는 중인지에 따라 다르게
    if (left_.fit && right_.fit) {
      center = left_.fit->blend(*right_.fit, 0.5);
      lane.confidence = (left_.miss == 0 && right_.miss == 0) ? 0.9f : 0.6f;
    } else if (left_.fit) {
      if (normal_shift) {
        center = normalShifted(*left_.fit, -lane_w / 2, h);
      }
      if (!center) {
        center = left_.fit->shifted(lane_w / 2);
      }
      lane.confidence = left_.miss == 0 ? 0.6f : 0.3f;
    } else if (right_.fit) {
      if (normal_shift) {
        center = normalShifted(*right_.fit, lane_w / 2, h);
      }
      if (!center) {
        center = right_.fit->shifted(-lane_w / 2);
      }
      lane.confidence = right_.miss == 0 ? 0.6f : 0.3f;
    }
    // 로봇은 BEV 맨 아래(+ eval_below_px)에 있다고 본다
    const double y_bot = h - 1 + dblParam("eval_below_px");
    // 곡선은 선 픽셀이 있는 곳까지만 믿고, 그보다 아래(로봇 쪽)는 접선으로 늘린다 (linear_below_pixels)
    //   2026-10-27 012701_corner (공사 구간 입구 왼쪽 코너): 섬 끝 U자 노란 선의 2차 곡선을 38px 더 아래로 늘리면 꼭짓점을
    //   넘어 왼쪽으로 가는 선이 angle +0.82 (오른쪽) 로 나옴. 위쪽에만 조금 보이는 흰 선 조각은 offset 이 -0.54 / +1.0 으로 튐
    double y_ref = y_bot;
    if (get_parameter("linear_below_pixels").as_bool()) {
      int low = -1;   // 이번 프레임 선 픽셀 중 가장 아래 줄 (이전 값으로 버티는 선은 맨 아래까지 있다고 봄)
      for (const Track * t : {&left_, &right_}) {
        if (!t->fit) {
          continue;
        }
        if (t->miss > 0) {
          low = h - 1;
        }
        for (const auto & p : t->pixels) {
          low = std::max(low, p.y);
        }
      }
      if (low >= 0) {
        y_ref = std::min(y_bot, static_cast<double>(low));
      }
    }
    lane.detected = center.has_value();
    // 왼쪽(노란) / 오른쪽(흰) 선을 이번 프레임에서 직접 찾았는지 (이전 값으로 버티는 중이면 false)
    lane.left_detected = left_.fit.has_value() && left_.miss == 0;
    lane.right_detected = right_.fit.has_value() && right_.miss == 0;
    if (center) {
      // 로봇 바로 앞(맨 아래 줄)에서 차선 중심이 로봇보다 얼마나 옆에 있는지, 반 차선 폭 = 1 로 정규화
      const double x_bot = center->x(y_ref) + center->dxdy(y_ref) * (y_bot - y_ref);
      lane.offset = static_cast<float>(std::clamp((x_bot - robot_x) / (lane_w / 2), -1.0, 1.0));
      // 위로(y 감소) 갈 때 x 가 늘면 오른쪽: dx/d(-y) = -dx/dy
      lane.angle = static_cast<float>(std::atan(-center->dxdy(y_ref)));
    }
    lane_pub_->publish(lane);

    // 디버그 화면은 GUI 가 보고 있을 때만 만든다 (그리기 + JPEG 인코딩이 꽤 무겁다)
    if (debug_pub_->get_subscription_count() > 0) {
      publishDebug(bev, yellow, white, center, lane, robot_x, msg->header);
    }
  }

  // GUI "선 · 벡터 검출" 화면
  //   노란/흰 칠: 색 마스크, 초록 상자: 슬라이딩 윈도우, 빨간 선: 차선 곡선(어두운 빨강 = 이전 값 유지 중)
  //   보라 선: 차선 중심, 하늘색 막대: 로봇 위치, 초록 영역: 추정한 차선
  void publishDebug(const cv::Mat & bev, const cv::Mat & yellow, const cv::Mat & white,
    const std::optional<Poly> & center, const interfaces::msg::LaneInfo & lane, double robot_x,
    const std_msgs::msg::Header & header)
  {
    const int h = bev.rows;
    const bool mask_only = get_parameter("debug_view").as_string() == "mask";
    // mask 모드는 검은 바탕, final 모드는 원본을 어둡게(0.6배) 깔고 그 위에 그린다
    cv::Mat out = mask_only ? cv::Mat(bev.size(), CV_8UC3, cv::Scalar(0, 0, 0)) : bev * 0.6;
    out.setTo(cv::Scalar(0, 220, 255), yellow);
    out.setTo(cv::Scalar(255, 255, 255), white);

    // 곡선을 그리기 위해 5px 간격으로 점을 찍어 꺾은선으로 만든다
    auto curve = [h](const Poly & p) {
        std::vector<cv::Point> pts;
        for (int y = 0; y < h; y += 5) {
          pts.emplace_back(cvRound(p.x(y)), y);
        }
        pts.emplace_back(cvRound(p.x(h - 1)), h - 1);
        return pts;
      };
    const double lane_w = dblParam("lane_width_px");
    if (!mask_only && center) {
      // 차선 사이 영역 채우기
      auto l = curve(center->shifted(-lane_w / 2)), r = curve(center->shifted(lane_w / 2));
      // 왼쪽 경계를 위->아래, 오른쪽 경계를 아래->위로 이어 하나의 닫힌 다각형으로
      std::vector<cv::Point> poly(l.begin(), l.end());
      poly.insert(poly.end(), r.rbegin(), r.rend());
      cv::Mat overlay = out.clone();
      cv::fillPoly(overlay, std::vector<std::vector<cv::Point>>{poly}, cv::Scalar(0, 200, 0));
      cv::addWeighted(overlay, 0.3, out, 0.7, 0, out);   // 30% 투명도로 겹치기
    }
    for (const Track * t : {&left_, &right_}) {
      for (const auto & win : t->windows) {
        cv::rectangle(out, win, cv::Scalar(0, 255, 0), 1);
      }
      if (t->fit) {
        cv::polylines(out, curve(*t->fit), false,
          t->miss == 0 ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 0, 128), 3);
      }
    }
    if (center) {
      cv::polylines(out, curve(*center), false, cv::Scalar(255, 0, 255), 2);
    }
    const int rx = cvRound(robot_x);
    cv::line(out, {rx, h - 40}, {rx, h - 1}, cv::Scalar(255, 255, 0), 3);   // 로봇 위치
    if (cross_.length > 0) {   // 교차로 가로 선: 주황 굵은 선
      cv::line(out, {cross_.seg[0], cross_.seg[1]}, {cross_.seg[2], cross_.seg[3]},
        cv::Scalar(0, 140, 255), 5);
      char ctext[32];
      std::snprintf(ctext, sizeof(ctext), "CROSS %+.0f deg", cross_.angle_deg);
      cv::putText(out, ctext, {6, 44}, cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 140, 255), 2);
    }
    // 왼쪽 위 글자: 차선 검출 여부(o/x)와 발행한 offset, angle
    char text[96];
    std::snprintf(text, sizeof(text), "L%s R%s off %+.2f ang %+.2f", left_.fit ? "o" : "x",
      right_.fit ? "o" : "x", lane.offset, lane.angle);
    cv::putText(out, text, {6, 20}, cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 255), 2);

    sensor_msgs::msg::CompressedImage jpeg;
    jpeg.header = header;
    jpeg.format = "jpeg";
    cv::imencode(".jpg", out, jpeg.data,
      {cv::IMWRITE_JPEG_QUALITY, intParam("debug_jpeg_quality")});
    debug_pub_->publish(jpeg);
  }

  Track left_, right_;   // 왼쪽 = 노란 선, 오른쪽 = 흰 선
  Cross cross_;          // 이번 프레임 가로 선 (디버그 그리기용)

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<interfaces::msg::LaneInfo>::SharedPtr lane_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr cross_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debug_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaneDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
