// 주행용 선 검출
//   구독: image_raw (sensor_msgs/Image)
//   발행: lane_info (interfaces/LaneInfo)
//         vision/lane_debug/compressed (GUI 선·벡터 검출 화면, 구독자가 있을 때만)
//   TODO: stop_line (interfaces/StopLine)
//
// 처리 순서
//   (1) 원본 영상 읽기
//   (2) 흰색 / 노란색 범위만 남겨 후보로 저장
//   (3) GrayScale 변환
//   (4) Gaussian 필터로 잡음 제거 후 Canny 에지 추출
//   (5) 진행 방향 바닥의 차선만 보도록 사다리꼴 관심 영역(ROI) 지정
//   (6) Hough 변환으로 직선 성분 추출
//   (7) 기울기/위치로 좌우 차선 후보를 나누고, 각각 선형 회귀로 가장 적합한 직선 계산
//       (선분 길이 가중 + 이상치 제거 후 재회귀)
//   (8) 두 차선의 소실점으로 진행 방향 예측
//   (9) 최종 차선을 선으로 그리고, 차선 사이 다각형을 색으로 채움
//
// bev:=true 면 Bird's Eye View 영상(image_bev)을 입력으로 쓴다. 차선이 세로로 평행하게 보이므로
//   (5) ROI 는 전체 화면, (7) 좌우는 기울기 부호 대신 흰/노란 픽셀 열 히스토그램의 좌/우 최고점
//   근처 선분만 쓰고 (BEV 에서 길게 늘어나는 바닥 반사광 제외), (8) 진행 방향은 소실점 대신
//   차선 중심선의 기울기로 판단한다. 로봇 위치는 BEV 화면 가운데가 아니라 bev_center_x.
//
// offset 은 두 방식 모두 화면 맨 아래에서 (차선 중심 - 로봇 위치) / (차선 폭 / 2)
//   0 = 차선 가운데, ±1 = 오른쪽/왼쪽 차선 위
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "cv_bridge/cv_bridge.hpp"
#include "interfaces/msg/lane_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace
{
// 직선 x = a*y + b (이미지 좌표, y 는 아래로 증가). 차선은 세로에 가까워 x 를 y 의 함수로 둔다.
struct Line
{
  double a;
  double b;
  double x(double y) const {return a * y + b;}
};

// 선분들의 끝점에 대해 x = a*y + b 가중 최소제곱 선형 회귀 (가중치 = 선분 길이)
// 짧은 잡음 선분(바닥 반사광 등)의 영향을 줄인다
std::optional<Line> linearRegression(const std::vector<cv::Vec4i> & segs)
{
  double sw = 0, sy = 0, sx = 0, syy = 0, sxy = 0;
  for (const auto & l : segs) {
    const double wgt = std::hypot(l[2] - l[0], l[3] - l[1]);
    for (const cv::Point p : {cv::Point(l[0], l[1]), cv::Point(l[2], l[3])}) {
      sw += wgt;
      sy += wgt * p.y;
      sx += wgt * p.x;
      syy += wgt * p.y * p.y;
      sxy += wgt * p.x * p.y;
    }
  }
  const double den = sw * syy - sy * sy;
  if (segs.empty() || std::abs(den) < 1e-6) {
    return std::nullopt;   // 선분 없음, 또는 모든 점의 y 가 같음 (수평선)
  }
  const double a = (sw * sxy - sx * sy) / den;
  return Line{a, (sx - a * sy) / sw};
}

// 한 번 회귀한 뒤 직선에서 max_dist[px] 넘게 떨어진 선분을 빼고 다시 회귀
std::optional<Line> robustFit(const std::vector<cv::Vec4i> & segs, double max_dist)
{
  const auto first = linearRegression(segs);
  if (!first) {
    return std::nullopt;
  }
  std::vector<cv::Vec4i> inliers;
  for (const auto & l : segs) {
    const double mid_x = (l[0] + l[2]) / 2.0, mid_y = (l[1] + l[3]) / 2.0;
    if (std::abs(first->x(mid_y) - mid_x) <= max_dist) {
      inliers.push_back(l);
    }
  }
  return inliers.empty() ? first : linearRegression(inliers);
}
}  // namespace

class LaneDetectionNode : public rclcpp::Node
{
public:
  LaneDetectionNode()
  : Node("lane_detection")
  {
    bev_ = declare_parameter("bev", false);
    const auto image_topic = declare_parameter(
      "image_topic", std::string(bev_ ? "image_bev" : "image_raw"));
    // (2) 색 범위 (OpenCV HSV: H 0~180)
    declare_parameter("white_s_max", 60);
    declare_parameter("white_v_min", 200);         // 바닥 반사광(V≈170)은 제외
    declare_parameter("yellow_h_min", 15);
    declare_parameter("yellow_h_max", 40);
    declare_parameter("yellow_s_min", 60);
    declare_parameter("yellow_v_min", 100);
    // (4) 에지
    declare_parameter("blur_kernel", 5);
    declare_parameter("canny_low", 50);
    declare_parameter("canny_high", 150);
    // (5) ROI 사다리꼴 (이미지 크기 비율). 아랫변은 화면 맨 아래 전체 폭
    declare_parameter("roi_top_y", 0.55);
    declare_parameter("roi_top_left_x", 0.2);
    declare_parameter("roi_top_right_x", 0.8);
    // (6) Hough
    declare_parameter("hough_threshold", 20);
    declare_parameter("hough_min_length", 20);     // [px]
    declare_parameter("hough_max_gap", 30);        // [px]
    // (7) 좌우 차선 후보: |기울기(dy/dx)| 가 이보다 작은 (수평에 가까운) 선은 버린다
    declare_parameter("min_slope", 0.3);
    declare_parameter("bev_min_angle", 45.0);      // (bev) 수평에서 이 각도[deg] 이상인 선만 차선 후보
    declare_parameter("bev_peak_margin", 25);      // (bev) 히스토그램 최고점에서 이 거리[px] 안의 선분만 차선
    // (bev) BEV 화면에서 로봇(카메라 중심)의 가로 위치 (폭 비율). bird_eye_view 보정값에서 계산:
    //       원본 맨 아래 가운데 (320, 480) 이 BEV 의 어디로 가는지
    declare_parameter("bev_center_x", 0.521);
    declare_parameter("outlier_dist", 25.0);       // 1차 회귀 직선에서 이 거리[px] 넘는 선분은 제외 후 재회귀
    declare_parameter("hold_frames", 5);           // 한쪽 차선을 놓쳐도 직전 값을 유지할 프레임 수
    // (8) 진행 방향: 소실점이 화면 중앙에서 이 비율 이상 벗어나면 좌/우 회전으로 판단
    declare_parameter("turn_threshold", 0.05);
    // GUI 에 보낼 화면: final / color / edges / roi
    declare_parameter("debug_view", std::string("final"));
    declare_parameter("debug_jpeg_quality", 70);

    const auto qos = rclcpp::SensorDataQoS();
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, qos, [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {onImage(msg);});
    lane_pub_ = create_publisher<interfaces::msg::LaneInfo>("lane_info", qos);
    debug_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      "vision/lane_debug/compressed", qos);

    RCLCPP_INFO(get_logger(), "%s 시작 (구독: %s, %s)", get_name(), image_topic.c_str(),
      bev_ ? "BEV" : "원근 영상");
  }

private:
  int intParam(const std::string & name) {return static_cast<int>(get_parameter(name).as_int());}
  double dblParam(const std::string & name) {return get_parameter(name).as_double();}

  // 이번 프레임에서 못 찾으면 hold_frames 동안 직전 차선을 유지
  std::optional<Line> hold(const std::optional<Line> & found, std::optional<Line> & last, int & miss)
  {
    if (found) {
      last = found;
      miss = 0;
    } else if (last && ++miss > intParam("hold_frames")) {
      last.reset();
    }
    return last;
  }

  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    const auto t0 = std::chrono::steady_clock::now();

    // (1) 원본 영상 읽기
    cv::Mat frame;
    try {
      frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "cv_bridge: %s", e.what());
      return;
    }
    const int w = frame.cols, h = frame.rows;

    // (2) 흰색 / 노란색 범위만 후보로 저장
    cv::Mat hsv, white, yellow, color_mask, candidate;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0, 0, intParam("white_v_min")),
      cv::Scalar(180, intParam("white_s_max"), 255), white);
    cv::inRange(hsv,
      cv::Scalar(intParam("yellow_h_min"), intParam("yellow_s_min"), intParam("yellow_v_min")),
      cv::Scalar(intParam("yellow_h_max"), 255, 255), yellow);
    cv::bitwise_or(white, yellow, color_mask);
    cv::bitwise_and(frame, frame, candidate, color_mask);

    // (3) GrayScale 변환
    cv::Mat gray;
    cv::cvtColor(candidate, gray, cv::COLOR_BGR2GRAY);

    // (4) Gaussian 필터 + Canny 에지
    const int k = std::max(1, intParam("blur_kernel")) | 1;   // 홀수
    cv::Mat blurred, edges;
    cv::GaussianBlur(gray, blurred, cv::Size(k, k), 0);
    cv::Canny(blurred, edges, intParam("canny_low"), intParam("canny_high"));

    // (5) 관심 영역: 진행 방향 바닥의 사다리꼴 (BEV 는 이미 바닥만 펼친 영상이라 전체)
    const int top_y = bev_ ? 0 :
      std::clamp(static_cast<int>(dblParam("roi_top_y") * h), 0, h - 1);
    const std::vector<cv::Point> roi_poly = bev_ ?
      std::vector<cv::Point>{{0, h - 1}, {0, 0}, {w - 1, 0}, {w - 1, h - 1}} :
      std::vector<cv::Point>{
      {0, h - 1},
      {static_cast<int>(dblParam("roi_top_left_x") * w), top_y},
      {static_cast<int>(dblParam("roi_top_right_x") * w), top_y},
      {w - 1, h - 1},
    };
    cv::Mat roi_mask = cv::Mat::zeros(edges.size(), CV_8U), roi_edges;
    cv::fillPoly(roi_mask, std::vector<std::vector<cv::Point>>{roi_poly}, 255);
    cv::bitwise_and(edges, roi_mask, roi_edges);

    // (6) Hough 변환으로 직선 성분 추출
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(roi_edges, lines, 1, CV_PI / 180, intParam("hough_threshold"),
      intParam("hough_min_length"), intParam("hough_max_gap"));

    // (7) 좌우 차선 후보 분리 -> 각각 선형 회귀
    //     이미지 좌표(y 아래로 증가)에서 왼쪽 차선은 기울기 < 0, 오른쪽 차선은 > 0
    //     기울기 부호와 화면 좌/우 위치가 모두 맞는 선만 쓴다
    //     (bev) 차선이 세로로 평행하므로 세로에 가까운 선을 화면 좌/우 위치로만 나눈다
    const double min_slope = dblParam("min_slope");
    const double bev_min_angle = dblParam("bev_min_angle");
    std::vector<cv::Vec4i> left_lines, right_lines;
    int left_peak = -1, right_peak = -1;
    if (bev_) {
      // 차선은 세로로 곧으므로 흰/노란 픽셀이 가장 많이 몰린 열이 차선 위치
      cv::Mat hist;
      cv::reduce(color_mask, hist, 0, cv::REDUCE_SUM, CV_32S);
      cv::Point loc;
      cv::minMaxLoc(hist.colRange(0, w / 2), nullptr, nullptr, nullptr, &loc);
      left_peak = loc.x;
      cv::minMaxLoc(hist.colRange(w / 2, w), nullptr, nullptr, nullptr, &loc);
      right_peak = w / 2 + loc.x;
    }
    const int peak_margin = intParam("bev_peak_margin");
    for (const auto & l : lines) {
      const double dx = l[2] - l[0], dy = l[3] - l[1];
      if (bev_) {
        const double deg = std::abs(std::atan2(dy, dx)) * 180.0 / CV_PI;
        if (std::min(deg, 180.0 - deg) < bev_min_angle) {
          continue;
        }
        auto near = [&](int peak) {
            return std::abs(l[0] - peak) <= peak_margin && std::abs(l[2] - peak) <= peak_margin;
          };
        if (near(left_peak)) {
          left_lines.push_back(l);
        } else if (near(right_peak)) {
          right_lines.push_back(l);
        }
        continue;
      }
      if (dx == 0) {
        continue;   // 완전한 세로선은 좌우 판단 불가
      }
      const double slope = dy / dx;
      if (std::abs(slope) < min_slope) {
        continue;
      }
      const double mid_x = (l[0] + l[2]) / 2.0;
      if (slope < 0 && mid_x < w / 2.0) {
        left_lines.push_back(l);
      } else if (slope > 0 && mid_x > w / 2.0) {
        right_lines.push_back(l);
      }
    }
    const double max_dist = dblParam("outlier_dist");
    const auto left = hold(robustFit(left_lines, max_dist), last_left_, left_miss_);
    const auto right = hold(robustFit(right_lines, max_dist), last_right_, right_miss_);

    // (8) 진행 방향 예측: 차선 중심선이 향하는 앞쪽 점이 화면 중앙에서 얼마나 벗어났는지
    //     원근 영상: 두 차선의 교점(소실점) / BEV: 차선 중심선의 맨 위 점
    interfaces::msg::LaneInfo lane;
    lane.header = msg->header;
    const double y_bot = h - 1;
    std::optional<cv::Point2d> ahead;
    std::string direction = "-";
    if (left && right) {
      const double cx_bot = (left->x(y_bot) + right->x(y_bot)) / 2.0;
      if (bev_) {
        ahead = cv::Point2d((left->x(0) + right->x(0)) / 2.0, 0.0);
      } else if (std::abs(left->a - right->a) > 1e-6) {
        // a1*y + b1 = a2*y + b2
        const double vy = (right->b - left->b) / (left->a - right->a);
        ahead = cv::Point2d(left->x(vy), vy);
      }
      lane.detected = true;
      lane.confidence = (left_miss_ == 0 && right_miss_ == 0) ? 0.9f : 0.6f;
      // offset +: 차선 중심이 로봇보다 오른쪽 (차선 폭/2 로 정규화) / angle +: 진행 방향이 오른쪽
      const double robot_x = bev_ ? dblParam("bev_center_x") * w : w / 2.0;
      const double half_lane = std::max(1.0, (right->x(y_bot) - left->x(y_bot)) / 2.0);
      lane.offset = static_cast<float>(std::clamp((cx_bot - robot_x) / half_lane, -1.0, 1.0));
      if (ahead && ahead->y < y_bot) {
        lane.angle = static_cast<float>(std::atan2(ahead->x - cx_bot, y_bot - ahead->y));
        const double shift = (ahead->x - w / 2.0) / w;
        const double thr = dblParam("turn_threshold");
        direction = shift < -thr ? "Left" : shift > thr ? "Right" : "Straight";
      }
    }
    lane_pub_->publish(lane);

    // 처리 시간 (원근 / BEV 비교용), 5초마다 평균 출력
    proc_ms_ += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    if (++proc_frames_ >= 150) {
      RCLCPP_INFO(get_logger(), "평균 처리 시간 %.2f ms (%d 프레임)", proc_ms_ / proc_frames_,
        proc_frames_);
      proc_ms_ = 0.0;
      proc_frames_ = 0;
    }

    // (9) 최종 차선 + 차선 사이 다각형 (GUI 선·벡터 검출 화면)
    if (debug_pub_->get_subscription_count() == 0) {
      return;
    }
    const std::string view = get_parameter("debug_view").as_string();
    cv::Mat out;
    if (view == "color") {
      out = candidate;
    } else if (view == "edges") {
      cv::cvtColor(edges, out, cv::COLOR_GRAY2BGR);
    } else if (view == "roi") {
      cv::cvtColor(roi_edges, out, cv::COLOR_GRAY2BGR);
      for (const auto & l : lines) {
        cv::line(out, {l[0], l[1]}, {l[2], l[3]}, cv::Scalar(0, 255, 255), 2);
      }
    } else {
      out = frame.clone();
      if (left && right) {
        const double y_top = top_y;
        const std::vector<cv::Point> lane_poly{
          {cvRound(left->x(y_bot)), cvRound(y_bot)}, {cvRound(left->x(y_top)), cvRound(y_top)},
          {cvRound(right->x(y_top)), cvRound(y_top)}, {cvRound(right->x(y_bot)), cvRound(y_bot)},
        };
        cv::Mat overlay = out.clone();
        cv::fillPoly(overlay, std::vector<std::vector<cv::Point>>{lane_poly}, cv::Scalar(0, 200, 0));
        cv::addWeighted(overlay, 0.3, out, 0.7, 0, out);
      }
      for (const auto & l : left_lines) {
        cv::line(out, {l[0], l[1]}, {l[2], l[3]}, cv::Scalar(255, 150, 0), 1);
      }
      for (const auto & l : right_lines) {
        cv::line(out, {l[0], l[1]}, {l[2], l[3]}, cv::Scalar(255, 150, 0), 1);
      }
      for (const auto & line : {left, right}) {
        if (line) {
          cv::line(out, {cvRound(line->x(y_bot)), cvRound(y_bot)},
            {cvRound(line->x(top_y)), top_y}, cv::Scalar(0, 0, 255), 4);
        }
      }
      if (lane.detected) {
        const cv::Point base(cvRound((left->x(y_bot) + right->x(y_bot)) / 2.0), cvRound(y_bot));
        const double len = y_bot - top_y;
        const cv::Point tip(cvRound(base.x + len * std::sin(lane.angle)),
          cvRound(base.y - len * std::cos(lane.angle)));
        cv::arrowedLine(out, base, tip, cv::Scalar(255, 0, 255), 3, cv::LINE_8, 0, 0.15);
      }
      cv::polylines(out, roi_poly, true, cv::Scalar(255, 200, 0), 1);
      if (bev_) {
        const int rx = cvRound(dblParam("bev_center_x") * w);
        cv::line(out, {rx, 0}, {rx, h - 1}, cv::Scalar(255, 255, 255), 1);   // 로봇 위치
        for (const int peak : {left_peak, right_peak}) {
          cv::rectangle(out, {peak - peak_margin, 0}, {peak + peak_margin, h - 1},
            cv::Scalar(0, 255, 255), 1);   // 차선 후보 창
        }
      }
      char text[96];
      std::snprintf(text, sizeof(text), "%s  offset %+.2f  angle %+.2f", direction.c_str(),
        lane.offset, lane.angle);
      cv::putText(out, text, {8, 24}, cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(0, 255, 255), 2);
    }

    sensor_msgs::msg::CompressedImage jpeg;
    jpeg.header = msg->header;
    jpeg.format = "jpeg";
    cv::imencode(".jpg", out, jpeg.data, {cv::IMWRITE_JPEG_QUALITY, intParam("debug_jpeg_quality")});
    debug_pub_->publish(jpeg);
  }

  bool bev_;
  double proc_ms_ = 0.0;
  int proc_frames_ = 0;
  std::optional<Line> last_left_, last_right_;
  int left_miss_ = 0, right_miss_ = 0;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<interfaces::msg::LaneInfo>::SharedPtr lane_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debug_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaneDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
