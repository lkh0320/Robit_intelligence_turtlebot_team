// 주행용 선 검출 (Bird's Eye View + 슬라이딩 윈도우 + 2차 곡선)
//   구독: image_bev (sensor_msgs/Image, bird_eye_view 노드)
//   발행: lane_info (interfaces/LaneInfo)
//         vision/lane_debug/compressed (GUI 선·벡터 검출 화면, 구독자가 있을 때만)
//   TODO: stop_line (interfaces/StopLine)
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
//      한쪽만 있으면 차선 폭의 절반만큼 옮겨 차선 중심을 추정한다
//   6. 곡선 계수를 직전 값과 섞어(smooth_alpha) 흔들림을 줄이고, 놓치면 hold_frames 동안 유지
//   7. 로봇 위치(BEV 아래, bev_center_x)에서 차선 중심까지 offset, 중심 곡선의 방향 angle
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
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace
{
// x = a*y^2 + b*y + c (BEV 이미지 좌표, y 는 아래로 증가)
struct Poly
{
  double a = 0, b = 0, c = 0;
  double x(double y) const {return (a * y + b) * y + c;}
  double dxdy(double y) const {return 2 * a * y + b;}
  Poly shifted(double dx) const {return {a, b, c + dx};}
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
  if (y_max - y_min >= min_span) {
    if (!cv::solve(A, X, coef, cv::DECOMP_QR)) {
      return std::nullopt;
    }
    return Poly{coef.at<double>(0), coef.at<double>(1), coef.at<double>(2)};
  }
  if (!cv::solve(A.colRange(1, 3), X, coef, cv::DECOMP_QR)) {
    return std::nullopt;
  }
  return Poly{0.0, coef.at<double>(0), coef.at<double>(1)};
}

// mask 의 [x0, x1) 열 중 아래 절반에서 픽셀이 가장 많은 열 (min_count 미만이면 -1)
int histogramPeak(const cv::Mat & mask, int x0, int x1, int min_count)
{
  x0 = std::clamp(x0, 0, mask.cols);
  x1 = std::clamp(x1, 0, mask.cols);
  if (x1 - x0 < 1) {
    return -1;
  }
  cv::Mat hist;
  cv::reduce(mask(cv::Rect(x0, mask.rows / 2, x1 - x0, mask.rows - mask.rows / 2)), hist, 0,
    cv::REDUCE_SUM, CV_32S);
  double max_val;
  cv::Point loc;
  cv::minMaxLoc(hist, nullptr, &max_val, nullptr, &loc);
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
    // 2~5. 차선 탐색
    declare_parameter("lane_width_px", 267.0);     // BEV 에서 두 차선 중심 사이 폭 (bird_eye_view 보정값)
    declare_parameter("bev_center_x", 0.521);      // BEV 에서 로봇(카메라 중심)의 가로 위치 (폭 비율)
    declare_parameter("base_min_pixels", 30);      // 시작점 열에 필요한 최소 픽셀 수
    declare_parameter("n_windows", 10);
    declare_parameter("window_margin", 30);        // 윈도우 반폭 / 직전 곡선 주변 탐색 폭 [px]
    declare_parameter("window_min_pixels", 20);    // 이보다 많으면 다음 윈도우 중심을 옮긴다
    declare_parameter("min_lane_pixels", 150);     // 곡선 맞춤에 필요한 최소 픽셀 수
    declare_parameter("min_curve_span", 0.4);      // 픽셀 세로 범위가 이 비율보다 짧으면 직선으로 맞춤
    declare_parameter("width_tolerance", 0.35);    // 차선 폭이 lane_width_px 에서 이 비율 넘게 다르면 이상
    // 6. 시간 필터
    declare_parameter("smooth_alpha", 0.5);        // 새 곡선 비중 (1 = 필터 없음)
    declare_parameter("hold_frames", 5);
    // 디버그: final (최종) / mask (색 마스크 + 윈도우)
    declare_parameter("debug_view", std::string("final"));
    declare_parameter("debug_jpeg_quality", 70);

    const auto qos = rclcpp::SensorDataQoS();
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, qos, [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {onImage(msg);});
    lane_pub_ = create_publisher<interfaces::msg::LaneInfo>("lane_info", qos);
    debug_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      "vision/lane_debug/compressed", qos);

    RCLCPP_INFO(get_logger(), "lane_detection 시작 (구독: %s)", image_topic.c_str());
  }

private:
  // 차선 하나의 추적 상태
  struct Track
  {
    std::optional<Poly> fit;   // 필터를 거친 곡선
    int miss = 0;              // 연속으로 놓친 프레임 수
    // 이번 프레임 결과 (디버그용)
    std::vector<cv::Point> pixels;
    std::vector<cv::Rect> windows;
  };

  int intParam(const std::string & name) {return static_cast<int>(get_parameter(name).as_int());}
  double dblParam(const std::string & name) {return get_parameter(name).as_double();}

  // 3. 차선 픽셀 모으기: 직전 곡선이 있으면 그 주변, 없으면 base_x 에서 슬라이딩 윈도우
  void collect(const std::vector<cv::Point> & nonzero, const cv::Mat & mask, int base_x,
    Track & t)
  {
    t.pixels.clear();
    t.windows.clear();
    const int margin = intParam("window_margin");
    if (t.fit && t.miss == 0) {
      for (const auto & p : nonzero) {
        if (std::abs(p.x - t.fit->x(p.y)) <= margin) {
          t.pixels.push_back(p);
        }
      }
      return;
    }
    if (base_x < 0) {
      return;
    }
    const int n = std::max(1, intParam("n_windows"));
    const int win_h = mask.rows / n;
    const int min_pix = intParam("window_min_pixels");
    int x = base_x;
    for (int i = 0; i < n; ++i) {
      const int y1 = mask.rows - i * win_h, y0 = std::max(0, y1 - win_h);
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
        p += win.tl();
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
      t.fit = t.fit ? found->blend(*t.fit, dblParam("smooth_alpha")) : *found;
      t.miss = 0;
    } else if (t.fit && ++t.miss > intParam("hold_frames")) {
      t.fit.reset();
    }
  }

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
    const double robot_x = dblParam("bev_center_x") * w;

    // 1. 색 마스크
    cv::Mat lab, yellow, white;
    cv::cvtColor(bev, lab, cv::COLOR_BGR2Lab);
    const int dev = intParam("white_ab_dev");
    cv::inRange(lab, cv::Scalar(intParam("yellow_l_min"), 0, intParam("yellow_b_min")),
      cv::Scalar(255, 255, 255), yellow);
    cv::inRange(lab, cv::Scalar(intParam("white_l_min"), 128 - dev, 128 - dev),
      cv::Scalar(255, 128 + dev, 128 + dev), white);
    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(yellow, yellow, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(white, white, cv::MORPH_OPEN, kernel);

    // 2. 시작점 (직전 곡선이 없을 때만 쓰인다)
    const int base_min = intParam("base_min_pixels");
    const int left_base = histogramPeak(yellow, 0, w, base_min);
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
    const int min_span = static_cast<int>(dblParam("min_curve_span") * h);
    auto left_fit = fitPoly(left_.pixels, min_pix, min_span);
    auto right_fit = fitPoly(right_.pixels, min_pix, min_span);

    // 5. 두 차선 폭 검사 (아래 / 위), 이상하면 픽셀이 적은 쪽을 버린다
    if (left_fit && right_fit) {
      const double tol = dblParam("width_tolerance") * lane_w;
      const double wb = right_fit->x(h - 1) - left_fit->x(h - 1);
      const double wt = right_fit->x(0) - left_fit->x(0);
      if (std::abs(wb - lane_w) > tol || std::abs(wt - lane_w) > tol) {
        (left_.pixels.size() < right_.pixels.size() ? left_fit : right_fit).reset();
      }
    }

    // 6. 시간 필터
    update(left_, left_fit);
    update(right_, right_fit);

    // 7. 차선 중심 -> offset, angle
    std::optional<Poly> center;
    interfaces::msg::LaneInfo lane;
    lane.header = msg->header;
    if (left_.fit && right_.fit) {
      center = left_.fit->blend(*right_.fit, 0.5);
      lane.confidence = (left_.miss == 0 && right_.miss == 0) ? 0.9f : 0.6f;
    } else if (left_.fit) {
      center = left_.fit->shifted(lane_w / 2);
      lane.confidence = left_.miss == 0 ? 0.6f : 0.3f;
    } else if (right_.fit) {
      center = right_.fit->shifted(-lane_w / 2);
      lane.confidence = right_.miss == 0 ? 0.6f : 0.3f;
    }
    const double y_bot = h - 1;
    lane.detected = center.has_value();
    if (center) {
      lane.offset = static_cast<float>(
        std::clamp((center->x(y_bot) - robot_x) / (lane_w / 2), -1.0, 1.0));
      // 위로(y 감소) 갈 때 x 가 늘면 오른쪽: dx/d(-y) = -dx/dy
      lane.angle = static_cast<float>(std::atan(-center->dxdy(y_bot)));
    }
    lane_pub_->publish(lane);

    if (debug_pub_->get_subscription_count() > 0) {
      publishDebug(bev, yellow, white, center, lane, robot_x, msg->header);
    }
  }

  void publishDebug(const cv::Mat & bev, const cv::Mat & yellow, const cv::Mat & white,
    const std::optional<Poly> & center, const interfaces::msg::LaneInfo & lane, double robot_x,
    const std_msgs::msg::Header & header)
  {
    const int h = bev.rows;
    const bool mask_only = get_parameter("debug_view").as_string() == "mask";
    cv::Mat out = mask_only ? cv::Mat(bev.size(), CV_8UC3, cv::Scalar(0, 0, 0)) : bev * 0.6;
    out.setTo(cv::Scalar(0, 220, 255), yellow);
    out.setTo(cv::Scalar(255, 255, 255), white);

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
      std::vector<cv::Point> poly(l.begin(), l.end());
      poly.insert(poly.end(), r.rbegin(), r.rend());
      cv::Mat overlay = out.clone();
      cv::fillPoly(overlay, std::vector<std::vector<cv::Point>>{poly}, cv::Scalar(0, 200, 0));
      cv::addWeighted(overlay, 0.3, out, 0.7, 0, out);
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

  Track left_, right_;

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
