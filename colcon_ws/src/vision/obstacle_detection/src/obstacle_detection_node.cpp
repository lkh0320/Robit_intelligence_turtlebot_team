// 장애물 인식 (차단바)
//   구독: image_raw (sensor_msgs/Image)
//   발행: barrier (Barrier), vision/obstacle_debug/compressed (검출 결과를 그린 화면, 구독자가 있을 때만)
// 처리: HSV 빨간색 마스크 -> 길쭉한 빨간 조각(차단바 줄무늬)이 min_segments 개 이상이면 검출
//       조각 중심들을 이은 직선이 수평에 가까우면 CLOSED(내려옴), 아니면 OPEN(올라감)
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "interfaces/msg/barrier.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace
{
// sensor_msgs/Image -> BGR cv::Mat
// cv_bridge 는 쓰지 않는다: Jetson 은 OpenCV 4.8(JetPack)인데 ROS cv_bridge 는 4.6 에 링크돼 있어
// 한 프로세스에 두 버전이 섞이기 때문
bool toBgr(const sensor_msgs::msg::Image & msg, cv::Mat & out)
{
  const int w = static_cast<int>(msg.width), h = static_cast<int>(msg.height);
  const auto step = static_cast<size_t>(msg.step);
  auto * data = const_cast<uint8_t *>(msg.data.data());
  if (msg.encoding == "bgr8") {
    out = cv::Mat(h, w, CV_8UC3, data, step).clone();
  } else if (msg.encoding == "rgb8") {
    cv::cvtColor(cv::Mat(h, w, CV_8UC3, data, step), out, cv::COLOR_RGB2BGR);
  } else if (msg.encoding == "mono8") {
    cv::cvtColor(cv::Mat(h, w, CV_8UC1, data, step), out, cv::COLOR_GRAY2BGR);
  } else if (msg.encoding == "yuv422_yuy2" || msg.encoding == "yuyv") {
    cv::cvtColor(cv::Mat(h, w, CV_8UC2, data, step), out, cv::COLOR_YUV2BGR_YUY2);
  } else if (msg.encoding == "bgra8") {
    cv::cvtColor(cv::Mat(h, w, CV_8UC4, data, step), out, cv::COLOR_BGRA2BGR);
  } else if (msg.encoding == "rgba8") {
    cv::cvtColor(cv::Mat(h, w, CV_8UC4, data, step), out, cv::COLOR_RGBA2BGR);
  } else {
    return false;
  }
  return true;
}
}  // namespace

class ObstacleDetectionNode : public rclcpp::Node
{
public:
  ObstacleDetectionNode()
  : Node("obstacle_detection")
  {
    const auto image_topic = declare_parameter("image_topic", std::string("image_raw"));
    // 빨간색은 H 가 0 근처와 180 근처로 나뉜다 (OpenCV HSV: H 0~180)
    declare_parameter("red_h_low_max", 10);
    declare_parameter("red_h_high_min", 170);
    declare_parameter("red_s_min", 100);
    declare_parameter("red_v_min", 70);
    declare_parameter("min_segment_area", 0.0005);  // 줄무늬 조각 최소 면적 (이미지 면적 비율)
    declare_parameter("min_elongation", 1.5);       // 조각 긴 변 / 짧은 변
    declare_parameter("min_segments", 2);           // 이 개수 이상이면 차단바로 판단
    declare_parameter("closed_max_angle", 30.0);    // 수평에서 이 각도[deg] 이하면 CLOSED
    declare_parameter("debug_jpeg_quality", 70);

    const auto qos = rclcpp::SensorDataQoS();
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, qos, [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {onImage(msg);});
    barrier_pub_ = create_publisher<interfaces::msg::Barrier>("barrier", qos);
    debug_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      "vision/obstacle_debug/compressed", qos);

    RCLCPP_INFO(get_logger(), "obstacle_detection 시작 (구독: %s)", image_topic.c_str());
  }

private:
  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    cv::Mat frame;
    if (!toBgr(*msg, frame)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "지원하지 않는 영상 인코딩: %s", msg->encoding.c_str());
      return;
    }

    // --- 빨간색 마스크 ---
    cv::Mat hsv, low, high, mask;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    const int s_min = static_cast<int>(get_parameter("red_s_min").as_int());
    const int v_min = static_cast<int>(get_parameter("red_v_min").as_int());
    cv::inRange(hsv, cv::Scalar(0, s_min, v_min),
      cv::Scalar(get_parameter("red_h_low_max").as_int(), 255, 255), low);
    cv::inRange(hsv, cv::Scalar(get_parameter("red_h_high_min").as_int(), s_min, v_min),
      cv::Scalar(180, 255, 255), high);
    mask = low | high;
    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

    // --- 길쭉한 빨간 조각 = 차단바 줄무늬 후보 ---
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    const double min_area =
      get_parameter("min_segment_area").as_double() * frame.cols * frame.rows;
    const double min_elong = get_parameter("min_elongation").as_double();
    std::vector<cv::RotatedRect> segments;
    for (const auto & c : contours) {
      if (cv::contourArea(c) < min_area) {
        continue;
      }
      const cv::RotatedRect r = cv::minAreaRect(c);
      const double long_side = std::max(r.size.width, r.size.height);
      const double short_side = std::max(1.0f, std::min(r.size.width, r.size.height));
      if (long_side / short_side >= min_elong) {
        segments.push_back(r);
      }
    }

    interfaces::msg::Barrier barrier;
    barrier.header = msg->header;
    barrier.state = interfaces::msg::Barrier::UNKNOWN;
    const int min_segments = static_cast<int>(get_parameter("min_segments").as_int());
    double bar_deg = 0.0;
    cv::Vec4f bar_line;
    barrier.detected = static_cast<int>(segments.size()) >= min_segments;
    if (barrier.detected) {
      // 조각 중심들을 이은 직선의 기울기로 차단바가 내려왔는지 판단
      std::vector<cv::Point2f> centers;
      for (const auto & s : segments) {
        centers.push_back(s.center);
      }
      cv::fitLine(centers, bar_line, cv::DIST_L2, 0, 0.01, 0.01);
      bar_deg = std::abs(std::atan2(bar_line[1], bar_line[0])) * 180.0 / CV_PI;
      bar_deg = std::min(bar_deg, 180.0 - bar_deg);
      barrier.state = bar_deg <= get_parameter("closed_max_angle").as_double() ?
        interfaces::msg::Barrier::CLOSED : interfaces::msg::Barrier::OPEN;
      barrier.confidence = static_cast<float>(
        std::min(1.0, static_cast<double>(segments.size()) / (min_segments + 2)));
    }
    barrier_pub_->publish(barrier);

    // --- 디버그 화면 ---
    if (debug_pub_->get_subscription_count() == 0) {
      return;
    }
    cv::Mat dbg = frame.clone();
    for (const auto & s : segments) {
      cv::Point2f pts[4];
      s.points(pts);
      for (int i = 0; i < 4; ++i) {
        cv::line(dbg, pts[i], pts[(i + 1) % 4], cv::Scalar(0, 255, 255), 2);
      }
    }
    if (barrier.detected) {
      const cv::Point2f p(bar_line[2], bar_line[3]), d(bar_line[0], bar_line[1]);
      const float len = static_cast<float>(std::max(frame.cols, frame.rows));
      cv::line(dbg, p - d * len, p + d * len, cv::Scalar(255, 0, 255), 2);
    }
    const char * state = barrier.state == interfaces::msg::Barrier::CLOSED ? "CLOSED" :
      barrier.state == interfaces::msg::Barrier::OPEN ? "OPEN" : "-";
    char text[96];
    std::snprintf(text, sizeof(text), "barrier %s  segments %zu  angle %.0f", state,
      segments.size(), bar_deg);
    cv::putText(dbg, text, cv::Point(8, 22), cv::FONT_HERSHEY_SIMPLEX, 0.6,
      cv::Scalar(0, 255, 255), 2);
    publishDebug(dbg, msg->header);
  }

  void publishDebug(const cv::Mat & img, const std_msgs::msg::Header & header)
  {
    sensor_msgs::msg::CompressedImage out;
    out.header = header;
    out.format = "jpeg";
    const std::vector<int> opts{cv::IMWRITE_JPEG_QUALITY,
      static_cast<int>(get_parameter("debug_jpeg_quality").as_int())};
    cv::imencode(".jpg", img, out.data, opts);
    debug_pub_->publish(out);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<interfaces::msg::Barrier>::SharedPtr barrier_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debug_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
