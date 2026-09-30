// 표지판용 객체 인식 (갈림길 방향 표지판)
//   구독: image_raw (sensor_msgs/Image)
//   발행: sign (Sign), vision/object_debug/compressed (검출 결과를 그린 화면, 구독자가 있을 때만)
// 처리: HSV 파란색 마스크 -> 원형에 가까운 가장 큰 윤곽 = 표지판
//       표지판 안 흰색(화살표) 픽셀의 무게중심이 중심보다 오른쪽이면 RIGHT, 왼쪽이면 LEFT
//       (화살표 머리 쪽에 흰 면적이 더 많다는 가정. 나중에 딥러닝 모델로 바꿀 때는 detect()만 교체)
#include <algorithm>
#include <climits>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "interfaces/msg/sign.hpp"
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

struct Detection
{
  cv::Rect box;
  uint8_t type;        // interfaces::msg::Sign 상수
  double confidence;
};

const char * typeName(uint8_t t)
{
  using S = interfaces::msg::Sign;
  switch (t) {
    case S::LEFT: return "LEFT";
    case S::RIGHT: return "RIGHT";
    default: return "?";
  }
}
}  // namespace

class SignDetectionNode : public rclcpp::Node
{
public:
  SignDetectionNode()
  : Node("sign_detection")
  {
    const auto image_topic = declare_parameter("image_topic", std::string("image_raw"));
    // 파란색 범위 (OpenCV HSV: H 0~180)
    declare_parameter("blue_h_min", 100);
    declare_parameter("blue_h_max", 130);
    declare_parameter("blue_s_min", 100);
    declare_parameter("blue_v_min", 50);
    // 표지판 안 화살표(흰색) 범위
    declare_parameter("white_s_max", 60);
    declare_parameter("white_v_min", 170);
    declare_parameter("min_area", 0.002);         // 최소 면적 (이미지 면적 비율)
    declare_parameter("min_circularity", 0.6);    // 4πA/P² (원 = 1)
    declare_parameter("direction_threshold", 0.05);  // 무게중심 치우침 / 표지판 폭
    declare_parameter("debug_jpeg_quality", 70);

    const auto qos = rclcpp::SensorDataQoS();
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, qos, [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {onImage(msg);});
    sign_pub_ = create_publisher<interfaces::msg::Sign>("sign", qos);
    debug_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      "vision/object_debug/compressed", qos);

    RCLCPP_INFO(get_logger(), "sign_detection 시작 (구독: %s)", image_topic.c_str());
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

    cv::Mat blue_mask;
    const auto det = detect(frame, blue_mask);

    interfaces::msg::Sign sign;
    sign.header = msg->header;
    sign.type = interfaces::msg::Sign::NONE;
    if (det) {
      sign.type = det->type;
      sign.area_ratio = static_cast<float>(
        static_cast<double>(det->box.area()) / (frame.cols * frame.rows));
      sign.confidence = static_cast<float>(det->confidence);
    }
    sign_pub_->publish(sign);

    // --- 디버그 화면 (GUI 객체 인식 창) ---
    if (debug_pub_->get_subscription_count() == 0) {
      return;
    }
    cv::Mat dbg = frame.clone();
    // 파란 마스크 영역을 옅게 표시
    cv::Mat tint(frame.size(), frame.type(), cv::Scalar(255, 120, 0));
    tint.copyTo(dbg, blue_mask);
    cv::addWeighted(dbg, 0.5, frame, 0.5, 0, dbg);
    if (det) {
      cv::rectangle(dbg, det->box, cv::Scalar(0, 255, 255), 2);
      char text[64];
      std::snprintf(text, sizeof(text), "%s %.2f", typeName(det->type), det->confidence);
      cv::putText(dbg, text, det->box.tl() + cv::Point(0, -6), cv::FONT_HERSHEY_SIMPLEX, 0.6,
        cv::Scalar(0, 255, 255), 2);
    }
    publishDebug(dbg, msg->header);
  }

  // 방향 표지판 1개 검출 (없으면 nullopt). blue_mask 는 디버그 표시용으로 채워 준다.
  std::optional<Detection> detect(const cv::Mat & frame, cv::Mat & blue_mask)
  {
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv,
      cv::Scalar(get_parameter("blue_h_min").as_int(), get_parameter("blue_s_min").as_int(),
      get_parameter("blue_v_min").as_int()),
      cv::Scalar(get_parameter("blue_h_max").as_int(), 255, 255), blue_mask);
    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(blue_mask, blue_mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(blue_mask, blue_mask, cv::MORPH_CLOSE, kernel);

    // 화살표가 파란 원 안의 구멍이 되므로 바깥 윤곽만 본다
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(blue_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    const double min_area = get_parameter("min_area").as_double() * frame.cols * frame.rows;
    const double min_circ = get_parameter("min_circularity").as_double();
    const std::vector<cv::Point> * best = nullptr;
    double best_area = 0.0, best_circ = 0.0;
    for (const auto & c : contours) {
      const double area = cv::contourArea(c);
      const double perim = cv::arcLength(c, true);
      if (area < min_area || perim <= 0) {
        continue;
      }
      const double circ = 4.0 * CV_PI * area / (perim * perim);
      if (circ >= min_circ && area > best_area) {
        best = &c;
        best_area = area;
        best_circ = circ;
      }
    }
    if (!best) {
      return std::nullopt;
    }

    // 표지판 내부(윤곽 안) 흰색 픽셀의 무게중심으로 화살표 방향 판단
    const cv::Rect box = cv::boundingRect(*best) & cv::Rect(0, 0, frame.cols, frame.rows);
    cv::Mat inside = cv::Mat::zeros(box.size(), CV_8U);
    cv::drawContours(inside, std::vector<std::vector<cv::Point>>{*best}, 0, 255, cv::FILLED,
      cv::LINE_8, cv::noArray(), INT_MAX, -box.tl());
    cv::Mat white;
    cv::inRange(hsv(box), cv::Scalar(0, 0, get_parameter("white_v_min").as_int()),
      cv::Scalar(180, get_parameter("white_s_max").as_int(), 255), white);
    white &= inside;

    const cv::Moments m = cv::moments(white, true);
    Detection det{box, interfaces::msg::Sign::NONE, 0.0};
    if (m.m00 < 0.02 * box.area()) {
      return std::nullopt;   // 화살표가 안 보이면 방향 표지판이 아님
    }
    const double shift = (m.m10 / m.m00 - box.width / 2.0) / box.width;
    const double thr = get_parameter("direction_threshold").as_double();
    if (shift > thr) {
      det.type = interfaces::msg::Sign::RIGHT;
    } else if (shift < -thr) {
      det.type = interfaces::msg::Sign::LEFT;
    } else {
      return std::nullopt;   // 방향을 판단할 수 없음
    }
    det.confidence = std::clamp(best_circ * std::min(1.0, std::abs(shift) / (2 * thr)), 0.0, 1.0);
    return det;
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
  rclcpp::Publisher<interfaces::msg::Sign>::SharedPtr sign_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debug_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SignDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
