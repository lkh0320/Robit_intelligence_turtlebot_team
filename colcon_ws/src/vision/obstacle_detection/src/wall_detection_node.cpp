// 장애물 인식: 벽 (좌우에서 번갈아 튀어나온 갈색 박스를 지그재그로 피해 가는 구간)
//   구독: image_raw (sensor_msgs/Image)
//   발행: wall_obstacle (WallObstacle), vision/wall_debug/compressed (검출 결과를 그린 화면, 구독자가 있을 때만)
// 처리: ROI -> HSV 갈색 마스크 -> 일정 크기 이상 덩어리 = 벽
//       아래 끝이 가장 아래(가장 가까운) 덩어리를 기준으로 막힌 쪽과 반대편 빈 공간 중심을 계산한다.
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "interfaces/msg/wall_obstacle.hpp"
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

class WallDetectionNode : public rclcpp::Node
{
public:
  WallDetectionNode()
  : Node("wall_detection")
  {
    const auto image_topic = declare_parameter("image_topic", std::string("image_raw"));
    declare_parameter("roi_top", 0.3);              // 이 비율 아래쪽만 처리 (먼 배경 제외)
    // 갈색 범위 (OpenCV HSV: H 0~180). 노란 차선은 더 밝아서 v_max 로 걸러진다.
    declare_parameter("brown_h_min", 5);
    declare_parameter("brown_h_max", 25);
    declare_parameter("brown_s_min", 70);
    declare_parameter("brown_v_min", 30);
    declare_parameter("brown_v_max", 200);
    declare_parameter("min_area", 0.01);            // 벽 최소 면적 (이미지 면적 비율)
    declare_parameter("debug_jpeg_quality", 70);

    const auto qos = rclcpp::SensorDataQoS();
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, qos, [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {onImage(msg);});
    wall_pub_ = create_publisher<interfaces::msg::WallObstacle>("wall_obstacle", qos);
    debug_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      "vision/wall_debug/compressed", qos);

    RCLCPP_INFO(get_logger(), "wall_detection 시작 (구독: %s)", image_topic.c_str());
  }

private:
  int intParam(const std::string & name) {return static_cast<int>(get_parameter(name).as_int());}

  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    cv::Mat frame;
    if (!toBgr(*msg, frame)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "지원하지 않는 영상 인코딩: %s", msg->encoding.c_str());
      return;
    }
    const int w = frame.cols, h = frame.rows;
    const int roi_y = std::clamp(static_cast<int>(h * get_parameter("roi_top").as_double()), 0, h - 2);
    const cv::Mat roi = frame(cv::Rect(0, roi_y, w, h - roi_y));

    // --- 갈색 마스크 ---
    cv::Mat hsv, mask;
    cv::GaussianBlur(roi, hsv, cv::Size(5, 5), 0);
    cv::cvtColor(hsv, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv,
      cv::Scalar(intParam("brown_h_min"), intParam("brown_s_min"), intParam("brown_v_min")),
      cv::Scalar(intParam("brown_h_max"), 255, intParam("brown_v_max")), mask);
    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    // --- 벽 후보 (좌표는 전체 이미지 기준) ---
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    const double min_area = get_parameter("min_area").as_double() * w * h;
    std::vector<cv::Rect> walls;
    for (const auto & c : contours) {
      if (cv::contourArea(c) >= min_area) {
        walls.push_back(cv::boundingRect(c) + cv::Point(0, roi_y));
      }
    }
    // 아래 끝이 가장 아래 = 가장 가까운 벽
    const auto nearest = std::max_element(walls.begin(), walls.end(),
        [](const cv::Rect & a, const cv::Rect & b) {return a.br().y < b.br().y;});

    interfaces::msg::WallObstacle wall;
    wall.header = msg->header;
    wall.side = interfaces::msg::WallObstacle::NONE;
    wall.detected = nearest != walls.end();
    double free_x = w / 2.0;
    if (wall.detected) {
      const cv::Rect & r = *nearest;
      const double cx = r.x + r.width / 2.0;
      const bool on_left = cx < w / 2.0;
      wall.side = on_left ? interfaces::msg::WallObstacle::LEFT : interfaces::msg::WallObstacle::RIGHT;
      // 벽 반대편 (벽 가장자리 ~ 화면 끝) 빈 공간의 중심을 회피 목표로
      free_x = on_left ? (r.br().x + w) / 2.0 : r.x / 2.0;
      wall.x_ratio = static_cast<float>((cx - w / 2.0) / (w / 2.0));
      wall.bottom_ratio = static_cast<float>(static_cast<double>(r.br().y) / h);
      wall.free_offset = static_cast<float>(std::clamp((free_x - w / 2.0) / (w / 2.0), -1.0, 1.0));
      wall.area_ratio = static_cast<float>(static_cast<double>(r.area()) / (w * h));
      wall.confidence = static_cast<float>(std::min(1.0, wall.area_ratio / 0.05));
    }
    wall_pub_->publish(wall);

    // --- 디버그 화면 ---
    if (debug_pub_->get_subscription_count() == 0) {
      return;
    }
    cv::Mat dbg = frame.clone();
    cv::Mat tint(roi.size(), frame.type(), cv::Scalar(0, 255, 255));
    cv::Mat dbg_roi = dbg(cv::Rect(0, roi_y, w, h - roi_y));
    cv::Mat blended;
    cv::addWeighted(dbg_roi, 0.6, tint, 0.4, 0, blended);
    blended.copyTo(dbg_roi, mask);
    cv::rectangle(dbg, cv::Rect(0, roi_y, w, h - roi_y), cv::Scalar(255, 200, 0), 1);
    for (auto it = walls.begin(); it != walls.end(); ++it) {
      cv::rectangle(dbg, *it, it == nearest ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 200, 255),
        it == nearest ? 3 : 1);
    }
    if (wall.detected) {
      cv::arrowedLine(dbg, cv::Point(w / 2, h - 1), cv::Point(cvRound(free_x), nearest->br().y),
        cv::Scalar(255, 0, 255), 3, cv::LINE_8, 0, 0.15);
    }
    const char * side = wall.side == interfaces::msg::WallObstacle::LEFT ? "LEFT" :
      wall.side == interfaces::msg::WallObstacle::RIGHT ? "RIGHT" : "-";
    char text[96];
    std::snprintf(text, sizeof(text), "wall %s  near %.2f  free %+.2f", side, wall.bottom_ratio,
      wall.free_offset);
    cv::putText(dbg, text, cv::Point(8, 22), cv::FONT_HERSHEY_SIMPLEX, 0.6,
      cv::Scalar(0, 255, 255), 2);
    publishDebug(dbg, msg->header);
  }

  void publishDebug(const cv::Mat & img, const std_msgs::msg::Header & header)
  {
    sensor_msgs::msg::CompressedImage out;
    out.header = header;
    out.format = "jpeg";
    const std::vector<int> opts{cv::IMWRITE_JPEG_QUALITY, intParam("debug_jpeg_quality")};
    cv::imencode(".jpg", img, out.data, opts);
    debug_pub_->publish(out);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<interfaces::msg::WallObstacle>::SharedPtr wall_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debug_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WallDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
