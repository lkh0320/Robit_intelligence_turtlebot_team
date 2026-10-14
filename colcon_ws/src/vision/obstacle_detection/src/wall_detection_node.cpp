// 장애물 인식: 공사 구간 갈색 박스(벽) + 빈 공간 방향
//   구독: image_raw (sensor_msgs/Image, 원본 카메라)
//   발행: wall_obstacle (interfaces/WallObstacle)
//           detected     박스가 보이나 (box_min_area 넘는 덩어리)
//           side         가장 가까운 박스가 화면 왼쪽(LEFT) / 오른쪽(RIGHT)
//           x_ratio      가장 가까운 박스 가로 중심 [-1, 1] (+ 오른쪽)
//           bottom_ratio 가장 가까운 박스 아래 끝 [0 위, 1 아래], 클수록 가까움
//           area_ratio   박스 덩어리 면적 합 / 화면 면적
//           free_offset  박스 / 선 없이 가장 멀리 비어 있는 방향 [-1, 1] (+ 오른쪽) -> lane_follower 공사 구간 조향
//           confidence   고른 방향이 얼마나 멀리 비었나 [0, 1] (1 = 화면 맨 위까지)
//           distance     0 (미터 보정 없음)
//         vision/wall_debug/compressed (보는 사람이 있을 때만)
//   계산은 include/obstacle_detection/free_space.hpp. 원본 30Hz 를 process_rate 로 솎아서 처리
//   BEV 대신 원본을 쓰는 이유: BEV 는 앞 18~58cm, 좌우 ±20cm 만 보여 박스를 늦게 본다 (원본은 1.2m 넘게)
#include <cstdio>
#include <memory>
#include <string>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "cv_bridge/cv_bridge.hpp"
#include "interfaces/msg/wall_obstacle.hpp"
#include "obstacle_detection/free_space.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

class WallDetectionNode : public rclcpp::Node
{
public:
  WallDetectionNode()
  : Node("wall_detection")
  {
    const obstacle_detection::FreeSpaceParams d;
    const auto image_topic = declare_parameter("image_topic", std::string("image_raw"));
    declare_parameter("process_rate", 10.0);   // [Hz] 0 이하 = 모든 프레임
    declare_parameter("work_width", d.work_width);
    declare_parameter("work_height", d.work_height);
    declare_parameter("box_l_min", d.box_l_min);
    declare_parameter("box_l_max", d.box_l_max);
    declare_parameter("box_a_min", d.box_a_min);
    declare_parameter("box_a_max", d.box_a_max);
    declare_parameter("box_b_min", d.box_b_min);
    declare_parameter("box_b_max", d.box_b_max);
    declare_parameter("box_min_area", d.box_min_area);
    declare_parameter("yellow_l_min", d.yellow_l_min);
    declare_parameter("yellow_a_max", d.yellow_a_max);
    declare_parameter("yellow_b_min", d.yellow_b_min);
    declare_parameter("white_l_min", d.white_l_min);
    declare_parameter("white_ab_dev", d.white_ab_dev);
    declare_parameter("robot_width_px", d.robot_width_px);
    declare_parameter("center_bias", d.center_bias);
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
  int intParam(const std::string & n) {return static_cast<int>(get_parameter(n).as_int());}

  obstacle_detection::FreeSpaceParams readParams()
  {
    obstacle_detection::FreeSpaceParams p;
    p.work_width = intParam("work_width");
    p.work_height = intParam("work_height");
    p.box_l_min = intParam("box_l_min");
    p.box_l_max = intParam("box_l_max");
    p.box_a_min = intParam("box_a_min");
    p.box_a_max = intParam("box_a_max");
    p.box_b_min = intParam("box_b_min");
    p.box_b_max = intParam("box_b_max");
    p.box_min_area = intParam("box_min_area");
    p.yellow_l_min = intParam("yellow_l_min");
    p.yellow_a_max = intParam("yellow_a_max");
    p.yellow_b_min = intParam("yellow_b_min");
    p.white_l_min = intParam("white_l_min");
    p.white_ab_dev = intParam("white_ab_dev");
    p.robot_width_px = intParam("robot_width_px");
    p.center_bias = get_parameter("center_bias").as_double();
    return p;
  }

  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    // 이미지 시각으로 솎는다 (bag 재생 속도를 바꿔도 같은 프레임을 고르게)
    const rclcpp::Time stamp(msg->header.stamp);
    const double rate = get_parameter("process_rate").as_double();
    if (rate > 0 && have_last_ && (stamp - last_).seconds() < 0.9 / rate &&
      (stamp - last_).seconds() >= 0)
    {
      return;
    }
    cv::Mat bgr;
    try {
      bgr = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "cv_bridge: %s", e.what());
      return;
    }
    last_ = stamp;
    have_last_ = true;

    const auto r = obstacle_detection::findFreeSpace(bgr, readParams());
    const double w = r.blocked.cols, h = r.blocked.rows;
    interfaces::msg::WallObstacle out;
    out.header = msg->header;
    out.detected = r.box_found;
    out.free_offset = static_cast<float>(r.offset());
    out.confidence = static_cast<float>(r.clearance());
    out.area_ratio = static_cast<float>(r.box_pixels / (w * h));
    if (r.box_found) {
      const auto & b = r.nearest.rect;
      out.x_ratio = static_cast<float>((b.x + b.width / 2.0) / (w / 2) - 1.0);
      out.bottom_ratio = static_cast<float>(b.br().y / h);
      out.side = out.x_ratio < 0 ? interfaces::msg::WallObstacle::LEFT :
        interfaces::msg::WallObstacle::RIGHT;
    }
    wall_pub_->publish(out);

    if (debug_pub_->get_subscription_count() > 0) {
      publishDebug(r, out);
    }
  }

  // 막힘 = 빨강, 열마다 빈 높이 = 초록 막대, 고른 통로 = 노란 선, 가장 가까운 박스 = 하늘색 상자
  void publishDebug(const obstacle_detection::FreeSpaceResult & r,
    const interfaces::msg::WallObstacle & o)
  {
    cv::Mat vis = r.small.clone();
    const int h = vis.rows;
    for (int x = 0; x < vis.cols; ++x) {
      cv::line(vis, {x, h - 1}, {x, h - 1 - static_cast<int>(r.free[x])}, cv::Scalar(0, 90, 0), 1);
    }
    cv::addWeighted(vis, 0.6, r.small, 0.4, 0, vis);
    vis.setTo(cv::Scalar(0, 0, 200), r.blocked);
    vis.setTo(cv::Scalar(0, 0, 120), r.box);
    if (r.box_found) {
      cv::rectangle(vis, r.nearest.rect, cv::Scalar(255, 255, 0), 1);
    }
    cv::line(vis, {vis.cols / 2, h - 1}, {r.column, h - 1 - static_cast<int>(r.corridor[r.column])},
      cv::Scalar(0, 255, 255), 2);
    char text[96];
    std::snprintf(text, sizeof(text), "free %+.2f clr %.2f box %s %.2f", o.free_offset, o.confidence,
      !o.detected ? "-" : o.side == interfaces::msg::WallObstacle::LEFT ? "L" : "R", o.bottom_ratio);
    cv::putText(vis, text, {4, 14}, cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(0, 255, 255), 1);

    sensor_msgs::msg::CompressedImage jpeg;
    jpeg.header = o.header;
    jpeg.format = "jpeg";
    cv::imencode(".jpg", vis, jpeg.data,
      {cv::IMWRITE_JPEG_QUALITY, intParam("debug_jpeg_quality")});
    debug_pub_->publish(jpeg);
  }

  rclcpp::Time last_;
  bool have_last_ = false;
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
