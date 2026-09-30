// 장애물 인식: 벽 (지그재그 회피 구간)
//   구독: image_raw (sensor_msgs/Image)
//   발행: wall_obstacle (interfaces/WallObstacle)
//         vision/wall_debug/compressed
// TODO: 구현
#include <memory>

#include "rclcpp/rclcpp.hpp"

class WallDetectionNode : public rclcpp::Node
{
public:
  WallDetectionNode()
  : Node("wall_detection")
  {
    RCLCPP_INFO(get_logger(), "wall_detection 시작");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WallDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
