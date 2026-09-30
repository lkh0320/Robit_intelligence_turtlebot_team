// 장애물 인식: 차단바
//   구독: image_raw (sensor_msgs/Image)
//   발행: barrier (interfaces/Barrier)
//         vision/barrier_debug/compressed
// TODO: 구현
#include <memory>

#include "rclcpp/rclcpp.hpp"

class BarrierDetectionNode : public rclcpp::Node
{
public:
  BarrierDetectionNode()
  : Node("barrier_detection")
  {
    RCLCPP_INFO(get_logger(), "barrier_detection 시작");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BarrierDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
