// 주행용 선 검출
//   구독: image_raw (sensor_msgs/Image)
//   발행: lane_info (interfaces/LaneInfo)
//         stop_line (interfaces/StopLine)
//         vision/lane_debug/compressed (GUI 선·벡터 검출 화면)
// TODO: 구현
#include <memory>

#include "rclcpp/rclcpp.hpp"

class LaneDetectionNode : public rclcpp::Node
{
public:
  LaneDetectionNode()
  : Node("lane_detection")
  {
    RCLCPP_INFO(get_logger(), "lane_detection 시작");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaneDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
