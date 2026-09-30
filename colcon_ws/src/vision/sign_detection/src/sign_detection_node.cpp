// 표지판용 객체 인식
//   구독: image_raw (sensor_msgs/Image)
//   발행: sign (interfaces/Sign)
//         vision/object_debug/compressed (GUI 객체 인식 화면)
// TODO: 구현
#include <memory>

#include "rclcpp/rclcpp.hpp"

class SignDetectionNode : public rclcpp::Node
{
public:
  SignDetectionNode()
  : Node("sign_detection")
  {
    RCLCPP_INFO(get_logger(), "sign_detection 시작");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SignDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
