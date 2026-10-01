// 표지판용 객체 인식
//   구독: image_raw (sensor_msgs/Image)
//   발행: sign (interfaces/Sign)
//         vision/object_debug/compressed (GUI 객체 인식 화면)
// TODO: 구현
// 지금은 노드만 뜨고 아무것도 구독/발행하지 않는 빈 껍데기다 (launch 구성을 미리 맞춰 두기 위함).
// 구현할 때 lane_detection_node.cpp 처럼 생성자에서 구독/발행을 만들고 이미지 콜백에서 처리하면 된다.
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
