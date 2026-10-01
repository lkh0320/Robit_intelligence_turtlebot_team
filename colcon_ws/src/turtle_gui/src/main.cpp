// turtle_gui 실행 진입점
//   ros2 run turtle_gui turtle_gui
// ROS 노드 하나를 만들어 MainWindow 에 넘기고 Qt 이벤트 루프를 돌린다.
// ROS 콜백 처리(spin)는 MainWindow 안의 QTimer 가 맡는다 (main_window.hpp 참고).
#include <memory>

#include <QApplication>

#include "rclcpp/rclcpp.hpp"
#include "turtle_gui/main_window.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("turtle_gui");
  MainWindow window(node);
  window.show();

  const int ret = app.exec();   // 창이 닫힐 때까지 여기서 대기
  rclcpp::shutdown();
  return ret;
}
