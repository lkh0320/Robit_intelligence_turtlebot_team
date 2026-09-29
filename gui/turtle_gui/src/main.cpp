#include <QApplication>

#include <rclcpp/rclcpp.hpp>

#include "turtle_gui/main_window.hpp"

// 프로그램 시작점: ROS 2 초기화 → Qt 앱 생성 → 창 띄우기 → 창이 닫힐 때까지 대기
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);                                           // ROS 2 먼저 초기화 (QNode 가 노드를 만들기 때문)
  QApplication app(argc, argv);

  int result = 0;
  {
    MainWindow window;                                                // 창이 닫히면 QNode 스레드도 여기서 정리됨
    window.show();
    result = app.exec();                                              // Qt 이벤트 루프 (창을 닫으면 반환)
  }

  if (rclcpp::ok()) rclcpp::shutdown();
  return result;
}
