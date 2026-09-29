#include <QApplication>

#include <rclcpp/rclcpp.hpp>

#include "turtle_gui/main_window.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);

  int result = 0;
  {
    MainWindow window;                                                // 창이 닫히면 QNode 스레드도 여기서 정리됨
    window.show();
    result = app.exec();
  }

  if (rclcpp::ok()) rclcpp::shutdown();
  return result;
}
