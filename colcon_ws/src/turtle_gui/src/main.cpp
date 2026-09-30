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

  const int ret = app.exec();
  rclcpp::shutdown();
  return ret;
}
