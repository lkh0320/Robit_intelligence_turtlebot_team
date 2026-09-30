// 노트북용 모니터링/조종 GUI
//   수신: 카메라, psd, dxl_state, cmd_vel, control_mode, 비전 인식 결과 6종
//   송신: cmd_vel (수동 주행), control_mode (모드 강제 변경)
// ROS 콜백은 QTimer에서 spin_some으로 GUI 스레드에서 처리한다 (스레드 동기화 불필요).
#ifndef TURTLE_GUI__MAIN_WINDOW_HPP_
#define TURTLE_GUI__MAIN_WINDOW_HPP_

#include <map>
#include <set>
#include <string>

#include <QElapsedTimer>
#include <QMainWindow>

#include "geometry_msgs/msg/twist.hpp"
#include "interfaces/msg/barrier.hpp"
#include "interfaces/msg/control_mode.hpp"
#include "interfaces/msg/dxl_state.hpp"
#include "interfaces/msg/lane_info.hpp"
#include "interfaces/msg/parking_spot.hpp"
#include "interfaces/msg/psd_array.hpp"
#include "interfaces/msg/sign.hpp"
#include "interfaces/msg/stop_line.hpp"
#include "interfaces/msg/traffic_light.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

class QCheckBox;
class QDoubleSpinBox;
class QGridLayout;
class QImage;
class QLabel;
class QPlainTextEdit;
class QProgressBar;
class QTimer;

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(rclcpp::Node::SharedPtr node, QWidget * parent = nullptr);

protected:
  // W/A/S/D/Space 는 어떤 위젯에 포커스가 있어도 주행키로 쓰기 위해 앱 전체에서 가로챈다
  bool eventFilter(QObject * obj, QEvent * event) override;

private:
  // 토픽별 수신 상태 (연결 램프 + Hz)
  struct TopicStat
  {
    QLabel * lamp = nullptr;
    QElapsedTimer last;
    int count = 0;
  };

  QWidget * buildStatusPanel();
  QWidget * buildControlPanel();
  void addTopicRow(QGridLayout * grid, int row, const QString & key, const QString & topic);
  void setupRos();

  void touch(const QString & key);
  void refreshLamps();
  void showImage(const QImage & image);
  void sendManualCmd();
  void sendControlMode(uint8_t mode);
  void log(const QString & text);

  rclcpp::Node::SharedPtr node_;
  double wheel_separation_;
  double psd_max_range_;

  // --- 위젯 ---
  QLabel * image_label_;
  QLabel * image_info_;
  QProgressBar * psd_bar_[3];
  QLabel * psd_text_[3];
  QLabel * dxl_label_;
  QLabel * cmd_label_;
  QLabel * mode_label_;
  QLabel * lane_label_;
  QLabel * light_label_;
  QLabel * stop_label_;
  QLabel * sign_label_;
  QLabel * barrier_label_;
  QLabel * parking_label_;
  QCheckBox * manual_enable_;
  QDoubleSpinBox * lin_speed_;
  QDoubleSpinBox * ang_speed_;
  QPlainTextEdit * log_view_;

  std::map<QString, TopicStat> stats_;
  std::set<int> pressed_keys_;
  bool was_moving_ = false;
  QElapsedTimer fps_timer_;
  int fps_frames_ = 0;
  double fps_ = 0.0;

  QTimer * spin_timer_;
  QTimer * cmd_timer_;
  QTimer * lamp_timer_;

  // --- ROS ---
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_sub_;
  rclcpp::Subscription<interfaces::msg::PsdArray>::SharedPtr psd_sub_;
  rclcpp::Subscription<interfaces::msg::DxlState>::SharedPtr dxl_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<interfaces::msg::ControlMode>::SharedPtr mode_sub_;
  rclcpp::Subscription<interfaces::msg::LaneInfo>::SharedPtr lane_sub_;
  rclcpp::Subscription<interfaces::msg::TrafficLight>::SharedPtr light_sub_;
  rclcpp::Subscription<interfaces::msg::StopLine>::SharedPtr stop_sub_;
  rclcpp::Subscription<interfaces::msg::Sign>::SharedPtr sign_sub_;
  rclcpp::Subscription<interfaces::msg::Barrier>::SharedPtr barrier_sub_;
  rclcpp::Subscription<interfaces::msg::ParkingSpot>::SharedPtr parking_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<interfaces::msg::ControlMode>::SharedPtr mode_pub_;
};

#endif  // TURTLE_GUI__MAIN_WINDOW_HPP_
