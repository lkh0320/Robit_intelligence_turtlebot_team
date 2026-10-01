// 노트북용 모니터링/조종 GUI
//   수신: 카메라 4종(원본 / Bird's Eye View / 선·벡터 검출 / 객체 인식), psd, dxl_state, cmd_vel, control_mode, 비전 인식 결과 6종
//   송신: cmd_vel (수동 주행), control_mode (모드 강제 변경)
// ROS 콜백은 QTimer에서 spin_some으로 GUI 스레드에서 처리한다 (스레드 동기화 불필요).
//   Qt 위젯은 GUI 스레드에서만 건드려야 하는데, ROS 콜백도 같은 스레드에서 돌기 때문에
//   콜백 안에서 바로 라벨/화면을 바꿔도 안전하다.
#ifndef TURTLE_GUI__MAIN_WINDOW_HPP_
#define TURTLE_GUI__MAIN_WINDOW_HPP_

#include <array>
#include <map>
#include <set>
#include <string>
#include <vector>

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

// Qt 클래스 전방 선언: 헤더에서는 포인터만 쓰므로 실제 include 는 main_window.cpp 에서 한다
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
    QLabel * lamp = nullptr;   // "● 30 Hz" 표시 라벨 (초록 = 수신 중, 빨강 = 끊김)
    QElapsedTimer last;        // 마지막 수신 후 지난 시간
    int count = 0;             // 최근 1초 동안 받은 개수 -> Hz
  };

  // 카메라 화면 1개 (토픽이 /compressed 로 끝나면 CompressedImage, 아니면 Image 로 구독)
  struct CameraView
  {
    QString key;                 // stats_ 에서 쓰는 이름 (cam_raw 등)
    QLabel * image = nullptr;    // 영상을 그리는 라벨
    QLabel * info = nullptr;     // 아래 "640 x 480  30 fps  토픽" 글자
    QString topic;
    QElapsedTimer fps_timer;     // fps 계산용
    int frames = 0;
    double fps = 0.0;
    rclcpp::SubscriptionBase::SharedPtr sub;   // 메시지 타입이 둘 중 하나라 공통 부모 타입으로 보관
  };

  // 화면 만들기
  QWidget * buildStatusPanel();
  QWidget * buildControlPanel();
  QWidget * buildCameraView(CameraView & view, const QString & title);
  void addTopicRow(QGridLayout * grid, int row, const QString & key, const QString & topic);
  // ROS 구독/발행 준비, 카메라 프레임 처리
  void setupRos();
  void subscribeCamera(CameraView & view);
  void onFrame(CameraView & view, const QImage & image);

  // 수신 기록 / 램프 갱신 / 송신 / 로그
  void touch(const QString & key);
  void refreshLamps();
  void sendManualCmd();
  void sendControlMode(uint8_t mode);
  void log(const QString & text);

  rclcpp::Node::SharedPtr node_;
  double wheel_separation_;   // cmd_vel -> 바퀴 목표속도 표시용 (stm_bridge 와 같은 값이어야 함)
  double psd_max_range_;      // PSD 막대 최대값 [m]

  // --- 위젯 ---
  std::array<CameraView, 4> cams_;   // 0 원본, 1 BEV, 2 선 검출, 3 객체 인식
  QProgressBar * psd_bar_[3];        // 0 왼쪽, 1 앞, 2 오른쪽
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

  std::map<QString, TopicStat> stats_;   // key -> 수신 상태
  std::set<int> pressed_keys_;           // 지금 누르고 있는 주행키 (W/A/S/D)
  bool was_moving_ = false;              // 직전 주기에 움직이는 명령을 보냈는지 (정지 명령 1회 송신용)

  QTimer * spin_timer_;   // 5ms: ROS 콜백 처리
  QTimer * cmd_timer_;    // 100ms: 수동 주행 cmd_vel 송신
  QTimer * lamp_timer_;   // 1s: 램프 색/Hz 갱신

  // --- ROS ---
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
