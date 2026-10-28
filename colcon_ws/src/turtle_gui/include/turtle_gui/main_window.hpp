// 노트북용 모니터링/조종 GUI
//   수신: 카메라 3종(원본 / grid_drive 계획 / 객체 인식), psd, dxl_state, cmd_vel, control_mode, 비전 인식 결과 3종
//   송신: control_mode STOP + cmd_vel 0 (정지), mission_start (자율주행 시작), cmd_vel (키보드 수동 주행)
//   파라미터: 오른쪽 아래 노드별 탭에서 실행 중인 노드 파라미터를 바로 바꾼다 (param_panel.hpp)
// ROS 콜백은 QTimer에서 spin_some으로 GUI 스레드에서 처리한다 (스레드 동기화 불필요).
//   Qt 위젯은 GUI 스레드에서만 건드려야 하는데, ROS 콜백도 같은 스레드에서 돌기 때문에
//   콜백 안에서 바로 라벨/화면을 바꿔도 안전하다.
#ifndef TURTLE_GUI__MAIN_WINDOW_HPP_
#define TURTLE_GUI__MAIN_WINDOW_HPP_

#include <array>
#include <map>
#include <set>
#include <string>

#include <QElapsedTimer>
#include <QMainWindow>

#include "geometry_msgs/msg/twist.hpp"
#include "interfaces/msg/barrier.hpp"
#include "interfaces/msg/control_mode.hpp"
#include "interfaces/msg/dxl_state.hpp"
#include "interfaces/msg/psd_array.hpp"
#include "interfaces/msg/sign.hpp"
#include "interfaces/msg/traffic_light.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/empty.hpp"

// Qt 클래스 전방 선언: 헤더에서는 포인터만 쓰므로 실제 include 는 main_window.cpp 에서 한다
class QCheckBox;
class QDoubleSpinBox;
class QHBoxLayout;
class QImage;
class QLabel;
class QPlainTextEdit;
class QTimer;
class ParamPanel;

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(rclcpp::Node::SharedPtr node, QWidget * parent = nullptr);

protected:
  // W/A/S/D/Space 는 글자 입력칸에 포커스가 없으면 주행키로 쓰기 위해 앱 전체에서 가로챈다
  bool eventFilter(QObject * obj, QEvent * event) override;

private:
  // 토픽별 수신 상태 (점 하나 + Hz 툴팁)
  struct TopicStat
  {
    QLabel * lamp = nullptr;   // "●" (초록 = 수신 중, 빨강 = 끊김, 회색 = 한 번도 안 옴)
    QString topic;             // 툴팁에 쓸 토픽 이름
    QElapsedTimer last;        // 마지막 수신 후 지난 시간
    int count = 0;             // 최근 1초 동안 받은 개수 -> Hz
  };

  // 카메라 화면 1개 (토픽이 /compressed 로 끝나면 CompressedImage, 아니면 Image 로 구독)
  struct CameraView
  {
    QString key;                 // stats_ 에서 쓰는 이름 (cam_raw 등)
    QLabel * image = nullptr;    // 영상을 그리는 라벨
    QLabel * info = nullptr;     // 제목 옆 "640x480  30fps" 글자
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
  void addLamp(QHBoxLayout * row, const QString & key, const QString & text, const QString & topic);
  // ROS 구독/발행 준비, 카메라 프레임 처리
  void setupRos();
  void subscribeCamera(CameraView & view);
  void onFrame(CameraView & view, const QImage & image);

  // 수신 기록 / 램프 갱신 / 송신 / 로그
  void touch(const QString & key);
  bool fresh(const QString & key) const;
  void refreshLamps();
  void refreshMotor();
  void sendManualCmd();
  void sendStop();
  void sendMissionStart();
  void log(const QString & text);

  rclcpp::Node::SharedPtr node_;
  double psd_warn_;           // PSD 가 이보다 가까우면 빨간 글씨 [m]
  double lin_speed_init_;     // 수동 주행 선속도 시작값 [m/s]
  double ang_speed_init_;     // 수동 주행 각속도 시작값 [rad/s]

  // --- 위젯 ---
  std::array<CameraView, 3> cams_;   // 0 grid_drive 계획, 1 원본, 2 객체 인식
  QLabel * mode_label_;      // 큰 글씨: 제어 모드 + 미션 상태
  QLabel * robot_label_;     // 배터리 / STM 상태 / 모터 이상
  QLabel * psd_value_[3];    // 0 왼쪽, 1 앞, 2 오른쪽
  QLabel * motor_label_;     // 바퀴 속도 + cmd_vel
  QLabel * light_chip_;
  QLabel * sign_chip_;
  QLabel * barrier_chip_;
  QCheckBox * manual_enable_;
  QDoubleSpinBox * lin_speed_;
  QDoubleSpinBox * ang_speed_;
  QPlainTextEdit * log_view_;
  ParamPanel * params_;

  std::map<QString, TopicStat> stats_;   // key -> 수신 상태
  std::set<int> pressed_keys_;           // 지금 누르고 있는 주행키 (W/A/S/D)
  bool was_moving_ = false;              // 직전 주기에 움직이는 명령을 보냈는지 (정지 명령 1회 송신용)
  QString wheel_text_;                   // 마지막 바퀴 속도 (모터 줄)
  QString cmd_text_;                     // 마지막 cmd_vel (모터 줄)

  QTimer * spin_timer_;   // 5ms: ROS 콜백 처리
  QTimer * cmd_timer_;    // 100ms: 수동 주행 cmd_vel 송신
  QTimer * lamp_timer_;   // 1s: 램프 색/Hz 갱신, 끊긴 인식 결과 지우기

  // --- ROS ---
  rclcpp::Subscription<interfaces::msg::PsdArray>::SharedPtr psd_sub_;
  rclcpp::Subscription<interfaces::msg::DxlState>::SharedPtr dxl_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<interfaces::msg::ControlMode>::SharedPtr mode_sub_;
  rclcpp::Subscription<interfaces::msg::TrafficLight>::SharedPtr light_sub_;
  rclcpp::Subscription<interfaces::msg::Sign>::SharedPtr sign_sub_;
  rclcpp::Subscription<interfaces::msg::Barrier>::SharedPtr barrier_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<interfaces::msg::ControlMode>::SharedPtr mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr start_pub_;
};

#endif  // TURTLE_GUI__MAIN_WINDOW_HPP_
