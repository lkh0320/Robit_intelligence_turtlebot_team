#ifndef TURTLE_GUI_QNODE_HPP
#define TURTLE_GUI_QNODE_HPP

#include <map>
#include <memory>
#include <string>

#include <QImage>
#include <QStringList>
#include <QThread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <turtle_interfaces/msg/motor_state.hpp>
#include <turtle_interfaces/msg/psd_array.hpp>
#include <turtle_interfaces/msg/robot_state.hpp>
#include <turtle_interfaces/msg/vision_result.hpp>
#include <turtle_interfaces/srv/set_mode.hpp>

#include "turtle_gui/gui_types.hpp"

// ROS 2 통신 전담. 별도 스레드에서 spin 하고, 받은 데이터는 signal 로 MainWindow 에 전달
//
// 구조 한눈에 보기
//   [ROS 2 토픽/서비스] <--> QNode (이 클래스, 별도 스레드) --signal--> MainWindow (화면, 메인 스레드)
//                                   <--함수 호출-- (버튼/키 입력)
//
//  - 받기: 구독 콜백에서 ROS 메시지를 gui_types.hpp 구조체로 바꿔 emit → MainWindow 의 slot 이 화면 갱신
//  - 보내기: MainWindow 가 publishManualCmd(), callRun() 같은 public 함수를 직접 호출
//  - 왜 스레드를 나눌까? ROS spin 이 화면(Qt 이벤트 루프)을 멈추게 하지 않으려고
class QNode : public QThread
{
  Q_OBJECT

public:
  QNode();                                                           // 구독/발행/서비스 클라이언트 생성 후 스레드 시작
  ~QNode() override;                                                 // 스레드 종료를 기다림

  void setImageSource(int source);                                   // 표시할 영상 토픽 변경
  void publishManualCmd(double linear, double angular);              // /cmd_vel_manual 발행
  void callRun(bool run);                                            // /robot/run
  void callEstop(bool on);                                           // /robot/estop
  void callSetMode(int mode);                                        // /robot/set_mode
  QStringList nodeNames();                                           // 현재 실행 중인 노드 목록
  void loadParameters(const QString & node_name);                    // 노드 파라미터 전체 읽기
  void setParameters(const QString & node_name, const QVector<ParamData> & params);

// ---- MainWindow 로 보내는 알림 (MainWindow 생성자에서 connect 됨) ----
signals:
  void imageReceived(const QImage & image);                          // JPEG 를 디코딩한 한 프레임
  void psdReceived(const PsdData & data);
  void motorReceived(const MotorData & data);
  void robotStateReceived(const RobotStateData & data);
  void visionReceived(const VisionData & data);
  void parametersLoaded(const QString & node_name, const QVector<ParamData> & params);
  void parametersApplied(const QString & node_name);
  void logMessage(const QString & text);                             // 로그 창에 표시할 문장
  void rosShutdown();                                                // Ctrl+C 등으로 ROS 가 종료됨 → 창 닫기

protected:
  void run() override;                                               // QThread 가 새 스레드에서 실행하는 함수 (spin 루프)

private:
  void callSetBool(rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client, const QString & name, bool value);  // run/estop 공통
  std::shared_ptr<rclcpp::AsyncParametersClient> paramClient(const std::string & node_name);  // 노드별로 한 번만 만들고 재사용

  rclcpp::Node::SharedPtr node_;                                     // GUI 의 ROS 노드 이름: /turtle_gui

  // 구독 (로봇 → GUI)

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
  rclcpp::Subscription<turtle_interfaces::msg::PsdArray>::SharedPtr psd_sub_;
  rclcpp::Subscription<turtle_interfaces::msg::MotorState>::SharedPtr motor_sub_;
  rclcpp::Subscription<turtle_interfaces::msg::RobotState>::SharedPtr state_sub_;
  rclcpp::Subscription<turtle_interfaces::msg::VisionResult>::SharedPtr vision_sub_;

  // 발행 (GUI → 로봇)
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;  // /cmd_vel_manual

  // 서비스 클라이언트 (GUI → control_node)
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr run_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr estop_client_;
  rclcpp::Client<turtle_interfaces::srv::SetMode>::SharedPtr mode_client_;

  std::map<std::string, std::shared_ptr<rclcpp::AsyncParametersClient>> param_clients_;  // 키: 노드 이름
};

#endif
