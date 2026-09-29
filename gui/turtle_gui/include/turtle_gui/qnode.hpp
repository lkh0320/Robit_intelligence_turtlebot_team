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
class QNode : public QThread
{
  Q_OBJECT

public:
  QNode();
  ~QNode() override;

  void setImageSource(int source);                                   // 표시할 영상 토픽 변경
  void publishManualCmd(double linear, double angular);              // /cmd_vel_manual 발행
  void callRun(bool run);                                            // /robot/run
  void callEstop(bool on);                                           // /robot/estop
  void callSetMode(int mode);                                        // /robot/set_mode
  QStringList nodeNames();                                           // 현재 실행 중인 노드 목록
  void loadParameters(const QString & node_name);                    // 노드 파라미터 전체 읽기
  void setParameters(const QString & node_name, const QVector<ParamData> & params);

signals:
  void imageReceived(const QImage & image);
  void psdReceived(const PsdData & data);
  void motorReceived(const MotorData & data);
  void robotStateReceived(const RobotStateData & data);
  void visionReceived(const VisionData & data);
  void parametersLoaded(const QString & node_name, const QVector<ParamData> & params);
  void parametersApplied(const QString & node_name);
  void logMessage(const QString & text);
  void rosShutdown();

protected:
  void run() override;

private:
  void callSetBool(rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client, const QString & name, bool value);
  std::shared_ptr<rclcpp::AsyncParametersClient> paramClient(const std::string & node_name);

  rclcpp::Node::SharedPtr node_;

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
  rclcpp::Subscription<turtle_interfaces::msg::PsdArray>::SharedPtr psd_sub_;
  rclcpp::Subscription<turtle_interfaces::msg::MotorState>::SharedPtr motor_sub_;
  rclcpp::Subscription<turtle_interfaces::msg::RobotState>::SharedPtr state_sub_;
  rclcpp::Subscription<turtle_interfaces::msg::VisionResult>::SharedPtr vision_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr run_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr estop_client_;
  rclcpp::Client<turtle_interfaces::srv::SetMode>::SharedPtr mode_client_;

  std::map<std::string, std::shared_ptr<rclcpp::AsyncParametersClient>> param_clients_;
};

#endif
