#include "turtle_gui/qnode.hpp"

#include <chrono>
#include <vector>

namespace
{

QStringList splitArray(const QString & text)                          // "[1, 2, 3]" -> {"1","2","3"}
{
  QString body = text.trimmed();
  if (body.startsWith('[')) body.remove(0, 1);
  if (body.endsWith(']')) body.chop(1);
  QStringList items;
  for (const QString & item : body.split(',')) {
    if (!item.trimmed().isEmpty()) items << item.trimmed();
  }
  return items;
}

bool toRosParameter(const ParamData & p, rclcpp::Parameter & out)     // 문자열 값을 원래 타입으로 변환
{
  const std::string name = p.name.toStdString();
  const QString text = p.value.trimmed();
  bool ok = true;

  switch (static_cast<rclcpp::ParameterType>(p.type)) {
    case rclcpp::ParameterType::PARAMETER_BOOL: {
      const QString t = text.toLower();
      if (t == "true" || t == "1") out = rclcpp::Parameter(name, true);
      else if (t == "false" || t == "0") out = rclcpp::Parameter(name, false);
      else ok = false;
      break;
    }
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      out = rclcpp::Parameter(name, static_cast<int64_t>(text.toLongLong(&ok)));
      break;
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      out = rclcpp::Parameter(name, text.toDouble(&ok));
      break;
    case rclcpp::ParameterType::PARAMETER_STRING:
      out = rclcpp::Parameter(name, text.toStdString());
      break;
    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY: {
      std::vector<int64_t> values;
      for (const QString & item : splitArray(text)) {
        values.push_back(item.toLongLong(&ok));
        if (!ok) break;
      }
      out = rclcpp::Parameter(name, values);
      break;
    }
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY: {
      std::vector<double> values;
      for (const QString & item : splitArray(text)) {
        values.push_back(item.toDouble(&ok));
        if (!ok) break;
      }
      out = rclcpp::Parameter(name, values);
      break;
    }
    default:
      ok = false;                                                     // 그 외 타입은 GUI에서 수정 안 함
  }
  return ok;
}

}  // namespace

QNode::QNode()
{
  qRegisterMetaType<PsdData>();
  qRegisterMetaType<MotorData>();
  qRegisterMetaType<RobotStateData>();
  qRegisterMetaType<VisionData>();
  qRegisterMetaType<ParamData>();
  qRegisterMetaType<QVector<ParamData>>();

  node_ = rclcpp::Node::make_shared("turtle_gui");
  const auto qos = rclcpp::SensorDataQoS();                           // best effort: 어떤 발행 QoS와도 연결됨

  psd_sub_ = node_->create_subscription<turtle_interfaces::msg::PsdArray>(
    "/sensor/psd", qos,
    [this](const turtle_interfaces::msg::PsdArray::SharedPtr msg) {
      PsdData d;
      for (int i = 0; i < 3; i++) {
        d.raw[i] = msg->raw[i];
        d.mm[i] = msg->distance_mm[i];
      }
      emit psdReceived(d);
    });

  motor_sub_ = node_->create_subscription<turtle_interfaces::msg::MotorState>(
    "/motor/state", qos,
    [this](const turtle_interfaces::msg::MotorState::SharedPtr msg) {
      emit motorReceived({msg->left_mps, msg->right_mps, msg->left_error, msg->right_error});
    });

  state_sub_ = node_->create_subscription<turtle_interfaces::msg::RobotState>(
    "/robot/state", qos,
    [this](const turtle_interfaces::msg::RobotState::SharedPtr msg) {
      emit robotStateReceived({msg->mode, msg->running, msg->estop, msg->stm32_connected,
                               msg->mcu_error_flags, QString::fromStdString(msg->message)});
    });

  vision_sub_ = node_->create_subscription<turtle_interfaces::msg::VisionResult>(
    "/vision/result", qos,
    [this](const turtle_interfaces::msg::VisionResult::SharedPtr msg) {
      emit visionReceived({msg->detected, msg->offset, QString::fromStdString(msg->label)});
    });

  cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_manual", 10);

  run_client_ = node_->create_client<std_srvs::srv::SetBool>("/robot/run");
  estop_client_ = node_->create_client<std_srvs::srv::SetBool>("/robot/estop");
  mode_client_ = node_->create_client<turtle_interfaces::srv::SetMode>("/robot/set_mode");

  setImageSource(IMAGE_CAMERA);
  start();                                                            // run() 스레드 시작
}

QNode::~QNode()
{
  requestInterruption();
  wait();
}

void QNode::run()
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);
  while (rclcpp::ok() && !isInterruptionRequested()) {
    executor.spin_once(std::chrono::milliseconds(50));
  }
  executor.remove_node(node_);
  if (!rclcpp::ok()) emit rosShutdown();                              // Ctrl+C 등으로 ROS 종료 시 창 닫기
}

void QNode::setImageSource(int source)
{
  const std::string topic = (source == IMAGE_VISION) ? "/vision/debug_image/compressed"
                                                     : "/camera/image_raw/compressed";
  image_sub_ = node_->create_subscription<sensor_msgs::msg::CompressedImage>(
    topic, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
      QImage image;
      if (image.loadFromData(msg->data.data(), static_cast<int>(msg->data.size()))) {
        emit imageReceived(image);                                    // JPEG/PNG 디코딩 (OpenCV 불필요)
      }
    });
  emit logMessage(QString("영상 토픽: %1").arg(QString::fromStdString(topic)));
}

void QNode::publishManualCmd(double linear, double angular)
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x = linear;
  msg.angular.z = angular;
  cmd_pub_->publish(msg);
}

void QNode::callSetBool(rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client, const QString & name, bool value)
{
  if (!client->service_is_ready()) {
    emit logMessage(QString("%1 서비스 없음 (control_node 실행 중인지 확인)").arg(name));
    return;
  }
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = value;
  client->async_send_request(
    request,
    [this, name, value](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
      const auto response = future.get();
      emit logMessage(QString("%1(%2) → %3 %4")
                        .arg(name, QString(value ? "true" : "false"), QString(response->success ? "성공" : "실패"),
                             QString::fromStdString(response->message)));
    });
}

void QNode::callRun(bool run)
{
  callSetBool(run_client_, "/robot/run", run);
}

void QNode::callEstop(bool on)
{
  callSetBool(estop_client_, "/robot/estop", on);
}

void QNode::callSetMode(int mode)
{
  if (!mode_client_->service_is_ready()) {
    emit logMessage("/robot/set_mode 서비스 없음 (control_node 실행 중인지 확인)");
    return;
  }
  auto request = std::make_shared<turtle_interfaces::srv::SetMode::Request>();
  request->mode = static_cast<uint8_t>(mode);
  mode_client_->async_send_request(
    request,
    [this, mode](rclcpp::Client<turtle_interfaces::srv::SetMode>::SharedFuture future) {
      const auto response = future.get();
      emit logMessage(QString("모드 변경(%1) → %2 %3")
                        .arg(QString(mode == 0 ? "수동" : "자율"), QString(response->success ? "성공" : "실패"),
                             QString::fromStdString(response->message)));
    });
}

QStringList QNode::nodeNames()
{
  QStringList names;
  for (const std::string & name : node_->get_node_names()) {
    const QString n = QString::fromStdString(name);
    if (n == "/turtle_gui" || n.startsWith("/_ros2cli")) continue;   // 자기 자신, ros2 CLI 데몬 제외
    if (!names.contains(n)) names << n;
  }
  names.sort();
  return names;
}

std::shared_ptr<rclcpp::AsyncParametersClient> QNode::paramClient(const std::string & node_name)
{
  auto it = param_clients_.find(node_name);
  if (it != param_clients_.end()) return it->second;
  auto client = std::make_shared<rclcpp::AsyncParametersClient>(node_, node_name);
  param_clients_[node_name] = client;
  return client;
}

void QNode::loadParameters(const QString & node_name)
{
  auto client = paramClient(node_name.toStdString());
  if (!client->service_is_ready()) {
    emit logMessage(QString("%1 파라미터 서비스 없음 (노드 이름, 실행 여부 확인)").arg(node_name));
    return;
  }

  client->list_parameters(
    std::vector<std::string>{}, 0,
    [this, client, node_name](std::shared_future<rcl_interfaces::msg::ListParametersResult> list_future) {
      client->get_parameters(
        list_future.get().names,
        [this, node_name](std::shared_future<std::vector<rclcpp::Parameter>> get_future) {
          QVector<ParamData> params;
          for (const rclcpp::Parameter & p : get_future.get()) {
            const QString name = QString::fromStdString(p.get_name());
            if (name == "use_sim_time" || name.startsWith("qos_overrides")) continue;  // 공통 파라미터 숨김
            params.push_back({name, static_cast<int>(p.get_type()),
                              QString::fromStdString(p.get_type_name()),
                              QString::fromStdString(p.value_to_string())});
          }
          emit parametersLoaded(node_name, params);
        });
    });
}

void QNode::setParameters(const QString & node_name, const QVector<ParamData> & params)
{
  std::vector<rclcpp::Parameter> ros_params;
  for (const ParamData & p : params) {
    rclcpp::Parameter rp;
    if (!toRosParameter(p, rp)) {
      emit logMessage(QString("%1 값 형식 오류: \"%2\" (%3)").arg(p.name, p.value, p.type_name));
      return;
    }
    ros_params.push_back(rp);
  }
  if (ros_params.empty()) return;

  auto client = paramClient(node_name.toStdString());
  if (!client->service_is_ready()) {
    emit logMessage(QString("%1 파라미터 서비스 없음").arg(node_name));
    return;
  }

  client->set_parameters(
    ros_params,
    [this, node_name, ros_params](
      std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
      const auto results = future.get();
      for (size_t i = 0; i < results.size() && i < ros_params.size(); i++) {
        const QString name = QString::fromStdString(ros_params[i].get_name());
        if (results[i].successful) {
          emit logMessage(QString("%1 %2 = %3 적용")
                            .arg(node_name, name, QString::fromStdString(ros_params[i].value_to_string())));
        } else {
          emit logMessage(QString("%1 %2 거부: %3")
                            .arg(node_name, name, QString::fromStdString(results[i].reason)));
        }
      }
      emit parametersApplied(node_name);                              // 실제 값 다시 읽기용
    });
}
