// STM32 시리얼 브릿지
//   sub: cmd_vel (geometry_msgs/Twist)  -> 차동구동 역기구학 -> WHEEL_CMD 프레임
//   pub: psd     (interfaces/PsdArray)  <- PSD 프레임
// 프로토콜은 include/stm/protocol.hpp 참고.
//
// 동작 요약
//   - 5ms 마다 시리얼을 읽어 PSD 프레임이 오면 psd 토픽으로 발행
//   - tx_rate(20Hz) 마다 마지막 cmd_vel 을 바퀴 속도로 바꿔 STM32 로 송신
//     cmd_vel 이 cmd_timeout 동안 안 오면 0 을 보내 로봇을 세운다 (상위 노드가 죽어도 안전)
//   - 시리얼이 끊기면 1초마다 다시 연결 시도 (USB 를 뽑았다 꽂아도 노드 재시작 불필요)
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "interfaces/msg/psd_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "stm/protocol.hpp"
#include "stm/serial_port.hpp"

using namespace std::chrono_literals;
namespace proto = stm::protocol;

class StmBridgeNode : public rclcpp::Node
{
public:
  StmBridgeNode()
  : Node("stm_bridge")
  {
    // 파라미터 (값은 config/stm_bridge.yaml)
    port_ = declare_parameter("port", "/dev/ttyUSB0");
    baud_ = declare_parameter("baud", 115200);
    wheel_separation_ = declare_parameter("wheel_separation", 0.160);
    max_wheel_speed_ = declare_parameter("max_wheel_speed", 0.26);
    cmd_timeout_ = declare_parameter("cmd_timeout", 0.5);
    const double tx_rate = declare_parameter("tx_rate", 20.0);
    psd_frame_id_ = declare_parameter("psd_frame_id", "base_link");

    psd_pub_ = create_publisher<interfaces::msg::PsdArray>("psd", rclcpp::SensorDataQoS());
    // cmd_vel 은 받을 때마다 저장만 해 두고, 실제 송신은 tx_timer_ 가 일정 주기로 한다
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        last_cmd_ = *msg;
        last_cmd_time_ = now();
      });

    last_cmd_time_ = now();
    // 타이머 3개: 수신(5ms), 송신(tx_rate), 재연결(1s). 전부 같은 스레드에서 돌아서 락이 필요 없다
    rx_timer_ = create_wall_timer(5ms, [this] {poll_rx();});
    tx_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / tx_rate), [this] {send_wheel_cmd();});
    reconnect_timer_ = create_wall_timer(1s, [this] {try_connect();});
    try_connect();
  }

  ~StmBridgeNode() override
  {
    // 종료 시 정지 명령을 한 번 보내고 닫는다.
    if (serial_.is_open()) {
      serial_.write(proto::encode_wheel_cmd(0, 0));
    }
  }

private:
  // 포트가 닫혀 있으면 열기를 시도 (실패 로그는 5초에 한 번만)
  void try_connect()
  {
    if (serial_.is_open()) {
      return;
    }
    if (serial_.open(port_, static_cast<int>(baud_))) {
      RCLCPP_INFO(get_logger(), "Opened %s @ %ld", port_.c_str(), baud_);
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Serial open failed: %s",
        serial_.error().c_str());
    }
  }

  // 읽기/쓰기 에러 시 포트를 닫는다 -> reconnect_timer_ 가 다시 열어 준다
  void disconnect()
  {
    RCLCPP_ERROR(get_logger(), "Serial error (%s), reconnecting", serial_.error().c_str());
    serial_.close();
  }

  // 시리얼에 쌓인 바이트를 읽어 파서에 한 바이트씩 넣고, 완성된 프레임을 처리
  void poll_rx()
  {
    if (!serial_.is_open()) {
      return;
    }
    std::array<uint8_t, 256> buf;
    const ssize_t n = serial_.read(buf.data(), buf.size());
    if (n < 0) {
      disconnect();
      return;
    }
    for (ssize_t i = 0; i < n; ++i) {
      if (parser_.feed(buf[i])) {
        handle_frame(parser_.frame());
      }
    }
  }

  // 완성된 프레임 하나 처리. 지금은 PSD 프레임만 있다
  void handle_frame(const proto::Frame & f)
  {
    if (f.id == proto::ID_PSD && f.payload.size() == proto::LEN_PSD) {
      interfaces::msg::PsdArray msg;
      msg.header.stamp = now();
      msg.header.frame_id = psd_frame_id_;
      // payload: left, front, right 각 2바이트 [mm] -> [m]
      msg.left = proto::get_u16(&f.payload[0]) * 1e-3f;
      msg.front = proto::get_u16(&f.payload[2]) * 1e-3f;
      msg.right = proto::get_u16(&f.payload[4]) * 1e-3f;
      psd_pub_->publish(msg);
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Unknown frame id=0x%02X len=%zu", f.id, f.payload.size());
    }
  }

  // cmd_vel(v, w) -> 좌/우 바퀴 선속도 -> WHEEL_CMD 프레임 송신
  void send_wheel_cmd()
  {
    if (!serial_.is_open()) {
      return;
    }
    // 마지막 cmd_vel 이 너무 오래됐으면 v = w = 0 (정지)
    double v = 0.0, w = 0.0;
    if ((now() - last_cmd_time_).seconds() < cmd_timeout_) {
      v = last_cmd_.linear.x;
      w = last_cmd_.angular.z;
    }

    // 차동구동 역기구학: 로봇이 w [rad/s] 로 돌면 바퀴는 중심에서 (바퀴 간격/2) 떨어져 있으므로
    // 왼쪽은 그만큼 느리게, 오른쪽은 빠르게 돈다 (w > 0 = 반시계 = 왼쪽으로 회전)
    double left = v - w * wheel_separation_ / 2.0;
    double right = v + w * wheel_separation_ / 2.0;
    // 한쪽이 한계를 넘으면 양쪽을 같은 비율로 줄여 회전반경을 유지
    const double peak = std::max(std::abs(left), std::abs(right));
    if (peak > max_wheel_speed_) {
      left *= max_wheel_speed_ / peak;
      right *= max_wheel_speed_ / peak;
    }

    // [m/s] -> [mm/s] 정수로 반올림해서 송신
    const auto frame = proto::encode_wheel_cmd(
      static_cast<int16_t>(std::lround(left * 1000.0)),
      static_cast<int16_t>(std::lround(right * 1000.0)));
    if (!serial_.write(frame)) {
      disconnect();
    }
  }

  // 파라미터
  std::string port_;
  int64_t baud_;
  double wheel_separation_;
  double max_wheel_speed_;
  double cmd_timeout_;
  std::string psd_frame_id_;

  stm::SerialPort serial_;
  proto::Parser parser_;                // 수신 바이트 -> 프레임 조립기
  geometry_msgs::msg::Twist last_cmd_;  // 마지막으로 받은 cmd_vel
  rclcpp::Time last_cmd_time_;          // 그 cmd_vel 을 받은 시각 (timeout 판단용)

  rclcpp::Publisher<interfaces::msg::PsdArray>::SharedPtr psd_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr rx_timer_, tx_timer_, reconnect_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StmBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
