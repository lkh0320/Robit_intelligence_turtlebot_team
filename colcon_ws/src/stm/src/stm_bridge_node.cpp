// STM32 시리얼 브릿지
//   sub: cmd_vel (geometry_msgs/Twist)  -> 차동구동 역기구학 -> WHEEL_CMD 프레임
//   pub: psd     (interfaces/PsdArray)  <- PSD 프레임
// 프로토콜은 include/stm/protocol.hpp 참고.
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
    port_ = declare_parameter("port", "/dev/ttyUSB0");
    baud_ = declare_parameter("baud", 115200);
    wheel_separation_ = declare_parameter("wheel_separation", 0.160);
    max_wheel_speed_ = declare_parameter("max_wheel_speed", 0.26);
    cmd_timeout_ = declare_parameter("cmd_timeout", 0.5);
    const double tx_rate = declare_parameter("tx_rate", 20.0);
    psd_frame_id_ = declare_parameter("psd_frame_id", "base_link");

    psd_pub_ = create_publisher<interfaces::msg::PsdArray>("psd", rclcpp::SensorDataQoS());
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        last_cmd_ = *msg;
        last_cmd_time_ = now();
      });

    last_cmd_time_ = now();
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

  void disconnect()
  {
    RCLCPP_ERROR(get_logger(), "Serial error (%s), reconnecting", serial_.error().c_str());
    serial_.close();
  }

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

  void handle_frame(const proto::Frame & f)
  {
    if (f.id == proto::ID_PSD && f.payload.size() == proto::LEN_PSD) {
      interfaces::msg::PsdArray msg;
      msg.header.stamp = now();
      msg.header.frame_id = psd_frame_id_;
      msg.left = proto::get_u16(&f.payload[0]) * 1e-3f;
      msg.front = proto::get_u16(&f.payload[2]) * 1e-3f;
      msg.right = proto::get_u16(&f.payload[4]) * 1e-3f;
      psd_pub_->publish(msg);
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Unknown frame id=0x%02X len=%zu", f.id, f.payload.size());
    }
  }

  void send_wheel_cmd()
  {
    if (!serial_.is_open()) {
      return;
    }
    double v = 0.0, w = 0.0;
    if ((now() - last_cmd_time_).seconds() < cmd_timeout_) {
      v = last_cmd_.linear.x;
      w = last_cmd_.angular.z;
    }

    double left = v - w * wheel_separation_ / 2.0;
    double right = v + w * wheel_separation_ / 2.0;
    // 한쪽이 한계를 넘으면 양쪽을 같은 비율로 줄여 회전반경을 유지
    const double peak = std::max(std::abs(left), std::abs(right));
    if (peak > max_wheel_speed_) {
      left *= max_wheel_speed_ / peak;
      right *= max_wheel_speed_ / peak;
    }

    const auto frame = proto::encode_wheel_cmd(
      static_cast<int16_t>(std::lround(left * 1000.0)),
      static_cast<int16_t>(std::lround(right * 1000.0)));
    if (!serial_.write(frame)) {
      disconnect();
    }
  }

  std::string port_;
  int64_t baud_;
  double wheel_separation_;
  double max_wheel_speed_;
  double cmd_timeout_;
  std::string psd_frame_id_;

  stm::SerialPort serial_;
  proto::Parser parser_;
  geometry_msgs::msg::Twist last_cmd_;
  rclcpp::Time last_cmd_time_;

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
