// 라인 추적 주행: lane_info -> cmd_vel
//   구독: lane_info (interfaces/LaneInfo)        lane_detection 또는 path_planner
//         control_mode (interfaces/ControlMode)  task_planner / GUI 모드 버튼
//         psd (interfaces/PsdArray)              정면 비상 정지
//   발행: cmd_vel (geometry_msgs/Twist)          stm_bridge 가 바퀴로 보낸다
//
// 운행 모드가 LANE / AVOID 일 때만 주행한다 (AVOID 도 주행: path_planner 는 회피를 경로로 처리).
//   그 외 모드(STOP, PARKING, MANUAL)가 되면 정지 명령을 한 번 보내고 조용히 있는다
//   (GUI 수동 주행이나 주차 기동 노드의 cmd_vel 과 싸우지 않도록)
// autostart: true 면 control_mode 를 받기 전에도 LANE 으로 시작 (task_planner 없이 주행 시험).
//   GUI STOP 버튼(control_mode STOP)을 누르면 그대로 멈춘다.
// 안전: lane_info 가 lane_timeout 동안 안 오거나 detected = false 면 정지,
//       정면 PSD 가 front_stop_distance 보다 가까우면 정지
// 제어 법칙은 include/lane_follower/controller.hpp
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "interfaces/msg/control_mode.hpp"
#include "interfaces/msg/lane_info.hpp"
#include "interfaces/msg/psd_array.hpp"
#include "lane_follower/controller.hpp"
#include "rclcpp/rclcpp.hpp"

class LaneFollowerNode : public rclcpp::Node
{
  using ControlMode = interfaces::msg::ControlMode;

public:
  LaneFollowerNode()
  : Node("lane_follower")
  {
    const lane_follower::Gains g;
    declare_parameter("linear_speed", g.linear_speed);
    declare_parameter("min_linear_speed", g.min_linear_speed);
    declare_parameter("kp", g.kp);
    declare_parameter("kd", g.kd);
    declare_parameter("ka", g.ka);
    declare_parameter("pp_lookahead", g.pp_lookahead);
    declare_parameter("max_angular", g.max_angular);
    declare_parameter("slow_down_angle", g.slow_down_angle);
    declare_parameter("low_confidence", g.low_confidence);
    declare_parameter("d_filter_alpha", g.d_filter_alpha);
    declare_parameter("lane_timeout", 0.3);
    declare_parameter("front_stop_distance", 0.10);   // 0 이하면 끔
    declare_parameter("psd_timeout", 0.5);
    const bool autostart = declare_parameter("autostart", false);
    const double rate = declare_parameter("control_rate", 20.0);

    mode_ = autostart ? ControlMode::LANE : ControlMode::STOP;

    const auto qos = rclcpp::SensorDataQoS();
    lane_sub_ = create_subscription<interfaces::msg::LaneInfo>(
      "lane_info", qos, [this](interfaces::msg::LaneInfo::ConstSharedPtr msg) {
        lane_ = *msg;
        lane_stamp_ = now();
        lane_received_ = true;
      });
    mode_sub_ = create_subscription<ControlMode>(
      "control_mode", 10, [this](ControlMode::ConstSharedPtr msg) {
        if (msg->mode != mode_) {
          RCLCPP_INFO(get_logger(), "control_mode %u -> %u (%s)", mode_, msg->mode,
            msg->mission_state.c_str());
        }
        mode_ = msg->mode;
      });
    psd_sub_ = create_subscription<interfaces::msg::PsdArray>(
      "psd", qos, [this](interfaces::msg::PsdArray::ConstSharedPtr msg) {
        psd_ = *msg;
        psd_stamp_ = now();
        psd_received_ = true;
      });
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this] {tick();});

    RCLCPP_INFO(get_logger(), "lane_follower 시작 (%s)",
      autostart ? "autostart: 바로 LANE 주행" : "control_mode LANE/AVOID 대기");
  }

private:
  double dbl(const std::string & name) {return get_parameter(name).as_double();}

  lane_follower::Gains readGains()
  {
    lane_follower::Gains g;
    g.linear_speed = dbl("linear_speed");
    g.min_linear_speed = dbl("min_linear_speed");
    g.kp = dbl("kp");
    g.kd = dbl("kd");
    g.ka = dbl("ka");
    g.pp_lookahead = dbl("pp_lookahead");
    g.max_angular = dbl("max_angular");
    g.slow_down_angle = dbl("slow_down_angle");
    g.low_confidence = dbl("low_confidence");
    g.d_filter_alpha = dbl("d_filter_alpha");
    return g;
  }

  void tick()
  {
    const auto t = now();
    const bool driving_mode = mode_ == ControlMode::LANE || mode_ == ControlMode::AVOID;
    if (!driving_mode) {
      // 주행 모드에서 막 빠져나왔으면 정지 한 번, 그 뒤로는 발행하지 않는다
      if (was_driving_) {
        cmd_pub_->publish(geometry_msgs::msg::Twist());
        controller_.reset();
        was_driving_ = false;
      }
      return;
    }
    was_driving_ = true;

    geometry_msgs::msg::Twist cmd;   // 기본 = 정지
    const bool lane_ok = lane_received_ && lane_.detected &&
      (t - lane_stamp_).seconds() <= dbl("lane_timeout");
    const double stop_dist = dbl("front_stop_distance");
    const bool front_blocked = stop_dist > 0 && psd_received_ &&
      (t - psd_stamp_).seconds() <= dbl("psd_timeout") && psd_.front > 0 && psd_.front < stop_dist;

    if (lane_ok && !front_blocked) {
      const double dt = last_tick_ok_ ? (t - last_tick_).seconds() : 0.0;
      const auto c = controller_.step(lane_.offset, lane_.angle, lane_.confidence, dt,
          readGains());
      cmd.linear.x = c.v;
      cmd.angular.z = c.w;
      last_tick_ok_ = true;
    } else {
      controller_.reset();
      last_tick_ok_ = false;
      const char * why = front_blocked ? "정면 PSD 가까움" :
        !lane_received_ ? "lane_info 없음" : !lane_.detected ? "차선 못 찾음" : "lane_info 끊김";
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "정지: %s", why);
    }
    last_tick_ = t;
    cmd_pub_->publish(cmd);
  }

  lane_follower::Controller controller_;
  uint8_t mode_;
  bool was_driving_ = false;

  interfaces::msg::LaneInfo lane_;
  rclcpp::Time lane_stamp_;
  bool lane_received_ = false;
  interfaces::msg::PsdArray psd_;
  rclcpp::Time psd_stamp_;
  bool psd_received_ = false;
  rclcpp::Time last_tick_;
  bool last_tick_ok_ = false;

  rclcpp::Subscription<interfaces::msg::LaneInfo>::SharedPtr lane_sub_;
  rclcpp::Subscription<ControlMode>::SharedPtr mode_sub_;
  rclcpp::Subscription<interfaces::msg::PsdArray>::SharedPtr psd_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaneFollowerNode>());
  rclcpp::shutdown();
  return 0;
}
