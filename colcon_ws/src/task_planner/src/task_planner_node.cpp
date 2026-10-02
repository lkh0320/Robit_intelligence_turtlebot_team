// 미션 상태머신: vision/psd 인식 결과를 보고 "지금 어떤 동작을 해야 하는지"만 결정해서
//   control_mode 토픽으로 지시한다. 실제 조향/속도 계산(PID, 회피 경로, 주차 기동)은
//   autonomous driving 쪽 책임이며 이 노드는 관여하지 않는다.
//
// sub: traffic_light, sign, psd, parking_spot, dxl_state, barrier,
//      control_mode (GUI 강제 오버라이드 감지용)
// pub: control_mode (interfaces/ControlMode)
//
// 상태 순서 (트랙 구성은 고정, 구간 내 장애물/표지판/주차 위치만 랜덤):
//   WAIT_LIGHT -> TO_OBSTACLE -> AVOIDING -> TO_PARKING -> PARKING
//   -> TO_BARRIER -> BARRIER_WAIT -> FINISHING -> DONE
// 갈림길(TO_OBSTACLE 구간 중 표지판 인식)은 별도 운행모드가 필요 없어서 mode는 계속 LANE 이고
// mission_state 문자열만 FORK_LEFT/FORK_RIGHT 로 바뀐다 (GUI 로그/디버깅용).
// 차단바 통과 후 "터널 직전 정지" 지점은 터널 인식 메시지가 아직 없어서 임시로 고정 시간
// (barrier_cross_duration) 주행 후 정지한다. vision에 터널 입구 인식이 추가되면 교체할 것.
#include <algorithm>
#include <memory>
#include <string>

#include "interfaces/msg/barrier.hpp"
#include "interfaces/msg/control_mode.hpp"
#include "interfaces/msg/dxl_state.hpp"
#include "interfaces/msg/parking_spot.hpp"
#include "interfaces/msg/psd_array.hpp"
#include "interfaces/msg/sign.hpp"
#include "interfaces/msg/traffic_light.hpp"
#include "rclcpp/rclcpp.hpp"

class TaskPlannerNode : public rclcpp::Node
{
  using ControlMode = interfaces::msg::ControlMode;

  enum class State
  {
    WAIT_LIGHT,
    TO_OBSTACLE,
    AVOIDING,
    TO_PARKING,
    PARKING,
    TO_BARRIER,
    BARRIER_WAIT,
    FINISHING,
    DONE,
  };

public:
  TaskPlannerNode()
  : Node("task_planner")
  {
    min_confidence_ = declare_parameter("min_confidence", 0.5);
    sign_area_ratio_threshold_ = declare_parameter("sign_area_ratio_threshold", 0.05);
    psd_obstacle_threshold_ = declare_parameter("psd_obstacle_threshold", 0.25);
    psd_clear_threshold_ = declare_parameter("psd_clear_threshold", 0.35);
    psd_clear_count_ = declare_parameter("psd_clear_count", 10);
    parking_offset_threshold_ = declare_parameter("parking_offset_threshold", 0.05);
    parking_stop_velocity_threshold_ = declare_parameter("parking_stop_velocity_threshold", 0.02);
    parking_stable_duration_ = declare_parameter("parking_stable_duration", 0.5);
    barrier_cross_duration_ = declare_parameter("barrier_cross_duration", 3.0);
    const double tick_rate = declare_parameter("tick_rate", 20.0);

    const auto qos = rclcpp::SensorDataQoS();
    light_sub_ = create_subscription<interfaces::msg::TrafficLight>(
      "traffic_light", qos, [this](interfaces::msg::TrafficLight::ConstSharedPtr msg) {
        light_ = *msg;
      });
    sign_sub_ = create_subscription<interfaces::msg::Sign>(
      "sign", qos, [this](interfaces::msg::Sign::ConstSharedPtr msg) {sign_ = *msg;});
    psd_sub_ = create_subscription<interfaces::msg::PsdArray>(
      "psd", qos, [this](interfaces::msg::PsdArray::ConstSharedPtr msg) {
        psd_ = *msg;
        psd_received_ = true;
      });
    parking_sub_ = create_subscription<interfaces::msg::ParkingSpot>(
      "parking_spot", qos, [this](interfaces::msg::ParkingSpot::ConstSharedPtr msg) {
        parking_ = *msg;
      });
    dxl_sub_ = create_subscription<interfaces::msg::DxlState>(
      "dxl_state", qos, [this](interfaces::msg::DxlState::ConstSharedPtr msg) {dxl_ = *msg;});
    barrier_sub_ = create_subscription<interfaces::msg::Barrier>(
      "barrier", qos, [this](interfaces::msg::Barrier::ConstSharedPtr msg) {barrier_ = *msg;});
    // GUI가 디버그 버튼으로 강제 모드 변경을 보낼 때 mission_state 에 "turtle_gui" 를 채워 보낸다
    // (turtle_gui/main_window.cpp sendControlMode 참고). 이걸 보면 수동 개입으로 보고 멈춘다.
    mode_sub_ = create_subscription<ControlMode>(
      "control_mode", 10, [this](ControlMode::ConstSharedPtr msg) {
        if (msg->mission_state == "turtle_gui" && !manual_override_) {
          manual_override_ = true;
          RCLCPP_WARN(get_logger(), "GUI 강제 모드 감지, 상태머신 정지 (노드 재시작 전까지 유지)");
        }
      });

    mode_pub_ = create_publisher<ControlMode>("control_mode", 10);
    tick_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / tick_rate), [this] {tick();});
  }

private:
  void tick()
  {
    if (manual_override_) {
      return;   // GUI가 제어권을 가져간 상태: 아무것도 보내지 않는다
    }

    std::string mission_state;
    switch (state_) {
      case State::WAIT_LIGHT:
        mission_state = "WAIT_LIGHT";
        if (light_.state == interfaces::msg::TrafficLight::GREEN &&
          light_.confidence >= min_confidence_)
        {
          state_ = State::TO_OBSTACLE;
        }
        break;

      case State::TO_OBSTACLE:
        mission_state = fork_mission_state();
        if (psd_blocked()) {
          state_ = State::AVOIDING;
          psd_clear_streak_ = 0;
        }
        break;

      case State::AVOIDING:
        mission_state = "AVOID";
        if (psd_clear()) {
          if (++psd_clear_streak_ >= psd_clear_count_) {
            state_ = State::TO_PARKING;
          }
        } else {
          psd_clear_streak_ = 0;
        }
        break;

      case State::TO_PARKING:
        mission_state = "LANE_FOLLOW";
        if (parking_.detected && parking_.confidence >= min_confidence_) {
          state_ = State::PARKING;
          parking_timer_running_ = false;
        }
        break;

      case State::PARKING:
        mission_state = "PARKING";
        if (parked()) {
          state_ = State::TO_BARRIER;
        }
        break;

      case State::TO_BARRIER:
        mission_state = "LANE_FOLLOW";
        if (barrier_.detected && barrier_.confidence >= min_confidence_) {
          if (barrier_.state == interfaces::msg::Barrier::OPEN) {
            state_ = State::FINISHING;
            finishing_since_ = now();
          } else {
            state_ = State::BARRIER_WAIT;
          }
        }
        break;

      case State::BARRIER_WAIT:
        mission_state = "BARRIER_WAIT";
        if (barrier_.state == interfaces::msg::Barrier::OPEN) {
          state_ = State::FINISHING;
          finishing_since_ = now();
        }
        break;

      case State::FINISHING:
        // TODO(vision): 터널 입구 인식 메시지가 생기면 고정 시간 대신 그걸로 판정할 것
        mission_state = "BARRIER_CROSSING";
        if ((now() - finishing_since_).seconds() >= barrier_cross_duration_) {
          state_ = State::DONE;
        }
        break;

      case State::DONE:
        mission_state = "DONE";
        break;
    }

    publish_mode(mission_state);
  }

  // sign 이 충분히 가까이 보이는 동안만 mission_state 를 FORK_LEFT/RIGHT 로 바꿔 GUI에 표시한다.
  // mode 자체는 계속 LANE: 갈림길은 평소 라인추적에 조향 편향만 더하는 거라 별도 운행모드가 필요 없다.
  std::string fork_mission_state() const
  {
    if (sign_.confidence >= min_confidence_ && sign_.area_ratio >= sign_area_ratio_threshold_) {
      if (sign_.type == interfaces::msg::Sign::LEFT) {return "FORK_LEFT";}
      if (sign_.type == interfaces::msg::Sign::RIGHT) {return "FORK_RIGHT";}
    }
    return "LANE_FOLLOW";
  }

  float min_psd() const {return std::min({psd_.left, psd_.front, psd_.right});}
  bool psd_blocked() const {return psd_received_ && min_psd() < psd_obstacle_threshold_;}
  bool psd_clear() const {return psd_received_ && min_psd() >= psd_clear_threshold_;}

  // 정렬(offset) + 정지(바퀴 속도 0) 가 일정 시간 유지되면 주차 완료로 판단
  bool parked()
  {
    const bool aligned = std::abs(parking_.offset) < parking_offset_threshold_;
    const bool stopped = std::abs(dxl_.left_velocity) < parking_stop_velocity_threshold_ &&
      std::abs(dxl_.right_velocity) < parking_stop_velocity_threshold_;
    if (!aligned || !stopped) {
      parking_timer_running_ = false;
      return false;
    }
    if (!parking_timer_running_) {
      parking_timer_running_ = true;
      parking_stable_since_ = now();
      return false;
    }
    return (now() - parking_stable_since_).seconds() >= parking_stable_duration_;
  }

  void publish_mode(const std::string & mission_state)
  {
    ControlMode msg;
    msg.header.stamp = now();
    msg.mode = mode_for_state();
    msg.mission_state = mission_state;
    mode_pub_->publish(msg);
  }

  uint8_t mode_for_state() const
  {
    switch (state_) {
      case State::WAIT_LIGHT: return ControlMode::STOP;
      case State::TO_OBSTACLE: return ControlMode::LANE;
      case State::AVOIDING: return ControlMode::AVOID;
      case State::TO_PARKING: return ControlMode::LANE;
      case State::PARKING: return ControlMode::PARKING;
      case State::TO_BARRIER: return ControlMode::LANE;
      case State::BARRIER_WAIT: return ControlMode::STOP;
      case State::FINISHING: return ControlMode::LANE;
      case State::DONE: return ControlMode::STOP;
    }
    return ControlMode::STOP;
  }

  // 파라미터
  double min_confidence_;
  double sign_area_ratio_threshold_;
  double psd_obstacle_threshold_;
  double psd_clear_threshold_;
  int64_t psd_clear_count_;
  double parking_offset_threshold_;
  double parking_stop_velocity_threshold_;
  double parking_stable_duration_;
  double barrier_cross_duration_;

  State state_ = State::WAIT_LIGHT;
  bool manual_override_ = false;
  int64_t psd_clear_streak_ = 0;
  bool parking_timer_running_ = false;
  rclcpp::Time parking_stable_since_;   // parking_timer_running_ 이 true 일 때만 유효
  rclcpp::Time finishing_since_;        // FINISHING 상태 진입 시점에만 설정됨

  bool psd_received_ = false;
  interfaces::msg::TrafficLight light_;
  interfaces::msg::Sign sign_;
  interfaces::msg::PsdArray psd_;
  interfaces::msg::ParkingSpot parking_;
  interfaces::msg::DxlState dxl_;
  interfaces::msg::Barrier barrier_;

  rclcpp::Subscription<interfaces::msg::TrafficLight>::SharedPtr light_sub_;
  rclcpp::Subscription<interfaces::msg::Sign>::SharedPtr sign_sub_;
  rclcpp::Subscription<interfaces::msg::PsdArray>::SharedPtr psd_sub_;
  rclcpp::Subscription<interfaces::msg::ParkingSpot>::SharedPtr parking_sub_;
  rclcpp::Subscription<interfaces::msg::DxlState>::SharedPtr dxl_sub_;
  rclcpp::Subscription<interfaces::msg::Barrier>::SharedPtr barrier_sub_;
  rclcpp::Subscription<ControlMode>::SharedPtr mode_sub_;
  rclcpp::Publisher<ControlMode>::SharedPtr mode_pub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TaskPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
