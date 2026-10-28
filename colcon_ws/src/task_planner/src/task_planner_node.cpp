// 미션 상태머신: vision/psd 인식 결과를 보고 "지금 어떤 동작을 해야 하는지"만 결정해서
//   control_mode 토픽으로 지시한다. 실제 조향/속도 계산(PID, 회피 경로, 주차 기동)은
//   autonomous driving 쪽 책임이며 이 노드는 관여하지 않는다.
//
// sub: traffic_light, sign, psd, parking_spot, dxl_state, barrier, parking_active, parking_done,
//      control_mode (GUI 강제 오버라이드 감지용), mission_start (GUI "자율주행 시작" 버튼)
// pub: control_mode (interfaces/ControlMode)
//
// 상태 순서 (트랙 구성은 고정, 구간 내 장애물/표지판/주차 위치만 랜덤):
//   WAIT_LIGHT -> TO_OBSTACLE -> AVOIDING -> TO_PARKING -> PARKING
//   -> TO_BARRIER -> BARRIER_WAIT -> FINISHING -> DONE
// 갈림길(TO_OBSTACLE 구간 중 표지판 인식)은 별도 운행모드가 필요 없어서 mode는 계속 LANE 이고
// mission_state 문자열만 FORK_LEFT/FORK_RIGHT 로 바뀐다 (GUI 로그/디버깅용).
// 차단바 통과 후 "터널 직전 정지" 지점은 터널 인식 메시지가 아직 없어서 임시로 고정 시간
// (barrier_cross_duration) 주행 후 정지한다. vision에 터널 입구 인식이 추가되면 교체할 것.
// wait_start=true 면 mission_start 를 받기 전까지 아무것도 안 보낸다 (GUI 모드 버튼/수동 주행 그대로 사용).
// mission_start 를 받으면 언제든 start_state(기본 WAIT_LIGHT)부터 다시 시작하고 GUI 오버라이드도 푼다.
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
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"

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
    // 차단바가 이 위치(이미지 세로 비율, 0 상단~1 하단)보다 가까워져야 정지/통과 판단.
    // 멀리서 잠깐 잘못 보이거나, 닫힌 채로 멀리 있을 때 미리 서버리지 않고 정해진 위치까지 접근하게 함
    barrier_stop_y_ratio_ = declare_parameter("barrier_stop_y_ratio", 0.5);
    // 모델이 "열림(bar_open)"을 잘 못 맞혀서, 차단바가 아예 안 보이는 것도 "치워짐"으로 봄.
    // 한 프레임만 놓쳐도 바로 출발하면 위험하니 이만큼 연속으로 못 봐야 인정 (psd_clear_count 와 같은 방식)
    barrier_lost_count_ = declare_parameter("barrier_lost_count", 10);
    // false: 차단바 인식으로 STOP 을 보내지 않음 (grid_drive 가 차단바 앞 막다른 길에서 알아서 멈추고,
    //   열리면 다시 감). TO_BARRIER 에서 계속 LANE
    barrier_stop_ = declare_parameter("barrier_stop", true);
    // 주차 노드의 PARKING 이 이만큼 안 오면 주차 노드가 운전을 그만둔 것으로 봄
    parking_quiet_sec_ = declare_parameter("parking_quiet_sec", 0.5);
    // 테스트용: 특정 상태부터 바로 시작 (기본 WAIT_LIGHT = 실제 미션과 동일하게 처음부터)
    //   예: start_state:=TO_BARRIER 로 차단바 구간만 따로 테스트
    start_state_ = stateFromName(declare_parameter("start_state", std::string("WAIT_LIGHT")));
    state_ = start_state_;
    // true: GUI "자율주행 시작"(mission_start) 을 받을 때까지 대기 (manual_bringup 에서 사용)
    started_ = !declare_parameter("wait_start", false);
    parking_sign_area_ratio_threshold_ =
      declare_parameter("parking_sign_area_ratio_threshold", 0.03);
    parking_sign_hits_needed_ = declare_parameter("parking_sign_hits_needed", 3);
    const double tick_rate = declare_parameter("tick_rate", 20.0);

    const auto qos = rclcpp::SensorDataQoS();
    light_sub_ = create_subscription<interfaces::msg::TrafficLight>(
      "traffic_light", qos, [this](interfaces::msg::TrafficLight::ConstSharedPtr msg) {
        light_ = *msg;
      });
    sign_sub_ = create_subscription<interfaces::msg::Sign>(
      "sign", qos, [this](interfaces::msg::Sign::ConstSharedPtr msg) {
        sign_ = *msg;
        // 주차 표지판이 가까이 연속으로 보인 횟수 (한 번이라도 안 보이면 0 부터 다시)
        const bool close_parking_sign = msg->type == interfaces::msg::Sign::PARKING &&
        msg->confidence >= min_confidence_ &&
        msg->area_ratio >= parking_sign_area_ratio_threshold_;
        parking_sign_hits_ = close_parking_sign ? parking_sign_hits_ + 1 : 0;
      });
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
    // parking_node 가 칸에 들어가고 나오는 동안(직접 운전 중) true. 그동안만 PARKING 모드
    parking_active_sub_ = create_subscription<std_msgs::msg::Bool>(
      "parking_active", 10, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        parking_active_ = msg->data;
      });
    // lane_follower 의 parking_node 가 주차 구간을 끝내면 true 를 보낸다
    parking_done_sub_ = create_subscription<std_msgs::msg::Bool>(
      "parking_done", 10, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        if (msg->data) {
          parking_done_ = true;
        }
      });
    barrier_sub_ = create_subscription<interfaces::msg::Barrier>(
      "barrier", qos, [this](interfaces::msg::Barrier::ConstSharedPtr msg) {barrier_ = *msg;});
    // GUI가 디버그 버튼으로 강제 모드 변경을 보낼 때 mission_state 에 "turtle_gui" 를 채워 보낸다
    // (turtle_gui/main_window.cpp sendControlMode 참고). 이걸 보면 수동 개입으로 보고 멈춘다.
    mode_sub_ = create_subscription<ControlMode>(
      "control_mode", 10, [this](ControlMode::ConstSharedPtr msg) {
        // 주차 노드 (parking_node.cpp) 는 칸에 들어갔다 나오는 동안 0.1초마다 PARKING 을 직접 보내고,
        // 끝나면 "parking_node:leave_done" 으로 LANE 을 보낸다. 그동안 이 노드가 LANE 을 보내면 모드가
        // 오가며 grid_drive / 주차 노드가 번갈아 운전해 덜덜 떤다 -> 주차 노드가 운전 중이면 아무것도 안 보냄
        if (msg->mission_state.rfind("parking_node:", 0) == 0) {
          if (msg->mode == ControlMode::PARKING) {
            parking_active_ = true;
            parking_seen_ = now();
          } else {
            parking_active_ = false;
            if (msg->mission_state == "parking_node:leave_done") {
              parking_done_ = true;
            }
          }
          return;
        }
        if (msg->mission_state == "turtle_gui" && !manual_override_) {
          manual_override_ = true;
          RCLCPP_WARN(get_logger(), "GUI 강제 모드 감지, 상태머신 정지 (노드 재시작 전까지 유지)");
        }
      });

    // GUI "자율주행 시작" 버튼: 처음부터(start_state) 다시 시작
    start_sub_ = create_subscription<std_msgs::msg::Empty>(
      "mission_start", 10, [this](std_msgs::msg::Empty::ConstSharedPtr) {start();});

    mode_pub_ = create_publisher<ControlMode>("control_mode", 10);
    tick_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / tick_rate), [this] {tick();});
  }

private:
  // 테스트용 start_state 파라미터(문자열) -> State. 모르는 이름이면 경고 찍고 WAIT_LIGHT
  State stateFromName(const std::string & name)
  {
    static const std::pair<const char *, State> kNames[] = {
      {"WAIT_LIGHT", State::WAIT_LIGHT}, {"TO_OBSTACLE", State::TO_OBSTACLE},
      {"AVOIDING", State::AVOIDING}, {"TO_PARKING", State::TO_PARKING},
      {"PARKING", State::PARKING}, {"TO_BARRIER", State::TO_BARRIER},
      {"BARRIER_WAIT", State::BARRIER_WAIT}, {"FINISHING", State::FINISHING},
      {"DONE", State::DONE},
    };
    for (const auto & [n, s] : kNames) {
      if (name == n) {
        return s;
      }
    }
    RCLCPP_WARN(get_logger(), "start_state '%s' 모름 -> WAIT_LIGHT", name.c_str());
    return State::WAIT_LIGHT;
  }

  // 상태머신을 start_state 부터 새로 시작 (지난 주행에서 남은 값 초기화)
  void start()
  {
    state_ = start_state_;
    started_ = true;
    manual_override_ = false;
    psd_clear_streak_ = 0;
    barrier_lost_streak_ = 0;
    parking_timer_running_ = false;
    parking_sign_hits_ = 0;
    parking_done_ = false;
    parking_active_ = false;
    light_ = interfaces::msg::TrafficLight();   // 시작 전에 본 GREEN 으로 바로 출발하지 않도록
    RCLCPP_INFO(get_logger(), "mission_start 수신: 자율주행 시작");
  }

  void tick()
  {
    if (!started_ || manual_override_) {
      return;   // 시작 전이거나 GUI가 제어권을 가져간 상태: 아무것도 보내지 않는다
    }
    // 주차 노드가 운전 중 (PARKING 을 받은 지 parking_quiet_sec 이내): 모드를 보내지 않음
    if (parking_active_ && (now() - parking_seen_).seconds() < parking_quiet_sec_) {
      if (state_ == State::TO_PARKING) {
        state_ = State::PARKING;   // 주차 노드가 표지판을 보고 먼저 시작함
      }
      return;
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
        if (parking_done_) {   // 주차 노드가 이미 주차를 끝냄
          state_ = State::TO_BARRIER;
          break;
        }
        // 주차 표지판을 가까이서 연속으로 보거나, 주차칸 인식 노드가 주차 구역을 알려주면 주차 시작
        if (parking_sign_hits_ >= parking_sign_hits_needed_ ||
          (parking_.detected && parking_.confidence >= min_confidence_))
        {
          state_ = State::PARKING;
          parking_timer_running_ = false;
        }
        break;

      case State::PARKING:
        mission_state = "PARKING";
        // parking_node 가 주차 동작(들어가기 -> 대기 -> 나오기)을 끝내면 다음 구간으로
        if (parking_done_) {
          state_ = State::TO_BARRIER;
        }
        break;

      case State::TO_BARRIER:
        mission_state = "LANE_FOLLOW";
        if (barrier_stop_ && barrier_.detected && barrier_.confidence >= min_confidence_) {
          if (barrier_.state == interfaces::msg::Barrier::OPEN) {
            // 이미 열려 있으면 정해진 위치까지 기다릴 필요 없이 바로 통과
            state_ = State::FINISHING;
            finishing_since_ = now();
          } else if (barrier_.y_ratio >= barrier_stop_y_ratio_) {
            // 닫혀 있고, 정해진 정지 위치(화면 세로 비율)까지 가까워졌을 때만 멈춘다
            // (멀리서 CLOSED 로 본 순간 바로 서버리면 정지 위치가 매번 달라짐)
            state_ = State::BARRIER_WAIT;
          }
        }
        break;

      case State::BARRIER_WAIT:
        mission_state = "BARRIER_WAIT";
        if (barrier_.state == interfaces::msg::Barrier::OPEN) {
          state_ = State::FINISHING;
          finishing_since_ = now();
          barrier_lost_streak_ = 0;
        } else if (!barrier_.detected) {
          if (++barrier_lost_streak_ >= barrier_lost_count_) {
            state_ = State::FINISHING;
            finishing_since_ = now();
          }
        } else {
          barrier_lost_streak_ = 0;   // 여전히 CLOSED 로 보이는 중
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
      // 주차 구역 안: LANE. 주차 노드가 칸에 들어갔다 나오는 동안은 주차 노드가 PARKING 을 직접 보내고
      // 이 노드는 아무것도 안 보냄 (tick 참고)
      case State::PARKING: return ControlMode::LANE;
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
  double barrier_stop_y_ratio_;
  int64_t barrier_lost_count_;
  int64_t barrier_lost_streak_ = 0;
  bool barrier_stop_ = true;
  double parking_quiet_sec_ = 0.5;
  rclcpp::Time parking_seen_{0, 0, RCL_ROS_TIME};
  double parking_sign_area_ratio_threshold_;
  int64_t parking_sign_hits_needed_;

  State start_state_ = State::WAIT_LIGHT;
  State state_ = State::WAIT_LIGHT;
  bool started_ = true;
  bool manual_override_ = false;
  int64_t psd_clear_streak_ = 0;
  bool parking_timer_running_ = false;
  int64_t parking_sign_hits_ = 0;
  bool parking_done_ = false;
  bool parking_active_ = false;
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
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr parking_active_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr parking_done_sub_;
  rclcpp::Subscription<ControlMode>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_sub_;
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
