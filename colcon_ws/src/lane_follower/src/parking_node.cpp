// ============================================================================
//  parking_node.cpp - 주차 표지판을 보면 주차를 하고, 끝나면 다시 라인 주행으로 돌려준다
// ============================================================================
//
// [전체 흐름]
//
//   주행 중 (lane_follower 가 차선 따라 운전)
//      │
//      │  sign_detection 이 주차 표지판을 충분히 가까이, 여러 번 연속으로 봄
//      ▼
//   주차 시작: control_mode 를 PARKING 으로 바꿈 -> lane_follower 는 스스로 멈춘다
//      │
//      │  아래 "단계" 를 차례로 실행 (각 단계 = 속도 v, 회전 w 를 정해진 시간 동안)
//      │    1) approach : 앞으로 조금 더 가기 (주차칸 입구까지)
//      │    2) turn_in  : 주차칸 쪽으로 90도 돌기
//      │    3) enter    : 주차칸 안으로 들어가기 (앞으로 또는 뒤로)
//      │    4) wait     : 주차칸 안에서 잠깐 서 있기
//      │    5) exit     : 주차칸에서 나오기 (들어간 반대 방향)
//      │    6) turn_out : 원래 방향으로 다시 90도 돌기
//      ▼
//   주차 끝: parking_done 을 알리고, control_mode 를 LANE 으로 돌려줌 -> 다시 차선 주행
//
// [두 가지 실행 방식]  (파라미터 use_task_planner)
//
//   false (기본, 시험용) : 이 노드가 직접 /sign 을 보고 주차를 시작한다.
//                         control_mode 도 이 노드가 PARKING -> LANE 으로 바꾼다.
//                         ros2 launch lane_follower autonomous.launch.py lane_method:=sliding
//
//   true (미션 전체)     : task_planner 가 /sign 을 보고 control_mode 를 PARKING 으로 바꾼다.
//                         이 노드는 PARKING 이 되면 단계를 실행하고, 끝나면 parking_done 만 보낸다.
//                         (GUI 의 PARKING 모드 버튼을 눌러도 같은 방식으로 시작된다)
//
// [안전]
//   - 앞으로 가는 단계에서 정면 PSD 가 front_stop_distance 보다 가까우면 멈추고 기다린다.
//   - GUI 에서 STOP / MANUAL 을 누르면 바로 멈추고 주차를 포기한다.
//   - 이 노드는 주차 중일 때만 cmd_vel 을 보낸다. 평소에는 아무것도 보내지 않는다.
//
// [주의] 거리와 각도는 "속도 x 시간" 으로 계산한다 (엔코더를 아직 안 씀).
//   바퀴 반지름(STM app.c)과 바퀴 사이 거리(stm_bridge.yaml)가 맞아야 정확하다.
//   실제로 돌려 보고 config/parking.yaml 의 거리 / 각도를 맞추면 된다.

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "interfaces/msg/control_mode.hpp"
#include "interfaces/msg/parking_spot.hpp"
#include "interfaces/msg/psd_array.hpp"
#include "interfaces/msg/sign.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

// 이 노드가 보내는 control_mode 의 mission_state 앞에 붙는 이름.
// 내가 보낸 메시지가 다시 나한테 들어올 때 "내 거네" 하고 무시하는 데 쓴다.
static const char * const MY_TAG = "parking_node";

// 주차 동작 한 단계: 이 속도(v)와 회전(w)으로 이 시간(duration) 동안 움직인다
struct Step
{
  std::string name;     // 단계 이름 (로그와 GUI 표시용)
  double v;             // 앞뒤 속도 [m/s]   + 앞, - 뒤
  double w;             // 회전 속도 [rad/s] + 왼쪽(반시계), - 오른쪽(시계)
  double duration;      // 이 단계를 하는 시간 [s]
};

class ParkingNode : public rclcpp::Node
{
  using ControlMode = interfaces::msg::ControlMode;
  using Sign = interfaces::msg::Sign;
  using ParkingSpot = interfaces::msg::ParkingSpot;

  // 이 노드의 상태
  enum class State
  {
    WAITING,    // 주차 표지판(또는 PARKING 모드)을 기다리는 중
    RUNNING,    // 주차 단계를 실행하는 중
    FINISHED,   // 주차를 끝냈거나 포기함 (다시 시작하지 않음)
  };

public:
  ParkingNode()
  : Node("parking")
  {
    // ---- 파라미터 (config/parking.yaml 에서 바꾼다) ----
    use_task_planner_ = declare_parameter("use_task_planner", false);
    min_confidence_ = declare_parameter("min_confidence", 0.5);
    sign_area_ratio_threshold_ = declare_parameter("sign_area_ratio_threshold", 0.03);
    sign_hits_needed_ = declare_parameter("sign_hits_needed", 3);
    parking_side_ = declare_parameter("parking_side", std::string("right"));
    use_parking_spot_ = declare_parameter("use_parking_spot", true);
    linear_speed_ = declare_parameter("linear_speed", 0.08);
    angular_speed_ = declare_parameter("angular_speed", 0.6);
    approach_distance_ = declare_parameter("approach_distance", 0.20);
    turn_angle_deg_ = declare_parameter("turn_angle_deg", 90.0);
    enter_distance_ = declare_parameter("enter_distance", 0.25);
    enter_reverse_ = declare_parameter("enter_reverse", false);
    park_wait_time_ = declare_parameter("park_wait_time", 2.0);
    front_stop_distance_ = declare_parameter("front_stop_distance", 0.10);
    const double rate = declare_parameter("control_rate", 20.0);
    dt_ = 1.0 / rate;

    // ---- 받는 토픽 ----
    const auto qos = rclcpp::SensorDataQoS();
    sign_sub_ = create_subscription<Sign>(
      "sign", qos, [this](Sign::ConstSharedPtr msg)
      {
        on_sign(*msg);
      });
    spot_sub_ = create_subscription<ParkingSpot>(
      "parking_spot", qos, [this](ParkingSpot::ConstSharedPtr msg)
      {
        spot_ = *msg;
        spot_received_ = true;
      });
    psd_sub_ = create_subscription<interfaces::msg::PsdArray>(
      "psd", qos, [this](interfaces::msg::PsdArray::ConstSharedPtr msg)
      {
        psd_ = *msg;
        psd_received_ = true;
      });
    mode_sub_ = create_subscription<ControlMode>(
      "control_mode", 10, [this](ControlMode::ConstSharedPtr msg)
      {
        on_mode(*msg);
      });

    // ---- 보내는 토픽 ----
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    mode_pub_ = create_publisher<ControlMode>("control_mode", 10);
    done_pub_ = create_publisher<std_msgs::msg::Bool>("parking_done", 10);

    // ---- 0.05초마다 step() 실행 ----
    timer_ = create_wall_timer(std::chrono::duration<double>(dt_), [this]
      {
        step();
      });

    RCLCPP_INFO(get_logger(), "주차 노드 시작 (%s)",
      use_task_planner_ ? "task_planner 의 PARKING 모드를 기다림" : "주차 표지판을 직접 기다림");
  }

private:
  // ==================== 받은 메시지 처리 ====================

  // 표지판을 받았을 때: 주차 표지판이 가까이, 여러 번 연속으로 보이면 주차 시작
  void on_sign(const Sign & sign)
  {
    const bool close_parking_sign =
      sign.type == Sign::PARKING &&
      sign.confidence >= min_confidence_ &&
      sign.area_ratio >= sign_area_ratio_threshold_;

    if (close_parking_sign)
    {
      sign_hits_++;           // 연속으로 본 횟수 1 증가
    }
    else
    {
      sign_hits_ = 0;         // 한 번이라도 안 보이면 처음부터 다시 센다 (잘못 본 것 걸러내기)
    }

    // task_planner 를 쓸 때는 시작 판단을 task_planner 가 한다
    if (use_task_planner_)
    {
      return;
    }
    if (state_ == State::WAITING && sign_hits_ >= sign_hits_needed_)
    {
      RCLCPP_INFO(get_logger(), "주차 표지판 인식 (크기 %.3f, 신뢰도 %.2f) -> 주차 시작",
        sign.area_ratio, sign.confidence);
      start();
    }
  }

  // control_mode 를 받았을 때
  void on_mode(const ControlMode & msg)
  {
    // 내가 보낸 메시지면 무시
    if (msg.mission_state.rfind(MY_TAG, 0) == 0)
    {
      return;
    }

    if (use_task_planner_)
    {
      // task_planner / GUI 가 PARKING 으로 바꾸면 시작, 다른 모드로 바꾸면 중단
      if (msg.mode == ControlMode::PARKING && state_ == State::WAITING)
      {
        RCLCPP_INFO(get_logger(), "PARKING 모드 받음 -> 주차 시작");
        start();
      }
      else if (msg.mode != ControlMode::PARKING && state_ == State::RUNNING)
      {
        abort("모드가 PARKING 이 아니게 바뀜");
      }
      return;
    }

    // 직접 시작하는 방식: GUI 에서 STOP 이나 MANUAL 을 누르면 중단
    if (state_ == State::RUNNING &&
      (msg.mode == ControlMode::STOP || msg.mode == ControlMode::MANUAL))
    {
      abort("GUI 에서 STOP/MANUAL");
    }
  }

  // ==================== 주차 시작 / 끝 / 중단 ====================

  // 주차 단계 목록을 만들고 시작한다
  void start()
  {
    // 1) 어느 쪽 칸에 넣을지 정한다. +1 = 왼쪽, -1 = 오른쪽
    double side = (parking_side_ == "left") ? +1.0 : -1.0;
    if (use_parking_spot_ && spot_received_ && spot_.detected)
    {
      // 주차칸 인식 노드가 빈 칸 방향을 알려주면 그걸 쓴다
      if (spot_.empty_side == ParkingSpot::LEFT)
      {
        side = +1.0;
      }
      else if (spot_.empty_side == ParkingSpot::RIGHT)
      {
        side = -1.0;
      }
    }

    // 2) 숫자 준비
    const double v = linear_speed_;
    const double w = angular_speed_;
    const double turn_rad = turn_angle_deg_ * M_PI / 180.0;   // 도 -> 라디안
    const double turn_time = turn_rad / w;                    // 이 각도만큼 도는 데 걸리는 시간
    const double enter_dir = enter_reverse_ ? -1.0 : +1.0;    // 들어갈 때 앞(+1) / 뒤(-1)

    // 앞으로 들어가면 칸 쪽으로 돌고, 뒤로 들어가면 칸 반대쪽으로 돈다 (엉덩이를 칸에 맞추려고)
    const double turn_in_w = side * enter_dir * w;

    // 3) 단계 목록. 시간 = 거리 / 속도
    steps_.clear();
    steps_.push_back({"approach", v, 0.0, approach_distance_ / v});
    steps_.push_back({"turn_in", 0.0, turn_in_w, turn_time});
    steps_.push_back({"enter", enter_dir * v, 0.0, enter_distance_ / v});
    steps_.push_back({"wait", 0.0, 0.0, park_wait_time_});
    steps_.push_back({"exit", -enter_dir * v, 0.0, enter_distance_ / v});
    steps_.push_back({"turn_out", 0.0, -turn_in_w, turn_time});

    step_index_ = 0;
    step_elapsed_ = 0.0;
    state_ = State::RUNNING;

    RCLCPP_INFO(get_logger(), "주차 방향: %s, 들어가기: %s",
      side > 0 ? "왼쪽" : "오른쪽", enter_reverse_ ? "후진" : "전진");
  }

  // 모든 단계를 끝냈을 때
  void finish()
  {
    stop_wheels();
    state_ = State::FINISHED;

    std_msgs::msg::Bool done;
    done.data = true;
    done_pub_->publish(done);           // task_planner 에게 "주차 끝" 알림

    if (!use_task_planner_)
    {
      publish_mode(ControlMode::LANE, "done");   // lane_follower 에게 "다시 운전해"
    }
    RCLCPP_INFO(get_logger(), "주차 완료 -> 라인 주행으로 복귀");
  }

  // 중간에 그만둘 때
  void abort(const char * reason)
  {
    stop_wheels();
    state_ = State::FINISHED;
    RCLCPP_WARN(get_logger(), "주차 중단: %s", reason);
  }

  // ==================== 0.05초마다 하는 일 ====================

  void step()
  {
    if (state_ != State::RUNNING)
    {
      return;                           // 주차 중이 아니면 아무것도 안 보낸다
    }

    const Step & s = steps_[step_index_];

    // 직접 시작하는 방식이면 "지금 주차 중" 을 계속 알려서 lane_follower 가 멈춰 있게 한다
    if (!use_task_planner_)
    {
      publish_mode(ControlMode::PARKING, s.name);
    }

    // 안전: 앞으로 가는데 앞이 막혀 있으면 멈추고 기다린다 (시간도 멈춤)
    if (s.v > 0.0 && front_blocked())
    {
      stop_wheels();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "앞이 막힘 -> 대기");
      return;
    }

    // 이 단계의 시간이 다 됐으면 다음 단계로
    step_elapsed_ += dt_;
    if (step_elapsed_ >= s.duration)
    {
      step_index_++;
      step_elapsed_ = 0.0;
      if (step_index_ >= steps_.size())
      {
        finish();
        return;
      }
      RCLCPP_INFO(get_logger(), "주차 단계: %s", steps_[step_index_].name.c_str());
      return;
    }

    // 이 단계의 속도로 움직이기
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = s.v;
    cmd.angular.z = s.w;
    cmd_pub_->publish(cmd);
  }

  // ==================== 작은 도우미 함수들 ====================

  // 정면 PSD 가 너무 가까우면 true
  bool front_blocked() const
  {
    if (front_stop_distance_ <= 0.0 || !psd_received_)
    {
      return false;
    }
    return psd_.front > 0.0 && psd_.front < front_stop_distance_;
  }

  // 바퀴 정지 명령 보내기 (속도 0)
  void stop_wheels()
  {
    cmd_pub_->publish(geometry_msgs::msg::Twist());
  }

  // control_mode 보내기. mission_state 는 "parking_node:단계이름" (GUI 에 보임)
  void publish_mode(uint8_t mode, const std::string & detail)
  {
    ControlMode msg;
    msg.header.stamp = now();
    msg.mode = mode;
    msg.mission_state = std::string(MY_TAG) + ":" + detail;
    mode_pub_->publish(msg);
  }

  // ==================== 변수 ====================

  // 파라미터
  bool use_task_planner_;
  double min_confidence_;
  double sign_area_ratio_threshold_;
  int64_t sign_hits_needed_;
  std::string parking_side_;
  bool use_parking_spot_;
  double linear_speed_;
  double angular_speed_;
  double approach_distance_;
  double turn_angle_deg_;
  double enter_distance_;
  bool enter_reverse_;
  double park_wait_time_;
  double front_stop_distance_;
  double dt_;

  // 상태
  State state_ = State::WAITING;
  int64_t sign_hits_ = 0;
  std::vector<Step> steps_;
  size_t step_index_ = 0;
  double step_elapsed_ = 0.0;

  // 마지막으로 받은 값
  ParkingSpot spot_;
  bool spot_received_ = false;
  interfaces::msg::PsdArray psd_;
  bool psd_received_ = false;

  // ROS 연결
  rclcpp::Subscription<Sign>::SharedPtr sign_sub_;
  rclcpp::Subscription<ParkingSpot>::SharedPtr spot_sub_;
  rclcpp::Subscription<interfaces::msg::PsdArray>::SharedPtr psd_sub_;
  rclcpp::Subscription<ControlMode>::SharedPtr mode_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<ControlMode>::SharedPtr mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr done_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParkingNode>());
  rclcpp::shutdown();
  return 0;
}
