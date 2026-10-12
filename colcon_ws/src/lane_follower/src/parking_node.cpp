// ============================================================================
//  parking_node.cpp - 주차 구간 담당
// ============================================================================
//
// [트랙 모양]
//   평소 차선 : 왼쪽 노란선, 오른쪽 흰선
//   주차 구역 : 왼쪽, 오른쪽 둘 다 노란선. 양옆에 주차칸이 있고, 한쪽 칸에는 장애물이 있다
//   -> 장애물이 없는 반대쪽 칸에 주차했다가 나와서 계속 간다
//
// [전체 흐름]
//
//   (1) WAITING      : 주차 표지판을 기다린다
//          │  주차 표지판이 가까이 3번 연속 보임
//          ▼
//   (2) ZONE         : 주차 구역 안. 운전은 lane_follower 가 한다 (왼쪽 노란선 따라가기)
//          │            lane_detection 은 오른쪽에서 흰선만 찾기 때문에, 오른쪽이 노란선인
//          │            주차 구역에서는 자동으로 왼쪽 노란선을 기준으로 주행한다.
//          │
//          │  왼쪽 PSD <= 0.2m  -> 왼쪽 칸에 장애물 -> 오른쪽 칸에 주차
//          │  오른쪽 PSD <= 0.2m -> 오른쪽 칸에 장애물 -> 왼쪽 칸에 주차
//          ▼
//   (3) MANEUVER     : 이 노드가 직접 바퀴를 움직인다 (lane_follower 는 멈춤)
//          │   ① 빈 칸 쪽으로 90도 회전
//          │   ② 2초 전진 (칸 안으로)
//          │   ③ 2초 대기 (주차 완료)
//          │   ④ 180도 회전 (칸 밖을 보게)
//          │   ⑤ 앞 PSD 가 exit_front_distance 이하가 될 때까지 전진
//          │      (건너편 장애물이 이만큼 가까워지면 = 원래 차선 위치로 돌아온 것)
//          │   ⑥ 90도 회전해서 원래 진행 방향으로
//          ▼
//   (2) ZONE 으로 복귀 : 다시 lane_follower 가 왼쪽 노란선 따라 운전
//          │  오른쪽 선이 다시 흰선으로 보임 (주차 구역 끝)
//          ▼
//   (4) FINISHED     : parking_done 을 보내고 끝. 평소 자율주행 계속
//
// [회전 방향] 오른쪽 칸에 주차하면 ①④⑥ 을 전부 오른쪽으로 돈다.
//   ① 오른쪽 90 -> ④ 오른쪽 180 -> ⑥ 오른쪽 90 = 한 바퀴(360도) -> 원래 방향.
//   (⑥ 을 반대로 돌면 180도가 되어 왔던 길로 돌아간다)
//
// [task_planner 와 함께 쓸 때] (use_task_planner: true)
//   control_mode 는 task_planner 가 보낸다. 이 노드는 parking_active (지금 직접 운전 중인지)와
//   parking_done (주차 구간 끝) 만 알려주고, task_planner 가 그걸 보고 모드를 정한다.
//
// [안전]
//   - 전진 중 정면 PSD 가 front_stop_distance 보다 가까우면 멈추고 기다린다.
//   - GUI 에서 STOP / MANUAL 을 누르면 바로 멈추고 주차를 포기한다.
//   - 각 단계에는 최대 시간이 있어서 조건이 안 맞아도 영원히 움직이지 않는다.

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "interfaces/msg/control_mode.hpp"
#include "interfaces/msg/lane_info.hpp"
#include "interfaces/msg/psd_array.hpp"
#include "interfaces/msg/sign.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

// 이 노드가 보내는 control_mode 의 mission_state 앞에 붙는 이름 (내가 보낸 걸 다시 받으면 무시하려고)
static const char * const MY_TAG = "parking_node";

// 주차 동작 한 단계
struct Step
{
  std::string name;          // 단계 이름 (로그, GUI 표시용)
  double v;                  // 앞뒤 속도 [m/s]   + 앞, - 뒤
  double w;                  // 회전 속도 [rad/s] + 왼쪽, - 오른쪽
  double duration;           // 이 시간 [s] 동안 한다 (until_front 가 있으면 "최대" 시간)
  double until_front;        // 0 보다 크면: 앞 PSD 가 이 거리 [m] 이하가 되면 바로 다음 단계로
};

class ParkingNode : public rclcpp::Node
{
  using ControlMode = interfaces::msg::ControlMode;
  using Sign = interfaces::msg::Sign;
  using LaneInfo = interfaces::msg::LaneInfo;

  enum class State
  {
    WAITING,     // 주차 표지판 기다리는 중
    ZONE,        // 주차 구역 안, lane_follower 가 운전 (왼쪽 노란선 따라가기)
    MANEUVER,    // 이 노드가 직접 주차 동작 중
    FINISHED,    // 주차 구간 끝 (또는 포기)
  };

public:
  ParkingNode()
  : Node("parking")
  {
    // ---- 파라미터 (config/parking.yaml) ----
    use_task_planner_ = declare_parameter("use_task_planner", false);
    // 표지판
    min_confidence_ = declare_parameter("min_confidence", 0.5);
    sign_area_ratio_threshold_ = declare_parameter("sign_area_ratio_threshold", 0.03);
    sign_hits_needed_ = declare_parameter("sign_hits_needed", 3);
    // 장애물 칸 찾기
    side_psd_threshold_ = declare_parameter("side_psd_threshold", 0.20);
    side_psd_hits_needed_ = declare_parameter("side_psd_hits_needed", 3);
    // 주차 구역 들어감 / 나옴 (오른쪽 흰선)
    lane_frames_needed_ = declare_parameter("lane_frames_needed", 5);
    zone_timeout_ = declare_parameter("zone_timeout", 40.0);
    // 주차 동작
    linear_speed_ = declare_parameter("linear_speed", 0.08);
    angular_speed_ = declare_parameter("angular_speed", 0.6);
    align_time_ = declare_parameter("align_time", 0.0);
    enter_time_ = declare_parameter("enter_time", 2.0);
    park_wait_time_ = declare_parameter("park_wait_time", 2.0);
    exit_front_distance_ = declare_parameter("exit_front_distance", 0.30);
    exit_timeout_ = declare_parameter("exit_timeout", 4.0);
    // 안전
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
    lane_sub_ = create_subscription<LaneInfo>(
      "lane_info", qos, [this](LaneInfo::ConstSharedPtr msg)
      {
        on_lane(*msg);
      });
    psd_sub_ = create_subscription<interfaces::msg::PsdArray>(
      "psd", qos, [this](interfaces::msg::PsdArray::ConstSharedPtr msg)
      {
        on_psd(*msg);
      });
    mode_sub_ = create_subscription<ControlMode>(
      "control_mode", 10, [this](ControlMode::ConstSharedPtr msg)
      {
        on_mode(*msg);
      });

    // ---- 보내는 토픽 ----
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    mode_pub_ = create_publisher<ControlMode>("control_mode", 10);
    active_pub_ = create_publisher<std_msgs::msg::Bool>("parking_active", 10);
    done_pub_ = create_publisher<std_msgs::msg::Bool>("parking_done", 10);

    // ---- 0.05초마다 step() ----
    timer_ = create_wall_timer(std::chrono::duration<double>(dt_), [this]
      {
        step();
      });

    RCLCPP_INFO(get_logger(), "주차 노드 시작: 주차 표지판 기다림 (task_planner %s)",
      use_task_planner_ ? "사용" : "없음");
  }

private:
  // ==================== 받은 메시지 처리 ====================

  // (1) 표지판: 주차 표지판이 가까이 연속으로 보이면 주차 구역 시작
  void on_sign(const Sign & sign)
  {
    const bool close_parking_sign =
      sign.type == Sign::PARKING &&
      sign.confidence >= min_confidence_ &&
      sign.area_ratio >= sign_area_ratio_threshold_;

    sign_hits_ = close_parking_sign ? sign_hits_ + 1 : 0;

    if (state_ == State::WAITING && sign_hits_ >= sign_hits_needed_)
    {
      RCLCPP_INFO(get_logger(), "주차 표지판 인식 -> 주차 구역 모드 (왼쪽 노란선 따라가기)");
      state_ = State::ZONE;
      zone_elapsed_ = 0.0;
      inside_zone_ = false;
      parked_ = false;
      left_hits_ = 0;
      right_hits_ = 0;
      white_seen_frames_ = 0;
      white_lost_frames_ = 0;
    }
  }

  // 차선 정보: 오른쪽 흰선이 사라지면 "주차 구역 안", 다시 보이면 "주차 구역 끝"
  void on_lane(const LaneInfo & lane)
  {
    if (state_ != State::ZONE)
    {
      return;
    }

    if (lane.right_detected)
    {
      white_seen_frames_++;
      white_lost_frames_ = 0;
    }
    else
    {
      white_lost_frames_++;
      white_seen_frames_ = 0;
    }

    // 오른쪽 흰선이 연속으로 안 보임 -> 오른쪽이 노란선인 주차 구역에 들어왔다
    if (!inside_zone_ && white_lost_frames_ >= lane_frames_needed_)
    {
      inside_zone_ = true;
      RCLCPP_INFO(get_logger(), "주차 구역 진입 (오른쪽 흰선 없음)");
    }

    // 구역 안에 있다가 오른쪽 흰선이 다시 연속으로 보임 -> 주차 구역 끝
    if (inside_zone_ && white_seen_frames_ >= lane_frames_needed_)
    {
      finish(parked_ ? "주차 구역 끝 (오른쪽 흰선 다시 보임)" :
        "주차 구역 끝, 하지만 주차할 칸을 못 찾음");
    }
  }

  // PSD: 구역 안에서 한쪽이 가까우면 그쪽 칸에 장애물 -> 반대쪽 칸에 주차
  void on_psd(const interfaces::msg::PsdArray & psd)
  {
    psd_ = psd;
    psd_received_ = true;

    if (state_ != State::ZONE || !inside_zone_ || parked_)
    {
      return;
    }

    left_hits_ = (psd.left > 0.0 && psd.left <= side_psd_threshold_) ? left_hits_ + 1 : 0;
    right_hits_ = (psd.right > 0.0 && psd.right <= side_psd_threshold_) ? right_hits_ + 1 : 0;

    if (left_hits_ >= side_psd_hits_needed_)
    {
      RCLCPP_INFO(get_logger(), "왼쪽 칸에 장애물 (%.2fm) -> 오른쪽 칸에 주차", psd.left);
      start_maneuver(-1.0);      // -1 = 오른쪽으로 돈다
    }
    else if (right_hits_ >= side_psd_hits_needed_)
    {
      RCLCPP_INFO(get_logger(), "오른쪽 칸에 장애물 (%.2fm) -> 왼쪽 칸에 주차", psd.right);
      start_maneuver(+1.0);      // +1 = 왼쪽으로 돈다
    }
  }

  // control_mode: GUI 에서 STOP / MANUAL 을 누르면 포기
  void on_mode(const ControlMode & msg)
  {
    if (msg.mission_state.rfind(MY_TAG, 0) == 0)
    {
      return;                    // 내가 보낸 것
    }
    const bool stop_request = msg.mode == ControlMode::STOP || msg.mode == ControlMode::MANUAL;
    const bool working = state_ == State::ZONE || state_ == State::MANEUVER;
    if (stop_request && working && msg.mission_state == "turtle_gui")
    {
      abort("GUI 에서 STOP/MANUAL");
    }
  }

  // ==================== 주차 동작 ====================

  // 주차 동작 단계 목록을 만들고 시작. turn = +1 왼쪽 칸, -1 오른쪽 칸
  void start_maneuver(double turn)
  {
    const double v = linear_speed_;
    const double w = turn * angular_speed_;              // 돌 방향 포함한 회전 속도
    const double t90 = (M_PI / 2.0) / angular_speed_;    // 90도 도는 시간
    const double t180 = M_PI / angular_speed_;           // 180도 도는 시간

    steps_.clear();
    if (align_time_ > 0.0)
    {
      steps_.push_back({"align", v, 0.0, align_time_, 0.0});        // 칸 가운데로 조금 더 가기
    }
    steps_.push_back({"turn_in", 0.0, w, t90, 0.0});                 // ① 빈 칸 쪽으로 90도
    steps_.push_back({"enter", v, 0.0, enter_time_, 0.0});           // ② 칸 안으로 전진
    steps_.push_back({"wait", 0.0, 0.0, park_wait_time_, 0.0});      // ③ 대기
    steps_.push_back({"turn_around", 0.0, w, t180, 0.0});            // ④ 180도
    steps_.push_back({"exit", v, 0.0, exit_timeout_, exit_front_distance_});  // ⑤ 앞 PSD 까지 전진
    steps_.push_back({"turn_back", 0.0, w, t90, 0.0});               // ⑥ 원래 방향으로 90도

    step_index_ = 0;
    step_elapsed_ = 0.0;
    state_ = State::MANEUVER;
    publish_active(true);
    RCLCPP_INFO(get_logger(), "주차 단계: %s", steps_[0].name.c_str());
  }

  // 주차 동작 끝 -> 다시 lane_follower 가 왼쪽 노란선 따라 운전
  void end_maneuver()
  {
    stop_wheels();
    parked_ = true;
    state_ = State::ZONE;
    publish_active(false);
    if (!use_task_planner_)
    {
      publish_mode(ControlMode::LANE, "zone");
    }
    RCLCPP_INFO(get_logger(), "주차 완료 -> 왼쪽 노란선 따라 계속 주행");
  }

  // 주차 구간 끝
  void finish(const char * reason)
  {
    if (state_ == State::MANEUVER)
    {
      stop_wheels();
      publish_active(false);
    }
    state_ = State::FINISHED;

    std_msgs::msg::Bool done;
    done.data = true;
    done_pub_->publish(done);
    if (!use_task_planner_)
    {
      publish_mode(ControlMode::LANE, "done");
    }
    RCLCPP_INFO(get_logger(), "%s -> 평소 자율주행으로", reason);
  }

  // 포기
  void abort(const char * reason)
  {
    if (state_ == State::MANEUVER)
    {
      stop_wheels();
      publish_active(false);
    }
    state_ = State::FINISHED;
    RCLCPP_WARN(get_logger(), "주차 중단: %s", reason);
  }

  // ==================== 0.05초마다 ====================

  void step()
  {
    // 주차 구역에서 너무 오래 있으면 (흰선을 못 찾는 등) 그냥 끝낸다
    if (state_ == State::ZONE)
    {
      zone_elapsed_ += dt_;
      if (zone_elapsed_ >= zone_timeout_)
      {
        finish("주차 구역 시간 초과");
      }
      return;                    // ZONE 에서는 lane_follower 가 운전하므로 바퀴 명령을 안 보낸다
    }
    if (state_ != State::MANEUVER)
    {
      return;
    }

    const Step & s = steps_[step_index_];

    // lane_follower 가 멈춰 있도록 "주차 중" 을 계속 알린다 (task_planner 가 있으면 그쪽이 한다)
    if (!use_task_planner_)
    {
      publish_mode(ControlMode::PARKING, s.name);
    }

    // 안전: 전진 중 앞이 너무 가까우면 멈추고 기다린다
    if (s.v > 0.0 && front_closer_than(front_stop_distance_))
    {
      stop_wheels();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "앞이 막힘 -> 대기");
      return;
    }

    // 다음 단계로 넘어갈지: 시간이 다 됐거나, 앞 PSD 조건을 만족했으면
    step_elapsed_ += dt_;
    const bool time_up = step_elapsed_ >= s.duration;
    const bool front_reached = s.until_front > 0.0 && front_closer_than(s.until_front);
    if (time_up || front_reached)
    {
      step_index_++;
      step_elapsed_ = 0.0;
      if (step_index_ >= steps_.size())
      {
        end_maneuver();
        return;
      }
      RCLCPP_INFO(get_logger(), "주차 단계: %s", steps_[step_index_].name.c_str());
      return;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = s.v;
    cmd.angular.z = s.w;
    cmd_pub_->publish(cmd);
  }

  // ==================== 도우미 ====================

  // 정면 PSD 가 distance [m] 보다 가까우면 true
  bool front_closer_than(double distance) const
  {
    if (distance <= 0.0 || !psd_received_)
    {
      return false;
    }
    return psd_.front > 0.0 && psd_.front <= distance;
  }

  void stop_wheels()
  {
    cmd_pub_->publish(geometry_msgs::msg::Twist());
  }

  void publish_mode(uint8_t mode, const std::string & detail)
  {
    ControlMode msg;
    msg.header.stamp = now();
    msg.mode = mode;
    msg.mission_state = std::string(MY_TAG) + ":" + detail;
    mode_pub_->publish(msg);
  }

  void publish_active(bool active)
  {
    std_msgs::msg::Bool msg;
    msg.data = active;
    active_pub_->publish(msg);
  }

  // ==================== 변수 ====================

  // 파라미터
  bool use_task_planner_;
  double min_confidence_;
  double sign_area_ratio_threshold_;
  int64_t sign_hits_needed_;
  double side_psd_threshold_;
  int64_t side_psd_hits_needed_;
  int64_t lane_frames_needed_;
  double zone_timeout_;
  double linear_speed_;
  double angular_speed_;
  double align_time_;
  double enter_time_;
  double park_wait_time_;
  double exit_front_distance_;
  double exit_timeout_;
  double front_stop_distance_;
  double dt_;

  // 상태
  State state_ = State::WAITING;
  int64_t sign_hits_ = 0;
  bool inside_zone_ = false;       // 오른쪽 흰선이 사라진 걸 확인했는지 (= 주차 구역 안)
  bool parked_ = false;            // 이번 구역에서 주차를 했는지
  double zone_elapsed_ = 0.0;
  int64_t left_hits_ = 0;
  int64_t right_hits_ = 0;
  int64_t white_seen_frames_ = 0;
  int64_t white_lost_frames_ = 0;
  std::vector<Step> steps_;
  size_t step_index_ = 0;
  double step_elapsed_ = 0.0;

  interfaces::msg::PsdArray psd_;
  bool psd_received_ = false;

  // ROS 연결
  rclcpp::Subscription<Sign>::SharedPtr sign_sub_;
  rclcpp::Subscription<LaneInfo>::SharedPtr lane_sub_;
  rclcpp::Subscription<interfaces::msg::PsdArray>::SharedPtr psd_sub_;
  rclcpp::Subscription<ControlMode>::SharedPtr mode_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<ControlMode>::SharedPtr mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_pub_;
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
