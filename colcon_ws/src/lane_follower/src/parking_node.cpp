// ============================================================================
//  parking_node.cpp - 주차 구간 (1~2단계: 주차장 입구 찾기 -> 엔코더로 정확히 가서 왼쪽 90도)
// ============================================================================
//
// [트랙 모양] (위에서 본 지도, 로봇은 아래 도로를 왼쪽 -> 오른쪽으로 달린다)
//
//              ┌──────┬──────┬──────┐
//              │ 왼쪽 │ 통로 │오른쪽│   <- 주차장
//              └──────┴┐    ┌┴──────┘
//                      │    │           <- 주차장 가는 길 (양옆 노란선)
//      노란선 ─────────┘ ★  └────────── 노란선
//      ─────►  로봇 진행 방향
//      흰선   ──────────────────────────
//
//   왼쪽 노란선이 입구에서 왼쪽으로 꺾인다. 카메라(위에서 본 영상 BEV)로 보면 "가로 노란선" 이다.
//   ★ = 입구 가운데 (여기서 왼쪽으로 90도 돌면 주차장 길을 바라본다)
//
// [전체 흐름]
//
//   (1) WAITING  : 주차 표지판을 기다린다 (운전은 lane_follower)
//          │  주차 표지판이 보임
//          ▼
//   (2) SEARCH   : 운전은 계속 lane_follower. 이 노드는 BEV 왼쪽에서 가로 노란선을 찾는다
//          │  가로 노란선이 연속으로 보임
//          │  -> 바퀴 축에서 ★ 까지 거리 계산, 그 순간의 로봇 위치(odom)를 기억
//          │  -> 가로선 아래의 차선(세로 노란선) 기울기로 "도로 방향" 을 계산
//          │     (로봇이 그 순간 살짝 틀어져 있어도 도로 기준으로 정확히 가고 돌기 위해)
//          ▼
//   (3) FORWARD  : 이 노드가 운전 (control_mode = PARKING -> lane_follower 멈춤)
//          │        도로 방향을 유지하며, odom 으로 잰 거리가 ★ 까지 거리가 될 때까지 직진
//          ▼
//   (4) TURN     : odom 으로 잰 방향이 "도로 방향 + 왼쪽 90도" 가 될 때까지 제자리 회전
//          ▼
//   (5) DONE     : 멈추고 control_mode = STOP ("parking_node:at_entrance")
//                  (다음 단계 - 길 올라가기, 빈 칸 고르기, 주차 - 는 아직 없음)
//
// [거리 계산] BEV 는 400 x 400 px, 1px = 1mm (bird_eye_view.yaml, 2026-10-09 보정)
//   화면 맨 아래 줄 = 바퀴 축에서 robot_y_offset_px (14.1cm) 앞
//   가로 노란선은 입구의 가까운 쪽 선. 입구 가운데는 거기서 차선 폭 절반만큼 더 앞
//   ★ 까지 거리 = (화면 맨 아래 - 가로선 + 차선 폭/2) + robot_y_offset_px   [px = mm]
//
//        x=0                    x=400
//    y=0  ┌──────────────────────┐   <- 바퀴 축에서 54.1cm 앞
//         │━━━━━┓                │   <- 가로 노란선
//         │     ┃(차선 노란선)   │
//  y=399  └──────────────────────┘   <- 바퀴 축에서 14.1cm 앞
//                    ▲ 로봇 (x = 200)
//
// [안전]
//   - 직진 중 정면 PSD 가 front_stop_distance 보다 가까우면 멈추고 기다린다
//   - odom 이 odom_timeout 초 동안 안 오면 멈추고 기다린다
//   - GUI 에서 STOP / MANUAL 을 누르면 바로 멈추고 주차를 그만둔다
//   - 표지판을 본 뒤 search_timeout 초 안에 가로선을 못 찾으면 그만둔다 (lane_follower 는 계속 달림)

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "interfaces/msg/control_mode.hpp"
#include "interfaces/msg/psd_array.hpp"
#include "interfaces/msg/sign.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

// 이 노드가 보내는 control_mode 의 mission_state 앞에 붙는 이름 (내가 보낸 걸 다시 받으면 무시하려고)
static const char * const MY_TAG = "parking_node";

// 각도를 -pi ~ pi 사이로 (예: 190도 -> -170도)
static double wrap_angle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

// 로봇 위치와 방향 (odom 기준)
struct Pose
{
  double x = 0.0;      // [m]
  double y = 0.0;      // [m]
  double yaw = 0.0;    // [rad] 왼쪽으로 돌면 +
};

class ParkingNode : public rclcpp::Node
{
  using ControlMode = interfaces::msg::ControlMode;
  using Sign = interfaces::msg::Sign;

  enum class State
  {
    WAITING,     // 주차 표지판 기다리는 중
    SEARCH,      // 가로 노란선 찾는 중 (운전은 lane_follower)
    FORWARD,     // 입구 가운데까지 직진 (이 노드가 운전)
    TURN,        // 왼쪽 90도 회전 (이 노드가 운전)
    DONE,        // 끝 (또는 그만둠)
  };

public:
  ParkingNode()
  : Node("parking")
  {
    declare_all_parameters();

    // ---- 받는 토픽 ----
    const auto qos = rclcpp::SensorDataQoS();
    sign_sub_ = create_subscription<Sign>(
      "sign", qos, [this](Sign::ConstSharedPtr msg)
      {
        on_sign(*msg);
      });
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, qos, [this](sensor_msgs::msg::Image::ConstSharedPtr msg)
      {
        on_image(msg);
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr msg)
      {
        on_odom(*msg);
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

    // ---- 0.05초마다 step() ----
    timer_ = create_wall_timer(std::chrono::duration<double>(dt_), [this]
      {
        step();
      });

    // 시험용: 표지판 없이 바로 가로선 찾기부터
    if (skip_sign_)
    {
      state_ = State::SEARCH;
      RCLCPP_INFO(get_logger(), "주차 노드 시작: skip_sign -> 표지판 없이 바로 가로 노란선 찾기");
      return;
    }
    RCLCPP_INFO(get_logger(), "주차 노드 시작: 주차 표지판 기다림");
  }

private:
  // ==================== 파라미터 (config/parking.yaml) ====================

  void declare_all_parameters()
  {
    // 1) 표지판
    min_confidence_ = declare_parameter("min_confidence", 0.5);
    sign_area_ratio_threshold_ = declare_parameter("sign_area_ratio_threshold", 0.0);
    sign_hits_needed_ = declare_parameter("sign_hits_needed", 1);
    search_timeout_ = declare_parameter("search_timeout", 40.0);
    skip_sign_ = declare_parameter("skip_sign", false);

    // 2) 가로 노란선 찾기 (BEV)
    image_topic_ = declare_parameter("image_topic", std::string("image_bev"));
    yellow_l_min_ = declare_parameter("yellow_l_min", 80);
    yellow_b_min_ = declare_parameter("yellow_b_min", 150);
    yellow_a_max_ = declare_parameter("yellow_a_max", 135);
    bev_center_x_ = declare_parameter("bev_center_x", 0.5);
    robot_y_offset_px_ = declare_parameter("robot_y_offset_px", 141.0);
    bev_m_per_px_ = declare_parameter("bev_m_per_px", 0.001);
    lane_width_px_ = declare_parameter("lane_width_px", 295.0);
    hline_min_pixels_ = declare_parameter("hline_min_pixels", 35);
    hline_min_rows_ = declare_parameter("hline_min_rows", 3);
    hline_hits_needed_ = declare_parameter("hline_hits_needed", 2);
    road_min_rows_ = declare_parameter("road_min_rows", 40);

    // 3) 움직이기 (odom)
    forward_speed_ = declare_parameter("forward_speed", 0.08);
    forward_extra_m_ = declare_parameter("forward_extra_m", 0.0);
    heading_kp_ = declare_parameter("heading_kp", 2.0);
    turn_angle_deg_ = declare_parameter("turn_angle_deg", 90.0);
    turn_kp_ = declare_parameter("turn_kp", 2.0);
    turn_max_speed_ = declare_parameter("turn_max_speed", 0.6);
    turn_min_speed_ = declare_parameter("turn_min_speed", 0.12);
    turn_tolerance_deg_ = declare_parameter("turn_tolerance_deg", 0.5);

    // 안전
    front_stop_distance_ = declare_parameter("front_stop_distance", 0.10);
    odom_timeout_ = declare_parameter("odom_timeout", 0.5);
    const double rate = declare_parameter("control_rate", 20.0);
    dt_ = 1.0 / rate;
  }

  // ==================== 받은 메시지 처리 ====================

  // (1) 주차 표지판이 보이면 SEARCH 시작
  void on_sign(const Sign & sign)
  {
    const bool parking_sign =
      sign.type == Sign::PARKING &&
      sign.confidence >= min_confidence_ &&
      sign.area_ratio >= sign_area_ratio_threshold_;

    sign_hits_ = parking_sign ? sign_hits_ + 1 : 0;

    if (state_ == State::WAITING && sign_hits_ >= sign_hits_needed_)
    {
      RCLCPP_INFO(get_logger(), "주차 표지판 인식 (신뢰도 %.2f) -> 가로 노란선 찾기", sign.confidence);
      state_ = State::SEARCH;
      search_elapsed_ = 0.0;
      hline_hits_ = 0;
    }
  }

  // (2) SEARCH 중에만 영상에서 가로 노란선을 찾는다
  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    if (state_ != State::SEARCH)
    {
      return;
    }

    cv_bridge::CvImageConstPtr cv;
    try
    {
      cv = cv_bridge::toCvShare(msg, "bgr8");
    }
    catch (const cv_bridge::Exception & e)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "영상 변환 실패: %s", e.what());
      return;
    }

    const cv::Mat yellow = yellow_mask(cv->image);
    const std::optional<double> line_y = nearest_horizontal_line(yellow);
    hline_hits_ = line_y ? hline_hits_ + 1 : 0;
    if (hline_hits_ >= hline_hits_needed_ && odom_received_)
    {
      start_forward(*line_y, yellow.rows, road_angle(yellow, *line_y));
    }
  }

  // odom: 로봇 위치/방향 저장
  void on_odom(const nav_msgs::msg::Odometry & msg)
  {
    const auto & q = msg.pose.pose.orientation;
    pose_.x = msg.pose.pose.position.x;
    pose_.y = msg.pose.pose.position.y;
    pose_.yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    odom_received_ = true;
    last_odom_time_ = now();
  }

  // GUI 에서 STOP / MANUAL 을 누르면 그만둔다
  void on_mode(const ControlMode & msg)
  {
    if (msg.mission_state.rfind(MY_TAG, 0) == 0)
    {
      return;                    // 내가 보낸 것
    }
    const bool stop_request = msg.mode == ControlMode::STOP || msg.mode == ControlMode::MANUAL;
    const bool working = state_ == State::SEARCH || i_am_driving();
    if (stop_request && working && msg.mission_state == "turtle_gui")
    {
      if (i_am_driving())
      {
        stop_wheels();
      }
      state_ = State::DONE;
      RCLCPP_WARN(get_logger(), "GUI 에서 STOP/MANUAL -> 주차 그만둠");
    }
  }

  // ==================== 가로 노란선 찾기 ====================

  // 영상에서 노란색인 곳만 255, 나머지 0 인 흑백 그림
  cv::Mat yellow_mask(const cv::Mat & bgr) const
  {
    cv::Mat lab;
    cv::cvtColor(bgr, lab, cv::COLOR_BGR2Lab);

    cv::Mat yellow;
    cv::inRange(lab, cv::Scalar(yellow_l_min_, 0, yellow_b_min_),
      cv::Scalar(255, yellow_a_max_, 255), yellow);

    // 작은 점(잡음) 지우기
    const cv::Mat k3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(yellow, yellow, cv::MORPH_OPEN, k3);
    return yellow;
  }

  // 로봇 왼쪽 절반에서 줄마다 노란 점 개수를 세고, 개수가 많은 줄이 연속으로 붙은 덩어리 = 가로선.
  //   (세로로 선 차선은 한 줄에 20개 정도, 가로로 누운 선은 훨씬 많다)
  // 가장 아래(= 로봇에 가장 가까운) 가로선의 가운데 줄 번호를 돌려준다. 없으면 비어 있음
  std::optional<double> nearest_horizontal_line(const cv::Mat & yellow) const
  {
    const int left_width = static_cast<int>(bev_center_x_ * yellow.cols);
    std::optional<double> nearest;
    int start = -1;              // 지금 덩어리가 시작한 줄 (-1 = 덩어리 밖)

    for (int y = 0; y <= yellow.rows; y++)
    {
      const bool full = y < yellow.rows &&
        cv::countNonZero(yellow.row(y).colRange(0, left_width)) >= hline_min_pixels_;
      if (full && start < 0)
      {
        start = y;               // 덩어리 시작
      }
      else if (!full && start >= 0)
      {
        if (y - start >= hline_min_rows_)    // 너무 얇으면 잡음
        {
          nearest = (start + y - 1) / 2.0;   // 아래로 갈수록 덮어써서, 마지막 = 가장 아래
        }
        start = -1;
      }
    }
    return nearest;
  }

  // 가로선 아래(로봇 쪽)에 보이는 차선(세로 노란선)이 로봇 정면에서 몇 도 틀어져 있는지 [rad]
  //   + = 도로가 로봇보다 왼쪽으로 향함. 차선이 충분히 안 보이면 0 (로봇 방향 = 도로 방향으로 봄)
  //   줄마다 노란 점들의 가운데 x 를 모아 직선 x = a + b*y 로 잇고, 그 기울기로 각도를 구한다
  double road_angle(const cv::Mat & yellow, double line_y) const
  {
    const int left_width = static_cast<int>(bev_center_x_ * yellow.cols);
    const int first_row = static_cast<int>(line_y) + 15;            // 가로선 두께만큼 아래부터
    double n = 0.0, sy = 0.0, sx = 0.0, syy = 0.0, sxy = 0.0;       // 최소제곱 합
    for (int y = std::max(0, first_row); y < yellow.rows; y++)
    {
      const cv::Moments m = cv::moments(yellow.row(y).colRange(0, left_width), true);
      if (m.m00 < 1.0 || m.m00 >= hline_min_pixels_)
      {
        continue;                // 노란 점 없음, 또는 가로선 줄
      }
      const double x = m.m10 / m.m00;
      n += 1.0;
      sy += y;
      sx += x;
      syy += y * y;
      sxy += x * y;
    }
    const double denom = n * syy - sy * sy;
    if (n < road_min_rows_ || std::abs(denom) < 1e-9)
    {
      return 0.0;
    }
    const double b = (n * sxy - sx * sy) / denom;   // 아래로 1 줄 갈 때 x 변화
    return std::atan(b);         // 위(앞)로 갈수록 x 가 작아지면(b > 0) 도로가 왼쪽으로 향함
  }

  // ==================== 움직이기 시작 ====================

  // (3) 가로선을 찾음 -> 입구 가운데까지 거리 계산, 지금 위치와 도로 방향 기억, 직진 시작
  void start_forward(double line_y, int rows, double road_angle_rad)
  {
    const double center_y = line_y - lane_width_px_ / 2.0;            // 입구 가운데 (화면 줄)
    const double ahead_px = (rows - 1 - center_y) + robot_y_offset_px_;
    goal_distance_ = ahead_px * bev_m_per_px_ + forward_extra_m_;
    start_pose_ = pose_;
    start_pose_.yaw = wrap_angle(pose_.yaw + road_angle_rad);         // 도로 방향

    state_ = State::FORWARD;
    RCLCPP_INFO(get_logger(),
      "가로 노란선 발견 (화면 y=%.0f, 도로가 로봇보다 %+.1f도) -> 입구 가운데까지 %.3fm 직진",
      line_y, road_angle_rad * 180.0 / M_PI, goal_distance_);
  }

  // (4) 직진 끝 -> 회전 시작. 목표 방향 = 도로 방향 + 90도
  void start_turn()
  {
    goal_yaw_ = wrap_angle(start_pose_.yaw + turn_angle_deg_ * M_PI / 180.0);
    state_ = State::TURN;
    RCLCPP_INFO(get_logger(), "직진 끝 (%.3fm) -> 왼쪽 %.0f도 회전", traveled(), turn_angle_deg_);
  }

  // (5) 회전 끝 -> 멈추고 STOP
  void finish()
  {
    stop_wheels();
    state_ = State::DONE;
    publish_mode(ControlMode::STOP, "at_entrance");
    RCLCPP_INFO(get_logger(), "입구 도착: 직진 %.3fm, 도로 기준 %.1f도 -> 정지 (다음 단계는 아직 없음)",
      traveled(), wrap_angle(pose_.yaw - start_pose_.yaw) * 180.0 / M_PI);
  }

  // ==================== 0.05초마다 ====================

  void step()
  {
    if (state_ == State::SEARCH)
    {
      check_search_timeout();
      return;                    // 운전은 lane_follower
    }
    if (!i_am_driving())
    {
      return;
    }

    // lane_follower 가 멈춰 있도록 "주차 중" 을 계속 알린다
    publish_mode(ControlMode::PARKING, state_ == State::FORWARD ? "forward" : "turn");

    // odom 이 끊기면 위치를 모르므로 멈추고 기다린다
    if ((now() - last_odom_time_).seconds() > odom_timeout_)
    {
      stop_wheels();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "odom 이 안 옴 -> 정지");
      return;
    }

    if (state_ == State::FORWARD)
    {
      step_forward();
    }
    else
    {
      step_turn();
    }
  }

  void check_search_timeout()
  {
    search_elapsed_ += dt_;
    if (search_timeout_ > 0.0 && search_elapsed_ >= search_timeout_)
    {
      state_ = State::DONE;
      RCLCPP_WARN(get_logger(), "가로 노란선을 못 찾음 (%.0f초) -> 주차 그만둠", search_timeout_);
    }
  }

  // 직진: 도로 방향을 유지하면서 목표 거리까지
  void step_forward()
  {
    if (traveled() >= goal_distance_)
    {
      stop_wheels();
      start_turn();
      return;
    }
    if (front_closer_than(front_stop_distance_))
    {
      stop_wheels();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "앞이 막힘 -> 대기");
      return;
    }
    // 도로 방향에서 틀어지면 되돌리기 (도로 방향 - 지금 방향) x heading_kp
    const double w = heading_kp_ * wrap_angle(start_pose_.yaw - pose_.yaw);
    drive(forward_speed_, w);
  }

  // 회전: 남은 각도에 비례해서 돌고, 거의 다 오면 멈춤
  void step_turn()
  {
    const double remain = wrap_angle(goal_yaw_ - pose_.yaw);          // + 면 왼쪽으로 더 돌아야 함
    if (std::abs(remain) <= turn_tolerance_deg_ * M_PI / 180.0)
    {
      finish();
      return;
    }
    double w = turn_kp_ * remain;
    w = std::clamp(std::abs(w), turn_min_speed_, turn_max_speed_);    // 너무 느리거나 빠르지 않게
    drive(0.0, std::copysign(w, remain));
  }

  // ==================== 도우미 ====================

  bool i_am_driving() const
  {
    return state_ == State::FORWARD || state_ == State::TURN;
  }

  // 직진을 시작한 곳에서 도로 방향으로 얼마나 왔는지 [m] (옆으로 간 건 빼고)
  double traveled() const
  {
    const double dx = pose_.x - start_pose_.x;
    const double dy = pose_.y - start_pose_.y;
    return dx * std::cos(start_pose_.yaw) + dy * std::sin(start_pose_.yaw);
  }

  // 정면 PSD 가 distance [m] 보다 가까우면 true
  bool front_closer_than(double distance) const
  {
    if (distance <= 0.0 || !psd_received_)
    {
      return false;
    }
    return psd_.front > 0.0 && psd_.front <= distance;
  }

  void drive(double v, double w)
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = v;
    cmd.angular.z = w;
    cmd_pub_->publish(cmd);
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

  // ==================== 변수 ====================

  // 파라미터
  double min_confidence_;
  double sign_area_ratio_threshold_;
  int64_t sign_hits_needed_;
  double search_timeout_;
  bool skip_sign_;
  std::string image_topic_;
  int64_t yellow_l_min_;
  int64_t yellow_b_min_;
  int64_t yellow_a_max_;
  double bev_center_x_;
  double robot_y_offset_px_;
  double bev_m_per_px_;
  double lane_width_px_;
  int64_t hline_min_pixels_;
  int64_t hline_min_rows_;
  int64_t hline_hits_needed_;
  int64_t road_min_rows_;
  double forward_speed_;
  double forward_extra_m_;
  double heading_kp_;
  double turn_angle_deg_;
  double turn_kp_;
  double turn_max_speed_;
  double turn_min_speed_;
  double turn_tolerance_deg_;
  double front_stop_distance_;
  double odom_timeout_;
  double dt_;

  // 상태
  State state_ = State::WAITING;
  int64_t sign_hits_ = 0;
  int64_t hline_hits_ = 0;
  double search_elapsed_ = 0.0;
  Pose pose_;                      // 지금 로봇 위치/방향 (odom)
  Pose start_pose_;                // 가로선을 찾은 순간의 위치, 도로 방향
  double goal_distance_ = 0.0;     // 입구 가운데까지 직진할 거리 [m]
  double goal_yaw_ = 0.0;          // 회전 목표 방향 [rad]
  bool odom_received_ = false;
  rclcpp::Time last_odom_time_;
  interfaces::msg::PsdArray psd_;
  bool psd_received_ = false;

  // ROS 연결
  rclcpp::Subscription<Sign>::SharedPtr sign_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<interfaces::msg::PsdArray>::SharedPtr psd_sub_;
  rclcpp::Subscription<ControlMode>::SharedPtr mode_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<ControlMode>::SharedPtr mode_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParkingNode>());
  rclcpp::shutdown();
  return 0;
}
