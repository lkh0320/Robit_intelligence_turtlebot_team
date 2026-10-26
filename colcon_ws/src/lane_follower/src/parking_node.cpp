// ============================================================================
//  parking_node.cpp - 주차 구간 (카메라만 보고: 입구에서 왼쪽으로 꺾기 -> 주차장 길 올라가기)
// ============================================================================
//
// [트랙 모양] (위에서 본 지도, 로봇은 아래 도로를 왼쪽 -> 오른쪽으로 달린다)
//
//              ┌──────┬──────┬──────┐
//              │ 왼쪽 │ 통로 │오른쪽│   <- 주차장
//              └──────┴╮    ╭┴──────┘
//                      │    │           <- 주차장 가는 길 (양옆 노란선)
//      노란선 ─────────╯ ★  ╰────────── 노란선  (입구 모서리는 둥글다)
//      ─────►  로봇 진행 방향
//      흰선   ──────────────────────────  (입구를 지나서도 곧게 이어진다)
//
// [전체 흐름]
//
//   (1) WAITING  : 주차 표지판을 기다린다 (운전은 lane_follower)
//          │  주차 표지판이 가까이 보임 (크기 sign_area_ratio_threshold 이상)
//          ▼
//   (2) APPROACH : 이 노드가 운전한다 (control_mode = PARKING -> lane_follower 멈춤)
//          │        오른쪽 흰선만 보고 똑바로 간다 (왼쪽 노란선이 입구로 휘어도 안 따라감)
//          │        동시에 BEV 왼쪽 아래 칸을 본다:
//          │          노란선 있음 -> 없음 (입구의 빈 곳이 지나감) -> 다시 있음 (입구 반대쪽 모서리)
//          │  "다시 있음" = 반대쪽 노란선이 화면 맨 아래에 닿음
//          │  입구 가운데는 반대쪽 선에서 차선 폭 절반(약 14.7cm) 앞.
//          │  화면 맨 아래가 바퀴 축에서 그보다 멀면(카메라 각도에 따라 14.1cm / 17.9cm),
//          │  바퀴 축이 아직 입구 가운데에 못 온 것 -> 남은 거리만큼 더 직진 (EXTRA)
//          ▼
//   (2-1) EXTRA  : 남은 거리 / 속도 만큼의 시간 동안 흰선 보고 더 직진 (남은 거리가 0 이면 건너뜀)
//          ▼
//   (3) TURN     : 제자리에서 왼쪽으로 돈다.
//          │        BEV 에 입구 길의 노란선 두 개(왼쪽, 오른쪽)가 세로로 똑바로 보이면 다음으로
//          │        (aligned_deg 4도 이내. 한 선만 보여도 길 옆선 자리에 세로로 서 있으면 됨)
//          ▼
//   (4) STEM     : 주차장 길을 올라간다. 양쪽이 노란선 -> 두 노란선의 가운데로
//          │        노란선이 끝나면 같은 줄로 흰 점선(주차장 통로와 칸을 나누는 선)이 이어진다
//          │        -> 노란선과 흰선을 둘 다 차선으로 보고 그 가운데로 계속 직진 (stem_use_white)
//          │        한쪽만 보이면 그 선에서 차선 폭 절반만큼 떨어져서 간다
//          │  차선(노란선 / 흰 점선)이 둘 다 연속으로 안 보임 = 통로 끝 -> (5)
//          │
//          │  옆 PSD 가 side_psd_threshold (0.15m) 이하로 연속 side_hits_needed 번 = 그쪽에 장애물 (기억).
//          │  계속 직진하다가 앞의 가로 흰 선(주차장 끝 테두리)이 화면 맨 아래 근처(white_trigger_y)에 오면:
//          │    왼쪽 장애물  -> 왼쪽으로 side_turn_deg (90도 = 가로 흰 선이 세로가 될 때까지) (SIDE_TURN)
//          │    오른쪽 장애물 -> 오른쪽으로 side_turn_deg (90도)
//          │    장애물을 못 봤으면 -> 그냥 멈춤 ("parking_node:stem_end")
//          │  side_turn_when 으로 도는 순간을 바꿀 수 있다:
//          │    white = 가로 흰 선이 화면 아래에 올 때 (기본), seen = PSD 로 보자마자,
//          │    clear = 장애물을 다 지나가서 그쪽 PSD 가 side_clear_threshold 보다 멀어질 때
//          │    (side_turn_toward: false 면 반대쪽으로)
//          │  (각도: 화면의 직선(흰선/노란선)들이 돈 각도를 카메라로 잰다. 못 재면 odom, odom 도 없으면 "속도 x 시간")
//          ▼
//   (4-2) BACKUP : 90도 돈 뒤 backup_time (3초) 동안 backup_speed 로 후진
//          │        (exit_enable: false 면 여기서 멈춤 - STOP "parking_node:turned_left_backed" / "turned_right_backed")
//          ▼
//   (4-3) EXIT_ARC : 두 바퀴 다 앞으로, 속도 차를 줘서 곡선으로 90도 돈다 (90도 돌았던 쪽과 같은 쪽)
//          │        반지름 = 후진한 거리 (backup_speed x backup_time) -> 다 돌면 통로 가운데에서 길 쪽을 본다
//          │        멈추는 때:
//          │          - 들어왔던 길의 노란선이 보이면: 그 선이 세로가 되거나 세로를 지나치는 순간
//          │            (실제로 도는 양이 명령과 달라도 화면을 보고 멈춘다)
//          │          - 노란선이 안 보이면: 화면 직선들이 돈 각도(SIDE_TURN 과 같은 방법)가 90도,
//          │            직선도 안 보이면 "회전 속도 x 시간" 이 90도
//          │        예) 왼쪽 장애물: 왼쪽 보고 돌아서 오른쪽 칸으로 후진한 상태
//          │            -> 왼쪽으로 곡선 (앞으로 가면서 왼쪽으로 휨) -> 아래(들어왔던 길)를 봄
//          ▼
//   (4-4) EXIT_STEM : 들어왔던 길을 반대로 내려간다 (흰 점선 -> 노란선, 두 차선 가운데로)
//          │        도로의 흰선(가로)이 화면 아래(exit_stop_y)에 오면 = 노란선을 지나 도로에 나옴
//          │        STOP ("parking_node:exited")
//          ▼
//   (5) DONE     : 멈추고 control_mode = STOP ("parking_node:exited", 장애물을 못 봤으면 "parking_node:stem_end")
//                  (다음 단계 - 도로로 다시 들어가기 - 는 아직 없음)
//          │  GUI 에서 LANE 을 누르면 (1) WAITING 으로 돌아가 다음 바퀴에 다시 주차한다
//          │  (skip_sign 이면 바로 (2) APPROACH 부터)
//
// [위에서 본 영상(BEV)]  400 x 400 px, 1px = 1mm. 화면 맨 아래 = 바퀴 축 17.9cm 앞 (2026-10-13 카메라 각도)
//
//        x=0                    x=400
//    y=0  ┌──────────────────────┐
//         │  ╭──                 │   <- 입구 반대쪽 모서리 (내려온다)
//         │  │       흰선 ┃      │
//         │  ╰──          ┃      │   <- 입구 이쪽 모서리
//         │┌────┐         ┃      │
//  y=399  └┴────┴─────────┴──────┘
//          └ 왼쪽 아래 칸 (watch_x_max x watch_rows)
//
// [안전]
//   - 직진 중 정면 PSD 가 front_stop_distance 보다 가까우면 멈추고 기다린다
//   - 영상이 image_timeout 초 동안 안 오면 멈춘다
//   - GUI 에서 STOP / MANUAL 을 누르면 바로 멈추고 주차를 그만둔다
//   - approach_timeout / turn_timeout / stem_timeout 이 지나면 멈추고 그만둔다

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

// 화면 위의 직선 하나: x = a + b * y  (y 는 화면 줄 번호, 아래로 갈수록 커짐)
struct Line
{
  double a = 0.0;
  double b = 0.0;
  double rms = 0.0;    // 점들이 직선에서 평균 얼마나 떨어져 있는지 [px]. 크면 두 선이 섞였거나 휜 선

  // 줄 y 에서의 가로 위치 [px]
  double x_at(double y) const
  {
    return a + b * y;
  }

  // 선이 로봇 정면에서 왼쪽으로 기운 각도 [rad]
  //   위(앞)로 갈수록 x 가 작아지면(b > 0) 선이 왼쪽으로 향한다
  double lean_left() const
  {
    return std::atan(b);
  }
};

class ParkingNode : public rclcpp::Node
{
  using ControlMode = interfaces::msg::ControlMode;
  using Sign = interfaces::msg::Sign;

  enum class State
  {
    WAITING,     // 주차 표지판 기다리는 중 (운전은 lane_follower)
    APPROACH,    // 흰선 보고 직진하며 입구 찾는 중 (이 노드가 운전)
    EXTRA,       // 입구 가운데까지 남은 거리만큼 더 직진 (이 노드가 운전)
    TURN,        // 입구에서 왼쪽으로 도는 중 (이 노드가 운전)
    STEM,        // 주차장 길 -> 통로를 두 차선(노란선 / 흰 점선) 가운데로 올라가는 중 (이 노드가 운전)
    SIDE_TURN,   // 장애물 쪽으로 90도 도는 중 (이 노드가 운전)
    BACKUP,      // 90도 돈 뒤 backup_time 초 후진 (이 노드가 운전)
    EXIT_ARC,    // 두 바퀴 다 앞으로, 곡선으로 90도 돌아 길 쪽 보기 (이 노드가 운전)
    EXIT_STEM,   // 들어왔던 길을 반대로 내려가 도로로 나가기 (이 노드가 운전)
    DONE,        // 끝 (또는 그만둠)
  };

  // 왼쪽 아래 칸의 노란선 변화 단계
  enum class Gap
  {
    BEFORE,      // 아직 입구 전 (노란선 있음)
    IN_GAP,      // 입구의 빈 곳 (노란선 없음)
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
    psd_sub_ = create_subscription<interfaces::msg::PsdArray>(
      "psd", qos, [this](interfaces::msg::PsdArray::ConstSharedPtr msg)
      {
        psd_ = *msg;
        psd_received_ = true;
        check_side_psd();
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr msg)
      {
        on_odom(*msg);
      });
    mode_sub_ = create_subscription<ControlMode>(
      "control_mode", 10, [this](ControlMode::ConstSharedPtr msg)
      {
        on_mode(*msg);
      });

    // ---- lane_follower 의 linear_speed 를 물어볼 때 쓴다 ----
    lane_params_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "lane_follower");

    // ---- 보내는 토픽 ----
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    mode_pub_ = create_publisher<ControlMode>("control_mode", 10);

    // ---- 0.1초마다: 모드 알림, 영상 끊김 / 시간 초과 확인 ----
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]
      {
        watchdog();
      });

    // 시험용: 표지판 없이 바로 시작
    if (skip_sign_)
    {
      RCLCPP_INFO(get_logger(), "주차 노드 시작: skip_sign -> 표지판 없이 바로 입구 찾기");
      start_approach();
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
    sign_area_ratio_threshold_ = declare_parameter("sign_area_ratio_threshold", 0.02);
    sign_hits_needed_ = declare_parameter("sign_hits_needed", 2);
    skip_sign_ = declare_parameter("skip_sign", false);

    // 2) 색 / 화면
    image_topic_ = declare_parameter("image_topic", std::string("image_bev"));
    yellow_l_min_ = declare_parameter("yellow_l_min", 80);
    yellow_b_min_ = declare_parameter("yellow_b_min", 150);
    yellow_a_max_ = declare_parameter("yellow_a_max", 135);
    white_l_min_ = declare_parameter("white_l_min", 190);
    white_ab_dev_ = declare_parameter("white_ab_dev", 15);
    bev_center_x_ = declare_parameter("bev_center_x", 0.5);
    lane_width_px_ = declare_parameter("lane_width_px", 295.0);
    line_min_rows_ = declare_parameter("line_min_rows", 40);
    flat_min_pixels_ = declare_parameter("flat_min_pixels", 35);

    // 3) 흰선 따라 직진
    // 직진 속도 [m/s]. use_lane_speed 가 true 면 주차를 시작할 때 lane_follower 의 linear_speed
    // (GUI 선속도 슬라이더 값) 를 받아와서 그 속도로 간다. 못 받으면 drive_speed 를 쓴다
    drive_speed_ = declare_parameter("drive_speed", 0.10);
    use_lane_speed_ = declare_parameter("use_lane_speed", true);
    heading_gain_ = declare_parameter("heading_gain", 1.0);
    offset_gain_ = declare_parameter("offset_gain", 1.0);
    offset_soft_px_ = declare_parameter("offset_soft_px", 300.0);
    max_angular_ = declare_parameter("max_angular", 0.6);
    robot_y_offset_px_ = declare_parameter("robot_y_offset_px", 179.0);

    // 4) 입구 찾기 (왼쪽 아래 칸)
    watch_x_max_ = declare_parameter("watch_x_max", 120);
    watch_rows_ = declare_parameter("watch_rows", 20);
    watch_bottom_ = declare_parameter("watch_bottom", 390);
    present_pixels_ = declare_parameter("present_pixels", 80);
    empty_pixels_ = declare_parameter("empty_pixels", 15);
    gap_frames_needed_ = declare_parameter("gap_frames_needed", 2);
    extra_forward_m_ = declare_parameter("extra_forward_m", -0.04);

    // 5) 회전
    turn_speed_ = declare_parameter("turn_speed", 0.4);
    turn_min_speed_ = declare_parameter("turn_min_speed", 0.08);
    turn_slow_angle_deg_ = declare_parameter("turn_slow_angle_deg", 60.0);
    turn_slow_power_ = declare_parameter("turn_slow_power", 2.0);
    aligned_deg_ = declare_parameter("aligned_deg", 4.0);
    parallel_deg_ = declare_parameter("parallel_deg", 15.0);
    pair_gap_min_px_ = declare_parameter("pair_gap_min_px", 150.0);
    pair_gap_max_px_ = declare_parameter("pair_gap_max_px", 420.0);
    single_line_ok_ = declare_parameter("single_line_ok", true);
    single_line_margin_px_ = declare_parameter("single_line_margin_px", 80.0);
    turn_min_time_ = declare_parameter("turn_min_time", 1.0);
    turn_fit_rows_ = declare_parameter("turn_fit_rows", 230);

    // 6) 주차장 길 올라가기
    stem_use_white_ = declare_parameter("stem_use_white", true);
    same_line_px_ = declare_parameter("same_line_px", 40.0);
    stem_end_frames_ = declare_parameter("stem_end_frames", 3);
    stem_min_time_ = declare_parameter("stem_min_time", 2.0);
    stem_timeout_ = declare_parameter("stem_timeout", 20.0);

    // 7) 옆 PSD 가 가까우면 반대쪽으로 90도
    side_psd_threshold_ = declare_parameter("side_psd_threshold", 0.15);
    side_hits_needed_ = declare_parameter("side_hits_needed", 3);
    side_turn_deg_ = declare_parameter("side_turn_deg", 90.0);
    side_turn_toward_ = declare_parameter("side_turn_toward", true);
    side_turn_when_ = declare_parameter("side_turn_when", std::string("white"));
    white_trigger_y_ = declare_parameter("white_trigger_y", 350);
    side_cam_ = declare_parameter("side_turn_by_camera", true);
    side_cam_min_line_px_ = declare_parameter("side_cam_min_line_px", 60);
    side_cam_max_step_deg_ = declare_parameter("side_cam_max_step_deg", 20.0);
    side_cam_lost_frames_ = declare_parameter("side_cam_lost_frames", 5);
    side_cam_max_extra_deg_ = declare_parameter("side_cam_max_extra_deg", 15.0);
    white_row_min_pixels_ = declare_parameter("white_row_min_pixels", 150);
    white_center_ratio_ = declare_parameter("white_center_ratio", 0.6);
    white_arm_y_ = declare_parameter("white_arm_y", 300);
    side_clear_threshold_ = declare_parameter("side_clear_threshold", 0.20);
    side_turn_speed_ = declare_parameter("side_turn_speed", 0.6);
    side_min_speed_ = declare_parameter("side_min_speed", 0.08);
    side_slow_angle_deg_ = declare_parameter("side_slow_angle_deg", 30.0);
    side_slow_power_ = declare_parameter("side_slow_power", 2.0);
    side_tolerance_deg_ = declare_parameter("side_tolerance_deg", 1.0);
    odom_timeout_ = declare_parameter("odom_timeout", 0.5);

    // 8) 90도 돈 뒤 후진
    backup_time_ = declare_parameter("backup_time", 3.0);
    backup_speed_ = declare_parameter("backup_speed", 0.06);

    // 9) 후진 뒤 곡선으로 돌아서 들어왔던 길로 나가기
    exit_enable_ = declare_parameter("exit_enable", true);
    exit_arc_speed_ = declare_parameter("exit_arc_speed", 0.08);
    exit_arc_radius_m_ = declare_parameter("exit_arc_radius_m", 0.0);
    exit_arc_radius_gain_ = declare_parameter("exit_arc_radius_gain", 2.0);
    exit_arc_min_radius_m_ = declare_parameter("exit_arc_min_radius_m", 0.12);
    exit_arc_deg_ = declare_parameter("exit_arc_deg", 90.0);
    exit_check_ratio_ = declare_parameter("exit_check_ratio", 0.3);
    exit_line_max_rms_px_ = declare_parameter("exit_line_max_rms_px", 8.0);
    exit_overshoot_deg_ = declare_parameter("exit_overshoot_deg", 15.0);
    exit_wait_yellow_deg_ = declare_parameter("exit_wait_yellow_deg", 15.0);
    exit_stop_y_ = declare_parameter("exit_stop_y", 350);
    exit_white_max_tilt_deg_ = declare_parameter("exit_white_max_tilt_deg", 30.0);
    exit_timeout_ = declare_parameter("exit_timeout", 20.0);

    // 안전
    front_stop_distance_ = declare_parameter("front_stop_distance", 0.10);
    image_timeout_ = declare_parameter("image_timeout", 0.5);
    approach_timeout_ = declare_parameter("approach_timeout", 30.0);
    turn_timeout_ = declare_parameter("turn_timeout", 15.0);
  }

  // ==================== 표지판 / 모드 ====================

  // 주차 표지판이 가까이 보이면 APPROACH 시작
  void on_sign(const Sign & sign)
  {
    const bool parking_sign =
      sign.type == Sign::PARKING &&
      sign.confidence >= min_confidence_ &&
      sign.area_ratio >= sign_area_ratio_threshold_;

    sign_hits_ = parking_sign ? sign_hits_ + 1 : 0;

    if (state_ == State::WAITING && sign_hits_ >= sign_hits_needed_)
    {
      RCLCPP_INFO(get_logger(), "주차 표지판 인식 (크기 %.3f, 신뢰도 %.2f) -> 흰선 보고 직진하며 입구 찾기",
        sign.area_ratio, sign.confidence);
      start_approach();
    }
  }

  // GUI 버튼
  //   STOP / MANUAL : 주차 중이면 멈추고 그만둔다
  //   LANE          : 주차가 끝났거나 그만둔 상태면 처음(표지판 기다리기)으로 돌아간다
  void on_mode(const ControlMode & msg)
  {
    if (msg.mission_state != "turtle_gui")
    {
      return;                    // GUI 가 보낸 것만 본다 (내가 보낸 것 등은 무시)
    }
    const bool stop_request = msg.mode == ControlMode::STOP || msg.mode == ControlMode::MANUAL;
    if (stop_request && i_am_driving())
    {
      give_up("GUI 에서 STOP/MANUAL");
      return;
    }
    if (msg.mode == ControlMode::LANE && state_ == State::DONE)
    {
      reset();
    }
  }

  // 처음 상태로 돌아가기 (다음 바퀴에 다시 주차)
  void reset()
  {
    sign_hits_ = 0;
    if (skip_sign_)
    {
      RCLCPP_INFO(get_logger(), "GUI LANE -> 다시 시작: skip_sign 이라 바로 입구 찾기");
      start_approach();
      return;
    }
    state_ = State::WAITING;
    RCLCPP_INFO(get_logger(), "GUI LANE -> 다시 시작: 주차 표지판 기다림");
  }

  // ==================== 영상 한 장마다 ====================

  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    if (!i_am_driving())
    {
      return;
    }
    last_image_time_ = now();

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

    cv::Mat lab;
    cv::cvtColor(cv->image, lab, cv::COLOR_BGR2Lab);
    const cv::Mat yellow = yellow_mask(lab);

    if (state_ == State::APPROACH)
    {
      if (entrance_reached(yellow))
      {
        start_extra();
        return;
      }
      drive_along_white(white_mask(lab));
    }
    else if (state_ == State::EXTRA)
    {
      drive_along_white(white_mask(lab));
    }
    else if (state_ == State::TURN)
    {
      turn_until_aligned(yellow);
    }
    else if (state_ == State::SIDE_TURN)
    {
      if (side_use_cam_)
      {
        cv::Mat lines_mask;
        cv::bitwise_or(yellow, white_mask(lab), lines_mask);
        side_turn_by_camera(lines_mask);
      }
    }
    else if (state_ == State::BACKUP)
    {
      // 후진은 시간으로 한다. 곡선으로 돌기 시작할 때의 기준(직선 방향 분포)만 계속 저장해 둔다
      cv::Mat lines_mask;
      cv::bitwise_or(yellow, white_mask(lab), lines_mask);
      last_scene_angles_ = scene_line_angles(lines_mask);
    }
    else if (state_ == State::EXIT_ARC)
    {
      cv::Mat lines_mask;
      cv::bitwise_or(yellow, white_mask(lab), lines_mask);
      exit_arc_by_camera(yellow, lines_mask);
    }
    else if (state_ == State::EXIT_STEM)
    {
      const cv::Mat white = white_mask(lab);
      if (white_line_passing(lowest_crossing_white(white), exit_stop_y_, exit_white_y_))
      {
        finish_exit();
        return;
      }
      cv::Mat lines;
      cv::bitwise_or(yellow, white, lines);
      drive_between_lines(lines, false);   // 차선이 잠깐 안 보여도 멈추지 않는다 (흰선을 보고 멈춤)
    }
    else if (state_ == State::STEM)
    {
      const cv::Mat white = white_mask(lab);
      if (side_cam_)
      {
        cv::Mat lines_mask;
        cv::bitwise_or(yellow, white, lines_mask);
        last_scene_angles_ = scene_line_angles(lines_mask);   // 돌기 시작하면 카메라 각도의 기준 (직선 방향 분포)
      }
      if (side_turn_when_ == "white" && check_white_trigger(white))
      {
        return;                  // 회전 시작 (또는 정지)
      }
      // 노란선 + (흰 점선). 가로로 누운 흰선(주차장 테두리)은 선을 이을 때 빠진다
      cv::Mat lines = yellow;
      if (stem_use_white_)
      {
        cv::bitwise_or(yellow, white, lines);
      }
      // side_turn_when: white 면 차선이 끝나도 멈추지 않고, 가로 흰 선이 화면 아래에 올 때까지 직진
      //   (그 흰 선을 보고 돌거나 멈춘다. 끝내 안 보이면 stem_timeout 에서 멈춤)
      drive_between_lines(lines, side_turn_when_ != "white");
    }
  }

  // ==================== 색 찾기 ====================

  // 노란색인 곳만 255
  cv::Mat yellow_mask(const cv::Mat & lab) const
  {
    cv::Mat mask;
    cv::inRange(lab, cv::Scalar(yellow_l_min_, 0, yellow_b_min_),
      cv::Scalar(255, yellow_a_max_, 255), mask);
    return clean(mask);
  }

  // 흰색인 곳만 255 (밝고, a/b 가 128 근처 = 색이 없음)
  cv::Mat white_mask(const cv::Mat & lab) const
  {
    cv::Mat mask;
    cv::inRange(lab, cv::Scalar(white_l_min_, 128 - white_ab_dev_, 128 - white_ab_dev_),
      cv::Scalar(255, 128 + white_ab_dev_, 128 + white_ab_dev_), mask);
    return clean(mask);
  }

  // 작은 점(잡음) 지우기
  static cv::Mat clean(const cv::Mat & mask)
  {
    cv::Mat out;
    const cv::Mat k3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(mask, out, cv::MORPH_OPEN, k3);
    return out;
  }

  // 가로 범위 [x0, x1), 세로 줄 [0, y_end) 안의 선을 직선 x = a + b*y 로 잇는다
  //   줄마다 점들의 가운데 x 를 모은다. 점이 너무 많은 줄(가로로 누운 부분)은 뺀다.
  //   점이 있는 줄이 line_min_rows 보다 적으면 "선 없음"
  std::optional<Line> fit_line(const cv::Mat & mask, int x0, int x1, int y_end) const
  {
    double n = 0.0, sy = 0.0, sx = 0.0, syy = 0.0, sxy = 0.0, sxx = 0.0;   // 최소제곱 합
    for (int y = 0; y < std::min(y_end, mask.rows); y++)
    {
      const cv::Moments m = cv::moments(mask.row(y).colRange(x0, x1), true);   // m00 = 점 개수
      if (m.m00 < 1.0 || m.m00 >= flat_min_pixels_)
      {
        continue;
      }
      const double x = x0 + m.m10 / m.m00;
      n += 1.0;
      sy += y;
      sx += x;
      syy += static_cast<double>(y) * y;
      sxy += x * y;
      sxx += x * x;
    }
    const double denom = n * syy - sy * sy;
    if (n < line_min_rows_ || std::abs(denom) < 1e-9)
    {
      return std::nullopt;
    }
    Line line;
    line.b = (n * sxy - sx * sy) / denom;
    line.a = (sx - line.b * sy) / n;
    // 남은 오차 제곱합 = sum (x - a - b*y)^2 를 합들로 계산
    const double sse = sxx - line.a * sx - line.b * sxy;
    line.rms = std::sqrt(std::max(0.0, sse) / n);
    return line;
  }

  // ==================== (2) APPROACH: 흰선 보고 직진 ====================

  void start_approach()
  {
    choose_speed();
    state_ = State::APPROACH;
    gap_ = Gap::BEFORE;
    gap_frames_ = 0;
    state_time_ = 0.0;
    last_image_time_ = now();
  }

  // 직진 속도 정하기: 우선 drive_speed 로 시작하고, lane_follower 가 답하면 그 linear_speed 로 바꾼다
  void choose_speed()
  {
    speed_ = drive_speed_;
    if (!use_lane_speed_ || !lane_params_->service_is_ready())
    {
      RCLCPP_INFO(get_logger(), "주차 직진 속도 %.2f m/s (drive_speed)", speed_);
      return;
    }
    lane_params_->get_parameters({"linear_speed"},
      [this](std::shared_future<std::vector<rclcpp::Parameter>> future)
      {
        const auto params = future.get();
        if (!params.empty() && params[0].get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
          params[0].as_double() > 0.0)
        {
          speed_ = params[0].as_double();
        }
        RCLCPP_INFO(get_logger(), "주차 직진 속도 %.2f m/s (lane_follower 와 같게)", speed_);
      });
  }

  // 오른쪽 흰선에서 차선 폭 절반만큼 왼쪽 = 차선 가운데를 따라간다. 흰선이 없으면 직진
  void drive_along_white(const cv::Mat & white)
  {
    const int mid = static_cast<int>(bev_center_x_ * white.cols);
    const std::optional<Line> line = fit_line(white, mid, white.cols, white.rows);
    if (!line)
    {
      drive(speed_, 0.0);
      return;
    }

    Line center = *line;
    center.a -= lane_width_px_ / 2.0;      // 흰선에서 차선 폭 절반만큼 왼쪽 = 차선 가운데
    drive(speed_, steer_to(center, white.rows, white.cols));
  }

  // 차선 가운데 선(center)을 따라가는 회전 속도 [rad/s]. + 왼쪽, - 오른쪽
  //   방향 오차: 가운데 선이 왼쪽으로 기운 만큼 왼쪽으로 돈다
  //   위치 오차: 가운데 선을 바퀴 축 높이까지 이어서, 로봇보다 오른쪽에 얼마나 있는지
  double steer_to(const Line & center, int rows, int cols) const
  {
    const double robot_x = bev_center_x_ * cols;
    const double bottom_y = rows - 1;
    const double center_at_axle = center.x_at(bottom_y) + center.b * robot_y_offset_px_;
    const double offset_right = center_at_axle - robot_x;
    const double w = heading_gain_ * center.lean_left() -
      offset_gain_ * std::atan2(offset_right, offset_soft_px_);
    return std::clamp(w, -max_angular_, max_angular_);
  }

  // 왼쪽 아래 칸의 노란 점 개수로 "입구 반대쪽 모서리가 화면 맨 아래에 닿았는지" 판단
  //   BEFORE: 노란선 있음. 연속으로 비면 -> IN_GAP
  //   IN_GAP: 연속으로 다시 차면 -> 도착 (true)
  bool entrance_reached(const cv::Mat & yellow)
  {
    const int y1 = std::min(static_cast<int>(watch_bottom_) + 1, yellow.rows);
    const int y0 = std::max(0, y1 - static_cast<int>(watch_rows_));
    const int x1 = std::min(static_cast<int>(watch_x_max_), yellow.cols);
    const int count = cv::countNonZero(yellow(cv::Range(y0, y1), cv::Range(0, x1)));

    if (gap_ == Gap::BEFORE)
    {
      gap_frames_ = count <= empty_pixels_ ? gap_frames_ + 1 : 0;
      if (gap_frames_ >= gap_frames_needed_)
      {
        gap_ = Gap::IN_GAP;
        gap_frames_ = 0;
        RCLCPP_INFO(get_logger(), "입구의 빈 곳 지나는 중 (왼쪽 아래 노란 점 %d개)", count);
      }
      return false;
    }

    gap_frames_ = count >= present_pixels_ ? gap_frames_ + 1 : 0;
    if (gap_frames_ >= gap_frames_needed_)
    {
      RCLCPP_INFO(get_logger(), "입구 반대쪽 노란선이 화면 맨 아래에 닿음 (%d개) -> 왼쪽으로 회전", count);
      return true;
    }
    return false;
  }

  // ==================== (2-1) EXTRA: 입구 가운데까지 남은 만큼 더 직진 ====================

  // 반대쪽 노란선이 watch_bottom 줄에 닿았을 때, 바퀴 축 ~ 그 선 거리 = (화면 맨 아래 - watch_bottom) + robot_y_offset
  // 입구 가운데 ~ 그 선 거리 = 차선 폭 / 2. 그 차이가 더 가야 할 거리 (음수면 0)
  void start_extra()
  {
    const double to_line_px = (399.0 - watch_bottom_) + robot_y_offset_px_;
    const double extra_m = std::max(0.0, (to_line_px - lane_width_px_ / 2.0) * 0.001 + extra_forward_m_);
    extra_time_ = extra_m / speed_;
    if (extra_time_ <= 0.0)
    {
      start_turn();
      return;
    }
    state_ = State::EXTRA;
    state_time_ = 0.0;
    RCLCPP_INFO(get_logger(), "입구 가운데까지 %.1fcm 더 직진 (%.1f초)", extra_m * 100.0, extra_time_);
  }

  // ==================== (3) TURN: 왼쪽으로 돌기 ====================

  void start_turn()
  {
    state_ = State::TURN;
    state_time_ = 0.0;
    drive(0.0, turn_speed_);
  }

  // 회전 속도: 길의 노란선이 왼쪽으로 많이 기울어 있으면(아직 많이 돌아야 함) 빠르게(turn_speed),
  //   세로에 가까워질수록 느리게(turn_min_speed). turn_slow_angle_deg 보다 많이 기울면 최고 속도
  //   ratio = 기운 각도 / turn_slow_angle_deg (0 ~ 1) 를 turn_slow_power 제곱해서 쓴다
  //     power 1 = 각도에 비례해서 고르게 느려짐, 2 = 처음엔 조금, 세로에 가까울수록 훨씬 많이 느려짐
  //   lean: 선이 왼쪽으로 기운 각도 [rad]. 음수(오른쪽으로 기움 = 지나침)도 크기로 계산
  double turn_speed_for(double lean) const
  {
    const double ratio = std::min(1.0, std::abs(lean) / (turn_slow_angle_deg_ * M_PI / 180.0));
    return turn_min_speed_ + (turn_speed_ - turn_min_speed_) * std::pow(ratio, turn_slow_power_);
  }

  // 왼쪽으로 돌면서:
  //   - 처음 turn_min_time 초는 최고 속도 (막 돌기 시작할 때 보이는 도로 노란선은 세로라서 헷갈리므로)
  //   - 그 뒤 왼쪽으로 기운 노란선(= 길의 선, 도는 쪽에서 들어옴)이 보이면 그 기울기에 맞춰 점점 느리게
  //   - 왼쪽/오른쪽 노란선이 둘 다 보이고, 간격이 길 폭 정도이고, 같은 쪽으로 기울어 있으면
  //     (= 같은 길의 두 선) 둘 다 거의 세로(aligned_deg 이내)가 되면 멈춘다
  void turn_until_aligned(const cv::Mat & yellow)
  {
    // 화면 아래쪽에는 입구의 둥근 모서리가 보여서 선이 바깥으로 휘어 보인다 -> 위쪽 줄만 쓴다
    const int mid = static_cast<int>(bev_center_x_ * yellow.cols);
    const int y_end = static_cast<int>(turn_fit_rows_);
    const std::optional<Line> left = fit_line(yellow, 0, mid, y_end);
    const std::optional<Line> right = fit_line(yellow, mid, yellow.cols, y_end);

    const double bottom_y = yellow.rows - 1;
    const bool pair = left && right &&
      right->x_at(bottom_y) - left->x_at(bottom_y) >= pair_gap_min_px_ &&
      right->x_at(bottom_y) - left->x_at(bottom_y) <= pair_gap_max_px_;

    if (state_time_ < turn_min_time_)
    {
      drive(0.0, turn_speed_);
      return;
    }
    const double aligned = aligned_deg_ * M_PI / 180.0;
    if (!pair)
    {
      // 두 선이 짝으로 안 보여도, 한 선이 길 옆선 자리에 거의 세로로 서 있으면 똑바로 본 걸로 친다
      const std::optional<double> single = single_aligned_line(left, right, bottom_y, yellow.cols);
      if (single)
      {
        start_stem(*single);
        return;
      }
      drive(0.0, turn_speed_for(leaning_left(left, right)));
      return;
    }

    const double lean_l = left->lean_left();
    const double lean_r = right->lean_left();
    if (std::abs(lean_l - lean_r) > parallel_deg_ * M_PI / 180.0)
    {
      // 두 선이 나란하지 않음 = 아직 길을 제대로 못 봄
      drive(0.0, turn_speed_for(leaning_left(left, right)));
      return;
    }
    const double lean = (lean_l + lean_r) / 2.0;   // + 면 더 왼쪽으로
    if (std::abs(lean_l) <= aligned && std::abs(lean_r) <= aligned)
    {
      start_stem(lean);
      return;
    }
    drive(0.0, std::copysign(turn_speed_for(lean), lean));   // 지나쳤으면(lean < 0) 오른쪽으로
  }

  // 왼쪽 / 오른쪽 선 중 하나가 "길의 옆선 자리" (로봇 가운데에서 차선 폭 절반 ± single_line_margin_px)에
  // 거의 세로(aligned_deg 이내)로 서 있으면 그 기울기 [rad]. 없으면 비어 있음 (single_line_ok: false 면 항상 비어 있음)
  std::optional<double> single_aligned_line(const std::optional<Line> & left,
    const std::optional<Line> & right, double bottom_y, int cols) const
  {
    if (!single_line_ok_)
    {
      return std::nullopt;
    }
    const double aligned = aligned_deg_ * M_PI / 180.0;
    const double robot_x = bev_center_x_ * cols;
    const double half_lane = lane_width_px_ / 2.0;
    if (left && std::abs(left->lean_left()) <= aligned &&
      std::abs(left->x_at(bottom_y) - (robot_x - half_lane)) <= single_line_margin_px_)
    {
      return left->lean_left();
    }
    if (right && std::abs(right->lean_left()) <= aligned &&
      std::abs(right->x_at(bottom_y) - (robot_x + half_lane)) <= single_line_margin_px_)
    {
      return right->lean_left();
    }
    return std::nullopt;
  }

  // 보이는 노란선 중 왼쪽으로 기운 것(= 아직 더 돌아야 하는 길의 선)의 기울기 [rad].
  // 왼쪽으로 기운 선이 없으면 아주 큰 값 (= 최고 속도)
  static double leaning_left(const std::optional<Line> & left, const std::optional<Line> & right)
  {
    double lean = 10.0;
    for (const auto & line : {left, right})
    {
      if (line && line->lean_left() > 0.0)
      {
        lean = std::min(lean, line->lean_left());
      }
    }
    return lean;
  }

  // ==================== (4) STEM: 두 차선 가운데로 길 / 통로 올라가기 ====================

  void start_stem(double lean)
  {
    state_ = State::STEM;
    state_time_ = 0.0;
    lines_missing_ = 0;
    side_seen_ = 0.0;
    left_hits_ = 0;
    right_hits_ = 0;
    clear_hits_ = 0;
    last_white_y_ = -1.0;
    last_scene_angles_.clear();
    side_prev_angles_.clear();
    RCLCPP_INFO(get_logger(), "입구 길을 똑바로 봄 (남은 기울기 %.1f도) -> 두 차선 가운데로 올라감",
      lean * 180.0 / M_PI);
  }

  // 왼쪽/오른쪽 차선(노란선, 흰 점선)을 잇고 그 가운데를 따라간다.
  //   둘 다 보이고 간격이 길 폭 정도 -> 두 선의 가운데
  //   하나만 보임                      -> 그 선에서 차선 폭 절반만큼 안쪽
  //   둘 다 안 보임이 연속              -> 통로 끝 -> 멈춤 (stop_when_lines_end 가 true 일 때만. 아니면 직진)
  void drive_between_lines(const cv::Mat & lines, bool stop_when_lines_end)
  {
    const int mid = static_cast<int>(bev_center_x_ * lines.cols);
    std::optional<Line> left = fit_line(lines, 0, mid, lines.rows);
    std::optional<Line> right = fit_line(lines, mid, lines.cols, lines.rows);

    // 두 선 간격이 길 폭과 너무 다르면 (잘못 찾은 선) 둘 다 믿지 않고 직진
    //   단, 아주 가깝고 나란하면 = 한 선이 화면 가운데에 걸쳐서 왼쪽 칸 / 오른쪽 칸에 둘 다 잡힌 것
    //   (로봇이 길에서 비켜서 비스듬할 때) -> 한 선으로 합치고, 바퀴 축 높이에서 로봇의 어느 쪽에 있는지로 왼쪽 / 오른쪽을 정한다
    const double bottom_y = lines.rows - 1;
    if (left && right)
    {
      const double gap = right->x_at(bottom_y) - left->x_at(bottom_y);
      if (is_same_line(*left, *right, bottom_y))
      {
        Line one;
        one.a = (left->a + right->a) / 2.0;
        one.b = (left->b + right->b) / 2.0;
        const double at_axle = one.x_at(bottom_y) + one.b * robot_y_offset_px_;
        const bool on_left = at_axle < bev_center_x_ * lines.cols;
        left.reset();
        right.reset();
        (on_left ? left : right) = one;
      }
      else if (gap < pair_gap_min_px_ || gap > pair_gap_max_px_)
      {
        left.reset();
        right.reset();
        drive(speed_, 0.0);
        return;
      }
    }

    if (!left && !right)
    {
      lines_missing_++;
      if (stop_when_lines_end && lines_missing_ >= stem_end_frames_ && state_time_ >= stem_min_time_)
      {
        finish();
        return;
      }
      drive(speed_, 0.0);     // 안 보이면 직진
      return;
    }
    lines_missing_ = 0;

    Line center;
    if (left && right)
    {
      center.a = (left->a + right->a) / 2.0;
      center.b = (left->b + right->b) / 2.0;
    }
    else if (left)
    {
      center = *left;
      center.a += lane_width_px_ / 2.0;
    }
    else
    {
      center = *right;
      center.a -= lane_width_px_ / 2.0;
    }
    drive(speed_, steer_to(center, lines.rows, lines.cols));
  }

  // 왼쪽 칸 / 오른쪽 칸에서 찾은 두 선이 사실 한 선인지: 화면 맨 아래에서 same_line_px 이내로 가깝고 나란함
  bool is_same_line(const Line & left, const Line & right, double bottom_y) const
  {
    const double gap = std::abs(right.x_at(bottom_y) - left.x_at(bottom_y));
    return gap <= same_line_px_ &&
      std::abs(left.lean_left() - right.lean_left()) <= parallel_deg_ * M_PI / 180.0;
  }

  // ==================== 옆 PSD 로 90도 돌기 ====================

  // STEM 중 PSD 가 올 때마다
  //   (1) 옆 PSD 가 side_psd_threshold 이하로 연속 side_hits_needed 번 -> 그쪽에 장애물 있음 (기억만, 계속 직진)
  //   (2) 그 뒤 그쪽 PSD 가 side_clear_threshold 보다 멀어진 게 연속 side_hits_needed 번
  //       -> 장애물을 다 지나감 -> 그쪽으로 (side_turn_toward: false 면 반대쪽으로) 돌기 시작
  //   side_turn_when: seen 이면 (1) 에서 바로 돈다, white 면 (1) 에서 기억만 하고
  //   가로 흰 선이 화면 아래에 올 때 돈다 (on_image -> check_white_trigger)
  void check_side_psd()
  {
    if (state_ != State::STEM)
    {
      left_hits_ = 0;
      right_hits_ = 0;
      return;
    }

    // (1) 아직 장애물을 못 봤으면: 가까운 쪽 찾기
    if (side_seen_ == 0.0)
    {
      left_hits_ = is_close(psd_.left) ? left_hits_ + 1 : 0;
      right_hits_ = is_close(psd_.right) ? right_hits_ + 1 : 0;
      if (left_hits_ >= side_hits_needed_)
      {
        side_seen_ = +1.0;
        RCLCPP_INFO(get_logger(), "왼쪽 PSD %.2fm (<= %.2f) -> 왼쪽에 장애물", psd_.left, side_psd_threshold_);
      }
      else if (right_hits_ >= side_hits_needed_)
      {
        side_seen_ = -1.0;
        RCLCPP_INFO(get_logger(), "오른쪽 PSD %.2fm (<= %.2f) -> 오른쪽에 장애물", psd_.right,
          side_psd_threshold_);
      }
      if (side_seen_ == 0.0)
      {
        return;
      }
      clear_hits_ = 0;
      if (side_turn_when_ == "seen")
      {
        turn_toward_seen_side();
      }
      else if (side_turn_when_ == "clear")
      {
        RCLCPP_INFO(get_logger(), "장애물 옆을 지나갈 때까지 계속 직진");
      }
      else
      {
        RCLCPP_INFO(get_logger(), "기억함. 앞의 가로 흰 선이 화면 아래에 올 때까지 계속 직진");
      }
      return;
    }

    // (2) side_turn_when: clear 일 때만 - 그쪽 PSD 가 멀어질 때까지 기다리기
    if (side_turn_when_ != "clear")
    {
      return;
    }
    const double value = side_seen_ > 0.0 ? psd_.left : psd_.right;
    clear_hits_ = (value > side_clear_threshold_) ? clear_hits_ + 1 : 0;
    if (clear_hits_ >= side_hits_needed_)
    {
      RCLCPP_INFO(get_logger(), "%s PSD %.2fm (> %.2f) -> 장애물을 지나감", side_seen_ > 0.0 ? "왼쪽" : "오른쪽",
        value, side_clear_threshold_);
      turn_toward_seen_side();
    }
  }

  // PSD 값이 정상(0 보다 큼)이고 side_psd_threshold 이하인지
  bool is_close(double value) const
  {
    return value > 0.0 && value <= side_psd_threshold_;
  }

  // 장애물을 본 쪽으로 (toward = false 면 반대쪽으로) 돌기 시작
  void turn_toward_seen_side()
  {
    const double toward = side_turn_toward_ ? 1.0 : -1.0;
    start_side_turn(side_seen_ * toward);
  }

  // 앞의 가로 흰 선(주차장 끝 테두리)이 화면에서 사라지기 직전인지 보고, 그렇다면
  //   장애물을 본 쪽이 있으면 그쪽으로 돌기 시작, 없으면 멈춤. 무언가 했으면 true
  //   "사라지기 직전" = (1) 흰 선이 white_trigger_y 줄보다 아래에 보임, 또는
  //                     (2) 바로 전 영상에서 white_arm_y 줄보다 아래에 있던 흰 선이 이번 영상에서 안 보임
  //   (카메라가 초당 몇 장이라, 흰 선이 맨 아래 몇 줄을 지나가는 걸 못 찍고 지나칠 수 있어서 (2) 도 본다)
  bool check_white_trigger(const cv::Mat & white)
  {
    if (!white_line_passing(lowest_horizontal_white(white), white_trigger_y_, last_white_y_))
    {
      return false;
    }
    const double fire_y = last_white_y_;   // 지금 (또는 사라지기 바로 전) 흰 선의 줄 번호

    if (side_seen_ != 0.0)
    {
      RCLCPP_INFO(get_logger(), "가로 흰 선이 화면에서 사라지기 직전 (y=%.0f) -> 기억한 %s으로 회전", fire_y,
        side_seen_ > 0.0 ? "왼쪽" : "오른쪽");
      side_prev_angles_ = last_scene_angles_;   // 돌기 전 화면의 직선 방향 분포 (카메라로 각도 잴 기준)
      turn_toward_seen_side();
    }
    else
    {
      RCLCPP_INFO(get_logger(), "가로 흰 선이 화면에서 사라지기 직전 (y=%.0f), 옆 장애물은 못 봄 -> 정지", fire_y);
      finish();
    }
    return true;
  }

  // 가로 흰 선이 화면 맨 아래 근처를 지나가는지 (STEM 의 회전 시작, EXIT_STEM 의 정지에 쓴다)
  //   (1) 흰 선이 trigger_y 줄보다 아래에 보임 -> true
  //   (2) 바로 전 영상에서 white_arm_y 줄보다 아래에 있던 흰 선이 이번 영상에서 안 보임 -> true
  //   last_y: 바로 전 영상의 흰 선 줄 번호 (-1 = 없음). 이 함수가 고쳐 둔다
  //   line_y: 이번 영상에서 찾은 가로 흰 선의 줄 번호 (없으면 비어 있음)
  bool white_line_passing(const std::optional<double> & line_y, int64_t trigger_y, double & last_y) const
  {
    if (line_y)
    {
      last_y = *line_y;
      return *line_y >= trigger_y;
    }
    if (last_y >= white_arm_y_)
    {
      return true;                 // 바로 전에 화면 아래쪽에 있던 흰 선이 이제 안 보임 = 막 지나감
    }
    last_y = -1.0;
    return false;
  }

  // 화면을 가로지르는 흰 선 중 가장 아래(= 로봇에 가장 가까운) 것의 줄 번호. 없으면 비어 있음
  //   한 줄에 흰 점이 white_row_min_pixels 이상이고, 화면 가운데 40% 구간도 white_center_ratio 이상 채워야 한다
  //   (주차장 입구 쪽 테두리는 길이 들어오는 가운데가 뚫려 있어서 빠진다)
  std::optional<double> lowest_horizontal_white(const cv::Mat & white) const
  {
    const int c0 = static_cast<int>(white.cols * 0.3);
    const int c1 = static_cast<int>(white.cols * 0.7);
    const double center_need = (c1 - c0) * white_center_ratio_;
    for (int y = white.rows - 1; y >= 0; y--)
    {
      const int all = cv::countNonZero(white.row(y));
      if (all < white_row_min_pixels_)
      {
        continue;
      }
      if (cv::countNonZero(white.row(y).colRange(c0, c1)) >= center_need)
      {
        return static_cast<double>(y);
      }
    }
    return std::nullopt;
  }

  // 화면을 가로지르는 흰 선 (조금 기울어도 됨) 중 가장 아래 것의, 화면 가운데 세로줄에서의 줄 번호. 없으면 비어 있음
  //   EXIT_STEM 에서 도로의 흰선을 찾을 때 쓴다. 로봇이 길을 비스듬히 나오면 도로 흰선이 화면에서 기울어 보여서
  //   lowest_horizontal_white (한 줄에 흰 점이 많아야 함) 로는 못 찾는다
  //   -> 직선 조각을 찾아서, 가로에서 exit_white_max_tilt_deg 이내로 기울고 길이가 white_row_min_pixels 이상이고
  //      화면 가운데 40% 구간을 끝까지 가로지르는 것만 본다
  //      (주차장 아래 테두리는 길이 들어오는 가운데가 뚫려 있어서 빠진다)
  std::optional<double> lowest_crossing_white(const cv::Mat & white) const
  {
    std::optional<double> best = lowest_horizontal_white(white);
    std::vector<cv::Vec4i> segments;
    cv::HoughLinesP(white, segments, 1.0, CV_PI / 180.0, 50, static_cast<double>(white_row_min_pixels_), 20.0);
    const double center_x = bev_center_x_ * white.cols;
    const double c0 = white.cols * 0.3;
    const double c1 = white.cols * 0.7;
    const double max_tilt = std::tan(exit_white_max_tilt_deg_ * M_PI / 180.0);
    for (const auto & seg : segments)
    {
      const double dx = seg[2] - seg[0];
      const double dy = seg[3] - seg[1];
      if (std::abs(dx) < 1.0 || std::abs(dy / dx) > max_tilt)
      {
        continue;                // 세로에 가까운 선 (길 옆선) 은 뺀다
      }
      if (std::min(seg[0], seg[2]) > c0 || std::max(seg[0], seg[2]) < c1)
      {
        continue;                // 화면 가운데를 가로지르지 않는 조각
      }
      // 화면 가운데 세로줄에서의 높이 (조각이 가운데까지 안 오면 이어서 계산)
      const double y = seg[1] + (center_x - seg[0]) * dy / dx;
      if (!best || y > *best)
      {
        best = y;
      }
    }
    return best;
  }

  // direction: +1 왼쪽, -1 오른쪽
  //   odom 이 최근에 왔으면: odom 으로 실제 돈 각도를 재며 돈다 (정확)
  //   아니면:               회전 시간 = 각도 / 회전 속도 만큼 돈다
  void start_side_turn(double direction)
  {
    stop_wheels();
    state_ = State::SIDE_TURN;
    state_time_ = 0.0;
    side_direction_ = direction;
    side_turn_time_ = (side_turn_deg_ * M_PI / 180.0) / side_turn_speed_;          // odom 도 카메라도 없을 때
    side_turn_limit_ = (side_turn_deg_ * M_PI / 180.0) / side_min_speed_ * 1.5;    // 너무 오래 걸리면 포기
    left_hits_ = 0;
    right_hits_ = 0;
    side_use_odom_ = odom_received_ && (now() - last_odom_time_).seconds() < odom_timeout_;
    side_start_yaw_ = odom_yaw_;
    side_turned_ = 0.0;
    // 가로 흰 선을 보고 돌기 시작했으면: 화면의 직선들이 돈 각도로 잰다 (카메라)
    side_use_cam_ = side_cam_ && !side_prev_angles_.empty();
    side_cam_lost_ = 0;
    side_cam_turned_ = 0.0;
    side_cam_sum_ = 0.0;
    const char * how = side_use_cam_ ? "카메라 (화면 직선들이 돈 각도) 로 잼" :
      (side_use_odom_ ? "odom 으로 각도 잼" : "odom 없음 -> 시간으로");
    RCLCPP_INFO(get_logger(), "%s으로 %.0f도 회전 시작 (%s)", direction < 0.0 ? "오른쪽" : "왼쪽",
      side_turn_deg_, how);
  }

  // 영상이 올 때마다 (SIDE_TURN + 카메라 사용 중):
  //   로봇이 제자리에서 돌면 화면(BEV)의 모든 직선이 같은 각도만큼 돈다.
  //   그래서 바로 전 영상과 이번 영상의 직선 방향 분포를 비교해 "이번에 더 돈 각도" 를 구하고 계속 더한다.
  //   (선 하나가 화면 밖으로 나가도 다른 선으로 이어서 잴 수 있다)
  //   더한 각도가 목표(side_turn_deg)가 되면 멈춤. 직선을 연속 side_cam_lost_frames 번 못 찾으면 odom 으로
  void side_turn_by_camera(const cv::Mat & lines_mask)
  {
    const std::vector<double> now_angles = scene_line_angles(lines_mask);
    const std::optional<double> step = rotation_between(side_prev_angles_, now_angles);
    if (!now_angles.empty())
    {
      side_prev_angles_ = now_angles;   // 다음 영상과 비교할 기준
    }
    if (!step)
    {
      side_cam_lost_++;
      if (side_cam_lost_ >= side_cam_lost_frames_)
      {
        side_use_cam_ = false;
        RCLCPP_WARN(get_logger(), "회전 중 직선을 놓침 (카메라로 %.1f도) -> %s 으로 이어서 잼",
          side_cam_turned_ * 180.0 / M_PI, side_use_odom_ ? "odom" : "시간");
      }
      return;                    // 바퀴는 마지막 명령대로 계속 돈다
    }
    side_cam_lost_ = 0;
    side_cam_sum_ += *step;
    side_cam_turned_ = std::abs(side_cam_sum_);
    const double remain = side_turn_deg_ * M_PI / 180.0 - side_cam_turned_;
    if (remain <= side_tolerance_deg_ * M_PI / 180.0)
    {
      end_side_turn();
      return;
    }
    drive(0.0, side_direction_ * side_speed_for(remain));
  }

  // 화면(흰선 + 노란선)의 직선 방향 분포: 180칸(1칸 = 1도), 각 칸 = 그 방향 직선 조각들의 길이 합
  //   (선은 방향이 없으니 0 ~ 180도). 살짝 번지게(±2칸) 해서 1~2도 차이도 겹치게 한다
  std::vector<double> scene_line_angles(const cv::Mat & lines_mask) const
  {
    std::vector<cv::Vec4i> segments;
    cv::HoughLinesP(lines_mask, segments, 1.0, CV_PI / 180.0, 30,
      static_cast<double>(side_cam_min_line_px_), 10.0);
    std::vector<double> hist(180, 0.0);
    if (segments.empty())
    {
      hist.clear();              // 직선이 없음
      return hist;
    }
    const double spread[5] = {1.0, 2.0, 3.0, 2.0, 1.0};
    for (const auto & seg : segments)
    {
      const double dx = seg[2] - seg[0];
      const double dy = seg[3] - seg[1];
      double deg = std::atan2(dy, dx) * 180.0 / M_PI;
      if (deg < 0.0)
      {
        deg += 180.0;
      }
      const int bin = static_cast<int>(std::lround(deg)) % 180;
      const double length = std::hypot(dx, dy);
      for (int k = -2; k <= 2; k++)
      {
        hist[(bin + k + 180) % 180] += length * spread[k + 2];
      }
    }
    return hist;
  }

  // 바로 전 영상의 방향 분포(before)를 몇 도 돌려야 이번 영상(after)과 가장 잘 겹치는지 = 화면이 돈 각도 [rad]
  //   -side_cam_max_step_deg ~ +side_cam_max_step_deg 를 1도씩 해 보고, 가장 잘 겹치는 곳 근처를 소수점까지 다듬는다
  //   어느 쪽이든 비어 있으면 (직선 없음) 비어 있음
  std::optional<double> rotation_between(const std::vector<double> & before,
    const std::vector<double> & after) const
  {
    if (before.empty() || after.empty())
    {
      return std::nullopt;
    }
    const int max_step = static_cast<int>(side_cam_max_step_deg_);
    std::vector<double> score(2 * max_step + 1, 0.0);
    for (int shift = -max_step; shift <= max_step; shift++)
    {
      double sum = 0.0;
      for (int a = 0; a < 180; a++)
      {
        sum += before[a] * after[(a + shift + 180) % 180];
      }
      score[shift + max_step] = sum;
    }
    const int best = static_cast<int>(std::max_element(score.begin(), score.end()) - score.begin());
    double shift = best - max_step;
    if (best > 0 && best < static_cast<int>(score.size()) - 1)
    {
      // 꼭짓점 양옆 점수로 소수점 위치 구하기 (포물선)
      const double l = score[best - 1];
      const double c = score[best];
      const double r = score[best + 1];
      const double denom = l - 2.0 * c + r;
      if (std::abs(denom) > 1e-9)
      {
        shift += 0.5 * (l - r) / denom;
      }
    }
    return shift * M_PI / 180.0;
  }

  // 옆 회전 속도: 남은 각도가 side_slow_angle_deg 보다 크면 최고 속도(side_turn_speed),
  //   가까워질수록 (남은 각도 / side_slow_angle_deg) 의 side_slow_power 제곱만큼 느려져 side_min_speed 까지
  double side_speed_for(double remain) const
  {
    const double ratio = std::clamp(remain / (side_slow_angle_deg_ * M_PI / 180.0), 0.0, 1.0);
    return side_min_speed_ + (side_turn_speed_ - side_min_speed_) * std::pow(ratio, side_slow_power_);
  }

  // odom 이 올 때마다 (SIDE_TURN + odom 사용 중일 때): 돈 각도를 재고, 남은 각도에 맞춰 속도 조절
  void step_side_turn_odom()
  {
    side_turned_ = std::abs(wrap_angle(odom_yaw_ - side_start_yaw_));
    const double remain = side_turn_deg_ * M_PI / 180.0 - side_turned_;
    if (remain <= side_tolerance_deg_ * M_PI / 180.0)
    {
      end_side_turn();
      return;
    }
    // 남은 각도에 비례해서 느려진다 (지나치지 않게). 너무 느리면 안 돌아서 최소 속도
    const double w = side_speed_for(remain);
    drive(0.0, side_direction_ * w);
  }

  // 0.1초마다: odom 을 안 쓰면 시간으로 돌기. odom 을 써도 너무 오래 걸리면 (odom 끊김 등) 멈춤
  void step_side_turn()
  {
    if (side_use_cam_)
    {
      if (state_time_ >= side_turn_limit_)
      {
        stop_wheels();
        give_up("카메라로 회전 중 너무 오래 걸림");
      }
      return;                    // 바퀴 명령은 영상이 올 때마다 side_turn_by_camera() 가 보낸다
    }
    if (side_use_odom_)
    {
      if ((now() - last_odom_time_).seconds() > odom_timeout_ || state_time_ >= side_turn_limit_)
      {
        stop_wheels();
        give_up("회전 중 odom 이 끊김 / 너무 오래 걸림");
      }
      return;                    // 바퀴 명령은 odom 이 올 때마다 step_side_turn_odom() 이 보낸다
    }
    if (state_time_ >= side_turn_time_)
    {
      end_side_turn();
      return;
    }
    drive(0.0, side_direction_ * side_turn_speed_);
  }

  void end_side_turn()
  {
    stop_wheels();
    const bool right = side_direction_ < 0.0;
    if (side_use_cam_)
    {
      RCLCPP_INFO(get_logger(), "%s으로 %.1f도 돌았음 (카메라, odom %.1f도)",
        right ? "오른쪽" : "왼쪽", side_cam_turned_ * 180.0 / M_PI, side_turned_ * 180.0 / M_PI);
    }
    else if (side_use_odom_)
    {
      RCLCPP_INFO(get_logger(), "%s으로 %.1f도 돌았음 (odom)", right ? "오른쪽" : "왼쪽",
        side_turned_ * 180.0 / M_PI);
    }
    else
    {
      RCLCPP_INFO(get_logger(), "%s으로 %.0f도 돌았음 (시간)", right ? "오른쪽" : "왼쪽", side_turn_deg_);
    }
    start_backup();
  }

  // ==================== (6) BACKUP: 90도 돈 뒤 후진 ====================

  // backup_time 초 동안 backup_speed 로 뒤로 간다 (뒤에는 센서가 없으니 천천히). 0 이면 바로 끝
  void start_backup()
  {
    if (backup_time_ <= 0.0)
    {
      finish_backup();
      return;
    }
    state_ = State::BACKUP;
    state_time_ = 0.0;
    RCLCPP_INFO(get_logger(), "%.1f초 후진", backup_time_);
    drive(-backup_speed_, 0.0);
  }

  // 0.1초마다: 시간이 될 때까지 후진, 다 되면 멈춤
  void step_backup()
  {
    if (state_time_ >= backup_time_)
    {
      finish_backup();
      return;
    }
    drive(-backup_speed_, 0.0);
  }

  void finish_backup()
  {
    stop_wheels();
    if (exit_enable_)
    {
      start_exit_arc();
      return;
    }
    state_ = State::DONE;
    const bool right = side_direction_ < 0.0;
    publish_mode(ControlMode::STOP, right ? "turned_right_backed" : "turned_left_backed");
    RCLCPP_INFO(get_logger(), "후진 끝 -> 정지 (exit_enable: false)");
  }

  // ==================== (7) EXIT_ARC: 곡선으로 90도 돌아 길 쪽 보기 ====================

  // 곡선 반지름 [m]: exit_arc_radius_m 이 0 이면 후진한 거리 (backup_speed x backup_time) x exit_arc_radius_gain.
  //   실제 반지름이 후진한 거리와 같으면 다 돌았을 때 바퀴 축이 후진을 시작한 곳(= 통로 가운데) 줄로 돌아온다.
  //   이 로봇은 회전 명령보다 약 2배 더 돌아서 (시험: 90도 명령 -> 약 200도) 실제 반지름이 명령의 절반쯤이 된다
  //   -> 명령 반지름을 2배 (exit_arc_radius_gain 2.0) 로 준다. 너무 크게 돌면 줄이고, 아직 확 꺾이면 키운다
  //   너무 작으면 안쪽 바퀴가 뒤로 돌거나 멈추므로 exit_arc_min_radius_m 보다 작게는 안 한다
  double exit_arc_radius()
  {
    double radius = exit_arc_radius_m_;
    if (radius <= 0.0)
    {
      radius = backup_speed_ * std::max(0.0, backup_time_) * exit_arc_radius_gain_;
    }
    if (radius < exit_arc_min_radius_m_)
    {
      RCLCPP_WARN(get_logger(), "곡선 반지름 %.2fm 가 너무 작음 -> %.2fm 로 (두 바퀴 다 앞으로 가게)", radius,
        exit_arc_min_radius_m_);
      radius = exit_arc_min_radius_m_;
    }
    return radius;
  }

  // 곡선 시작: 앞으로 exit_arc_speed, 회전 = 속도 / 반지름 (90도 돌았던 쪽과 같은 쪽)
  //   두 바퀴 속도 = 속도 x (1 -+ 바퀴 간격 절반 / 반지름) -> 반지름이 바퀴 간격 절반보다 크면 둘 다 앞으로
  void start_exit_arc()
  {
    state_ = State::EXIT_ARC;
    state_time_ = 0.0;
    const double radius = exit_arc_radius();
    exit_w_ = side_direction_ * exit_arc_speed_ / radius;
    exit_arc_time_ = (exit_arc_deg_ * M_PI / 180.0) / std::abs(exit_w_);   // 카메라로 못 잴 때
    exit_arc_time_full_ = exit_arc_time_;
    exit_arc_limit_ = exit_arc_time_ * 2.0;                                  // 너무 오래 걸리면 포기
    exit_prev_angles_ = last_scene_angles_;     // 후진 중 마지막 화면의 직선 방향 분포 (카메라로 각도 잴 기준)
    exit_use_cam_ = side_cam_ && !exit_prev_angles_.empty();
    exit_cam_lost_ = 0;
    exit_cam_sum_ = 0.0;
    exit_yellow_seen_ = false;
    RCLCPP_INFO(get_logger(), "%s으로 곡선 %.0f도 (반지름 %.0fcm, 속도 %.2fm/s, %s)",
      side_direction_ < 0.0 ? "오른쪽" : "왼쪽", exit_arc_deg_, radius * 100.0, exit_arc_speed_,
      exit_use_cam_ ? "카메라로 각도 잼" : "시간으로");
    drive(exit_arc_speed_, exit_w_);
  }

  // 영상이 올 때마다 (EXIT_ARC)
  //   (1) 들어왔던 길의 노란선이 보이면 그것만 본다: 아직 덜 돌았으면 계속, 세로이거나 지나쳤으면 끝
  //       (시간으로 어림한 각도가 exit_check_ratio (30%) 를 넘은 뒤부터. 막 돌기 시작할 때 착각하지 않게)
  //   (2) 노란선이 안 보이면 SIDE_TURN 과 같은 방법으로 화면 직선들이 돈 각도를 더해 90도면 끝
  //       (곡선으로 가면 화면이 옆으로도 밀리지만, 직선의 "방향" 은 로봇이 돈 만큼만 바뀐다)
  void exit_arc_by_camera(const cv::Mat & yellow, const cv::Mat & lines_mask)
  {
    exit_yellow_seen_ = false;
    if (state_time_ >= exit_arc_time_full_ * exit_check_ratio_)
    {
      const std::optional<double> lean = exit_yellow_lean(yellow);
      // 남은 기울기: 왼쪽으로 돌 때는 왼쪽으로 기운 만큼 (+), 오른쪽으로 돌 때는 오른쪽으로 기운 만큼
      //   한 영상 사이에 exit_overshoot_deg (15도) 넘게 지나칠 수는 없으니, 그보다 많이 지나친 걸로 보이면
      //   선을 잘못 찾은 것 -> 이번 영상은 노란선이 없는 걸로 친다
      const double remain = lean ? side_direction_ * *lean : 0.0;
      if (lean && remain >= -exit_overshoot_deg_ * M_PI / 180.0)
      {
        exit_yellow_seen_ = true;
        if (remain <= aligned_deg_ * M_PI / 180.0)
        {
          RCLCPP_INFO(get_logger(), "곡선 중 길의 노란선이 세로가 됨 (기울기 %.1f도) -> 길 내려가기",
            *lean * 180.0 / M_PI);
          start_exit_stem();
          return;
        }
        drive(exit_arc_speed_, exit_w_);
        return;
      }
    }
    exit_by_scene_angles(lines_mask);
  }

  // 곡선 중 노란선이 안 보일 때: 화면 직선들이 돈 각도로 잰다 (못 재면 시간으로 넘어감)
  void exit_by_scene_angles(const cv::Mat & lines_mask)
  {
    if (!exit_use_cam_)
    {
      return;                    // 시간으로 도는 중 (watchdog 이 본다)
    }
    const std::vector<double> now_angles = scene_line_angles(lines_mask);
    const std::optional<double> step = rotation_between(exit_prev_angles_, now_angles);
    if (!now_angles.empty())
    {
      exit_prev_angles_ = now_angles;
    }
    if (!step)
    {
      exit_cam_lost_++;
      if (exit_cam_lost_ >= side_cam_lost_frames_)
      {
        // 남은 각도를 시간으로: 지금까지 걸린 시간 + 남은 각도 / 회전 속도
        exit_use_cam_ = false;
        const double remain = std::max(0.0, exit_arc_deg_ * M_PI / 180.0 - std::abs(exit_cam_sum_));
        exit_arc_time_ = state_time_ + remain / std::abs(exit_w_);
        RCLCPP_WARN(get_logger(), "곡선 중 직선을 놓침 (카메라로 %.1f도) -> 남은 만큼 시간으로 (노란선이 보이면 그걸로)",
          std::abs(exit_cam_sum_) * 180.0 / M_PI);
      }
      return;                    // 바퀴는 마지막 명령대로 계속
    }
    exit_cam_lost_ = 0;
    exit_cam_sum_ += *step;
    // 곡선으로 가면 새 직선이 화면에 들어와서 이 방법이 실제보다 조금 크게 잰다 (시뮬레이션: 75도를 90도로)
    //   -> 노란선이 안 보이는 동안은 exit_wait_yellow_deg (15도) 더 돌면서 노란선을 기다린다
    const double target = (exit_arc_deg_ + exit_wait_yellow_deg_) * M_PI / 180.0;
    const double remain = target - std::abs(exit_cam_sum_);
    if (remain <= side_tolerance_deg_ * M_PI / 180.0)
    {
      RCLCPP_INFO(get_logger(), "곡선 %.1f도 돌았음 (카메라)", std::abs(exit_cam_sum_) * 180.0 / M_PI);
      start_exit_stem();
      return;
    }
    drive(exit_arc_speed_, exit_w_);
  }

  // 들어왔던 길의 노란선 기울기 [rad] (+ 왼쪽으로 기움). 안 보이면 비어 있음
  //   두 선이 길 폭만큼 떨어져 나란하면 둘의 평균, 아니면 보이는 한 선
  //   (곡선은 실제 반지름이 조금 달라 옆으로 밀릴 수 있어서 선의 자리는 따지지 않는다)
  //   점들이 직선에서 exit_line_max_rms_px 보다 많이 벗어난 선(두 노란선이 한쪽 칸에 같이 들어와 섞인 것)은 안 쓴다
  std::optional<double> exit_yellow_lean(const cv::Mat & yellow)
  {
    const int mid = static_cast<int>(bev_center_x_ * yellow.cols);
    std::optional<Line> left = fit_line(yellow, 0, mid, yellow.rows);
    std::optional<Line> right = fit_line(yellow, mid, yellow.cols, yellow.rows);
    if (left && left->rms > exit_line_max_rms_px_)
    {
      left.reset();
    }
    if (right && right->rms > exit_line_max_rms_px_)
    {
      right.reset();
    }
    if (left && right)
    {
      const double bottom_y = yellow.rows - 1;
      const double gap = right->x_at(bottom_y) - left->x_at(bottom_y);
      if (gap >= pair_gap_min_px_ && gap <= pair_gap_max_px_ &&
        std::abs(left->lean_left() - right->lean_left()) <= parallel_deg_ * M_PI / 180.0)
      {
        return (left->lean_left() + right->lean_left()) / 2.0;
      }
    }
    if (left)
    {
      return left->lean_left();
    }
    if (right)
    {
      return right->lean_left();
    }
    return std::nullopt;
  }

  // 0.1초마다 (EXIT_ARC): 시간으로 도는 중이면 시간이 되면 끝 (노란선이 보이는 중이면 그 선을 보고 끝낸다),
  //   카메라로 도는 중이어도 너무 오래 걸리면 (exit_arc_limit = 시간의 2배) 포기
  void step_exit_arc()
  {
    if (state_time_ >= exit_arc_limit_)
    {
      give_up("곡선으로 도는 데 너무 오래 걸림");
      return;
    }
    if (exit_use_cam_)
    {
      return;                    // 바퀴 명령은 영상이 올 때마다 exit_arc_by_camera() 가 보낸다
    }
    if (state_time_ >= exit_arc_time_ && !exit_yellow_seen_)
    {
      RCLCPP_INFO(get_logger(), "곡선 %.0f도 돌았음 (시간)", exit_arc_deg_);
      start_exit_stem();
      return;
    }
    drive(exit_arc_speed_, exit_w_);
  }

  // ==================== (8) EXIT_STEM: 들어왔던 길을 반대로 내려가기 ====================

  void start_exit_stem()
  {
    state_ = State::EXIT_STEM;
    state_time_ = 0.0;
    lines_missing_ = 0;
    exit_white_y_ = -1.0;
    RCLCPP_INFO(get_logger(), "들어왔던 길을 두 차선 가운데로 내려감 (도로 흰선이 보이면 멈춤)");
  }

  // 도로의 흰선이 화면 아래에 옴 = 노란선을 지나 도로에 나옴 -> 정지
  void finish_exit()
  {
    stop_wheels();
    state_ = State::DONE;
    publish_mode(ControlMode::STOP, "exited");
    RCLCPP_INFO(get_logger(), "도로 흰선이 화면 아래에 옴 (%.1f초 내려감) -> 노란선을 지나 나옴, 정지 (다음 단계는 아직 없음)",
      state_time_);
  }

  // odom: 로봇 방향(yaw) 저장
  void on_odom(const nav_msgs::msg::Odometry & msg)
  {
    const auto & q = msg.pose.pose.orientation;
    odom_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    odom_received_ = true;
    last_odom_time_ = now();
    if (state_ != State::SIDE_TURN || !side_use_odom_)
    {
      return;
    }
    if (!side_use_cam_)
    {
      step_side_turn_odom();
      return;
    }
    // 카메라로 도는 중: odom 으로 잰 각도가 목표 + side_cam_max_extra_deg 를 넘으면 무조건 멈춤 (안전)
    side_turned_ = std::abs(wrap_angle(odom_yaw_ - side_start_yaw_));
    if (side_turned_ >= (side_turn_deg_ + side_cam_max_extra_deg_) * M_PI / 180.0)
    {
      RCLCPP_WARN(get_logger(), "카메라 각도 %.1f도인데 odom 으로 %.1f도 -> 안전하게 멈춤",
        side_cam_turned_ * 180.0 / M_PI, side_turned_ * 180.0 / M_PI);
      end_side_turn();
    }
  }

  // 각도를 -pi ~ pi 사이로
  static double wrap_angle(double a)
  {
    return std::atan2(std::sin(a), std::cos(a));
  }

  // ==================== 끝내기 ====================

  void finish()
  {
    stop_wheels();
    state_ = State::DONE;
    publish_mode(ControlMode::STOP, "stem_end");
    RCLCPP_INFO(get_logger(), "차선이 끝남 (통로 끝, %.1f초 올라감) -> 정지 (다음 단계는 아직 없음)",
      state_time_);
  }

  void give_up(const char * reason)
  {
    stop_wheels();
    state_ = State::DONE;
    RCLCPP_WARN(get_logger(), "주차 그만둠: %s", reason);
  }

  // ==================== 0.1초마다 ====================

  void watchdog()
  {
    if (!i_am_driving())
    {
      return;
    }

    // lane_follower 가 멈춰 있도록 "주차 중" 을 계속 알린다
    publish_mode(ControlMode::PARKING, state_name());

    state_time_ += 0.1;
    if (state_ == State::EXTRA && state_time_ >= extra_time_)
    {
      start_turn();
      return;
    }
    if (state_ == State::APPROACH && approach_timeout_ > 0.0 && state_time_ >= approach_timeout_)
    {
      give_up("입구를 못 찾음 (approach_timeout)");
      return;
    }
    if (state_ == State::TURN && turn_timeout_ > 0.0 && state_time_ >= turn_timeout_)
    {
      give_up("입구 길이 똑바로 안 보임 (turn_timeout)");
      return;
    }
    if (state_ == State::STEM && stem_timeout_ > 0.0 && state_time_ >= stem_timeout_)
    {
      give_up("길 끝을 못 찾음 (stem_timeout)");
      return;
    }
    if (state_ == State::SIDE_TURN)
    {
      step_side_turn();          // 시간으로 도는 거라 영상이 없어도 계속
      return;
    }
    if (state_ == State::BACKUP)
    {
      step_backup();             // 시간으로 후진 (영상 안 봄)
      return;
    }
    if (state_ == State::EXIT_ARC)
    {
      step_exit_arc();           // 시간 초과 확인 (카메라로 못 재면 시간으로 돌기)
      return;
    }
    if (state_ == State::EXIT_STEM && exit_timeout_ > 0.0 && state_time_ >= exit_timeout_)
    {
      give_up("도로 흰선을 못 찾음 (exit_timeout)");
      return;
    }

    // 영상이 끊기면 멈춤
    if ((now() - last_image_time_).seconds() > image_timeout_)
    {
      stop_wheels();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "영상이 안 옴 -> 정지");
    }
  }

  // ==================== 도우미 ====================

  bool i_am_driving() const
  {
    return state_ == State::APPROACH || state_ == State::EXTRA || state_ == State::TURN ||
           state_ == State::STEM || state_ == State::SIDE_TURN || state_ == State::BACKUP ||
           state_ == State::EXIT_ARC || state_ == State::EXIT_STEM;
  }

  // GUI 에 보일 단계 이름
  const char * state_name() const
  {
    switch (state_)
    {
      case State::APPROACH:
        return "approach";
      case State::EXTRA:
        return "extra";
      case State::TURN:
        return "turn";
      case State::STEM:
        return "stem";
      case State::SIDE_TURN:
        return side_direction_ < 0.0 ? "side_turn_right" : "side_turn_left";
      case State::BACKUP:
        return "backup";
      case State::EXIT_ARC:
        return "exit_arc";
      case State::EXIT_STEM:
        return "exit_stem";
      default:
        return "-";
    }
  }

  // 바퀴 명령. 앞으로 갈 때 앞이 막혔으면 멈춤
  void drive(double v, double w)
  {
    if (v > 0.0 && front_closer_than(front_stop_distance_))
    {
      stop_wheels();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "앞이 막힘 -> 대기");
      return;
    }
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = v;
    cmd.angular.z = w;
    cmd_pub_->publish(cmd);
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
  bool skip_sign_;
  std::string image_topic_;
  int64_t yellow_l_min_;
  int64_t yellow_b_min_;
  int64_t yellow_a_max_;
  int64_t white_l_min_;
  int64_t white_ab_dev_;
  double bev_center_x_;
  double lane_width_px_;
  int64_t line_min_rows_;
  int64_t flat_min_pixels_;
  double drive_speed_;
  bool use_lane_speed_;
  double heading_gain_;
  double offset_gain_;
  double offset_soft_px_;
  double max_angular_;
  double robot_y_offset_px_;
  int64_t watch_x_max_;
  int64_t watch_rows_;
  int64_t watch_bottom_;
  int64_t present_pixels_;
  int64_t empty_pixels_;
  int64_t gap_frames_needed_;
  double extra_forward_m_;
  double turn_speed_;
  double turn_min_speed_;
  double turn_slow_angle_deg_;
  double turn_slow_power_;
  double aligned_deg_;
  double parallel_deg_;
  bool single_line_ok_;
  double single_line_margin_px_;
  double pair_gap_min_px_;
  double pair_gap_max_px_;
  double turn_min_time_;
  int64_t turn_fit_rows_;
  bool stem_use_white_;
  double same_line_px_;
  int64_t stem_end_frames_;
  double stem_min_time_;
  double stem_timeout_;
  double side_psd_threshold_;
  int64_t side_hits_needed_;
  double side_turn_deg_;
  bool side_turn_toward_;
  std::string side_turn_when_;
  bool side_cam_;
  int64_t side_cam_min_line_px_;
  double side_cam_max_step_deg_;
  int64_t side_cam_lost_frames_;
  double side_cam_max_extra_deg_;
  int64_t white_trigger_y_;
  int64_t white_row_min_pixels_;
  double white_center_ratio_;
  int64_t white_arm_y_;
  double side_clear_threshold_;
  double side_turn_speed_;
  double side_slow_angle_deg_;
  double side_slow_power_;
  double side_min_speed_;
  double side_tolerance_deg_;
  double odom_timeout_;
  double backup_time_;
  double backup_speed_;
  bool exit_enable_;
  double exit_arc_speed_;
  double exit_arc_radius_m_;
  double exit_arc_radius_gain_;
  double exit_arc_min_radius_m_;
  double exit_arc_deg_;
  double exit_check_ratio_;
  double exit_line_max_rms_px_;
  double exit_overshoot_deg_;
  double exit_wait_yellow_deg_;
  int64_t exit_stop_y_;
  double exit_white_max_tilt_deg_;
  double exit_timeout_;
  double front_stop_distance_;
  double image_timeout_;
  double approach_timeout_;
  double turn_timeout_;

  // 상태
  State state_ = State::WAITING;
  Gap gap_ = Gap::BEFORE;
  int64_t gap_frames_ = 0;         // 지금 조건이 연속으로 맞은 프레임 수
  int64_t sign_hits_ = 0;
  double state_time_ = 0.0;        // 지금 단계에 들어온 뒤 지난 시간 [s]
  double extra_time_ = 0.0;        // EXTRA 단계에서 더 직진할 시간 [s]
  double speed_ = 0.10;            // 지금 쓰는 직진 속도 [m/s] (주차 시작할 때 정함)
  int64_t lines_missing_ = 0;      // STEM 에서 차선이 둘 다 연속으로 안 보인 프레임 수
  int64_t left_hits_ = 0;          // 왼쪽 PSD 가 연속으로 가까웠던 횟수
  int64_t right_hits_ = 0;         // 오른쪽 PSD 가 연속으로 가까웠던 횟수
  double side_seen_ = 0.0;         // 장애물을 본 쪽 (+1 왼쪽, -1 오른쪽, 0 아직 못 봄)
  int64_t clear_hits_ = 0;         // 장애물을 본 쪽 PSD 가 연속으로 멀었던 횟수
  double last_white_y_ = -1.0;     // 바로 전 영상에서 가로 흰 선의 줄 번호 (-1 = 없음)
  std::vector<double> last_scene_angles_;  // STEM 중 바로 전 영상의 직선 방향 분포 (180칸)
  double side_direction_ = 0.0;    // SIDE_TURN 방향 (+1 왼쪽, -1 오른쪽)
  double side_turn_time_ = 0.0;    // SIDE_TURN 할 시간 [s] (odom 없을 때)
  double side_turn_limit_ = 0.0;   // SIDE_TURN 최대 시간 [s] (넘으면 포기)
  bool side_use_odom_ = false;     // SIDE_TURN 을 odom 으로 재는지
  bool side_use_cam_ = false;      // SIDE_TURN 을 카메라 (화면 직선들이 돈 각도) 로 재는지
  std::vector<double> side_prev_angles_;   // SIDE_TURN 중 바로 전 영상의 직선 방향 분포 (180칸)
  double side_cam_sum_ = 0.0;      // 카메라로 잰 돈 각도 합 (부호 있음) [rad]
  int64_t side_cam_lost_ = 0;      // 흰 선을 연속으로 못 찾은 프레임 수
  double side_cam_turned_ = 0.0;   // 카메라로 잰 돈 각도 [rad]
  double side_start_yaw_ = 0.0;    // SIDE_TURN 시작할 때 방향 [rad]
  double side_turned_ = 0.0;       // 지금까지 돈 각도 [rad]
  double exit_w_ = 0.0;            // EXIT_ARC 회전 속도 [rad/s] (+ 왼쪽)
  double exit_arc_time_ = 0.0;     // EXIT_ARC 를 시간으로 끝낼 때의 시간 [s]
  double exit_arc_time_full_ = 0.0;  // 처음 계산한 "90도 / 회전 속도" 시간 [s]
  double exit_arc_limit_ = 0.0;    // EXIT_ARC 최대 시간 [s] (넘으면 포기)
  bool exit_use_cam_ = false;      // EXIT_ARC 를 카메라로 재는지
  bool exit_yellow_seen_ = false;  // EXIT_ARC 중 바로 전 영상에서 길의 노란선이 보였는지
  std::vector<double> exit_prev_angles_;   // EXIT_ARC 중 바로 전 영상의 직선 방향 분포
  int64_t exit_cam_lost_ = 0;      // EXIT_ARC 중 직선을 연속으로 못 찾은 프레임 수
  double exit_cam_sum_ = 0.0;      // EXIT_ARC 중 카메라로 잰 돈 각도 합 (부호 있음) [rad]
  double exit_white_y_ = -1.0;     // EXIT_STEM 중 바로 전 영상의 가로 흰 선 줄 번호 (-1 = 없음)
  double odom_yaw_ = 0.0;          // odom 방향 [rad]
  bool odom_received_ = false;
  rclcpp::Time last_odom_time_;
  rclcpp::Time last_image_time_;
  interfaces::msg::PsdArray psd_;
  bool psd_received_ = false;

  // ROS 연결
  rclcpp::Subscription<Sign>::SharedPtr sign_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<interfaces::msg::PsdArray>::SharedPtr psd_sub_;
  rclcpp::Subscription<ControlMode>::SharedPtr mode_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<ControlMode>::SharedPtr mode_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::AsyncParametersClient::SharedPtr lane_params_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParkingNode>());
  rclcpp::shutdown();
  return 0;
}
