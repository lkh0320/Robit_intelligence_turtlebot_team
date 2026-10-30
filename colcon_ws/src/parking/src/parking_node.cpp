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
//   (1) WAITING  : 주차 표지판을 기다린다 (운전은 grid_drive)
//          │  주차 표지판이 가까이 보임 (크기 sign_area_ratio_threshold 이상)
//          ▼
//   (2) APPROACH : 이 노드가 운전한다 (control_mode = PARKING -> grid_drive 멈춤)
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
//   ※ entry_arc: true (기본) 면 (2) ~ (3) 대신 곡선으로 부드럽게 들어간다:
//      (2) 에서 왼쪽 아래 노란선이 "끊기는" 순간 (입구가 시작됨, 바퀴 축 ~ 입구 가운데 약 entry_gap_to_center_m)
//      -> (2-1) (entry_gap_to_center_m - entry_arc_radius_m) 만큼 더 직진
//      -> (3-1) ENTRY_ARC: 두 바퀴 다 앞으로, 반지름 entry_arc_radius_m 로 왼쪽 곡선
//               (곡선 중심이 입구 가운데 줄 위에 오므로, 다 돌면 길 가운데로 들어간다)
//               화면 직선들이 entry_check_deg (45도) 넘게 돈 뒤부터 길의 노란선을 보고,
//               노란선이 세로가 되면 (4) STEM 으로
//
//         ──────●╮        (●: 노란선이 끊긴 곳에서 조금 더 간 곳)
//                ╰─╮
//                  │      <- 노란선이 세로로 보이면 곡선 끝
//
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
//   ※ side_arc: true (기본) 면 "제자리 90도 + 후진" 대신 뒤로 가면서 도는 곡선 한 번으로 칸에 들어간다:
//      (4-0) SIDE_FORWARD : side_arc_forward_m 만큼 더 올라감 (기본 0 = 안 올라가고 바로 곡선)
//                           (뒤로 곡선을 그리면 반지름만큼 통로 아래쪽으로 내려온다)
//      (4-1) SIDE_TURN    : 뒤로 가면서 장애물 쪽으로 90도 (side_arc_speed, 반지름 side_arc_radius_m)
//                           -> 다 돌면 반대쪽 칸 안에서 장애물 쪽을 보고 있다
//      (4-2) BACKUP       : side_arc_backup_time 초 더 후진 (0 이면 건너뜀)
//
//   90도를 언제 멈추나 (제자리 / 곡선 모두, 카메라 - side_turn_by_camera() 설명 참고):
//      주차장의 선(서로 직각인 격자)의 기울기를 영상마다 재서 끊김 없이 이어 붙인다 -> 격자 기준 90도에서 멈춤
//      선이 안 보이는 동안만 odom 으로 대충 세고, 다시 보이면 격자로 바로잡는다
//   side_turn_fixed: true 면 카메라를 안 보고 odom (x side_odom_scale) 각도가 side_turn_deg 가 되면 멈춘다
//
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
//          │        leave_turn: true (기본) 면: 왼쪽 아래 노란선이 보이다가 끊기면 (길이 도로와 만나는 곳)
//          │          -> 들어갈 때 (entry_arc) 와 같은 방법으로 조금 직진 + 왼쪽 곡선
//          │          -> 도로의 노란선이 세로로 보이면 control_mode = LANE ("parking_node:leave_done") -> 일반 주행
//          │          (주차 노드는 다시 표지판 기다리기. leave_rearm_delay 초 동안은 표지판을 무시)
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
#include "std_msgs/msg/empty.hpp"

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
    WAITING,     // 주차 표지판 기다리는 중 (운전은 grid_drive)
    APPROACH,    // 흰선 보고 직진하며 입구 찾는 중 (이 노드가 운전)
    EXTRA,       // 입구 가운데까지 남은 거리만큼 더 직진 (이 노드가 운전)
    TURN,        // 입구에서 왼쪽으로 도는 중 (이 노드가 운전)
    ENTRY_ARC,   // 입구로 곡선을 그리며 왼쪽으로 들어가는 중 (entry_arc: true 일 때, 이 노드가 운전)
    STEM,        // 주차장 길 -> 통로를 두 차선(노란선 / 흰 점선) 가운데로 올라가는 중 (이 노드가 운전)
    SIDE_FORWARD,  // 뒤로 곡선을 그리기 전에 반지름만큼 더 올라가는 중 (side_arc: true, 이 노드가 운전)
    SIDE_TURN,   // 장애물 쪽으로 90도 도는 중 (제자리, 또는 side_arc 면 뒤로 가면서. 이 노드가 운전)
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
    loadParams();
    // 시작할 때만 읽는 파라미터. 설명의 "재시작" 을 보고 turtle_gui 가 ↻ 표시
    rcl_interfaces::msg::ParameterDescriptor restart;
    restart.description = "재시작해야 반영 (시작할 때만 읽음)";
    image_topic_ = declare_parameter("image_topic", std::string("image_bev"), restart);
    skip_sign_ = declare_parameter("skip_sign", false, restart);
    // 나머지는 실행 중에 바꾸면 loadParams 로 바로 반영
    rcl_interfaces::msg::ParameterDescriptor ro;
    ro.read_only = true;
    ro.description = "turtle_gui 표시용: 파라미터를 바꾸면 바로 반영되는 노드";
    declare_parameter("live_params", true, ro);
    param_cb_ = add_post_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & ps)
      {
        loadParams();
        for (const auto & q : ps)
        {
          RCLCPP_INFO(get_logger(), "파라미터 %s = %s", q.get_name().c_str(), q.value_to_string().c_str());
        }
      });

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
    // GUI "자율주행 시작" (mission_start): 지난 시험에서 그만둔 / 끝난 상태를 지우고 표지판 기다리기부터
    //   (예전에는 GUI LANE 버튼만 다시 시작시켜서, STOP 으로 그만둔 뒤 미션을 시작하면 주차를 안 했다)
    start_sub_ = create_subscription<std_msgs::msg::Empty>(
      "mission_start", 10, [this](std_msgs::msg::Empty::ConstSharedPtr)
      {
        if (i_am_driving())
        {
          give_up("자율주행 다시 시작");
        }
        rearm_until_s_ = 0.0;
        reset();
      });
    mode_sub_ = create_subscription<ControlMode>(
      "control_mode", 10, [this](ControlMode::ConstSharedPtr msg)
      {
        on_mode(*msg);
      });

    // ---- 보내는 토픽 ----
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    // 일반 주행 (grid_drive) 이 보내는 속도를 기억해 둔다 (주차를 같은 속도로 이어가려고)
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr msg)
      {
        if (!i_am_driving() && msg->linear.x > 0.0)
        {
          seen_drive_v_ = msg->linear.x;
          seen_drive_t_ = now().seconds();
        }
      });
    mode_pub_ = create_publisher<ControlMode>("control_mode", 10);

    // ---- 0.1초마다: 모드 알림, 영상 끊김 / 시간 초과 확인 ----
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]
      {
        watchdog();
      });
    // 0.02초마다: 부드럽게 바꾼 바퀴 명령 보내기 (smooth)
    smooth_timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]
      {
        smooth_step();
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

  // 처음이면 선언, 이미 있으면 지금 값 (실행 중 바뀐 값) 을 읽는다
  template<typename T>
  T param(const std::string & name, const T & def)
  {
    if (!has_parameter(name))
    {
      return declare_parameter(name, def);
    }
    return get_parameter(name).get_value<T>();
  }

  // 실행 중에 바꿀 수 있는 파라미터 전부. 생성자에서 한 번, 파라미터가 바뀔 때마다 다시
  //   (turtle_gui 파라미터 탭 / ros2 param set). 주차 도중에 바꿔도 다음 영상 / 단계부터 그 값
  void loadParams()
  {
    // 1) 표지판
    min_confidence_ = param("min_confidence", 0.5);
    sign_area_ratio_threshold_ = param("sign_area_ratio_threshold", 0.02);
    sign_hits_needed_ = param("sign_hits_needed", 2);

    // 2) 색 / 화면
    yellow_l_min_ = param("yellow_l_min", 80);
    yellow_b_min_ = param("yellow_b_min", 150);
    yellow_a_max_ = param("yellow_a_max", 135);
    white_l_min_ = param("white_l_min", 190);
    white_ab_dev_ = param("white_ab_dev", 15);
    bev_center_x_ = param("bev_center_x", 0.5);
    lane_width_px_ = param("lane_width_px", 295.0);
    line_min_rows_ = param("line_min_rows", 40);
    flat_min_pixels_ = param("flat_min_pixels", 35);

    // 3) 흰선 따라 직진
    // 직진 속도 [m/s]
    drive_speed_ = param("drive_speed", 0.10);
    heading_gain_ = param("heading_gain", 1.0);
    offset_gain_ = param("offset_gain", 1.0);
    offset_soft_px_ = param("offset_soft_px", 300.0);
    max_angular_ = param("max_angular", 0.6);
    robot_y_offset_px_ = param("robot_y_offset_px", 179.0);

    // 4) 입구 찾기 (왼쪽 아래 칸)
    watch_x_max_ = param("watch_x_max", 120);
    watch_rows_ = param("watch_rows", 20);
    watch_bottom_ = param("watch_bottom", 390);
    present_pixels_ = param("present_pixels", 80);
    empty_pixels_ = param("empty_pixels", 15);
    gap_frames_needed_ = param("gap_frames_needed", 2);
    extra_forward_m_ = param("extra_forward_m", -0.04);

    // 5) 회전
    turn_speed_ = param("turn_speed", 0.4);
    turn_min_speed_ = param("turn_min_speed", 0.08);
    turn_slow_angle_deg_ = param("turn_slow_angle_deg", 60.0);
    turn_slow_power_ = param("turn_slow_power", 2.0);
    aligned_deg_ = param("aligned_deg", 4.0);
    parallel_deg_ = param("parallel_deg", 15.0);
    pair_gap_min_px_ = param("pair_gap_min_px", 150.0);
    pair_gap_max_px_ = param("pair_gap_max_px", 420.0);
    single_line_ok_ = param("single_line_ok", true);
    single_line_margin_px_ = param("single_line_margin_px", 80.0);
    turn_min_time_ = param("turn_min_time", 1.0);
    turn_fit_rows_ = param("turn_fit_rows", 230);

    // 5-1) 제자리 회전 대신 곡선으로 입구에 들어가기
    entry_arc_ = param("entry_arc", true);
    entry_gap_to_center_m_ = param("entry_gap_to_center_m", 0.33);
    entry_forward_m_ = param("entry_forward_m", 0.0);
    // 노란선 끊김을 알아챘을 때 로봇은 이미 (카메라 + 계산 지연) x 속도 만큼 지나와 있다 -> 그만큼 덜 직진
    //   (park5 녹화: 3번 모두 곡선 끝이 3~4cm 늦음 = 실제 속도 약 0.25 m/s x 0.14초)
    entry_latency_s_ = param("entry_latency_s", 0.14);
    entry_arc_radius_m_ = param("entry_arc_radius_m", 0.08);
    entry_arc_radius_gain_ = param("entry_arc_radius_gain", 1.0);
    entry_arc_min_radius_m_ = param("entry_arc_min_radius_m", 0.05);
    entry_arc_speed_base_ = param("entry_arc_speed", 0.08);
    // 주차장으로 들어가는 곡선만 일반 주행 코너 빠르기로 (곡선 모양은 그대로)
    entry_turn_match_ = param("entry_turn_match", true);
    entry_turn_slow_deg_ = param("entry_turn_slow_deg", 45.0);   // 카메라로 이만큼 돈 뒤에는 원래 빠르기 (entry_arc_speed)
    leave_turn_match_ = param("leave_turn_match", true);   // 도로로 나가는 곡선도 일반 주행 코너 빠르기로
    leave_slow_ = param("leave_slow", false);              // 나갈 때 노란선 끝이 가까워지면 줄이기
    entry_turn_w_ = param("entry_turn_w", 0.6);   // [rad/s 명령] 일반 주행 코너 회전 (grid_drive planner.max_angular 1.5 실제 / 2.45)
    entry_check_deg_ = param("entry_check_deg", 45.0);
    entry_pair_stop_ = param("entry_pair_stop", true);
    entry_pair_max_lean_deg_ = param("entry_pair_max_lean_deg", 30.0);
    entry_pair_check_deg_ = param("entry_pair_check_deg", 20.0);

    // 6) 주차장 길 올라가기
    stem_use_white_ = param("stem_use_white", true);
    same_line_px_ = param("same_line_px", 40.0);
    stem_end_frames_ = param("stem_end_frames", 3);
    stem_min_time_ = param("stem_min_time", 2.0);
    stem_timeout_ = param("stem_timeout", 20.0);

    // 7) 옆 PSD 가 가까우면 반대쪽으로 90도
    side_psd_threshold_ = param("side_psd_threshold", 0.15);
    side_hits_needed_ = param("side_hits_needed", 3);
    side_turn_base_deg_ = param("side_turn_deg", 90.0);
    // 왼쪽 / 오른쪽으로 돌 때 각도를 따로 (0 보다 작으면 side_turn_deg 를 씀)
    side_turn_deg_left_ = param("side_turn_deg_left", -1.0);
    side_turn_deg_right_ = param("side_turn_deg_right", -1.0);
    side_turn_toward_ = param("side_turn_toward", true);
    side_turn_when_ = param("side_turn_when", std::string("white"));
    white_trigger_y_ = param("white_trigger_y", 350);
    side_cam_ = param("side_turn_by_camera", true);
    side_turn_fixed_ = param("side_turn_fixed", false);
    side_odom_scale_ = param("side_odom_scale", 1.5);
    side_cam_min_line_px_ = param("side_cam_min_line_px", 60);
    side_cam_max_step_deg_ = param("side_cam_max_step_deg", 20.0);
    side_cam_lost_frames_ = param("side_cam_lost_frames", 5);
    side_cam_max_extra_deg_ = param("side_cam_max_extra_deg", 15.0);
    white_row_min_pixels_ = param("white_row_min_pixels", 150);
    white_center_ratio_ = param("white_center_ratio", 0.6);
    white_arm_y_ = param("white_arm_y", 300);
    side_clear_threshold_ = param("side_clear_threshold", 0.20);
    side_turn_speed_ = param("side_turn_speed", 0.6);
    side_min_speed_ = param("side_min_speed", 0.08);
    side_slow_angle_deg_ = param("side_slow_angle_deg", 30.0);
    side_slow_power_ = param("side_slow_power", 2.0);
    side_tolerance_deg_ = param("side_tolerance_deg", 1.0);
    odom_timeout_ = param("odom_timeout", 0.5);

    // 7-1) 90도 멈추는 기준: 격자 맞추기
    side_snap_ = param("side_snap", true);
    side_snap_min_ratio_ = param("side_snap_min_ratio", 0.5);

    // 7-2) 제자리 90도 + 후진 대신 뒤로 가면서 곡선으로 칸에 들어가기
    side_arc_ = param("side_arc", true);
    side_arc_radius_m_ = param("side_arc_radius_m", 0.12);
    side_arc_radius_gain_ = param("side_arc_radius_gain", 1.0);
    side_arc_speed_ = param("side_arc_speed", 0.06);
    side_arc_forward_m_ = param("side_arc_forward_m", 0.0);
    side_arc_forward_max_m_ = param("side_arc_forward_max_m", 0.30);
    // 0 보다 크면 거리 (side_arc_forward_m) 대신 이 시간 [s] 만큼 더 전진한 뒤 곡선
    side_arc_forward_time_ = param("side_arc_forward_time", 0.0);
    side_arc_backup_time_ = param("side_arc_backup_time", 0.0);

    // 8) 90도 돈 뒤 후진
    backup_time_ = param("backup_time", 3.0);
    backup_speed_ = param("backup_speed", 0.06);

    // 9) 후진 뒤 곡선으로 돌아서 들어왔던 길로 나가기
    exit_enable_ = param("exit_enable", true);
    exit_arc_speed_base_ = param("exit_arc_speed", 0.08);
    // 일반 주행과 같은 속도로: 주차를 시작할 때 grid_drive 가 보내던 속도를 직진 / 입구 곡선 / 나오는 곡선 / 길 내려가기 /
    //   도로로 나가는 곡선에 쓴다. 칸으로 들어가는 동안 (흰 선 뒤 전진, 뒤로 곡선, 후진) 은 원래 값 그대로
    match_drive_speed_ = param("match_drive_speed", true);
    match_speed_min_ = param("match_speed_min", 0.05);   // 이 범위 밖이면 잘라서 씀 [m/s, 명령 단위]
    match_speed_max_ = param("match_speed_max", 0.25);
    match_turn_speed_ = param("match_turn_speed", false);   // false = 곡선은 원래 속도 (entry_arc_speed / exit_arc_speed)
    // 주차 중이면 start_approach 에서 정한 속도 (일반 주행 속도일 수 있음) 를 그대로 둔다
    if (!i_am_driving() || !match_turn_speed_)
    {
      entry_arc_speed_ = entry_arc_speed_base_;
      exit_arc_speed_ = exit_arc_speed_base_;
    }
    exit_arc_radius_m_ = param("exit_arc_radius_m", 0.0);
    exit_arc_radius_gain_ = param("exit_arc_radius_gain", 2.0);
    exit_arc_min_radius_m_ = param("exit_arc_min_radius_m", 0.12);
    // 9-2) 보면서 나가기: 곡선 중 안내 노란선 (칸 모서리에 붙은 길의 노란선) 이 보이면
    //   그 선까지 거리 D 와 기운 각 phi 로 "이 반지름으로 돌면 선과 나란해질 때 선에서 exit_guide_offset_m 떨어진다"
    //   R = (D - offset) / (1 - cos phi) 를 매 영상 다시 계산해서 돈다. phi 가 exit_guide_align_deg 이하면 길 내려가기
    exit_guide_ = param("exit_guide", true);
    exit_guide_offset_m_ = param("exit_guide_offset_m", 0.15);     // 끝났을 때 선에서 로봇 가운데까지 (= 길 폭 / 2)
    exit_guide_align_deg_ = param("exit_guide_align_deg", 4.0);
    exit_guide_max_radius_m_ = param("exit_guide_max_radius_m", 2.0);
    // 9-3) 도로로 나가는 왼쪽 곡선도 보면서: 도로 바깥 흰 선에서 leave_guide_offset_m 떨어져 끝나게
    leave_guide_ = param("leave_guide", true);
    leave_guide_offset_m_ = param("leave_guide_offset_m", 0.15);   // 끝났을 때 흰 선에서 로봇 가운데까지 (= 차선 폭 / 2)
    leave_guide_align_deg_ = param("leave_guide_align_deg", 4.0);
    leave_guide_min_radius_m_ = param("leave_guide_min_radius_m", 0.10);
    leave_guide_max_rms_px_ = param("leave_guide_max_rms_px", 10.0);   // 이보다 두꺼운 흰 덩어리는 선이 아님 (상자 등)

    // 9-1) 흰 점선 (맞은편 칸 입구) 으로 후진 방향 맞추기 + 나가는 곡선 반지름
    //   후진하는 동안 점선의 기울기 x dash_gain 만큼 돌면서 후진 -> 점선이 화면에서 가로가 되게 (= 칸과 똑바로)
    //   나갈 때: 점선까지 거리 - 통로 폭 / 2 = 통로 가운데까지 거리 -> 이 반지름으로 돌면 통로 가운데로 나감
    dash_align_ = param("dash_align", true);
    dash_gain_ = param("dash_gain", 0.6);                // 기울기 [rad] x 이 값 = 회전 명령 [rad/s]
    dash_max_w_ = param("dash_max_w", 0.4);              // [rad/s] 맞추는 회전 최대
    dash_max_deg_ = param("dash_max_deg", 30.0);         // 이보다 기운 조각은 점선으로 안 봄
    dash_min_len_px_ = param("dash_min_len_px", 35.0);   // 점선 한 조각 길이 범위 [px] (실제 약 110)
    dash_max_len_px_ = param("dash_max_len_px", 130.0);
    dash_max_width_px_ = param("dash_max_width_px", 30.0);   // 조각 두께 최대 [px] (실제 약 20)
    bev_near_m_ = param("bev_near_m", 0.179);           // BEV 맨 아래 = 바퀴 축 앞 [m]
    aisle_width_m_ = param("aisle_width_m", 0.31);       // 통로 양쪽 점선 사이 [m] (녹화로 잰 값)
    exit_dash_gain_ = param("exit_dash_gain", 1.25);     // 실제 반지름 -> 명령 반지름 (0 = 점선 안 씀)
    // 왼쪽 / 오른쪽으로 나갈 때 따로 (0 보다 작으면 exit_dash_gain 을 씀)
    exit_dash_gain_left_ = param("exit_dash_gain_left", -1.0);
    exit_dash_gain_right_ = param("exit_dash_gain_right", -1.0);
    exit_arc_deg_ = param("exit_arc_deg", 90.0);
    exit_check_ratio_ = param("exit_check_ratio", 0.3);
    exit_line_max_rms_px_ = param("exit_line_max_rms_px", 8.0);
    exit_overshoot_deg_ = param("exit_overshoot_deg", 15.0);
    exit_wait_yellow_deg_ = param("exit_wait_yellow_deg", 15.0);
    exit_stop_y_ = param("exit_stop_y", 350);
    exit_white_max_tilt_deg_ = param("exit_white_max_tilt_deg", 30.0);
    exit_timeout_ = param("exit_timeout", 20.0);

    // 9-1) 길을 내려와 도로로 나갈 때: 들어갈 때처럼 곡선으로 왼쪽 -> 일반 주행(LANE)
    leave_turn_ = param("leave_turn", true);
    leave_forward_m_ = param("leave_forward_m", 0.0);
    leave_arc_radius_m_ = param("leave_arc_radius_m", 0.12);   // 도로로 나가는 곡선 처음 반지름 (흰 선 밟으면 줄이기, 노란선 쪽으로 붙으면 키우기)
    entry_yellow_min_s_ = param("entry_yellow_min_s", 0.5);   // 끊기기 전에 왼쪽 노란선이 이만큼 계속 보여야 입구 (0 = 안 봄)
    leave_latency_s_ = param("leave_latency_s", 0.0);   // 나갈 때 끊김을 늦게 알아채는 시간 (park12: 늦지 않음 -> 0)
    leave_yellow_frames_ = param("leave_yellow_frames", 2);
    // 나가는 길을 이 시간 [s] 내려간 뒤부터 왼쪽 노란선 끊김을 본다
    leave_min_time_ = param("leave_min_time", 1.0);
    // 가로 흰 선 / 노란선 끝이 가까워질수록 일반 주행 속도 -> drive_speed 로 줄이기 (돌기 시작할 때는 예전처럼 drive_speed)
    slow_near_ = param("slow_near", true);
    stem_slow_start_y_ = param("stem_slow_start_y", 100.0);    // 가로 흰 선이 이 줄에 오면 줄이기 시작, white_trigger_y 에서 drive_speed
    leave_slow_start_y_ = param("leave_slow_start_y", 100.0);  // 노란선 끝이 이 줄에 오면 줄이기 시작, 끊김 보는 칸 위에서 drive_speed
    leave_rearm_delay_ = param("leave_rearm_delay", 20.0);

    // 안전
    front_stop_distance_ = param("front_stop_distance", 0.10);
    image_timeout_ = param("image_timeout", 0.5);
    // 바퀴 명령 부드럽게 (속도는 smooth_accel 씩, 곡선은 smooth_curve_m 동안 서서히)
    smooth_ = param("smooth", true);
    smooth_accel_ = param("smooth_accel", 0.4);       // [m/s^2 명령] 속도 바뀌는 빠르기 (0.2 -> 0 이 0.5초)
    smooth_curve_m_ = param("smooth_curve_m", 0.02);  // [m 명령] 반지름 20cm 곡선으로 다 꺾을 때까지 가는 거리
    approach_timeout_ = param("approach_timeout", 30.0);
    turn_timeout_ = param("turn_timeout", 15.0);
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

    if (state_ == State::WAITING && now().seconds() < rearm_until_s_)
    {
      sign_hits_ = 0;            // 주차장에서 막 나왔음: 같은 표지판을 다시 보고 들어가지 않게
      return;
    }
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
      if (state_ == State::WAITING && entry_yellow_min_s_ > 0.0)
      {
        watch_left_yellow(msg);   // 표지판 기다리는 동안: 왼쪽 아래 노란선이 계속 보이는지만
      }
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
      if (entry_arc_ && gap_ == Gap::IN_GAP)
      {
        start_entry_straight();  // 곡선으로 들어가기: 노란선이 끊기자마자 곡선 시작점까지 직진
        return;
      }
      drive_along_white(white_mask(lab));
    }
    else if (state_ == State::EXTRA)
    {
      if (leaving_)
      {
        drive(speed_, 0.0);      // 도로로 나가는 중: 앞에 도로 흰선이 가로로 있어서 그냥 직진
      }
      else
      {
        drive_along_white(white_mask(lab));
      }
    }
    else if (state_ == State::TURN)
    {
      turn_until_aligned(yellow);
    }
    else if (state_ == State::ENTRY_ARC)
    {
      const cv::Mat white = white_mask(lab);
      // 도로로 나가는 곡선이면 도로 바깥 흰 선을 보고 돈다 (못 찾으면 원래 방식)
      if (!(leaving_ && leave_guide_ && leave_arc_by_guide(white)))
      {
        cv::Mat lines_mask;
        cv::bitwise_or(yellow, white, lines_mask);
        entry_arc_step(yellow, lines_mask);
      }
    }
    else if (state_ == State::SIDE_FORWARD)
    {
      // 반지름만큼 더 올라가는 동안에도 두 차선 가운데로. 돌기 시작할 때 쓸 직선 방향 분포도 계속 저장
      const cv::Mat white = white_mask(lab);
      cv::Mat lines;
      cv::bitwise_or(yellow, white, lines);
      if (side_cam_)
      {
        std::vector<double> angles = scene_line_angles(lines);
        if (!angles.empty())
        {
          last_scene_angles_ = angles;   // 주차장 끝 너머라 직선이 안 보이면 마지막으로 본 것을 둔다
        }
      }
      drive_between_lines(lines, false);
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
      // 후진은 시간으로 한다. 곡선으로 돌기 시작할 때의 기준(직선 방향 분포)도 계속 저장해 둔다
      const cv::Mat white = white_mask(lab);
      cv::Mat lines_mask;
      cv::bitwise_or(yellow, white, lines_mask);
      last_scene_angles_ = scene_line_angles(lines_mask);
      update_dash(white);        // 흰 점선 기울기 / 거리 (step_backup 이 보고 방향을 맞춤)
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
      // 곡선이 막 끝나 길 가운데로 오는 동안은 끊김을 안 봄 (한쪽 노란선이 화면 밖으로 잠깐 빠져서 착각함)
      if (leave_turn_ && leave_slow_)
      {
        slow_by_row(yellow_end_y(yellow), leave_slow_start_y_,
          static_cast<double>(watch_bottom_ - watch_rows_), "노란선 끝");
      }
      // 빨리 가면 leave_min_time 안에 끊김을 지나칠 수 있어서, 같은 거리 (leave_min_time x drive_speed) 를 가도 본다
      const bool leave_min_ok = state_time_ >= leave_min_time_ || exit_travel_ >= leave_min_time_ * drive_speed_;
      if (leave_turn_ && leave_gap_reached(yellow, leave_min_ok))
      {
        start_leave_straight();  // 길이 도로와 만남 -> 들어갈 때처럼 조금 직진 + 왼쪽 곡선
        return;
      }
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
      if (side_turn_when_ == "white")
      {
        slow_by_row(last_white_y_, stem_slow_start_y_, static_cast<double>(white_trigger_y_), "가로 흰 선");
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
    speed_ = drive_speed_;
    entry_arc_speed_ = entry_arc_speed_base_;
    exit_arc_speed_ = exit_arc_speed_base_;
    const bool fresh = seen_drive_t_ >= 0.0 && now().seconds() - seen_drive_t_ < 1.0;
    if (match_drive_speed_ && fresh)
    {
      // 직진은 일반 주행 속도 그대로. 곡선은 match_turn_speed 일 때만. 곡선 반지름은 속도와 상관없이 그대로다
      speed_ = std::clamp(seen_drive_v_, match_speed_min_, match_speed_max_);
      if (match_turn_speed_)   // false 면 곡선은 entry_arc_speed / exit_arc_speed 그대로
      {
        entry_arc_speed_ = speed_;
        exit_arc_speed_ = speed_;
      }
      RCLCPP_INFO(get_logger(), "주차 직진 속도 = 일반 주행 속도 %.2f m/s (곡선 / 칸 안은 원래 속도)", speed_);
    }
    else
    {
      RCLCPP_INFO(get_logger(), "주차 직진 속도 %.2f m/s (drive_speed)", speed_);
    }
    run_speed_ = speed_;
    v_out_ = fresh ? std::clamp(seen_drive_v_, 0.0, 0.5) : 0.0;   // 일반 주행이 가던 속도에서 이어서
    w_out_ = 0.0;
    k_out_ = 0.0;
    tgt_v_ = v_out_;           // 첫 영상이 오기 전에 "명령 끊김" 으로 멈추지 않게 (그동안 지금 속도로 직진)
    tgt_w_ = 0.0;
    tgt_t_ = now().seconds();
    state_ = State::APPROACH;
    leaving_ = false;
    gap_ = Gap::BEFORE;
    gap_frames_ = 0;
    state_time_ = 0.0;
    last_image_time_ = now();
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
  // 왼쪽 아래 칸 (watch_*) 의 노란 점 수로 "노란선이 계속 보인 시간" 을 잰다
  //   present_pixels 이상이면 보임 (이어짐), empty_pixels 이하가 0.15초 넘게 이어지면 끊김 (새로 잼)
  void track_left_yellow(int count)
  {
    const double t = now().seconds();
    if (count >= present_pixels_)
    {
      if (yellow_run_start_ < 0.0)
      {
        yellow_run_start_ = t;
      }
      yellow_run_last_ = t;
    }
    else if (count <= empty_pixels_ && yellow_run_start_ >= 0.0 && t - yellow_run_last_ > 0.15)
    {
      yellow_run_start_ = -1.0;
    }
  }

  // 마지막으로 노란선이 계속 보인 시간 [s] (없으면 0)
  double left_yellow_run() const
  {
    return yellow_run_start_ < 0.0 ? 0.0 : yellow_run_last_ - yellow_run_start_;
  }

  // 표지판 기다리는 동안: 영상의 왼쪽 아래 칸만 잘라서 노란 점 수를 센다 (가볍게)
  void watch_left_yellow(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    cv_bridge::CvImageConstPtr cv;
    try
    {
      cv = cv_bridge::toCvShare(msg, "bgr8");
    }
    catch (const cv_bridge::Exception &)
    {
      return;
    }
    const cv::Mat & img = cv->image;
    const int y1 = std::min(static_cast<int>(watch_bottom_) + 1, img.rows);
    const int y0 = std::max(0, y1 - static_cast<int>(watch_rows_));
    const int x1 = std::min(static_cast<int>(watch_x_max_), img.cols);
    if (y1 <= y0 || x1 <= 0)
    {
      return;
    }
    cv::Mat lab;
    cv::cvtColor(img(cv::Range(y0, y1), cv::Range(0, x1)), lab, cv::COLOR_BGR2Lab);
    track_left_yellow(cv::countNonZero(yellow_mask(lab)));
  }

  bool entrance_reached(const cv::Mat & yellow)
  {
    const int y1 = std::min(static_cast<int>(watch_bottom_) + 1, yellow.rows);
    const int y0 = std::max(0, y1 - static_cast<int>(watch_rows_));
    const int x1 = std::min(static_cast<int>(watch_x_max_), yellow.cols);
    const int count = cv::countNonZero(yellow(cv::Range(y0, y1), cv::Range(0, x1)));

    if (gap_ == Gap::BEFORE)
    {
      track_left_yellow(count);
      gap_frames_ = count <= empty_pixels_ ? gap_frames_ + 1 : 0;
      if (gap_frames_ >= gap_frames_needed_ && entry_yellow_min_s_ > 0.0 && left_yellow_run() < entry_yellow_min_s_)
      {
        RCLCPP_WARN(get_logger(), "끊기기 전에 왼쪽 노란선을 %.2f초만 봄 (< %.2f초) -> 입구가 아님 (표지판을 늦게 봄). 이번엔 주차 안 함",
          left_yellow_run(), entry_yellow_min_s_);
        finish_leave();          // 일반 주행에 넘기고 다시 표지판 기다리기
        return false;
      }
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
      start_turn_or_arc();
      return;
    }
    state_ = State::EXTRA;
    state_time_ = 0.0;
    RCLCPP_INFO(get_logger(), "입구 가운데까지 %.1fcm 더 직진 (%.1f초)", extra_m * 100.0, extra_time_);
  }

  // ==================== (2-1) 곡선으로 들어가기: 곡선 시작점까지 직진 ====================

  // 왼쪽 아래 노란선이 끊긴 순간 = 바퀴 축이 입구 가운데보다 약 entry_gap_to_center_m 앞.
  // 반지름 entry_arc_radius_m 인 곡선의 중심이 입구 가운데 줄 위에 오려면, 입구 가운데보다 반지름만큼 앞에서 꺾기 시작
  //   -> 더 갈 거리 = entry_gap_to_center_m - entry_arc_radius_m + entry_forward_m (음수면 바로 꺾음)
  void start_entry_straight()
  {
    const double latency_m = speed_ * std::max(0.0, entry_latency_s_);   // 알아채는 동안 이미 간 거리
    const double extra_m = std::max(0.0, entry_gap_to_center_m_ - entry_arc_radius_m_ + entry_forward_m_ - latency_m -
      curve_ramp_shift(entry_radius_cmd()));   // 서서히 꺾는 만큼 일찍
    extra_time_ = extra_m / speed_;
    state_ = State::EXTRA;
    state_time_ = 0.0;
    RCLCPP_INFO(get_logger(), "입구 시작 (노란선 끊김) -> %.1fcm 더 직진한 뒤 곡선으로 왼쪽 (지연 보정 -%.1fcm)",
      extra_m * 100.0, latency_m * 100.0);
    if (extra_time_ <= 0.0)
    {
      start_turn_or_arc();
    }
  }

  // EXTRA 가 끝나면: entry_arc 면 곡선, 아니면 제자리 회전
  void start_turn_or_arc()
  {
    if (entry_arc_ || leaving_)
    {
      start_entry_arc();
      return;
    }
    start_turn();
  }

  // ==================== (3-1) ENTRY_ARC: 곡선으로 왼쪽으로 들어가기 ====================

  // 앞 속도 entry_arc_speed, 회전 = 속도 / (반지름 x entry_arc_radius_gain)
  //   기본: 반지름 8cm, gain 1.0 -> 노란선이 끊기고 25cm 직진한 뒤 확 꺾는다 (실제 시험에서 꺾는 시기가 좋았던 곳)
  void start_entry_arc()
  {
    state_ = State::ENTRY_ARC;
    state_time_ = 0.0;
    // 입구는 확 꺾어도 되니 나가는 곡선의 최소 반지름 (12cm, 두 바퀴 다 앞으로) 대신 entry_arc_min_radius_m 만 지킨다
    //   (반지름이 바퀴 간격 절반 9.5cm 보다 작으면 안쪽 바퀴는 살짝 뒤로 돈다)
    const double radius_cmd = leaving_ ? leave_radius_cmd() :
      std::max(entry_arc_radius_m_ * entry_arc_radius_gain_, entry_arc_min_radius_m_);
    entry_v_ = entry_arc_speed_;
    if (((!leaving_ && entry_turn_match_) || (leaving_ && leave_turn_match_)) && run_speed_ > entry_v_)
    {
      // 주차장으로 들어가는 곡선: 일반 주행이 코너에서 도는 빠르기 (entry_turn_w) 로. 반지름(길)은 그대로라 직진 속도도 같이 오른다
      entry_v_ = std::max(entry_v_, std::min(run_speed_, entry_turn_w_ * radius_cmd));
    }
    entry_w_ = entry_v_ / radius_cmd;
    entry_v_max_ = entry_v_;
    entry_arc_time_full_ = (M_PI / 2.0) / entry_w_;   // 명령대로 돈다면 90도 걸리는 시간
    entry_arc_limit_ = entry_arc_time_full_ * 2.5;     // 너무 오래 걸리면 포기
    entry_prev_angles_.clear();
    entry_cam_sum_ = 0.0;
    entry_cam_lost_ = 0;
    entry_cam_failed_ = false;
    leave_guide_active_ = false;
    leave_last_lean_ = -M_PI / 2.0;
    RCLCPP_INFO(get_logger(), "곡선으로 왼쪽 (반지름 %.0fcm, 명령 반지름 %.0fcm, 속도 %.2fm/s)",
      entry_arc_radius_m_ * 100.0, radius_cmd * 100.0, entry_v_);
    drive(entry_v_, entry_w_);
  }

  // 영상이 올 때마다 (ENTRY_ARC):
  //   (1) 화면 직선들이 돈 각도를 더한다 (SIDE_TURN 과 같은 방법). 처음엔 도로의 노란선이 세로로 보여서
  //       길의 선으로 착각하지 않게, entry_check_deg (45도) 넘게 돈 뒤부터 (2) 를 본다
  //       (직선을 계속 못 찾으면: 명령대로 45도 돌 시간이 지난 뒤부터)
  //   (2) 길의 노란선(화면 위쪽 turn_fit_rows 줄)이 세로 (aligned_deg 이내) 가 되면 -> STEM
  //       조금 지나쳤어도 (exit_overshoot_deg 이내) STEM 으로 (올라가며 바로잡음)
  void entry_arc_step(const cv::Mat & yellow, const cv::Mat & lines_mask)
  {
    const std::vector<double> now_angles = scene_line_angles(lines_mask);
    const std::optional<double> step = rotation_between(entry_prev_angles_, now_angles);
    if (step)
    {
      entry_cam_sum_ += *step;
      entry_cam_lost_ = 0;
    }
    else if (!entry_prev_angles_.empty() && ++entry_cam_lost_ >= side_cam_lost_frames_ && !entry_cam_failed_)
    {
      entry_cam_failed_ = true;
      RCLCPP_WARN(get_logger(), "곡선 중 직선을 놓침 (%.1f도) -> 시간으로 어림해서 노란선 보기 시작",
        std::abs(entry_cam_sum_) * 180.0 / M_PI);
    }
    if (!now_angles.empty())
    {
      entry_prev_angles_ = now_angles;
    }

    // 입구로 들어가는 중: 길의 노란선 두 개가 짝으로 보이면 (덜 돌았어도) 바로 멈추고 두 선 가운데로 올라간다
    //   (entry_pair_stop. 선이 entry_pair_max_lean_deg 보다 많이 기울어 있으면 아직 너무 비스듬해서 더 돈다)
    const double cam_deg = std::abs(entry_cam_sum_) * 180.0 / M_PI;
    // 빠르게 도는 중이면 entry_turn_slow_deg 넘게 돈 뒤 원래 빠르기로 (반지름은 그대로). 끝까지 빠르면 더 돌아버림
    if (!leaving_ && entry_v_ > entry_arc_speed_ && (entry_cam_failed_ || cam_deg >= entry_turn_slow_deg_))
    {
      entry_w_ *= entry_arc_speed_ / entry_v_;
      entry_v_ = entry_arc_speed_;
      RCLCPP_INFO(get_logger(), "입구 곡선 %.0f도 돔 -> 원래 빠르기 (%.2fm/s, %.2frad/s)", cam_deg, entry_v_, entry_w_);
      drive(entry_v_, entry_w_);
    }
    if (!leaving_ && entry_pair_stop_ && (entry_cam_failed_ || cam_deg >= entry_pair_check_deg_))
    {
      const std::optional<double> pair_lean = yellow_pair_lean(yellow, static_cast<int>(turn_fit_rows_));
      if (pair_lean && std::abs(*pair_lean) <= entry_pair_max_lean_deg_ * M_PI / 180.0)
      {
        RCLCPP_INFO(get_logger(), "곡선 중 길의 노란선 두 개가 보임 (기울기 %.1f도, 카메라 각도 %.1f도) -> 바로 올라감",
          *pair_lean * 180.0 / M_PI, cam_deg);
        end_curve();
        start_stem(*pair_lean);
        return;
      }
    }

    const bool turned_enough = entry_cam_failed_ ?
      state_time_ >= entry_arc_time_full_ * 0.5 :
      std::abs(entry_cam_sum_) >= entry_check_deg_ * M_PI / 180.0;
    if (turned_enough)
    {
      const std::optional<double> lean = yellow_lean(yellow, static_cast<int>(turn_fit_rows_));
      // 왼쪽으로 돌고 있으니 길의 선은 왼쪽으로 기울어 있다가 (+) 세로 (0) 가 된다
      if (lean && *lean <= aligned_deg_ * M_PI / 180.0 && *lean >= -exit_overshoot_deg_ * M_PI / 180.0)
      {
        RCLCPP_INFO(get_logger(), "곡선 중 %s의 노란선이 세로가 됨 (카메라 각도 %.1f도)", leaving_ ? "도로" : "길",
          std::abs(entry_cam_sum_) * 180.0 / M_PI);
        end_curve();
        if (leaving_)
        {
          finish_leave();
          return;
        }
        start_stem(*lean);
        return;
      }
    }
    drive(entry_v_, entry_w_);
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
    slow_logged_ = false;
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
  //   side_arc 면 먼저 반지름만큼 더 올라간 뒤 (SIDE_FORWARD) 뒤로 곡선
  void turn_toward_seen_side()
  {
    const double toward = side_turn_toward_ ? 1.0 : -1.0;
    if (side_arc_)
    {
      start_side_forward(side_seen_ * toward);
      return;
    }
    start_side_turn(side_seen_ * toward);
  }

  // 곡선 전에 side_arc_forward_m 만큼 더 올라간다 (기본 0 = 가로 흰 선이 사라지면 바로 곡선).
  //   잘못 넣어도 위험하지 않게 0 ~ side_arc_forward_max_m (30cm) 로 자른다
  //   (뒤로 곡선을 그리면 로봇이 반지름만큼 통로 아래쪽으로도 내려온다. 칸 안에서 너무 아래에 서면 이 값을 조금 키운다)
  void start_side_forward(double direction)
  {
    side_direction_ = direction;
    speed_ = drive_speed_;       // 칸으로 들어가는 동안은 원래 속도 (흰 선 뒤 전진 거리가 맞춰 둔 그대로 되게)
    double extra_m = std::clamp(side_arc_forward_m_, 0.0, side_arc_forward_max_m_);
    if (extra_m != side_arc_forward_m_)
    {
      RCLCPP_WARN(get_logger(), "side_arc_forward_m %.2fm 는 범위 밖 -> %.2fm 로", side_arc_forward_m_, extra_m);
    }
    // side_arc_forward_time 이 있으면 시간으로 (최대 2초), 없으면 거리 / 속도
    const double extra_time = std::clamp(side_arc_forward_time_, 0.0, 2.0);
    if (extra_time > 0.0)
    {
      extra_m = extra_time * speed_;
    }
    if (extra_m <= 0.0)
    {
      side_forward_time_ = 0.0;
      end_side_forward();        // 더 안 올라가고 바로 뒤로 곡선
      return;
    }
    side_forward_time_ = extra_time > 0.0 ? extra_time : extra_m / speed_;
    if (smooth_ && smooth_accel_ > 0.0)
    {
      side_forward_time_ = std::max(0.0, side_forward_time_ - speed_ / (2.0 * smooth_accel_));   // 서서히 멈추며 더 가는 만큼
    }
    state_ = State::SIDE_FORWARD;
    state_time_ = 0.0;
    lines_missing_ = 0;
    RCLCPP_INFO(get_logger(), "%.1fcm 더 올라간 뒤 뒤로 가면서 %s으로 곡선", extra_m * 100.0,
      direction < 0.0 ? "오른쪽" : "왼쪽");
    if (side_forward_time_ <= 0.0)
    {
      end_side_forward();
    }
  }

  void end_side_forward()
  {
    side_prev_angles_ = last_scene_angles_;   // 돌기 시작할 때의 직선 방향 분포 (카메라로 각도 잴 기준)
    start_side_turn(side_direction_);
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
    // 이번에 돌 각도: 오른쪽이면 side_turn_deg_right, 왼쪽이면 side_turn_deg_left (0 보다 작으면 side_turn_deg)
    const double side_deg = direction < 0.0 ? side_turn_deg_right_ : side_turn_deg_left_;
    side_turn_deg_ = side_deg >= 0.0 ? side_deg : side_turn_base_deg_;
    // 명령 반지름 = 반지름 x gain. 잘못 넣어도 위험하지 않게 5cm ~ 50cm 로 자른다
    side_arc_radius_cmd_ = std::clamp(side_arc_radius_m_ * side_arc_radius_gain_, 0.05, 0.50);
    if (side_arc_ && side_arc_radius_cmd_ != side_arc_radius_m_ * side_arc_radius_gain_)
    {
      RCLCPP_WARN(get_logger(), "곡선 명령 반지름 %.2fm 는 범위 밖 -> %.2fm 로 (side_arc_radius_m, side_arc_radius_gain 확인)",
        side_arc_radius_m_ * side_arc_radius_gain_, side_arc_radius_cmd_);
    }
    // 가장 빠를 때 / 가장 느릴 때 회전 속도 [rad/s] (곡선이면 뒤 속도 / 반지름)
    const double w_max = side_arc_ ? side_arc_speed_ / side_arc_radius_cmd_ : side_turn_speed_;
    const double w_min = w_max * side_min_speed_ / side_turn_speed_;
    side_turn_time_ = (side_turn_deg_ * M_PI / 180.0) / w_max;          // odom 도 카메라도 없을 때
    side_turn_limit_ = (side_turn_deg_ * M_PI / 180.0) / w_min * 1.5;    // 너무 오래 걸리면 포기
    side_rough_ = 0.0;
    side_w_cmd_ = 0.0;
    side_gap_est_ = 0.0;
    side_gap_other_ = 0.0;
    side_last_yaw_ = odom_yaw_;
    // 시작할 때의 격자 기울기 (길을 올라올 때 본 마지막 화면). 로봇이 격자에서 조금 틀어져 있으면 그만큼에서 시작
    {
      const std::optional<double> tilt0 = grid_tilt(side_prev_angles_);
      side_phi_ = tilt0 ? side_direction_ * *tilt0 : 0.0;
    }
    left_hits_ = 0;
    right_hits_ = 0;
    side_use_odom_ = odom_received_ && (now() - last_odom_time_).seconds() < odom_timeout_;
    side_start_yaw_ = odom_yaw_;
    side_turned_ = 0.0;
    // 가로 흰 선을 보고 돌기 시작했으면: 화면의 직선들이 돈 각도로 잰다 (카메라)
    //   (뒤로 곡선이면 시작할 때 화면에 직선이 없을 수 있다 -> 보일 때까지는 시간으로 어림하고 카메라로 이어서 잰다)
    //   side_turn_fixed 면 카메라 / 격자는 안 보고 odom (없으면 시간) 으로 side_turn_deg 만큼만 돈다
    side_use_cam_ = !side_turn_fixed_ && side_cam_ && (side_arc_ || !side_prev_angles_.empty());
    side_cam_lost_ = side_prev_angles_.empty() ? 1 : 0;
    side_grid_seen_ = false;
    side_cam_turned_ = 0.0;
    side_cam_sum_ = 0.0;
    const char * how = side_use_cam_ ? "카메라 (화면 직선들이 돈 각도) 로 잼" :
      (side_use_odom_ ? "odom 으로 각도 잼 (무조건 이 각도)" : "odom 없음 -> 시간으로 (무조건 이 각도)");
    if (side_arc_)
    {
      RCLCPP_INFO(get_logger(), "뒤로 가면서 %s으로 %.0f도 곡선 시작 (반지름 %.0fcm, 명령 반지름 %.0fcm, 뒤 속도 %.2fm/s, %s)",
        direction < 0.0 ? "오른쪽" : "왼쪽", side_turn_deg_, side_arc_radius_m_ * 100.0,
        side_arc_radius_cmd_ * 100.0, side_arc_speed_, how);
    }
    else
    {
      RCLCPP_INFO(get_logger(), "%s으로 %.0f도 회전 시작 (%s)", direction < 0.0 ? "오른쪽" : "왼쪽",
        side_turn_deg_, how);
    }
    side_drive(side_turn_deg_ * M_PI / 180.0);
  }

  // SIDE_TURN 바퀴 명령. remain: 남은 각도 [rad] (작을수록 느려짐)
  //   제자리: 회전 속도 side_speed_for(remain)
  //   곡선  : 같은 비율로 뒤 속도를 줄이고, 회전 = 뒤 속도 / 명령 반지름 (반지름은 그대로)
  void side_drive(double remain)
  {
    const double w = side_speed_for(remain);
    if (!side_arc_)
    {
      side_w_cmd_ = w;
      drive(0.0, side_direction_ * w);
      return;
    }
    const double v = side_arc_speed_ * w / side_turn_speed_;
    side_w_cmd_ = v / side_arc_radius_cmd_;
    drive(-v, side_direction_ * side_w_cmd_);
  }

  // 영상이 올 때마다 (SIDE_TURN + 카메라 사용 중): 격자 각도를 끊김 없이 이어서 센다
  //   주차장 선들은 서로 직각(격자)이라 카메라로는 "선들이 똑바로에서 몇 도 기울었나" (-45 ~ +45도) 만 알 수 있다.
  //   (0도와 90도는 화면이 똑같다)
  //   -> 영상마다 기울기를 재서 이어 붙인다: 44도 다음에 -43도가 나오면 격자 한 칸을 넘은 것 -> 47도.
  //      영상 사이에 45도 넘게 돌 일은 없어서 이렇게 이으면 오차가 쌓이지 않는다 (side_phi_ = 격자 기준 돈 각도)
  //   격자가 안 보이는 동안은 대충 더해 두고 (side_gap_est_):
  //     odom (x side_odom_scale) -> odom 이 없으면 바로 전 영상과 비교한 회전량 -> 그것도 없으면 시간
  //   다시 보이면 "대충 센 값에 가장 가까운 격자 각도" 로 바로잡는다 -> 안 보이는 구간에서 45도 넘게 틀리지 않으면 정확
  //   격자 기준 90도 (side_tolerance_deg 이내) 가 되면 멈춘다
  //   격자를 한 번도 못 보면: 대충 센 각도가 side_turn_deg 가 되면 멈춘다
  void side_turn_by_camera(const cv::Mat & lines_mask)
  {
    const double deg = M_PI / 180.0;
    const std::vector<double> now_angles = scene_line_angles(lines_mask);
    std::optional<double> tilt;
    if (side_snap_)
    {
      tilt = grid_tilt(now_angles);
    }

    // 격자가 안 보일 때 대충 더하기 (side_gap_est_):
    //   odom 이 있으면 odom (x side_odom_scale). 크기는 조금 틀려도 가만히 있는데 늘어나는 일은 없다
    //   odom 이 없으면 바로 전 영상과 비교한 회전량 (도는 방향 부호를 지켜서. 흔들림이 +/- 로 상쇄되게),
    //   직선도 없으면 "명령 회전 속도 x 시간"
    const std::optional<double> step = rotation_between(side_prev_angles_, now_angles);
    if (side_use_odom_ || !step)
    {
      side_gap_est_ += side_gap_other_;
    }
    else
    {
      side_gap_est_ += side_direction_ * *step;   // 왼쪽으로 돌면 +, 오른쪽이면 - 로 나온다
    }
    side_gap_other_ = 0.0;
    if (!now_angles.empty())
    {
      side_prev_angles_ = now_angles;
    }
    const double predicted = side_phi_ + side_gap_est_;   // 지난번 격자 각도 + 그 뒤 대충 돈 각도

    if (tilt)
    {
      // 도는 방향으로 본 기울기 (-45 ~ +45도). 실제로 돈 각도 = 이 값 + 90도 x k
      const double phi = side_direction_ * *tilt;
      const double k = std::round((predicted - phi) / (90.0 * deg));
      const double phi_now = phi + k * 90.0 * deg;
      if (!side_grid_seen_ && side_gap_est_ > 30.0 * deg)
      {
        RCLCPP_INFO(get_logger(), "격자 보임: 대충 %.1f도 -> 격자로 %.1f도", predicted / deg, phi_now / deg);
      }
      side_phi_ = phi_now;
      side_gap_est_ = 0.0;
      side_grid_seen_ = true;
      side_snap_tilt_ = *tilt;
    }
    side_cam_lost_ = tilt ? 0 : side_cam_lost_ + 1;
    const double turned = tilt ? side_phi_ : predicted;
    side_rough_ = turned;
    side_cam_turned_ = turned;

    // 목표: 격자를 봤으면 격자 기준 90도 (다시 똑바로), 한 번도 못 봤으면 side_turn_deg
    const double target = side_turn_deg_ * deg;    
    const double remain = target - turned;
    if (remain <= side_tolerance_deg_ * deg)
    {
      if (!tilt)
      {
        RCLCPP_WARN(get_logger(), "격자가 안 보이는 채로 대충 %.1f도 -> 멈춤", turned / deg);
      }
      end_side_turn();
      return;
    }
    side_drive(remain);
  }

  // 화면 선들이 가로/세로에서 얼마나 틀어져 있는지 [rad] (-45도 ~ +45도). 직각 격자가 안 보이면 비어 있음
  //   주차장의 선들은 서로 직각이라, 로봇이 격자에 맞게 서 있으면 직선 방향 분포가 0도(가로)와 90도(세로)에 몰린다.
  //   0 ~ 89도 중 "그 각도 근처 + 그 각도 + 90도 근처" 의 길이 합이 가장 큰 각도를 찾는다 (소수점까지 다듬음)
  //   그 합이 전체의 side_snap_min_ratio 보다 작으면 격자가 아닌 걸로 본다
  std::optional<double> grid_tilt(const std::vector<double> & hist) const
  {
    if (hist.size() != 180)
    {
      return std::nullopt;
    }
    double total = 0.0;
    for (const double h : hist)
    {
      total += h;
    }
    // 분포를 ±2칸 번지게 만들었으니 점수도 ±2칸을 합쳐서 본다
    std::vector<double> score(90, 0.0);
    for (int k = 0; k < 90; k++)
    {
      for (int j = -2; j <= 2; j++)
      {
        score[k] += hist[(k + j + 180) % 180] + hist[(k + 90 + j + 180) % 180];
      }
    }
    const int best = static_cast<int>(std::max_element(score.begin(), score.end()) - score.begin());
    if (total <= 0.0 || score[best] < total * side_snap_min_ratio_)
    {
      return std::nullopt;
    }
    // 꼭짓점 양옆 점수로 소수점 위치 (90도마다 반복되니 양 끝은 이어서 본다)
    const double l = score[(best + 89) % 90];
    const double c = score[best];
    const double r = score[(best + 1) % 90];
    double deg = best;
    const double denom = l - 2.0 * c + r;
    if (std::abs(denom) > 1e-9)
    {
      deg += 0.5 * (l - r) / denom;
    }
    if (deg >= 45.0)
    {
      deg -= 90.0;               // 0 ~ 90 -> -45 ~ 45
    }
    return deg * M_PI / 180.0;
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
  //   side_odom_scale: odom 이 실제보다 작게 / 크게 잴 때 곱하는 배율 (실제 각도 = odom 각도 x 배율)
  void step_side_turn_odom()
  {
    side_turned_ = std::abs(wrap_angle(odom_yaw_ - side_start_yaw_)) * side_odom_scale_;
    const double remain = side_turn_deg_ * M_PI / 180.0 - side_turned_;
    if (remain <= side_tolerance_deg_ * M_PI / 180.0)
    {
      end_side_turn();
      return;
    }
    // 남은 각도에 비례해서 느려진다 (지나치지 않게). 너무 느리면 안 돌아서 최소 속도
    side_drive(remain);
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
        return;
      }
      if (!side_use_odom_)
      {
        side_gap_other_ += std::abs(side_w_cmd_) * 0.1;   // odom 이 없으면: 명령한 회전 속도 x 0.1초 로 대충 더함
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
    side_drive(side_turn_deg_ * M_PI / 180.0);   // 시간으로 돌 때는 늘 최고 속도
  }

  void end_side_turn()
  {
    stop_wheels();
    const bool right = side_direction_ < 0.0;
    if (side_use_cam_ && side_grid_seen_)
    {
      RCLCPP_INFO(get_logger(), "%s으로 격자 기준 %.1f도 돌았음 (남은 틀어짐 %.1f도, odom x %.2f = %.1f도)",
        right ? "오른쪽" : "왼쪽", side_cam_turned_ * 180.0 / M_PI, side_snap_tilt_ * 180.0 / M_PI,
        side_odom_scale_, side_turned_ * 180.0 / M_PI);
    }
    else if (side_use_cam_)
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
  //   side_arc (뒤로 곡선) 로 칸에 들어왔으면 side_arc_backup_time 초만 (기본 0 = 후진 안 함)
  void start_backup()
  {
    backup_duration_ = side_arc_ ? side_arc_backup_time_ : backup_time_;
    if (backup_duration_ <= 0.0)
    {
      finish_backup();
      return;
    }
    state_ = State::BACKUP;
    state_time_ = 0.0;
    dash_time_ = -1.0;           // 이번 후진에서 본 점선만 쓴다
    RCLCPP_INFO(get_logger(), "%.1f초 후진%s", backup_duration_, dash_align_ ? " (흰 점선이 보이면 가로가 되게 돌면서)" : "");
    drive(-backup_speed_, 0.0);
  }

  // 0.1초마다: 시간이 될 때까지 후진, 다 되면 멈춤
  void step_backup()
  {
    if (state_time_ >= backup_duration_)
    {
      finish_backup();
      return;
    }
    drive(-backup_speed_, dash_turn());
  }

  // 후진 중 회전 명령: 점선이 최근 (0.3초 안) 보였으면 기울기에 비례해서, 아니면 0 (똑바로)
  //   점선이 화면에서 a 만큼 기울었으면 (y 아래 방향 기준, + = 오른쪽이 아래) 로봇이 -a 만큼 돌아야 가로가 된다
  double dash_turn()
  {
    if (!dash_align_ || !dash_fresh(0.3))
    {
      return 0.0;
    }
    const double w = std::clamp(-dash_gain_ * dash_angle_, -dash_max_w_, dash_max_w_);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "흰 점선 기울기 %.1f도 -> 돌면서 후진 (%.2f rad/s)",
      dash_angle_ * 180.0 / M_PI, w);
    return w;
  }

  bool dash_fresh(double within_s)
  {
    return dash_time_ >= 0.0 && now().seconds() - dash_time_ <= within_s;
  }

  // 흰 마스크 (BEV) 에서 점선 한 조각을 찾아 기울기 / 거리를 기억한다
  void update_dash(const cv::Mat & white)
  {
    const std::optional<cv::Vec2d> dash = find_dash(white);
    if (!dash)
    {
      return;
    }
    dash_angle_ = (*dash)[0];
    dash_dist_ = bev_near_m_ + (white.rows - (*dash)[1]) * 0.001;   // 1px = 1mm
    dash_time_ = now().seconds();
  }

  // 점선 조각 찾기: 길이 dash_min_len_px ~ dash_max_len_px, 두께 dash_max_width_px 이하, 가로에서 dash_max_deg 이내인
  //   흰 덩어리 중 로봇에 가장 가까운 것 (화면 아래). 돌려주는 값: (기울기 [rad, + = 오른쪽이 아래], 가운데 y [px])
  //   (칸 테두리 실선은 길어서 빠지고, 실선 끝에 붙은 짧은 선도 실선과 한 덩어리라 빠진다)
  std::optional<cv::Vec2d> find_dash(const cv::Mat & white) const
  {
    cv::Mat labels, stats, centroids;
    const int n = cv::connectedComponentsWithStats(white, labels, stats, centroids, 8, CV_32S);
    std::optional<cv::Vec2d> best;
    for (int i = 1; i < n; ++i)
    {
      if (stats.at<int>(i, cv::CC_STAT_AREA) < 150)
      {
        continue;
      }
      std::vector<cv::Point> pts;
      cv::findNonZero(labels == i, pts);
      const cv::RotatedRect box = cv::minAreaRect(pts);
      const double len = std::max(box.size.width, box.size.height);
      const double wid = std::min(box.size.width, box.size.height);
      if (len < dash_min_len_px_ || len > dash_max_len_px_ || wid > dash_max_width_px_ || len < 2.5 * wid)
      {
        continue;
      }
      cv::Vec4f fit;
      cv::fitLine(pts, fit, cv::DIST_L2, 0, 0.01, 0.01);
      double ang = std::atan2(fit[1], fit[0]);
      if (ang > M_PI / 2.0)
      {
        ang -= M_PI;
      }
      if (ang <= -M_PI / 2.0)
      {
        ang += M_PI;
      }
      if (std::abs(ang) > dash_max_deg_ * M_PI / 180.0)
      {
        continue;
      }
      const double cy = box.center.y;
      if (!best || cy > (*best)[1])
      {
        best = cv::Vec2d(ang, cy);
      }
    }
    return best;
  }

  void finish_backup()
  {
    stop_wheels();
    if (dash_align_ && dash_fresh(0.5))
    {
      RCLCPP_INFO(get_logger(), "후진 끝: 흰 점선 기울기 %.1f도, 점선까지 %.0fcm", dash_angle_ * 180.0 / M_PI,
        dash_dist_ * 100.0);
    }
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
    // 흰 점선을 방금 봤으면: 통로 가운데까지 거리 (점선까지 - 통로 폭 / 2) 를 실제 반지름으로
    // 이번에 쓸 비율: 오른쪽으로 나가면 exit_dash_gain_right, 왼쪽이면 _left (0 보다 작으면 exit_dash_gain)
    const double side_gain = side_direction_ < 0.0 ? exit_dash_gain_right_ : exit_dash_gain_left_;
    const double gain = side_gain >= 0.0 ? side_gain : exit_dash_gain_;
    if (dash_align_ && gain > 0.0 && dash_fresh(1.0))
    {
      const double real = dash_dist_ - aisle_width_m_ / 2.0;
      const double cmd = std::clamp(real * gain, exit_arc_min_radius_m_, 0.6);
      RCLCPP_INFO(get_logger(), "흰 점선까지 %.0fcm -> 통로 가운데까지 %.0fcm -> 명령 반지름 %.0fcm",
        dash_dist_ * 100.0, real * 100.0, cmd * 100.0);
      return cmd;
    }
    double radius = exit_arc_radius_m_;
    if (radius <= 0.0)
    {
      // 통로 가운데에서 칸 안으로 들어간 거리: 뒤로 곡선이면 곡선 반지름 + 더 후진한 거리, 아니면 후진한 거리
      const double into_slot = side_arc_ ?
        side_arc_radius_m_ + backup_speed_ * std::max(0.0, side_arc_backup_time_) :
        backup_speed_ * std::max(0.0, backup_time_);
      radius = into_slot * exit_arc_radius_gain_;
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
    exit_guide_active_ = false;
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
  // 보면서 나가기 (EXIT_ARC, 영상마다). 안내선을 못 찾으면 false (원래 방식으로)
  //   안내선 = 나가는 쪽 반대편에 보이는 노란선 (왼쪽으로 나가면 오른쪽, 오른쪽으로 나가면 왼쪽)
  bool exit_arc_by_guide(const cv::Mat & yellow)
  {
    const std::optional<cv::Vec2d> g = find_exit_guide(yellow, side_direction_);
    if (!g)
    {
      if (exit_guide_active_)
      {
        drive(exit_arc_speed_, exit_w_);   // 잠깐 놓치면 마지막 곡선 그대로
        return true;
      }
      return false;
    }
    const double dist = (*g)[0];
    const double lean = (*g)[1];
    const double phi = std::abs(lean);
    // 나란해졌거나 지나쳤으면 (나가는 방향 x 기울기 > 0) 길 내려가기로
    if (phi <= exit_guide_align_deg_ * M_PI / 180.0 || side_direction_ * lean > 0.0)
    {
      RCLCPP_INFO(get_logger(), "안내 노란선과 나란해짐 (선까지 %.0fcm, 기울기 %.1f도) -> 길 내려가기",
        dist * 100.0, lean * 180.0 / M_PI);
      start_exit_stem();
      return true;
    }
    const double need = dist - exit_guide_offset_m_;
    double radius = exit_arc_min_radius_m_;
    if (need > 0.0)
    {
      radius = std::clamp(need / (1.0 - std::cos(phi)), exit_arc_min_radius_m_, exit_guide_max_radius_m_);
    }
    if (!exit_guide_active_)
    {
      RCLCPP_INFO(get_logger(), "안내 노란선 보임 (선까지 %.0fcm, 기울기 %.0f도) -> 보면서 나가기 (반지름 %.0fcm)",
        dist * 100.0, lean * 180.0 / M_PI, radius * 100.0);
    }
    exit_guide_active_ = true;
    exit_w_ = side_direction_ * exit_arc_speed_ / radius;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "보면서 나가기: 선까지 %.0fcm, 기울기 %.0f도 -> 반지름 %.0fcm",
      dist * 100.0, lean * 180.0 / M_PI, radius * 100.0);
    drive(exit_arc_speed_, exit_w_);
    return true;
  }

  // 안내 노란선 찾기 (BEV, 1px = 1mm). 돌려주는 값: (바퀴 축에서 선까지 수직 거리 [m], 선이 기운 각 [rad, + = 앞쪽이 오른쪽])
  //   노란 덩어리마다 아래쪽 150줄로 직선 x = a + b*y 를 맞추고, 나가는 반대쪽에 있는 것 중 가장 가까운 것
  std::optional<cv::Vec2d> find_exit_guide(const cv::Mat & yellow, double direction, double max_rms_px = 0.0) const
  {
    cv::Mat labels, stats, centroids;
    const int n = cv::connectedComponentsWithStats(yellow, labels, stats, centroids, 8, CV_32S);
    const double axle_y = yellow.rows + bev_near_m_ * 1000.0;   // 바퀴 축의 화면 y (화면 아래 밖)
    const double cx = bev_center_x_ * yellow.cols;
    std::optional<cv::Vec2d> best;
    for (int i = 1; i < n; ++i)
    {
      if (stats.at<int>(i, cv::CC_STAT_AREA) < 300)
      {
        continue;
      }
      std::vector<cv::Point> pts;
      cv::findNonZero(labels == i, pts);
      int y_max = 0;
      for (const auto & p : pts)
      {
        y_max = std::max(y_max, p.y);
      }
      // 최소제곱 직선 x = a + b*y (아래쪽 150줄만: 위쪽은 길이 휘는 곳일 수 있음)
      double sn = 0, sy = 0, sx = 0, syy = 0, sxy = 0;
      for (const auto & p : pts)
      {
        if (p.y < y_max - 150)
        {
          continue;
        }
        sn += 1; sy += p.y; sx += p.x; syy += double(p.y) * p.y; sxy += double(p.x) * p.y;
      }
      const double den = sn * syy - sy * sy;
      if (sn < 50 || std::abs(den) < 1e-6)
      {
        continue;
      }
      const double b = (sn * sxy - sy * sx) / den;
      const double a = (sx - b * sy) / sn;
      const double lean = std::atan(-b);                       // 앞으로 갈수록 x 가 커지면 +
      if (max_rms_px > 0.0)
      {
        // 선이 아닌 덩어리 (도로 옆 흰 상자 등) 는 뺀다: 직선에서 벗어난 정도 (= 두께) 가 크면
        double se = 0.0;
        for (const auto & p : pts)
        {
          if (p.y >= y_max - 150)
          {
            const double e = p.x - (a + b * p.y);
            se += e * e;
          }
        }
        if (std::sqrt(se / sn) * std::cos(lean) > max_rms_px)
        {
          continue;
        }
      }
      if (std::abs(lean) > 70.0 * M_PI / 180.0)
      {
        continue;
      }
      const double x_axle = (a + b * axle_y - cx) * 0.001;     // 바퀴 축 줄에서 선의 가로 위치 [m] (+ 오른쪽)
      if (x_axle * direction < 0.0)                            // 왼쪽으로 나가면 (+1) 선은 오른쪽 (+)
      {
        continue;
      }
      const double dist = std::abs(x_axle) * std::cos(lean);
      if (!best || dist < (*best)[0])
      {
        best = cv::Vec2d(dist, lean);
      }
    }
    return best;
  }

  void exit_arc_by_camera(const cv::Mat & yellow, const cv::Mat & lines_mask)
  {
    if (exit_guide_ && exit_arc_by_guide(yellow))
    {
      return;                    // 안내선을 보고 돌았음
    }
    exit_yellow_seen_ = false;
    if (state_time_ >= exit_arc_time_full_ * exit_check_ratio_)
    {
      const std::optional<double> lean = yellow_lean(yellow, yellow.rows);
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

  // 길의 노란선 두 개가 짝으로 보이면 그 평균 기울기 [rad] (+ 왼쪽으로 기움), 아니면 비어 있음
  //   왼쪽 칸 / 오른쪽 칸에 곧은 노란선이 하나씩 (exit_line_max_rms_px 이내), 간격이 길 폭 정도 (pair_gap_min/max_px),
  //   서로 나란함 (parallel_deg 이내). 화면 맨 위부터 y_end 줄까지만 본다 (아래쪽은 입구의 둥근 모서리)
  std::optional<double> yellow_pair_lean(const cv::Mat & yellow, int y_end)
  {
    const int mid = static_cast<int>(bev_center_x_ * yellow.cols);
    const std::optional<Line> left = fit_line(yellow, 0, mid, y_end);
    const std::optional<Line> right = fit_line(yellow, mid, yellow.cols, y_end);
    if (!left || !right || left->rms > exit_line_max_rms_px_ || right->rms > exit_line_max_rms_px_)
    {
      return std::nullopt;
    }
    const double bottom_y = yellow.rows - 1;
    const double gap = right->x_at(bottom_y) - left->x_at(bottom_y);
    if (gap < pair_gap_min_px_ || gap > pair_gap_max_px_ ||
      std::abs(left->lean_left() - right->lean_left()) > parallel_deg_ * M_PI / 180.0)
    {
      return std::nullopt;
    }
    return (left->lean_left() + right->lean_left()) / 2.0;
  }

  // 길의 노란선 기울기 [rad] (+ 왼쪽으로 기움). 화면 맨 위부터 y_end 줄까지만 본다. 안 보이면 비어 있음
  //   (EXIT_ARC: 들어왔던 길로 나갈 때, ENTRY_ARC: 곡선으로 들어갈 때)
  //   두 선이 길 폭만큼 떨어져 나란하면 둘의 평균, 아니면 보이는 한 선
  //   (곡선은 실제 반지름이 조금 달라 옆으로 밀릴 수 있어서 선의 자리는 따지지 않는다)
  //   점들이 직선에서 exit_line_max_rms_px 보다 많이 벗어난 선(두 노란선이 한쪽 칸에 같이 들어와 섞인 것)은 안 쓴다
  std::optional<double> yellow_lean(const cv::Mat & yellow, int y_end)
  {
    const int mid = static_cast<int>(bev_center_x_ * yellow.cols);
    std::optional<Line> left = fit_line(yellow, 0, mid, y_end);
    std::optional<Line> right = fit_line(yellow, mid, yellow.cols, y_end);
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
    if (exit_guide_active_)
    {
      return;                    // 안내선을 본 뒤로는 영상이 올 때마다 exit_arc_by_guide() 가 바퀴 명령을 보낸다
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
    if (smooth_)
    {
      end_curve();
    }
    speed_ = run_speed_;         // 칸에서 나왔으니 다시 일반 주행 속도
    state_time_ = 0.0;
    lines_missing_ = 0;
    exit_white_y_ = -1.0;
    leave_present_ = 0;
    leave_seen_ = false;
    leave_empty_ = 0;
    exit_travel_ = 0.0;
    slow_logged_ = false;
    RCLCPP_INFO(get_logger(), "들어왔던 길을 두 차선 가운데로 내려감 (%s)",
      leave_turn_ ? "왼쪽 노란선이 끊기면 왼쪽 곡선으로 도로에" : "도로 흰선이 보이면 멈춤");
  }

  // ==================== (9) 도로로 나가기: 들어갈 때처럼 곡선으로 왼쪽 -> LANE ====================

  // 왼쪽 아래 칸 (들어갈 때 입구를 찾던 칸과 같음) 의 노란선:
  //   연속 leave_yellow_frames 번 보임 (길의 왼쪽 노란선) -> 그 뒤 연속 gap_frames_needed 번 안 보임 = 길이 도로와 만남
  //   (길 위쪽은 흰 점선이라 처음부터 비어 있다 -> 노란선을 먼저 본 뒤에만 "끊김" 으로 친다)
  bool leave_gap_reached(const cv::Mat & yellow, bool allow_break)
  {
    const int y1 = std::min(static_cast<int>(watch_bottom_) + 1, yellow.rows);
    const int y0 = std::max(0, y1 - static_cast<int>(watch_rows_));
    const int x1 = std::min(static_cast<int>(watch_x_max_), yellow.cols);
    const int count = cv::countNonZero(yellow(cv::Range(y0, y1), cv::Range(0, x1)));
    if (!leave_seen_)
    {
      leave_present_ = count >= present_pixels_ ? leave_present_ + 1 : 0;
      if (leave_present_ >= leave_yellow_frames_)
      {
        leave_seen_ = true;
        RCLCPP_INFO(get_logger(), "길의 왼쪽 노란선 보임 (%d개) -> 끊기면 도로", count);
      }
      return false;
    }
    if (!allow_break)
    {
      leave_empty_ = 0;          // 곡선 직후 (leave_min) 에는 끊김을 세지 않음 ("보임" 은 미리 세 둠)
      return false;
    }
    leave_empty_ = count <= empty_pixels_ ? leave_empty_ + 1 : 0;
    if (leave_empty_ >= gap_frames_needed_)
    {
      RCLCPP_INFO(get_logger(), "왼쪽 노란선 끊김 (%d개) -> 도로", count);
      return true;
    }
    return false;
  }

  // 들어갈 때와 같은 계산: 더 갈 거리 = entry_gap_to_center_m - entry_arc_radius_m + entry_forward_m + leave_forward_m
  //   (길 폭과 도로 폭이 같고 모서리 모양도 같아서, 들어갈 때 맞춘 entry_forward_m 을 그대로 쓰고 leave_forward_m 으로 더 조절)
  void start_leave_straight()
  {
    leaving_ = true;
    const double extra_m = std::max(0.0,
      entry_gap_to_center_m_ - entry_arc_radius_m_ + entry_forward_m_ + leave_forward_m_ -
      std::max(0.0, speed_ - drive_speed_) * std::max(0.0, leave_latency_s_) -   // 빠른 만큼 늦게 알아채면 (기본 0)
      curve_ramp_shift(leave_radius_cmd()));   // 서서히 꺾는 만큼 일찍
    extra_time_ = extra_m / speed_;
    state_ = State::EXTRA;
    state_time_ = 0.0;
    RCLCPP_INFO(get_logger(), "%.1fcm 더 직진한 뒤 곡선으로 왼쪽 (도로로)", extra_m * 100.0);
    if (extra_time_ <= 0.0)
    {
      start_turn_or_arc();
    }
  }

  // 도로를 똑바로 봄 -> grid_drive 에게 넘긴다 (control_mode = LANE), 주차 노드는 다시 표지판 기다리기
  // 도로로 나가는 왼쪽 곡선 (ENTRY_ARC + leaving_, 영상마다): 도로 바깥 흰 선 (왼쪽으로 돌면 오른쪽에 보임) 까지
  //   거리 D 와 기운 각 phi 로 반지름 R = (D - leave_guide_offset_m) / (1 - cos phi) 를 매번 다시 계산한다
  //   (칸에서 나올 때 exit_arc_by_guide 와 같은 방법). 나란해지면 (또는 지나치면) 일반 주행으로 넘긴다
  bool leave_arc_by_guide(const cv::Mat & white)
  {
    const std::optional<cv::Vec2d> g = find_exit_guide(white, 1.0, leave_guide_max_rms_px_);
    if (!g)
    {
      if (leave_guide_active_)
      {
        drive(entry_v_, entry_w_);   // 잠깐 놓치면 마지막 곡선 그대로
        return true;
      }
      return false;
    }
    const double dist = (*g)[0];
    const double lean = (*g)[1];
    const double phi = std::abs(lean);
    // 왼쪽으로 돌고 있으니 선은 왼쪽으로 기울어 있다가 (-) 세로가 된다. + 면 지나침
    // 멀리 기울어 있다가 (-25도 아래) 갑자기 + (지나침) 로 바뀜 = 잘못 봄 -> 하던 곡선 그대로
    if (lean > 0.0 && leave_last_lean_ <= -25.0 * M_PI / 180.0)
    {
      drive(entry_v_, entry_w_);
      return true;
    }
    leave_last_lean_ = lean;
    if (phi <= leave_guide_align_deg_ * M_PI / 180.0 || lean > 0.0)
    {
      RCLCPP_INFO(get_logger(), "도로 흰 선과 나란해짐 (선까지 %.0fcm, 기울기 %.1f도)", dist * 100.0, lean * 180.0 / M_PI);
      end_curve();
      finish_leave();
      return true;
    }
    const double need = dist - leave_guide_offset_m_;
    double radius = leave_guide_min_radius_m_;
    if (need > 0.0)
    {
      radius = std::clamp(need / (1.0 - std::cos(phi)), leave_guide_min_radius_m_, exit_guide_max_radius_m_);
    }
    if (!leave_guide_active_)
    {
      RCLCPP_INFO(get_logger(), "도로 흰 선 보임 (선까지 %.0fcm, 기울기 %.0f도) -> 보면서 도로로 (반지름 %.0fcm)",
        dist * 100.0, lean * 180.0 / M_PI, radius * 100.0);
    }
    leave_guide_active_ = true;
    // 빠르게 나가는 중이어도 급한 곡선 (작은 반지름) 은 회전이 entry_turn_w 를 넘지 않게 (예전보다 빨리 돌지는 않음)
    entry_v_ = std::min(entry_v_max_, std::max(entry_arc_speed_, entry_turn_w_ * radius));
    entry_w_ = entry_v_ / radius;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "보면서 도로로: 선까지 %.0fcm, 기울기 %.0f도 -> 반지름 %.0fcm",
      dist * 100.0, lean * 180.0 / M_PI, radius * 100.0);
    drive(entry_v_, entry_w_);
    return true;
  }

  void finish_leave()
  {
    if (smooth_)
    {
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = v_out_;
      cmd_pub_->publish(cmd);    // 일반 주행이 이어받을 때까지 0 으로 서지 않게
      v_out_ = w_out_ = k_out_ = 0.0;
      tgt_v_ = tgt_w_ = 0.0;
    }
    leaving_ = false;
    state_ = State::WAITING;
    sign_hits_ = 0;
    rearm_until_s_ = now().seconds() + leave_rearm_delay_;
    handover_ticks_ = 5;         // 0.5초 동안 LANE 을 몇 번 더 보낸다 (못 받는 일 없게)
    publish_mode(ControlMode::LANE, "leave_done");
    RCLCPP_INFO(get_logger(), "도로로 나옴 -> 주차 끝, 일반 주행 (LANE). %.0f초 동안 주차 표지판 무시",
      leave_rearm_delay_);
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
    // 카메라로 도는 중: 격자가 안 보이는 동안 쓸 "대충 돈 각도" 에 odom 변화를 더한다
    side_turned_ = std::abs(wrap_angle(odom_yaw_ - side_start_yaw_)) * side_odom_scale_;
    side_gap_other_ += std::abs(wrap_angle(odom_yaw_ - side_last_yaw_)) * side_odom_scale_;
    side_last_yaw_ = odom_yaw_;
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
    if (handover_ticks_ > 0)
    {
      handover_ticks_--;
      publish_mode(ControlMode::LANE, "leave_done");
    }
    if (!i_am_driving())
    {
      return;
    }

    // grid_drive 가 멈춰 있도록 "주차 중" 을 계속 알린다
    publish_mode(ControlMode::PARKING, state_name());

    state_time_ += 0.1;
    if (state_ == State::EXIT_STEM)
    {
      exit_travel_ += 0.1 * speed_;
    }
    if (state_ == State::EXTRA && state_time_ >= extra_time_)
    {
      start_turn_or_arc();
      return;
    }
    if (state_ == State::ENTRY_ARC && state_time_ >= entry_arc_limit_)
    {
      give_up("입구 곡선이 너무 오래 걸림 (길의 노란선이 세로로 안 보임)");
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
    if (state_ == State::SIDE_FORWARD && state_time_ >= side_forward_time_)
    {
      end_side_forward();
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
      hard_stop();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "영상이 안 옴 -> 정지");
    }
  }

  // ==================== 도우미 ====================

  // 선이 화면 아래로 (가까이) 올수록 일반 주행 속도 (run_speed) -> drive_speed 로 줄인다. 다시 빨라지지는 않는다
  //   y: 선의 줄 번호 (-1 = 안 보임 -> 그대로). start_y 에서 줄이기 시작, end_y 에서 drive_speed
  void slow_by_row(double y, double start_y, double end_y, const char * what)
  {
    if (!slow_near_ || y < 0.0 || run_speed_ <= drive_speed_ || end_y <= start_y)
    {
      return;
    }
    const double r = std::clamp((y - start_y) / (end_y - start_y), 0.0, 1.0);
    const double target = run_speed_ + (drive_speed_ - run_speed_) * r;
    if (target < speed_)
    {
      speed_ = target;
      if (!slow_logged_)
      {
        slow_logged_ = true;
        RCLCPP_INFO(get_logger(), "%s 가까워짐 (y=%.0f) -> 천천히 (%.2f -> %.2f m/s)", what, y, run_speed_, drive_speed_);
      }
    }
  }

  // 왼쪽 칸 [0, watch_x_max) 에서 노란선이 아래 (watch_bottom) 부터 위로 이어지다 끝나는 줄 번호. 없으면 -1
  //   (길을 내려갈 때 길이 도로와 만나는 곳 = 노란선 끝이 화면 위에서 내려온다. park5 녹화로 확인: 0.1초에 약 25줄씩)
  double yellow_end_y(const cv::Mat & yellow) const
  {
    const int x1 = std::min(static_cast<int>(watch_x_max_), yellow.cols);
    double end = -1.0;
    int gap = 0;
    for (int row = std::min(static_cast<int>(watch_bottom_), yellow.rows - 1); row >= 0; row--)
    {
      if (cv::countNonZero(yellow.row(row).colRange(0, x1)) >= 3)
      {
        end = row;
        gap = 0;
      }
      else if (++gap > 15)
      {
        break;
      }
    }
    return end;
  }

  bool i_am_driving() const
  {
    return state_ == State::APPROACH || state_ == State::EXTRA || state_ == State::TURN ||
           state_ == State::ENTRY_ARC ||
           state_ == State::STEM || state_ == State::SIDE_FORWARD || state_ == State::SIDE_TURN ||
           state_ == State::BACKUP ||
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
        return leaving_ ? "leave_straight" : "extra";
      case State::TURN:
        return "turn";
      case State::ENTRY_ARC:
        return leaving_ ? "leave_arc" : "entry_arc";
      case State::STEM:
        return "stem";
      case State::SIDE_FORWARD:
        return "side_forward";
      case State::SIDE_TURN:
        if (side_arc_)
        {
          return side_direction_ < 0.0 ? "side_arc_right" : "side_arc_left";
        }
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
      hard_stop();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "앞이 막힘 -> 대기");
      return;
    }
    if (smooth_)
    {
      tgt_v_ = v;                // 실제로 보내는 건 smooth_step (0.02초마다)
      tgt_w_ = w;
      tgt_t_ = now().seconds();
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
    if (smooth_ && i_am_driving())
    {
      tgt_v_ = 0.0;              // 서서히 멈춤 (지금 곡선 그대로)
      tgt_w_ = 0.0;
      tgt_t_ = now().seconds();
      return;
    }
    hard_stop();
  }

  // 바로 정지 (정면 막힘 / 영상 끊김 / 그만둠)
  void hard_stop()
  {
    v_out_ = w_out_ = k_out_ = 0.0;
    tgt_v_ = tgt_w_ = 0.0;
    cmd_pub_->publish(geometry_msgs::msg::Twist());
  }

  // 곡선 끝 -> 직진: 회전은 바로 멈추고 속도는 그대로 (smooth 가 아니면 예전처럼 정지)
  void end_curve()
  {
    if (!smooth_)
    {
      stop_wheels();
      return;
    }
    k_out_ = 0.0;
    w_out_ = 0.0;
    tgt_w_ = 0.0;
  }

  static double move_toward(double x, double target, double step)
  {
    return x + std::clamp(target - x, -step, step);
  }

  // 입구 / 도로로 나가는 곡선의 명령 반지름 [m]
  // 도로로 나가는 곡선의 처음 명령 반지름 [m] (leave_arc_radius_m, 0 이면 들어갈 때와 같게)
  double leave_radius_cmd() const
  {
    if (leave_arc_radius_m_ > 0.0)
    {
      return std::max(leave_arc_radius_m_, entry_arc_min_radius_m_);
    }
    return entry_radius_cmd();
  }

  double entry_radius_cmd() const
  {
    return std::max(entry_arc_radius_m_ * entry_arc_radius_gain_, entry_arc_min_radius_m_);
  }

  // 곡선을 서서히 꺾으면 그 거리의 절반만큼 늦게 꺾은 것과 같다 -> 그만큼 일찍 시작 [m 명령]
  double curve_ramp_shift(double radius) const
  {
    if (!smooth_ || smooth_curve_m_ <= 0.0 || radius <= 0.0)
    {
      return 0.0;
    }
    return 0.5 * smooth_curve_m_ * (0.2 / radius);
  }

  // 0.02초마다: 목표 (tgt_v_, tgt_w_) 쪽으로 속도는 smooth_accel 씩, 곡률은 간 거리만큼씩 바꿔서 보낸다
  void smooth_step()
  {
    if (!smooth_)
    {
      return;
    }
    if (!i_am_driving() || now().seconds() - tgt_t_ > 0.3)
    {
      if (v_out_ != 0.0 || w_out_ != 0.0)
      {
        hard_stop();             // 주차 끝 / 그만둠 / 명령이 끊김 -> 바로 정지
      }
      return;
    }
    const double dt = 0.02;
    const double dv = std::max(0.0, smooth_accel_) * dt;
    if (std::abs(tgt_v_) < 1e-4 && std::abs(tgt_w_) > 1e-4)
    {
      v_out_ = move_toward(v_out_, 0.0, dv);            // 제자리 회전
      w_out_ = move_toward(w_out_, tgt_w_, 3.0 * dt);
      k_out_ = 0.0;
    }
    else
    {
      if (std::abs(tgt_v_) >= 1e-4 && v_out_ * tgt_v_ >= 0.0)
      {
        const double kt = tgt_w_ / tgt_v_;
        if (std::abs(v_out_) < 0.02 || smooth_curve_m_ <= 0.0 || tgt_v_ < 0.0)
        {
          k_out_ = kt;           // 거의 서 있을 때 (길이 안 바뀜) / 뒤로 갈 때 (점선 맞추기가 바로 반응하게) 는 바로
        }
        else
        {
          const double rate = (1.0 / 0.2) / smooth_curve_m_;   // [1/m^2] 반지름 20cm 를 smooth_curve_m 동안
          k_out_ = move_toward(k_out_, kt, rate * std::abs(v_out_) * dt);
        }
      }
      // 멈추는 중이거나 앞뒤가 바뀌는 중이면 지금 곡선 그대로 줄인다
      v_out_ = move_toward(v_out_, tgt_v_, dv);
      w_out_ = k_out_ * v_out_;
    }
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = v_out_;
    cmd.angular.z = w_out_;
    cmd_pub_->publish(cmd);
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
  bool entry_arc_;
  double entry_gap_to_center_m_;
  double entry_forward_m_;
  double entry_latency_s_;
  double entry_arc_radius_m_;
  double entry_arc_radius_gain_;
  double entry_arc_min_radius_m_;
  double entry_arc_speed_;
  double entry_v_ = 0.08;          // 이번 ENTRY_ARC 의 직진 속도 [m/s]
  bool entry_turn_match_;
  double entry_turn_w_;
  bool leave_turn_match_, leave_slow_;
  double entry_v_max_ = 0.08;      // 이번 ENTRY_ARC 의 최고 직진 속도 [m/s]
  double entry_turn_slow_deg_;
  double entry_check_deg_;
  bool entry_pair_stop_;
  double entry_pair_max_lean_deg_;
  double entry_pair_check_deg_;
  bool stem_use_white_;
  double same_line_px_;
  int64_t stem_end_frames_;
  double stem_min_time_;
  double stem_timeout_;
  double side_psd_threshold_;
  int64_t side_hits_needed_;
  double side_turn_deg_ = 90.0;    // 이번에 돌 각도 [도] (돌기 시작할 때 왼쪽 / 오른쪽 값으로 정함)
  double side_turn_base_deg_;
  double side_turn_deg_left_;
  double side_turn_deg_right_;
  bool side_turn_toward_;
  std::string side_turn_when_;
  bool side_cam_;
  bool side_turn_fixed_;
  double side_odom_scale_;
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
  bool side_snap_;
  double side_snap_min_ratio_;
  bool side_arc_;
  double side_arc_radius_m_;
  double side_arc_radius_gain_;
  double side_arc_speed_;
  double side_arc_forward_m_;
  double side_arc_forward_max_m_;
  double side_arc_forward_time_;
  double side_arc_backup_time_;
  double backup_time_;
  double backup_speed_;
  bool exit_enable_;
  double exit_arc_speed_;
  double exit_arc_radius_m_;
  double exit_arc_radius_gain_;
  double exit_arc_min_radius_m_;
  bool exit_guide_;
  double exit_guide_offset_m_, exit_guide_align_deg_, exit_guide_max_radius_m_;
  bool exit_guide_active_ = false; // 이번 EXIT_ARC 에서 안내선을 보고 도는 중
  bool leave_guide_;
  double leave_guide_offset_m_, leave_guide_align_deg_, leave_guide_min_radius_m_;
  double leave_guide_max_rms_px_ = 10.0;
  double leave_last_lean_ = -M_PI / 2.0;   // 보면서 도로로: 바로 전 영상의 흰 선 기울기 [rad]
  bool leave_guide_active_ = false; // 이번 도로로 나가는 곡선에서 흰 선을 보고 도는 중
  bool dash_align_;
  double dash_gain_, dash_max_w_, dash_max_deg_, dash_min_len_px_, dash_max_len_px_, dash_max_width_px_;
  double bev_near_m_, aisle_width_m_, exit_dash_gain_;
  double exit_dash_gain_left_, exit_dash_gain_right_;
  double leave_min_time_;
  double leave_latency_s_ = 0.0;
  double entry_yellow_min_s_ = 0.5;
  double yellow_run_start_ = -1.0, yellow_run_last_ = -1.0;   // 왼쪽 아래 노란선이 계속 보이기 시작한 / 마지막 시각
  double leave_arc_radius_m_ = 0.12;
  bool slow_near_;
  double stem_slow_start_y_, leave_slow_start_y_;
  double exit_travel_ = 0.0;       // EXIT_STEM 에서 간 거리 [m, 명령 단위]
  bool slow_logged_ = false;
  double dash_angle_ = 0.0;        // 마지막으로 본 흰 점선 기울기 [rad] (+ = 오른쪽이 아래)
  double dash_dist_ = 0.0;         // 바퀴 축에서 그 점선까지 [m]
  double dash_time_ = -1.0;        // 그 점선을 본 시각 [s] (-1 = 못 봄)
  double exit_arc_deg_;
  double exit_check_ratio_;
  double exit_line_max_rms_px_;
  double exit_overshoot_deg_;
  double exit_wait_yellow_deg_;
  int64_t exit_stop_y_;
  double exit_white_max_tilt_deg_;
  double exit_timeout_;
  bool leave_turn_;
  double leave_forward_m_;
  int64_t leave_yellow_frames_;
  double leave_rearm_delay_;
  double front_stop_distance_;
  double image_timeout_;
  bool smooth_;
  double smooth_accel_, smooth_curve_m_;
  double tgt_v_ = 0.0, tgt_w_ = 0.0, tgt_t_ = 0.0;   // 바퀴 명령 목표 (drive)
  double v_out_ = 0.0, w_out_ = 0.0, k_out_ = 0.0;   // 실제로 보내는 값, 곡률 [1/m]
  rclcpp::TimerBase::SharedPtr smooth_timer_;
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
  bool match_drive_speed_;
  double match_speed_min_, match_speed_max_;
  bool match_turn_speed_ = false;
  double entry_arc_speed_base_ = 0.08, exit_arc_speed_base_ = 0.08;   // yaml 값 (일반 주행 속도로 바꾸기 전)
  double run_speed_ = 0.10;        // 이번 주차에서 쓰는 "일반 주행 속도"
  double seen_drive_v_ = 0.0;      // 마지막으로 본 일반 주행 직진 속도 [m/s, 명령 단위]
  double seen_drive_t_ = -1.0;     // 그 시각 [s]
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  int64_t lines_missing_ = 0;      // STEM 에서 차선이 둘 다 연속으로 안 보인 프레임 수
  int64_t left_hits_ = 0;          // 왼쪽 PSD 가 연속으로 가까웠던 횟수
  int64_t right_hits_ = 0;         // 오른쪽 PSD 가 연속으로 가까웠던 횟수
  double side_seen_ = 0.0;         // 장애물을 본 쪽 (+1 왼쪽, -1 오른쪽, 0 아직 못 봄)
  int64_t clear_hits_ = 0;         // 장애물을 본 쪽 PSD 가 연속으로 멀었던 횟수
  double last_white_y_ = -1.0;     // 바로 전 영상에서 가로 흰 선의 줄 번호 (-1 = 없음)
  std::vector<double> last_scene_angles_;  // STEM 중 바로 전 영상의 직선 방향 분포 (180칸)
  double side_direction_ = 0.0;    // SIDE_TURN 방향 (+1 왼쪽, -1 오른쪽)
  double side_forward_time_ = 0.0; // SIDE_FORWARD 시간 [s]
  double side_arc_radius_cmd_ = 0.1;   // SIDE_TURN 곡선의 명령 반지름 [m] (side_arc)
  bool side_grid_seen_ = false;    // SIDE_TURN 중 격자를 한 번이라도 봤는지
  double side_rough_ = 0.0;        // SIDE_TURN 중 돈 각도 [rad] (로그용)
  double side_phi_ = 0.0;          // SIDE_TURN 중 격자 기준으로 돈 각도 [rad] (끊김 없이 이어 붙인 값)
  double side_gap_est_ = 0.0;      // 마지막으로 격자를 본 뒤 대충 돈 각도 [rad] (odom 또는 시간)
  double side_gap_other_ = 0.0;    // 바로 전 영상 뒤로 odom (또는 시간) 으로 잰 회전 [rad]
  double side_last_yaw_ = 0.0;     // 바로 전 odom 방향 [rad]
  double side_w_cmd_ = 0.0;        // SIDE_TURN 마지막으로 명령한 회전 속도 크기 [rad/s]
  double side_snap_tilt_ = 0.0;    // 마지막으로 잰 격자 틀어짐 [rad]
  double backup_duration_ = 0.0;   // 이번 BACKUP 시간 [s]
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
  double entry_w_ = 0.0;           // ENTRY_ARC 회전 속도 [rad/s] (+ 왼쪽)
  double entry_arc_time_full_ = 0.0;  // ENTRY_ARC 명령대로 90도 도는 시간 [s]
  double entry_arc_limit_ = 0.0;   // ENTRY_ARC 최대 시간 [s] (넘으면 포기)
  std::vector<double> entry_prev_angles_;  // ENTRY_ARC 중 바로 전 영상의 직선 방향 분포
  double entry_cam_sum_ = 0.0;     // ENTRY_ARC 중 카메라로 잰 돈 각도 합 [rad]
  int64_t entry_cam_lost_ = 0;     // ENTRY_ARC 중 직선을 연속으로 못 찾은 프레임 수
  bool entry_cam_failed_ = false;  // ENTRY_ARC 중 카메라 각도를 포기했는지 (그 뒤로는 시간으로 어림)
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
  bool leaving_ = false;           // 도로로 나가는 곡선 중인지 (EXTRA / ENTRY_ARC 를 같이 쓴다)
  int64_t leave_present_ = 0;      // EXIT_STEM 중 왼쪽 아래 노란선이 연속으로 보인 프레임 수
  bool leave_seen_ = false;        // EXIT_STEM 중 왼쪽 아래 노란선을 봤는지 (본 뒤에 끊겨야 도로)
  int64_t leave_empty_ = 0;        // 그 뒤 연속으로 안 보인 프레임 수
  int64_t handover_ticks_ = 0;     // LANE 을 몇 번 더 보낼지 (0.1초마다)
  double rearm_until_s_ = 0.0;     // 이 시각 [s] 전에는 주차 표지판 무시
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
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<ControlMode>::SharedPtr mode_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParkingNode>());
  rclcpp::shutdown();
  return 0;
}
