// 격자 계획 주행: 카메라 -> 바닥 지도 (색 + 도로 세그멘테이션) -> grid_planner -> cmd_vel
//   구독: image_raw (sensor_msgs/Image)              v4l2_camera 640x480
//         odom (nav_msgs/Odometry)                   stm_bridge, 프레임 사이 이동량 -> planner.onMove
//         control_mode (interfaces/ControlMode)      LANE / AVOID 일 때만 주행
//         sign (interfaces/Sign)                     LEFT / RIGHT 를 보면 turn_hold_sec 동안 그쪽 목표 가점
//         lane_info, cross_line (lane_detection)     intersection_lane 이 true 면 교차로를 예전 sliding 방식으로
//
// 교차로 (intersection_lane, 2026-10-28): 격자 계획은 최단 경로라 교차로 U턴에서 안쪽 선을 밟았다.
//   lane_follower 의 교차로 상태 기계 (include/lane_follower/intersection_turn.hpp, 표지판 -> T자 가로 선 -> 한쪽 바퀴 회전
//   -> 고리 -> 출구) 와 차선 추종 (controller.hpp, lane_info 의 차선 가운데) 을 그대로 쓴다. 상태가 IDLE 이 아니면
//   (표지판 확인 ~ 출구) 그 명령을 보내고, IDLE 로 돌아오면 다시 격자 계획.
//   intersection.* / lane.* 값은 lane_follower.yaml (2026-10-27 튜닝) 에서 복사. 그때는 stm wheel_radius 0.05 라
//   속도 / odom 회전이 모두 cmd_vel 단위 -> 그 값을 그대로 쓰려고 odom 이동량에 cmd_scale 을 곱해 넣고 명령은 그대로 보냄
//   발행: cmd_vel (geometry_msgs/Twist)
//         grid_drive/debug/compressed               원본 + 도로 마스크 | 계획 그림 (구독자가 있을 때만)
//
// 관측 (include/grid_drive/floor_obs.hpp)
//   선 / 박스: 색 방식 (observeColor). 주행 불가: 도로 모델 (road_seg.hpp) 의 내 차선 마스크 -> observeSegmentation
//   두 관측을 merge. 엔진이 없거나 use_road=false 면 색 방식만 (주행 불가 모름)
//   주행 불가 (차선 밖) 는 공사 구간에서도 항상 진입 금지 (2026-10-28 사용자: 차선 넘으면 안 됨)
//
// 속도: 계획기 v / w 는 실제 [m/s] 로 보고 cmd_scale 을 곱해 보낸다.
//   펌웨어 WHEEL_RADIUS_M 0.05 그대로라 실제 속도 = cmd_vel x 2.45 (벨트 비, stm wheel_radius 0.1225)
//   -> cmd_scale = 0.05 / 0.1225 = 0.408. 펌웨어를 고치면 1.0
// 안전: 영상이 image_timeout 동안 안 오면 정지. 주행 모드가 아니면 정지 명령 한 번 후 조용히 (다른 노드와 안 싸움)
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "grid_drive/grid_planner.hpp"
#include "grid_drive/road_seg.hpp"
#include "interfaces/msg/barrier.hpp"
#include "interfaces/msg/control_mode.hpp"
#include "interfaces/msg/lane_info.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "lane_follower/controller.hpp"
#include "lane_follower/intersection_turn.hpp"
#include "interfaces/msg/sign.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#ifdef GRID_DRIVE_TENSORRT
#include "grid_drive/trt_engine.hpp"
#endif

using interfaces::msg::ControlMode;
using interfaces::msg::Sign;
using interfaces::msg::Barrier;

namespace
{
std::string expandHome(const std::string & path)
{
  const char * home = std::getenv("HOME");
  if (home && !path.empty() && path[0] == '~') {
    return home + path.substr(1);
  }
  return path;
}

double yawOf(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}
}  // namespace

class GridDriveNode : public rclcpp::Node
{
public:
  GridDriveNode()
  : Node("grid_drive")
  {
    const auto image_topic = declare_parameter("image_topic", std::string("image_raw"));
    const auto engine_path = expandHome(
      declare_parameter("engine_path", std::string("~/yolo26_check/road_r2_320_fp16.engine")));
    use_road_ = declare_parameter("use_road", true);
    seg_.conf = static_cast<float>(declare_parameter("road_conf", 0.5));
    seg_.mask_thr = static_cast<float>(declare_parameter("road_mask_thr", 0.5));
    mode_ = declare_parameter("autostart", false) ? ControlMode::LANE : ControlMode::STOP;
    cmd_scale_ = declare_parameter("cmd_scale", 0.408);
    process_rate_ = declare_parameter("process_rate", 15.0);
    image_timeout_ = declare_parameter("image_timeout", 0.5);
    turn_hold_ = declare_parameter("turn_hold_sec", 6.0);
    sign_conf_ = declare_parameter("sign_min_conf", 0.6);
    jpeg_quality_ = static_cast<int>(declare_parameter("debug_jpeg_quality", 70));
    // 명령 부드럽게: 매번 새 값 쪽으로 이 비율만큼만 다가감 (1 = 필터 없음, 작을수록 부드럽지만 늦음)
    smooth_ = declare_parameter("cmd_smooth", 0.3);
    // 코너 진입: 곡선이 처음 보이면 corner_delay_sec 동안 직진한 뒤 계획대로 회전
    corner_delay_ = declare_parameter("corner_delay_sec", 1.5);
    corner_k_ = declare_parameter("corner_k_min", 2.0);          // [1/m] 이보다 급한 곡선 = 코너 (2.0 = 반지름 50cm)
    corner_rearm_ = declare_parameter("corner_rearm_sec", 0.5);  // 곡선이 이만큼 안 보이면 다음 코너 준비
    // 좌/우 기억 중 곡선 방향이 바뀌면 (진입 좌회전 -> U턴 우회전 등) 이 시간 [s] 직진한 뒤 회전 (0 = 끔).
    //   U턴을 너무 일찍 시작해 안쪽 고리 선을 밟았다 (2026-10-28 drive21)
    turn_flip_delay_ = declare_parameter("turn_flip_delay_sec", 0.0);
    // 교차로: 좌/우 표지판 기억 중에는 turn_linear 까지 감속 (0 = 감속 안 함)
    // 도로 모델이 있으면 주행 가능 구역 안에서만 계획 (구역이 안 보이면 계획 안 하고 정지)
    p_.road_only = declare_parameter("road_only", true);
    road_hold_sec_ = declare_parameter("road_hold_sec", 0.15);
    // 막다른 길: 막는 것이 화면 아래로 빠져 안 보여도, 마지막으로 잰 거리에서 정지 거리까지 남은 만큼 odom 으로 천천히 직진
    creep_linear_ = declare_parameter("dead_end_creep_linear", 0.05);
    turn_linear_ = declare_parameter("turn_linear", 0.12);
    // 교차로에서 꺾을 때: 안쪽 선이 화면 아래로 빠지면 지도에서 사라져 안쪽으로 파고듦 -> odom 기억 켬
    turn_memory_ = declare_parameter("turn_memory", true);
    // 공사 구간: 표지판을 본 뒤 construction_hold_sec 동안 (박스가 보이면 계속 연장)
    //   차선 폭 지키기 끔,
    //   odom 기억 켬 (벽 옆면이 화면 밖으로 빠져도 기억해서 덜 돌고 박는 것 막음)
    construction_hold_ = declare_parameter("construction_hold_sec", 20.0);
    construction_box_hold_ = declare_parameter("construction_box_hold_sec", 3.0);
    construction_box_min_ = declare_parameter("construction_box_min_ratio", 0.002);
    construction_memory_ = declare_parameter("construction_memory", true);
    construction_memory_lines_ = declare_parameter("construction_memory_lines", false);
    // 공사 모드 회전: 최대 회전 속도 (0 = planner.max_angular 그대로), 곡률 바꿈 벌점 (0 = planner.change_weight 그대로)
    //   박스 사이에서 크게 꺾고 되꺾는 지그재그 줄이기. 속도는 그대로
    construction_max_angular_ = declare_parameter("construction_max_angular", 0.0);
    construction_change_weight_ = declare_parameter("construction_change_weight", 0.0);
    p_.memory_max_frames = static_cast<int>(declare_parameter("planner.memory_max_frames", 15));
    p_.box_memory_max_frames = static_cast<int>(declare_parameter("planner.box_memory_max_frames", -1));
    // 한 프레임만 튀는 회전 명령 거르기: 최근 w_median 프레임의 가운데 값 (1 = 끔)
    w_median_ = std::max<int64_t>(1, declare_parameter("w_median", 3));
    // 출발 / 가속 제한 [m/s^2 실제]: 정지에서 급출발하면 앞이 들림. 감속은 제한 안 함
    accel_limit_ = declare_parameter("accel_limit", 0.3);

    // 계획기 파라미터 (나머지는 grid_planner.hpp 기본값)
    const std::vector<std::pair<const char *, double *>> dparams = {
      {"linear", &p_.linear}, {"min_linear", &p_.min_linear}, {"slow_curvature", &p_.slow_curvature},
      {"max_angular", &p_.max_angular}, {"pivot_angular", &p_.pivot_angular},
      {"soft_speed_ratio", &p_.soft_speed_ratio}, {"follow_mm", &p_.follow_mm},
      {"robot_half_width_mm", &p_.robot_half_width_mm}, {"line_margin_mm", &p_.line_margin_mm},
      {"recover_mm", &p_.recover_mm}, {"hard_line_mm", &p_.hard_line_mm}, {"near_line_cost", &p_.near_line_cost},
      {"box_margin_mm", &p_.box_margin_mm}, {"map_y_scale", &p_.map_y_scale},
      {"road_min_ratio", &p_.road_min_ratio},
      {"lane_half_mm", &p_.lane_half_mm}, {"far_line_cost", &p_.far_line_cost},
      {"box_depth_mm", &p_.box_depth_mm}, {"box_gap_mm", &p_.box_gap_mm},
      {"memory_back_mm", &p_.memory_back_mm}, {"box_memory_back_mm", &p_.box_memory_back_mm},
      {"dead_end_stop_mm", &p_.dead_end_stop_mm}, {"dead_end_line_mm", &p_.dead_end_line_mm},
      {"dead_end_slow_mm", &p_.dead_end_slow_mm},
      {"curvature_weight", &p_.curvature_weight}, {"change_weight", &p_.change_weight},
    };
    for (const auto & [name, ptr] : dparams) {
      *ptr = declare_parameter(std::string("planner.") + name, *ptr);
    }
    p_.memory = declare_parameter("planner.memory", p_.memory);
    // 좌/우 기억 중 선 여유 [mm] (planner.line_margin_mm 대신). 교차로 진입 때 고리 선과 바깥 선의 빨간 띠가
    // 붙어 표지판 쪽 길이 막히면 낮추기
    turn_line_margin_ = declare_parameter("turn_line_margin_mm", p_.line_margin_mm);
    // 왼쪽 표지판 진입만 따로 (오른쪽은 띠가 길을 막아 0 이 필요했고, 왼쪽은 0 이면 왼쪽 코너 안쪽 선을 밟음)
    turn_line_margin_left_ = declare_parameter("turn_line_margin_left_mm", turn_line_margin_);
    // 위 여유는 좌/우 표지판을 마지막으로 본 뒤 이 시간 [s] 동안만 (진입). 그 뒤 (U턴 등) 는 planner.line_margin_mm
    //   (안쪽 선 밟음 막음). 0 = 좌/우 기억 내내
    turn_line_margin_sec_ = declare_parameter("turn_line_margin_sec", 0.0);
    // 진입 뒤 (위 시간 지난 뒤) 좌/우 기억 중 선 여유 [mm]. 크면 U턴을 크게 돌아 안쪽 선에서 멀어짐
    turn_after_margin_ = declare_parameter("turn_after_margin_mm", p_.line_margin_mm);
    // 교차로 출구: 길은 표지판 쪽으로 들어가 고리를 반대쪽으로 U턴한 뒤 다시 표지판 쪽으로 나감 (drive24 odom:
    //   오른쪽 -70도 -> 고리 왼쪽 +155도 -> 출구 오른쪽). 표지판을 본 뒤 (가장 표지판 쪽으로 꺾인 곳부터) 반대쪽으로
    //   turn_exit_yaw_deg [odom 도] 돌면 U턴 끝 = 출구 앞으로 보고 좌/우 기억을 turn_exit_hold_sec 동안 다시 켬. 0 = 끔.
    //   출구를 기다리는 동안 반대 방향 표지판은 무시 (drive24: 출구에서 섬 안 표지판을 LEFT 0.61 로 한 번 보고 밖으로 나감)
    //   turn_hold_sec 를 짧게 (진입만) 하자 출구에서 직진해 오른쪽 출구를 지나쳤다 (2026-10-28)
    turn_exit_yaw_ = declare_parameter("turn_exit_yaw_deg", 0.0) * M_PI / 180.0;
    turn_exit_hold_ = declare_parameter("turn_exit_hold_sec", 3.0);
    turn_exit_max_ = declare_parameter("turn_exit_max_sec", 20.0);   // 표지판 뒤 이 시간 안에 출구가 없으면 잊음
    // 좌/우 기억 중 선 기억 프레임 수 (0 이하 = planner.memory_max_frames). U턴 중 안쪽 고리 선이 화면 오른쪽 아래로
    //   빠진 뒤 0.5초 만에 잊어서 그 위로 경로를 냈다 (2026-10-28 drive19)
    turn_memory_frames_ = static_cast<int>(declare_parameter("turn_memory_max_frames", 0));
    // 좌/우 기억 중 가장 급한 원호 반지름 [mm 실제] (0 = 기본 150). 교차로 U턴이 150 으로 못 돌아 바깥 선을 넘었다
    turn_min_radius_ = declare_parameter("turn_min_radius_mm", 0.0);
    // 좌/우 기억 중 곡률 바꿈 벌점 (0 = planner.change_weight). 진입 좌회전 -> U턴 우회전으로 빨리 바꾸게 낮춤
    turn_change_weight_ = declare_parameter("turn_change_weight", 0.0);
    // 좌/우 기억 중 가운데 따라가기 비용 (planner.center_cost, 0 = 끔) 과 가운데를 찾는 반경 [mm]
    turn_center_cost_ = declare_parameter("turn_center_cost", 0.0);
    p_.center_radius_mm = declare_parameter("turn_center_radius_mm", p_.center_radius_mm);

#ifdef GRID_DRIVE_TENSORRT
    if (use_road_) {
      try {
        trt_ = std::make_unique<grid_drive::TrtEngine>(engine_path);
      } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "도로 엔진을 못 씀 -> 색 방식만: %s", e.what());
      }
    }
#else
    if (use_road_) {
      RCLCPP_WARN(get_logger(), "TensorRT 없이 빌드됨 -> 색 방식만");
    }
#endif

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    debug_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      "grid_drive/debug/compressed", rclcpp::SensorDataQoS());
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 20, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {odom_ = *m; have_odom_ = true;});
    mode_sub_ = create_subscription<ControlMode>(
      "control_mode", 10, [this](ControlMode::ConstSharedPtr m) {
        if (m->mode != mode_) {
          RCLCPP_INFO(get_logger(), "control_mode %u -> %u (%s)", mode_, m->mode,
          m->mission_state.c_str());
          mode_ = m->mode;
        }
      });
    // 비전 차단바 인식 (sign_detection): 닫힌 차단바를 본 시각
    barrier_hold_ = declare_parameter("barrier_hold_sec", 1.5);
    barrier_sub_ = create_subscription<Barrier>(
      "barrier", rclcpp::SensorDataQoS(), [this](Barrier::ConstSharedPtr m) {
        if (m->detected && m->state != Barrier::OPEN && m->confidence >= 0.5) {
          bar_closed_t_ = now();
        }
      });
    // 교차로를 예전 sliding 방식으로 (맨 위 설명)
    xturn_on_ = declare_parameter("intersection_lane", false);
    xp_.enabled = declare_parameter("intersection.enabled", xp_.enabled);
    xp_.sign_min_confidence = declare_parameter("intersection.sign_min_confidence", xp_.sign_min_confidence);
    xp_.sign_min_area = declare_parameter("intersection.sign_min_area", xp_.sign_min_area);
    xp_.sign_confirm_count = static_cast<int>(declare_parameter("intersection.sign_confirm_count", static_cast<int64_t>(xp_.sign_confirm_count)));
    xp_.sign_confirm_window = static_cast<int>(declare_parameter("intersection.sign_confirm_window", static_cast<int64_t>(xp_.sign_confirm_window)));
    xp_.sign_instant_confidence = declare_parameter("intersection.sign_instant_confidence", xp_.sign_instant_confidence);
    xp_.arm_max_dist = declare_parameter("intersection.arm_max_dist", xp_.arm_max_dist);
    xp_.require_cross_line = declare_parameter("intersection.require_cross_line", xp_.require_cross_line);
    xp_.require_cross_line_grace = declare_parameter("intersection.require_cross_line_grace", xp_.require_cross_line_grace);
    xp_.approach_align = declare_parameter("intersection.approach_align", xp_.approach_align);
    xp_.approach_align_kp = declare_parameter("intersection.approach_align_kp", xp_.approach_align_kp);
    xp_.approach_align_max = declare_parameter("intersection.approach_align_max", xp_.approach_align_max);
    xp_.approach_align_fresh = declare_parameter("intersection.approach_align_fresh", xp_.approach_align_fresh);
    xp_.approach_slow_linear = declare_parameter("intersection.approach_slow_linear", xp_.approach_slow_linear);
    xp_.align_enabled = declare_parameter("intersection.align_enabled", xp_.align_enabled);
    xp_.align_kp = declare_parameter("intersection.align_kp", xp_.align_kp);
    xp_.align_max_angular = declare_parameter("intersection.align_max_angular", xp_.align_max_angular);
    xp_.align_min_angular = declare_parameter("intersection.align_min_angular", xp_.align_min_angular);
    xp_.align_tol = declare_parameter("intersection.align_tol", xp_.align_tol);
    xp_.align_ok_count = static_cast<int>(declare_parameter("intersection.align_ok_count", static_cast<int64_t>(xp_.align_ok_count)));
    xp_.align_max_time = declare_parameter("intersection.align_max_time", xp_.align_max_time);
    xp_.align_lost_time = declare_parameter("intersection.align_lost_time", xp_.align_lost_time);
    xp_.cross_recent_time = declare_parameter("intersection.cross_recent_time", xp_.cross_recent_time);
    xp_.approach_time = declare_parameter("intersection.approach_time", xp_.approach_time);
    xp_.approach_linear = declare_parameter("intersection.approach_linear", xp_.approach_linear);
    xp_.turn_pivot = declare_parameter("intersection.turn_pivot", xp_.turn_pivot);
    xp_.wheel_separation = declare_parameter("intersection.wheel_separation", xp_.wheel_separation);
    xp_.turn_linear = declare_parameter("intersection.turn_linear", xp_.turn_linear);
    xp_.turn_angular = declare_parameter("intersection.turn_angular", xp_.turn_angular);
    xp_.turn_search_angular = declare_parameter("intersection.turn_search_angular", xp_.turn_search_angular);
    xp_.turn_min_yaw = declare_parameter("intersection.turn_min_yaw", xp_.turn_min_yaw);
    xp_.turn_switch_max_yaw = declare_parameter("intersection.turn_switch_max_yaw", xp_.turn_switch_max_yaw);
    xp_.turn_max_yaw = declare_parameter("intersection.turn_max_yaw", xp_.turn_max_yaw);
    xp_.exit_ok_count = static_cast<int>(declare_parameter("intersection.exit_ok_count", static_cast<int64_t>(xp_.exit_ok_count)));
    xp_.exit_min_confidence = declare_parameter("intersection.exit_min_confidence", xp_.exit_min_confidence);
    xp_.exit_max_angle = declare_parameter("intersection.exit_max_angle", xp_.exit_max_angle);
    xp_.exit_max_offset = declare_parameter("intersection.exit_max_offset", xp_.exit_max_offset);
    xp_.exit_lead_angle = declare_parameter("intersection.exit_lead_angle", xp_.exit_lead_angle);
    xp_.exit_turn_side_only = declare_parameter("intersection.exit_turn_side_only", xp_.exit_turn_side_only);
    xp_.rearm_dist = declare_parameter("intersection.rearm_dist", xp_.rearm_dist);
    xp_.after_enabled = declare_parameter("intersection.after_enabled", xp_.after_enabled);
    xp_.after_dist = declare_parameter("intersection.after_dist", xp_.after_dist);
    xp_.after_max_yaw = declare_parameter("intersection.after_max_yaw", xp_.after_max_yaw);
    xp_.after_max_retries = static_cast<int>(declare_parameter("intersection.after_max_retries", static_cast<int64_t>(xp_.after_max_retries)));
    xp_.out_enabled = declare_parameter("intersection.out_enabled", xp_.out_enabled);
    xp_.out_right_enabled = declare_parameter("intersection.out_right_enabled", xp_.out_right_enabled);
    xp_.out_trigger_yaw = declare_parameter("intersection.out_trigger_yaw", xp_.out_trigger_yaw);
    xp_.out_island_offset = declare_parameter("intersection.out_island_offset", xp_.out_island_offset);
    xp_.out_island_min_yaw = declare_parameter("intersection.out_island_min_yaw", xp_.out_island_min_yaw);
    xp_.out_min_dist = declare_parameter("intersection.out_min_dist", xp_.out_min_dist);
    xp_.out_max_dist = declare_parameter("intersection.out_max_dist", xp_.out_max_dist);
    xp_.out_approach_time = declare_parameter("intersection.out_approach_time", xp_.out_approach_time);
    xp_.out_turn_min_yaw = declare_parameter("intersection.out_turn_min_yaw", xp_.out_turn_min_yaw);
    xp_.out_turn_max_yaw = declare_parameter("intersection.out_turn_max_yaw", xp_.out_turn_max_yaw);
    xp_.out_follow = declare_parameter("intersection.out_follow", xp_.out_follow);
    xp_.out_guard_dist = declare_parameter("intersection.out_guard_dist", xp_.out_guard_dist);
    xp_.out_guard_max_offset = declare_parameter("intersection.out_guard_max_offset", xp_.out_guard_max_offset);
    xp_.out_arc_linear = declare_parameter("intersection.out_arc_linear", xp_.out_arc_linear);
    xp_.out_arc_angular = declare_parameter("intersection.out_arc_angular", xp_.out_arc_angular);
    xp_.out_arc_max_dist = declare_parameter("intersection.out_arc_max_dist", xp_.out_arc_max_dist);
    xp_.out_hold_time = declare_parameter("intersection.out_hold_time", xp_.out_hold_time);
    gains_.linear_speed = declare_parameter("lane.linear_speed", gains_.linear_speed);
    gains_.min_linear_speed = declare_parameter("lane.min_linear_speed", gains_.min_linear_speed);
    gains_.kp = declare_parameter("lane.kp", gains_.kp);
    gains_.kd = declare_parameter("lane.kd", gains_.kd);
    gains_.ka = declare_parameter("lane.ka", gains_.ka);
    gains_.max_angular = declare_parameter("lane.max_angular", gains_.max_angular);
    gains_.slow_down_angle = declare_parameter("lane.slow_down_angle", gains_.slow_down_angle);
    gains_.low_confidence = declare_parameter("lane.low_confidence", gains_.low_confidence);
    gains_.d_filter_alpha = declare_parameter("lane.d_filter_alpha", gains_.d_filter_alpha);
    lane_timeout_ = declare_parameter("lane.lane_timeout", 0.7);
    lane_lost_hold_ = declare_parameter("lane.lost_hold_sec", 0.5);
    if (xturn_on_) {
      lane_sub_ = create_subscription<interfaces::msg::LaneInfo>(
        "lane_info", rclcpp::SensorDataQoS(), [this](interfaces::msg::LaneInfo::ConstSharedPtr m) {
          lane_ = *m;
          lane_stamp_ = now();
          lane_rx_ = true;
        });
      cross_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
        "cross_line", rclcpp::SensorDataQoS(), [this](geometry_msgs::msg::Vector3Stamped::ConstSharedPtr m) {
          xturn_.onCrossLine(m->vector.z > 0, m->vector.y);
        });
    }
    sign_sub_ = create_subscription<Sign>(
      "sign", rclcpp::SensorDataQoS(), [this](Sign::ConstSharedPtr m) {
        if (m->type == Sign::CONSTRUCTION && m->confidence >= sign_conf_) {
          if (now() > construction_until_) {
            RCLCPP_INFO(get_logger(), "공사 구간 표지판 -> 공사 모드 %.0fs", construction_hold_);
          }
          construction_until_ = now() + rclcpp::Duration::from_seconds(construction_hold_);
        }
        if (xturn_on_) {
          xturn_.onSign(m->type, m->confidence, m->area_ratio, xp_);
          logXturn();
          return;   // 좌/우는 교차로 상태 기계가 맡음 (격자 turn_ 안 씀)
        }
        if ((m->type == Sign::LEFT || m->type == Sign::RIGHT) && m->confidence >= sign_conf_) {
          const auto dir = m->type == Sign::LEFT ? grid_drive::Turn::LEFT : grid_drive::Turn::RIGHT;
          if (exit_armed_ && dir != exit_dir_) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "교차로 출구 기다리는 중 반대 표지판 무시 (%.2f)",
              m->confidence);
            return;
          }
          turn_ = dir;
          turn_until_ = now() + rclcpp::Duration::from_seconds(turn_hold_);
          turn_sign_t_ = now();
          // 출구 기다림: 처음 보거나 방향이 바뀌면 회전량을 새로 셈 (계속 보이는 / 섬 안에서 다시 본 같은 표지판은 이어 셈)
          if (turn_exit_yaw_ > 0 && (!exit_armed_ || exit_dir_ != turn_)) {
            exit_armed_ = true;
            exit_dir_ = turn_;
            exit_acc_ = exit_lo_ = exit_hi_ = 0;
          }
          if (exit_armed_) {
            exit_deadline_ = now() + rclcpp::Duration::from_seconds(turn_exit_max_);
          }
        }
      });
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::Image::ConstSharedPtr m) {onImage(m);});
    watchdog_ = create_wall_timer(std::chrono::milliseconds(100), [this] {
          if (driving() && sent_moving_ && (now() - last_image_).seconds() > image_timeout_) {
            RCLCPP_WARN(get_logger(), "영상 %.1fs 끊김 -> 정지", image_timeout_);
            publishCmd(0, 0);
          }
        });

    RCLCPP_INFO(get_logger(), "grid_drive 시작 (구독 %s, 도로 %s, %s, cmd_scale %.3f)",
      image_topic.c_str(), haveRoad() ? engine_path.c_str() : "없음 (색 방식만)",
      mode_ == ControlMode::LANE ? "autostart" : "control_mode 대기", cmd_scale_);
  }

private:
  bool driving() const {return mode_ == ControlMode::LANE || mode_ == ControlMode::AVOID;}

  bool haveRoad() const
  {
#ifdef GRID_DRIVE_TENSORRT
    return trt_ != nullptr;
#else
    return false;
#endif
  }

  // 코너 진입 지연: 곡선을 처음 보면 corner_delay_ 초 동안 회전을 0 으로 (직진), 그 뒤 계획대로
  void cornerDelay(const grid_drive::Result & r, double & w)
  {
    const double t = now().seconds();
    const bool curve = r.follow_ok && !r.pivot && std::abs(r.curvature) * 1000.0 > corner_k_;
    if (curve && turn_ != grid_drive::Turn::NONE && turn_flip_delay_ > 0) {
      const int sign = r.curvature > 0 ? 1 : -1;
      if (last_curve_sign_ != 0 && sign != last_curve_sign_) {
        corner_until_ = std::max(corner_until_, t + turn_flip_delay_);
        RCLCPP_INFO(get_logger(), "교차로 곡선 방향 바뀜 -> %.2f초 직진 후 회전", turn_flip_delay_);
      }
      last_curve_sign_ = sign;
    }
    if (turn_ == grid_drive::Turn::NONE) {
      last_curve_sign_ = 0;
    }
    if (corner_delay_ <= 0) {
      if (t < corner_until_) {
        w = 0.0;
      }
      return;
    }
    if (curve) {
      last_curve_t_ = t;
      if (corner_armed_) {
        corner_armed_ = false;
        corner_until_ = t + corner_delay_;
        RCLCPP_INFO(get_logger(), "코너 발견 -> %.1f초 직진 후 회전", corner_delay_);
      }
    } else if (t - last_curve_t_ > corner_rearm_) {
      corner_armed_ = true;
    }
    if (t < corner_until_) {
      w = 0.0;
    }
  }

  void logXturn()
  {
    if (xturn_.takeChanged()) {
      RCLCPP_INFO(get_logger(), "[교차로] %s -> %s", xturn_.lastReason().c_str(), xturn_.describe().c_str());
    }
  }

  // 교차로 상태 기계 + 차선 추종. 이번 프레임을 맡았으면 true (명령은 cmd_vel 단위로 이미 보냄)
  bool intersectionDrive()
  {
    using IT = lane_follower::IntersectionTurn;
    const auto t = now();
    const double dt = have_x_tick_ ? std::clamp((t - x_tick_).seconds(), 0.0, 0.3) : 0.0;
    x_tick_ = t;
    have_x_tick_ = true;
    const bool lane_fresh = lane_rx_ && (t - lane_stamp_).seconds() <= lane_timeout_;
    IT::LaneObs obs;
    obs.lost = lane_fresh && !lane_.detected;
    obs.ok = lane_fresh && lane_.detected;
    obs.fresh = lane_rx_ && lane_stamp_ != x_lane_used_;
    if (lane_rx_) {
      x_lane_used_ = lane_stamp_;
    }
    obs.confidence = lane_.confidence;
    obs.angle = lane_.angle;
    obs.offset = lane_.offset;
    obs.left = lane_.left_detected;
    obs.right = lane_.right_detected;
    const IT::Cmd c = xturn_.update(obs, xp_, dt);
    logXturn();
    if (xturn_.phase() == IT::Phase::IDLE) {
      if (x_active_) {
        x_active_ = false;
        planner_.reset();   // 교차로 동안 쌓인 기억 / 직전 경로 버림
        RCLCPP_INFO(get_logger(), "[교차로] 끝 -> 격자 계획");
      }
      return false;
    }
    if (!x_active_) {
      x_active_ = true;
      lane_ctl_.reset();
      have_lane_cmd_ = false;
      RCLCPP_INFO(get_logger(), "[교차로] 시작 -> sliding 차선 추종 / 회전");
    }
    double v = 0, w = 0;
    if (c.active) {
      lane_ctl_.reset();
      v = c.v;
      w = c.w;
      if (c.hold && have_lane_cmd_) {
        v = lane_v_;
        w = lane_w_;
      }
    } else if (obs.ok) {
      const auto lc = lane_ctl_.step(lane_.offset, lane_.angle, lane_.confidence,
          lane_ok_prev_ ? dt : 0.0, gains_);
      v = lc.v;
      w = lc.w;
      double w_add = 0, v_max = 0;
      if (xturn_.crossAssist(xp_, w_add, v_max)) {
        w = std::clamp(w + w_add, -gains_.max_angular, gains_.max_angular);
        if (v_max > 0) {
          v = std::min(v, v_max);
        }
      }
      lane_v_ = v;
      lane_w_ = w;
      have_lane_cmd_ = true;
      last_lane_ok_ = t;
    } else if (have_lane_cmd_ && (t - last_lane_ok_).seconds() <= lane_lost_hold_) {
      v = lane_v_;   // 차선을 잠깐 놓침 -> 직전 명령
      w = lane_w_;
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "[교차로] 차선 못 찾음 -> 정지 (%s)",
        xturn_.describe().c_str());
    }
    lane_ok_prev_ = obs.ok && !c.active;
    geometry_msgs::msg::Twist tw;   // cmd_vel 단위 그대로 (lane_follower 튜닝값)
    tw.linear.x = v;
    tw.angular.z = w;
    cmd_pub_->publish(tw);
    sent_moving_ = v != 0 || w != 0;
    // 격자로 돌아갈 때 이어지게 실제 단위로 필터 상태를 맞춤
    v_f_ = v / cmd_scale_;
    w_f_ = w / cmd_scale_;
    w_hist_.clear();
    stop_sent_ = false;
    return true;
  }

  void publishCmd(double v, double w)
  {
    geometry_msgs::msg::Twist t;
    t.linear.x = v * cmd_scale_;
    t.angular.z = w * cmd_scale_;
    cmd_pub_->publish(t);
    sent_moving_ = v != 0 || w != 0;
  }

  // 도로 모델 -> 원본 크기 마스크 (엔진이 없으면 빈 Mat)
  grid_drive::RoadSeg road(const cv::Mat & img, double & ms)
  {
    grid_drive::RoadSeg rs;
#ifdef GRID_DRIVE_TENSORRT
    if (trt_) {
      const auto t0 = std::chrono::steady_clock::now();
      cv::Mat input;
      const auto lb = grid_drive::letterbox(img, trt_->inputSize(), input);
      const auto outs = trt_->infer(grid_drive::toBlob(input));
      if (outs.size() == 2) {
        rs = grid_drive::decodeRoad(outs[0], outs[1], trt_->inputSize(), lb, img.size(), seg_);
      }
      ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    }
#else
    (void)img;
    (void)ms;
#endif
    return rs;
  }

  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    const rclcpp::Time stamp(msg->header.stamp);
    if (process_rate_ > 0 && have_stamp_ && (stamp - last_stamp_).seconds() >= 0 &&
      (stamp - last_stamp_).seconds() < 0.9 / process_rate_)
    {
      return;
    }
    last_stamp_ = stamp;
    have_stamp_ = true;
    last_image_ = now();

    cv::Mat img;
    try {
      img = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "cv_bridge: %s", e.what());
      return;
    }

    // 주행 시작할 때 이전 경로 / 기억을 버린다
    if (driving() && !was_driving_) {
      planner_.reset();
      corner_armed_ = true;
      corner_until_ = 0;
      // 주차 노드 등에서 넘겨받을 때 이미 달리는 중이면 그 속도에서 이어감 (0 부터 가속하면 한 번 멈칫함)
      v_f_ = have_odom_ ? std::max(0.0, odom_.twist.twist.linear.x) : 0.0;
    }
    if (!driving() && was_driving_ && xturn_on_) {
      xturn_.cancelManeuver();
      logXturn();
      x_active_ = false;
    }
    was_driving_ = driving();

    // 프레임 사이 바퀴 이동 (실제 mm, 왼쪽 + rad)
    if (have_odom_) {
      const auto & q = odom_.pose.pose;
      const double yaw = yawOf(q.orientation);
      if (have_prev_odom_) {
        const double dx = q.position.x - prev_x_, dy = q.position.y - prev_y_;
        const double ds = dx * std::cos(prev_yaw_) + dy * std::sin(prev_yaw_);
        planner_.onMove(ds * 1000.0, std::remainder(yaw - prev_yaw_, 2 * M_PI), p_);
        if (exit_armed_) {
          exit_acc_ += std::remainder(yaw - prev_yaw_, 2 * M_PI);   // + 왼쪽
          exit_lo_ = std::min(exit_lo_, exit_acc_);
          exit_hi_ = std::max(exit_hi_, exit_acc_);
          // U턴 (표지판 반대쪽) 으로 돈 양: 오른쪽 표지판 = 가장 오른쪽으로 꺾였던 곳에서 왼쪽으로 올라온 만큼
          const double prog = exit_dir_ == grid_drive::Turn::RIGHT ? exit_acc_ - exit_lo_ : exit_hi_ - exit_acc_;
          if (prog >= turn_exit_yaw_) {
            exit_armed_ = false;
            turn_ = exit_dir_;
            turn_until_ = now() + rclcpp::Duration::from_seconds(turn_exit_hold_);
            RCLCPP_INFO(get_logger(), "교차로 U턴 %.0f도 -> 출구, %s 쪽 %.1f초 다시 우선", prog * 180.0 / M_PI,
              exit_dir_ == grid_drive::Turn::RIGHT ? "오른" : "왼", turn_exit_hold_);
          } else if (now() > exit_deadline_) {
            exit_armed_ = false;
          }
        }
        if (xturn_on_) {   // lane_follower 튜닝 단위 (cmd_vel) 로
          xturn_.onMove(ds * cmd_scale_, std::remainder(yaw - prev_yaw_, 2 * M_PI) * cmd_scale_, xp_);
          logXturn();
        }
      }
      prev_x_ = q.position.x;
      prev_y_ = q.position.y;
      prev_yaw_ = yaw;
      have_prev_odom_ = true;
    }

    const auto t0 = std::chrono::steady_clock::now();
    double ms_road = 0;
    const grid_drive::RoadSeg rs = road(img, ms_road);
    grid_drive::FloorObs obs = grid_drive::observeColor(img, p_, colors_);
    if (turn_ != grid_drive::Turn::NONE && now() > turn_until_) {
      turn_ = grid_drive::Turn::NONE;
    }
    const bool in_turn = turn_ != grid_drive::Turn::NONE;
    // 공사 구간: 박스가 보이는 동안은 끝나지 않게 연장
    bool in_construction = now() < construction_until_;
    if (in_construction && !obs.box.empty() &&
      cv::countNonZero(obs.box) > construction_box_min_ * obs.box.total())
    {
      const auto until = now() + rclcpp::Duration::from_seconds(construction_box_hold_);
      construction_until_ = std::max(construction_until_, until);
    }
    in_construction = now() < construction_until_;
    if (!rs.mask.empty()) {
      grid_drive::SegMasks sm;
      sm.road = rs.mask;
      grid_drive::merge(obs, grid_drive::observeSegmentation(sm, p_));
    }
    // 모든 계획은 도로 모델 기준: 도로 모델이 주행 가능 구역을 하나도 못 찾으면
    // 색 (선) 만으로 계획하지 않고 경로 없음 -> 정지. 도로가 지도 칸 밖에만 있으면 지도 전체가 주행 불가 -> 경로 없음
    const bool no_road = haveRoad() && (rs.mask.empty() || rs.detections == 0 || obs.offroad.empty());
    grid_drive::Params pp = p_;
    if (in_turn || in_construction) {
      pp.far_line_cost = 0.0;   // 교차로 / 공사 구간은 넓은 바닥을 가로지름
    }
    if (in_turn) {
      const bool entry = turn_line_margin_sec_ <= 0 || (now() - turn_sign_t_).seconds() < turn_line_margin_sec_;
      pp.line_margin_mm = !entry ? turn_after_margin_ :
        turn_ == grid_drive::Turn::LEFT ? turn_line_margin_left_ : turn_line_margin_;
      if (turn_memory_frames_ > 0) {
        pp.memory_max_frames = turn_memory_frames_;
      }
      if (turn_min_radius_ > 0) {
        pp.max_curvature = 1.0 / turn_min_radius_;
      }
      if (turn_change_weight_ > 0) {
        pp.change_weight = turn_change_weight_;
      }
      pp.center_cost = turn_center_cost_;
    }
    if (in_construction && !in_turn) {
      if (construction_max_angular_ > 0) {
        pp.max_angular = construction_max_angular_;
      }
      if (construction_change_weight_ > 0) {
        pp.change_weight = construction_change_weight_;
      }
    }
    if ((in_turn && turn_memory_) || (in_construction && construction_memory_)) {
      pp.memory = true;
      // 공사 구간: 박스만 기억 (선은 계속 보여서 기억할 필요가 적고, 어긋난 선 기억이 경로를 뒤집음)
      pp.memory_lines = in_turn || p_.memory_lines;
      if (in_construction && !in_turn) {
        pp.memory_lines = construction_memory_lines_;
      }
    }
    // 비전이 닫힌 차단바를 barrier_hold_sec 안에 봄: 막다른 길을 차단바로 (색으로는 흰 줄무늬가 선으로 잡힘)
    const bool bar_near = (now() - bar_closed_t_).seconds() < barrier_hold_;
    pp.dead_end_force_box = bar_near;
    grid_drive::Result r;
    if (no_road) {
      r.reason = "주행 가능 구역 없음";
      planner_.reset();
    } else {
      r = planner_.plan(std::move(obs), pp, turn_);
    }
    const double ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();

    if (no_road) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "주행 가능 구역이 안 보임 -> 정지");
    }
    // 막다른 길: 막는 것까지 거리를 잴 때마다 기억 (odom 위치와 함께)
    const double odom_x = odom_.pose.pose.position.x, odom_y = odom_.pose.pose.position.y;
    // 차단바 (박스) 는 공중에 뜬 막대라 바닥으로 편 지도에서 실제보다 멀게 재짐 -> 남은 거리 직진은 코스 끝 (선 / 도로 끝) 만
    if (r.dead_end && r.path_ok && !r.dead_end_box) {
      end_seen_ = true;
      end_left_mm_ = r.dead_end_mm - r.dead_end_target_mm;
      end_x_ = odom_x;
      end_y_ = odom_y;
      end_t_ = now();
    } else if (r.path_ok || (r.dead_end && !r.path_ok)) {
      end_seen_ = false;   // 보통 경로 / 정지 거리에 이미 옴
    }
    // 차단바 앞에서 멈췄으면 차단바가 barrier_hold_sec 동안 안 보일 때까지 (올라감) 계속 정지
    if (r.dead_end && r.dead_end_box && !r.path_ok && bar_near) {
      bar_stopped_ = true;
    }
    if (!bar_near || !driving()) {
      bar_stopped_ = false;
    }
    if (bar_stopped_) {
      r.v = 0;
      r.w = 0;
      r.reason = "차단바 앞 정지 (올라가길 기다림)";
      end_seen_ = false;
    }
    // 막는 것이 화면 아래로 빠짐 (목표 없음, 코스 끝이면 도로도 안 보임): 마지막으로 잰 거리에서 남은 만큼 직진
    double creep_left = 0;
    if (end_seen_ && !r.path_ok && have_odom_ && (now() - end_t_).seconds() < 3.0) {
      creep_left = end_left_mm_ - 1000.0 * std::hypot(odom_x - end_x_, odom_y - end_y_);
      if (creep_left <= 0) {
        end_seen_ = false;
      }
    }
    const bool creep = driving() && !bar_stopped_ && creep_left > 0 && creep_linear_ > 0;
    if (creep) {
      r.v = creep_linear_;
      r.w = 0;
      r.reason = "막는 것 앞까지 남은 거리 직진";
    }

    // 도로 / 경로를 잠깐 (road_hold_sec 이하) 놓치면 직전 명령을 계속 보냄 (U턴 중 1~2 프레임 놓침).
    // 막다른 길 정지 (차단바 앞) 는 바로 멈춤
    // 원호가 전부 막혀 제자리 회전 (pivot) 도 한두 프레임이면 직전 명령 유지 (drive26: 교차로 진입 왼쪽 회전 중 한 프레임
    //   pivot -> 속도 0.27 -> 0.14 로 멈칫). road_hold_sec 넘게 계속되면 그때 제자리 회전
    const bool lost = !creep && (no_road || (!r.path_ok && !r.dead_end) || r.pivot);
    if (!lost) {
      last_ok_ = now();
    }
    const bool hold = driving() && !bar_stopped_ && lost && sent_moving_ && (now() - last_ok_).seconds() <= road_hold_sec_;
    if (xturn_on_ && driving() && !bar_stopped_ && intersectionDrive()) {
      // 교차로: 위에서 보냄
    } else if (hold) {
      publishCmd(v_f_, w_f_);
      stop_sent_ = false;
    } else if (creep) {
      v_f_ = creep_linear_;
      w_f_ = 0;
      w_hist_.clear();
      publishCmd(v_f_, w_f_);
      stop_sent_ = false;
    } else if (driving() && !no_road && !bar_stopped_) {
      double v = r.v, w = r.w;
      // 교차로 (좌/우 기억 중) 에서는 turn_linear 까지 감속 (꺾을 시간 확보). 곡률은 그대로
      if (turn_ != grid_drive::Turn::NONE && turn_linear_ > 0 && v > turn_linear_) {
        w *= turn_linear_ / v;
        v = turn_linear_;
      }
      cornerDelay(r, w);
      w = medianW(w);
      const double v_prev = v_f_;
      v_f_ += smooth_ * (v - v_f_);
      w_f_ += smooth_ * (w - w_f_);
      if (accel_limit_ > 0 && v_f_ > v_prev) {
        const double dt = std::clamp((now() - last_cmd_t_).seconds(), 0.0, 0.1);
        v_f_ = std::min(v_f_, v_prev + accel_limit_ * dt);
      }
      last_cmd_t_ = now();
      publishCmd(v_f_, w_f_);
      stop_sent_ = false;
    } else {
      v_f_ = 0;
      w_f_ = 0;
      w_hist_.clear();
      if (!stop_sent_) {
        publishCmd(0, 0);
        stop_sent_ = true;
      }
    }
    if (!r.path_ok || r.pivot) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "%s: %s",
        r.pivot ? "제자리 회전" : "경로 없음", r.reason.c_str());
    }
    RCLCPP_DEBUG(get_logger(), "v %.3f w %.3f soft %d road %d (%.1fms) 전체 %.1fms",
      r.v, r.w, r.soft, rs.detections, ms_road, ms);

    if (debug_pub_->get_subscription_count() > 0) {
      publishDebug(img, rs, r, ms_road, ms, msg->header, no_road);
    }
  }

  // 최근 w_median_ 개 중 가운데 값 (한 프레임만 반대로 튀는 경로 선택을 무시, 지연 1 프레임)
  double medianW(double w)
  {
    w_hist_.push_back(w);
    while (static_cast<int64_t>(w_hist_.size()) > w_median_) {
      w_hist_.pop_front();
    }
    std::vector<double> s(w_hist_.begin(), w_hist_.end());
    std::nth_element(s.begin(), s.begin() + s.size() / 2, s.end());
    return s[s.size() / 2];
  }

  // 왼쪽: 원본 + 도로 마스크 (초록), 오른쪽: 계획 그림 (GridPlanner::draw)
  void publishDebug(const cv::Mat & img, const grid_drive::RoadSeg & rs, const grid_drive::Result & r,
    double ms_road, double ms, const std_msgs::msg::Header & header, bool no_road)
  {
    cv::Mat left = img.clone();
    if (!rs.mask.empty()) {
      cv::Mat tint(img.size(), CV_8UC3, cv::Scalar(0, 200, 0));
      cv::Mat mixed;
      cv::addWeighted(img, 0.6, tint, 0.4, 0, mixed);
      mixed.copyTo(left, rs.mask);
    }
    const char * turn = turn_ == grid_drive::Turn::LEFT ? "L" : turn_ == grid_drive::Turn::RIGHT ? "R" :
      now() < construction_until_ ? "C" : "-";
    char text[160];
    std::snprintf(text, sizeof(text), "%s v%.2f w%.2f %s%s turn %s road %d %.2f  %.0f/%.0fms",
      !driving() ? "STOP" : bar_stopped_ ? "BAR WAIT" : no_road ? "NO ROAD" : "RUN", r.v, r.w, r.dead_end ? "END " : r.soft ? "SOFT " : "", r.pivot ? "PIVOT " : "", turn,
      rs.detections, rs.best_conf, ms_road, ms);
    cv::putText(left, text, {6, 20}, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2);
    if (xturn_on_ && xturn_.phase() != lane_follower::IntersectionTurn::Phase::IDLE) {
      // 교차로 상태 (이 동안 명령은 sliding 차선 추종 / 회전, 오른쪽 격자 그림은 참고용)
      const std::string xs = "X " + xturn_.describe() + (lane_.detected ? "  lane o" : "  lane x");
      cv::putText(left, xs, {6, 44}, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 2);
    }

    cv::Mat right, out;
    if (no_road) {   // 계획 안 함: 지도 칸은 비우고 표시만
      right = cv::Mat(img.rows, img.rows, CV_8UC3, cv::Scalar(40, 25, 25));
      cv::putText(right, "NO ROAD", {right.cols / 2 - 70, right.rows / 2}, cv::FONT_HERSHEY_SIMPLEX, 1.0,
        cv::Scalar(0, 0, 255), 2);
    } else {
      right = planner_.draw(r, p_);
    }
    cv::resize(right, right, {right.cols * img.rows / right.rows, img.rows});
    cv::hconcat(left, right, out);

    sensor_msgs::msg::CompressedImage jpeg;
    jpeg.header = header;
    jpeg.format = "jpeg";
    cv::imencode(".jpg", out, jpeg.data, {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_});
    debug_pub_->publish(jpeg);
  }

  grid_drive::Params p_;
  obstacle_detection::ColorParams colors_;
  grid_drive::GridPlanner planner_;
  grid_drive::RoadSegParams seg_;
#ifdef GRID_DRIVE_TENSORRT
  std::unique_ptr<grid_drive::TrtEngine> trt_;
#endif
  bool use_road_ = true;
  uint8_t mode_ = ControlMode::STOP;
  double cmd_scale_ = 1.0, process_rate_ = 0, image_timeout_ = 0.5, turn_hold_ = 6.0, sign_conf_ = 0.6;
  int jpeg_quality_ = 70;
  double smooth_ = 0.3, v_f_ = 0, w_f_ = 0;   // 명령 필터 (cmd_smooth)
  double corner_delay_ = 1.5, corner_k_ = 2.0, corner_rearm_ = 0.5;   // 코너 진입 지연
  double turn_linear_ = 0.12;   // 교차로 속도 [m/s 실제]
  bool turn_memory_ = true;
  double construction_hold_ = 20.0, construction_box_hold_ = 3.0, construction_box_min_ = 0.002;
  rclcpp::Time construction_until_{0, 0, RCL_ROS_TIME};
  int64_t w_median_ = 3;
  double accel_limit_ = 0.3;
  rclcpp::Time last_cmd_t_{0, 0, RCL_ROS_TIME};
  bool construction_memory_ = true, construction_memory_lines_ = false;
  double construction_max_angular_ = 0, construction_change_weight_ = 0;
  double turn_line_margin_ = 10.0, turn_line_margin_sec_ = 0.0, turn_after_margin_ = 10.0;
  double turn_line_margin_left_ = 10.0;
  int turn_memory_frames_ = 0;
  double turn_exit_yaw_ = 0, turn_exit_hold_ = 3.0, turn_exit_max_ = 20.0;
  bool exit_armed_ = false;
  grid_drive::Turn exit_dir_ = grid_drive::Turn::NONE;
  double exit_acc_ = 0, exit_lo_ = 0, exit_hi_ = 0;
  rclcpp::Time exit_deadline_{0, 0, RCL_ROS_TIME};
  double turn_min_radius_ = 0, turn_change_weight_ = 0, turn_center_cost_ = 0;
  rclcpp::Time turn_sign_t_{0, 0, RCL_ROS_TIME};
  double road_hold_sec_ = 0.15;
  double creep_linear_ = 0.05;
  bool end_seen_ = false;
  double end_left_mm_ = 0, end_x_ = 0, end_y_ = 0;
  rclcpp::Time end_t_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_ok_{0, 0, RCL_ROS_TIME};
  std::deque<double> w_hist_;
  bool corner_armed_ = true;
  double corner_until_ = 0, last_curve_t_ = 0;
  double turn_flip_delay_ = 0;
  int last_curve_sign_ = 0;
  // 교차로 sliding 방식 (intersection_lane)
  bool xturn_on_ = false, x_active_ = false, have_x_tick_ = false, lane_rx_ = false;
  bool have_lane_cmd_ = false, lane_ok_prev_ = false;
  lane_follower::IntersectionTurn xturn_;
  lane_follower::TurnParams xp_;
  lane_follower::Gains gains_;
  lane_follower::Controller lane_ctl_;
  interfaces::msg::LaneInfo lane_;
  rclcpp::Time lane_stamp_{0, 0, RCL_ROS_TIME}, x_lane_used_{0, 0, RCL_ROS_TIME}, x_tick_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_lane_ok_{0, 0, RCL_ROS_TIME};
  double lane_timeout_ = 0.7, lane_lost_hold_ = 0.5, lane_v_ = 0, lane_w_ = 0;
  rclcpp::Subscription<interfaces::msg::LaneInfo>::SharedPtr lane_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr cross_sub_;   // 좌/우 기억 중 마지막 곡선 방향 (+1 오른쪽, -1 왼쪽, 0 없음)
  grid_drive::Turn turn_ = grid_drive::Turn::NONE;
  rclcpp::Time turn_until_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_stamp_, last_image_{0, 0, RCL_ROS_TIME};
  bool have_stamp_ = false, was_driving_ = false, sent_moving_ = false, stop_sent_ = false;
  nav_msgs::msg::Odometry odom_;
  bool have_odom_ = false, have_prev_odom_ = false;
  double prev_x_ = 0, prev_y_ = 0, prev_yaw_ = 0;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<ControlMode>::SharedPtr mode_sub_;
  rclcpp::Subscription<Sign>::SharedPtr sign_sub_;
  rclcpp::Subscription<Barrier>::SharedPtr barrier_sub_;
  double barrier_hold_ = 1.5;
  rclcpp::Time bar_closed_t_{0, 0, RCL_ROS_TIME};
  bool bar_stopped_ = false;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GridDriveNode>());
  rclcpp::shutdown();
  return 0;
}
