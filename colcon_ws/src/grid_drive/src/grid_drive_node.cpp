// 격자 계획 주행: 카메라 -> 바닥 지도 (색 + 도로 세그멘테이션) -> grid_planner -> cmd_vel
//   구독: image_raw (sensor_msgs/Image)              v4l2_camera 640x480
//         odom (nav_msgs/Odometry)                   stm_bridge, 프레임 사이 이동량 -> planner.onMove
//         control_mode (interfaces/ControlMode)      LANE / AVOID 일 때만 주행
//         sign (interfaces/Sign)                     LEFT / RIGHT 를 보면 turn_hold_sec 동안 그쪽 목표 가점
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
    // 시작할 때만 읽는 파라미터 (구독 / 엔진 / 첫 모드). 설명의 "재시작" 을 보고 turtle_gui 가 ↻ 표시
    rcl_interfaces::msg::ParameterDescriptor restart;
    restart.description = "재시작해야 반영 (시작할 때만 읽음)";
    const auto image_topic = declare_parameter("image_topic", std::string("image_raw"), restart);
    const auto engine_path = expandHome(
      declare_parameter("engine_path", std::string("~/yolo26_check/road_r2_320_fp16.engine"), restart));
    use_road_ = declare_parameter("use_road", true, restart);
    mode_ = declare_parameter("autostart", false, restart) ? ControlMode::LANE : ControlMode::STOP;
    loadParams();
    // 나머지 파라미터는 실행 중에 바꾸면 (turtle_gui 파라미터 탭, ros2 param set) loadParams 로 다시 읽어 바로 반영.
    //   spin 이 한 스레드라 영상 처리 도중에 값이 바뀌지는 않는다
    rcl_interfaces::msg::ParameterDescriptor ro;
    ro.read_only = true;
    ro.description = "turtle_gui 표시용: 파라미터를 바꾸면 바로 반영되는 노드";
    declare_parameter("live_params", true, ro);
    param_cb_ = add_post_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & ps) {
        loadParams();
        for (const auto & q : ps) {
          RCLCPP_INFO(get_logger(), "파라미터 %s = %s", q.get_name().c_str(), q.value_to_string().c_str());
        }
      });

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
    // 비전 차단바 인식 (sign_detection): 닫힌 차단바를 본 시각 (barrier_hold_sec 동안 기억)
    barrier_sub_ = create_subscription<Barrier>(
      "barrier", rclcpp::SensorDataQoS(), [this](Barrier::ConstSharedPtr m) {
        // 열림을 본 뒤 3초는 닫힘을 무시 (올라가는 중 닫힘으로 한두 번 보여 다시 서지 않게)
        const bool after_open = bar_open_t_.nanoseconds() > 0 && (now() - bar_open_t_).seconds() < 3.0;
        if (m->detected && m->state != Barrier::OPEN && m->confidence >= 0.5 && !after_open) {
          bar_closed_t_ = now();
          bar_y_ = m->y_ratio;
        } else if (m->detected && m->state == Barrier::OPEN && m->confidence >= barrier_open_conf_ &&
          bar_closed_t_.nanoseconds() > 0)
        {
          // 열린 차단바를 보면 barrier_hold_sec 를 기다리지 않고 바로 출발 (열린 뒤 한참 서 있었음, 2026-10-29)
          bar_closed_t_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
          bar_open_t_ = now();
          RCLCPP_INFO(get_logger(), "차단바 열림 (%.2f) -> 출발", m->confidence);
        }
      });
    sign_sub_ = create_subscription<Sign>(
      "sign", rclcpp::SensorDataQoS(), [this](Sign::ConstSharedPtr m) {
        if (m->type == Sign::CONSTRUCTION && m->confidence >= sign_conf_) {
          if (now() > construction_until_) {
            RCLCPP_INFO(get_logger(), "공사 구간 표지판 -> 공사 모드 %.0fs", construction_hold_);
          }
          // 교차로는 공사 구간 앞이라 아직 출구를 기다리면 잊음 (drive32 66s: 박스 사이 회전을 U턴으로 세어
          //   공사 구간 한가운데서 오른쪽 기억이 켜짐)
          if (exit_armed_) {
            exit_armed_ = false;
            RCLCPP_INFO(get_logger(), "공사 구간 -> 교차로 출구 기다리기 취소");
          }
          construction_until_ = now() + rclcpp::Duration::from_seconds(construction_hold_);
        }
        if ((m->type == Sign::LEFT || m->type == Sign::RIGHT) && m->confidence >= sign_conf_) {
          const auto dir = m->type == Sign::LEFT ? grid_drive::Turn::LEFT : grid_drive::Turn::RIGHT;
          if (exit_armed_ && dir != exit_dir_) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "교차로 출구 기다리는 중 반대 표지판 무시 (%.2f)",
              m->confidence);
            return;
          }
          if (turn_ != dir) {
            RCLCPP_INFO(get_logger(), "%s 표지판 (%.2f) -> 그쪽 우선 %.1fs, 선 여유 %.0fmm %.1fs",
              dir == grid_drive::Turn::LEFT ? "왼쪽" : "오른쪽", m->confidence, turn_hold_,
              dir == grid_drive::Turn::LEFT ? turn_line_margin_left_ : turn_line_margin_, turn_line_margin_sec_);
          }
          turn_ = dir;
          turn_until_ = now() + rclcpp::Duration::from_seconds(turn_hold_);
          turn_sign_t_ = now();
          turn_sign_dir_ = dir;
          // 출구 기다림: 처음 보거나 방향이 바뀌면 회전량을 새로 셈 (계속 보이는 / 섬 안에서 다시 본 같은 표지판은 이어 셈)
          if (turn_exit_yaw_ > 0 && (!exit_armed_ || exit_dir_ != turn_)) {
            exit_armed_ = true;
            exit_entered_ = false;
            entry_side_seen_ = false;
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
  // 처음이면 선언, 이미 있으면 지금 값 (실행 중 바뀐 값) 을 읽는다
  template<typename T>
  T param(const std::string & name, const T & def)
  {
    if (!has_parameter(name)) {
      return declare_parameter(name, def);
    }
    return get_parameter(name).get_value<T>();
  }

  // 실행 중에 바꿀 수 있는 파라미터 전부. 생성자에서 한 번, 파라미터가 바뀔 때마다 다시
  void loadParams()
  {
    seg_.conf = static_cast<float>(param("road_conf", 0.5));
    seg_.mask_thr = static_cast<float>(param("road_mask_thr", 0.5));
    cmd_scale_ = param("cmd_scale", 0.408);
    process_rate_ = param("process_rate", 15.0);
    image_timeout_ = param("image_timeout", 0.5);
    turn_hold_ = param("turn_hold_sec", 6.0);
    sign_conf_ = param("sign_min_conf", 0.6);
    jpeg_quality_ = static_cast<int>(param("debug_jpeg_quality", 70));
    // 명령 부드럽게: 매번 새 값 쪽으로 이 비율만큼만 다가감 (1 = 필터 없음, 작을수록 부드럽지만 늦음)
    smooth_ = param("cmd_smooth", 0.3);
    // 코너 진입: 곡선이 처음 보이면 corner_delay_sec 동안 직진한 뒤 계획대로 회전
    corner_delay_ = param("corner_delay_sec", 1.5);
    corner_k_ = param("corner_k_min", 2.0);          // [1/m] 이보다 급한 곡선 = 코너 (2.0 = 반지름 50cm)
    corner_rearm_ = param("corner_rearm_sec", 0.5);  // 곡선이 이만큼 안 보이면 다음 코너 준비
    // 좌/우 기억 중 곡선 방향이 바뀌면 (진입 좌회전 -> U턴 우회전 등) 이 시간 [s] 직진한 뒤 회전 (0 = 끔).
    //   U턴을 너무 일찍 시작해 안쪽 고리 선을 밟았다 (2026-10-28 drive21)
    turn_flip_delay_ = param("turn_flip_delay_sec", 0.0);
    // 교차로: 좌/우 표지판 기억 중에는 turn_linear 까지 감속 (0 = 감속 안 함)
    // 도로 모델이 있으면 주행 가능 구역 안에서만 계획 (구역이 안 보이면 계획 안 하고 정지)
    p_.road_only = param("road_only", true);
    road_hold_sec_ = param("road_hold_sec", 0.15);
    // 막다른 길: 막는 것이 화면 아래로 빠져 안 보여도, 마지막으로 잰 거리에서 정지 거리까지 남은 만큼 odom 으로 천천히 직진
    creep_linear_ = param("dead_end_creep_linear", 0.05);
    // 모터 반응 지연 [s]: 감속 없이 서므로 코스 끝 정지 거리를 (속도 x 이 시간) 만큼 앞당김 (drive30: 명령 -> odom 0.2~0.3s)
    dead_end_lag_ = param("dead_end_lag_sec", 0.25);
    turn_linear_ = param("turn_linear", 0.12);
    // 교차로에서 꺾을 때: 안쪽 선이 화면 아래로 빠지면 지도에서 사라져 안쪽으로 파고듦 -> odom 기억 켬
    turn_memory_ = param("turn_memory", true);
    // 공사 구간: 표지판을 본 뒤 construction_hold_sec 동안 (박스가 보이면 계속 연장)
    //   차선 폭 지키기 끔,
    //   odom 기억 켬 (벽 옆면이 화면 밖으로 빠져도 기억해서 덜 돌고 박는 것 막음)
    construction_hold_ = param("construction_hold_sec", 20.0);
    construction_box_hold_ = param("construction_box_hold_sec", 3.0);
    construction_box_min_ = param("construction_box_min_ratio", 0.002);
    construction_memory_ = param("construction_memory", true);
    construction_memory_lines_ = param("construction_memory_lines", false);
    // 공사 모드 회전: 최대 회전 속도 (0 = planner.max_angular 그대로), 곡률 바꿈 벌점 (0 = planner.change_weight 그대로)
    //   박스 사이에서 크게 꺾고 되꺾는 지그재그 줄이기. 속도는 그대로
    construction_max_angular_ = param("construction_max_angular", 0.0);
    construction_change_weight_ = param("construction_change_weight", 0.0);
    // 공사 모드 속도 [m/s 실제] (0 = 감속 안 함). 직선 0.50 으로 올린 뒤 박스 사이를 너무 빨리 지나감 (2026-10-29 drive29)
    construction_linear_ = param("construction_linear", 0.0);
    // 공사 구간에서 원호가 전부 막힘: 제자리 회전 대신 이 속도 [m/s 실제] 로 가며 최대 회전 (0 = 제자리 회전).
    //   박스가 처음 보일 때 몇 프레임 막혀 0.49 -> 0.12 로 멈칫했음 (2026-10-29 drive34 재생)
    construction_pivot_linear_ = param("construction_pivot_linear", 0.0);
    // 공사 구간 가속 제한 [m/s^2 실제] (음수 = accel_limit). 멈칫 뒤 다시 빨라지는 데 1초 넘게 걸렸음
    construction_accel_limit_ = param("construction_accel_limit", -1.0);
    // 지연 보정 [s]: 카메라 ~ 바퀴 반응까지. 원호를 지금 명령으로 이 시간 더 간 자리에서 고름 (0 = 끔).
    //   drive30: 명령 -> odom 회전 0.2~0.3s 늦음, 0.50 m/s 직선에서 +-0.6 rad/s 출렁임 + 코너 안쪽 밟음
    lead_sec_ = param("lead_sec", 0.0);
    p_.memory_max_frames = static_cast<int>(param("planner.memory_max_frames", 15));
    p_.box_memory_max_frames = static_cast<int>(param("planner.box_memory_max_frames", -1));
    // 한 프레임만 튀는 회전 명령 거르기: 최근 w_median 프레임의 가운데 값 (1 = 끔)
    w_median_ = std::max<int64_t>(1, param("w_median", 3));
    // 출발 / 가속 제한 [m/s^2 실제]: 정지에서 급출발하면 앞이 들림. 감속은 제한 안 함
    accel_limit_ = param("accel_limit", 0.3);

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
      {"soft_depth_cost", &p_.soft_depth_cost},
      {"track_len_mm", &p_.track_len_mm}, {"track_preview_mm", &p_.track_preview_mm},
      {"track_outer_mm", &p_.track_outer_mm}, {"track_bend_mm", &p_.track_bend_mm},
      {"track_outer_fit", &p_.track_outer_fit}, {"track_slow_ahead", &p_.track_slow_ahead},
      {"track_block_ok_mm", &p_.track_block_ok_mm},
      {"soft_line_block_mm", &p_.soft_line_block_mm}, {"soft_line_keep_mm", &p_.soft_line_keep_mm},
    };
    for (const auto & [name, ptr] : dparams) {
      *ptr = param(std::string("planner.") + name, *ptr);
    }
    p_.memory = param("planner.memory", p_.memory);
    // 좌/우 기억 중 선 여유 [mm] (planner.line_margin_mm 대신). 교차로 진입 때 고리 선과 바깥 선의 빨간 띠가
    // 붙어 표지판 쪽 길이 막히면 낮추기
    turn_line_margin_ = param("turn_line_margin_mm", p_.line_margin_mm);
    // 왼쪽 표지판 진입만 따로 (오른쪽은 띠가 길을 막아 0 이 필요했고, 왼쪽은 0 이면 왼쪽 코너 안쪽 선을 밟음)
    turn_line_margin_left_ = param("turn_line_margin_left_mm", turn_line_margin_);
    // 위 여유는 좌/우 표지판을 마지막으로 본 뒤 이 시간 [s] 동안만 (진입). 그 뒤 (U턴 등) 는 planner.line_margin_mm
    //   (안쪽 선 밟음 막음). 0 = 좌/우 기억 내내
    turn_line_margin_sec_ = param("turn_line_margin_sec", 0.0);
    // 진입 뒤 (위 시간 지난 뒤) 좌/우 기억 중 선 여유 [mm]. 크면 U턴을 크게 돌아 안쪽 선에서 멀어짐
    turn_after_margin_ = param("turn_after_margin_mm", p_.line_margin_mm);
    // true: 위 진입 여유는 표지판 쪽으로 turn_exit_entry_deg 만큼 들어가면 바로 끝 (그 뒤 U턴은 turn_after_margin_mm)
    turn_entry_margin_until_entered_ = param("turn_entry_margin_until_entered", true);
    // 교차로 출구: 길은 표지판 쪽으로 들어가 고리를 반대쪽으로 U턴한 뒤 다시 표지판 쪽으로 나감 (drive24 odom:
    //   오른쪽 -70도 -> 고리 왼쪽 +155도 -> 출구 오른쪽). 표지판을 본 뒤 (가장 표지판 쪽으로 꺾인 곳부터) 반대쪽으로
    //   turn_exit_yaw_deg [odom 도] 돌면 U턴 끝 = 출구 앞으로 보고 좌/우 기억을 turn_exit_hold_sec 동안 다시 켬. 0 = 끔.
    //   출구를 기다리는 동안 반대 방향 표지판은 무시 (drive24: 출구에서 섬 안 표지판을 LEFT 0.61 로 한 번 보고 밖으로 나감)
    //   turn_hold_sec 를 짧게 (진입만) 하자 출구에서 직진해 오른쪽 출구를 지나쳤다 (2026-10-28)
    turn_exit_yaw_ = param("turn_exit_yaw_deg", 0.0) * M_PI / 180.0;
    turn_exit_hold_ = param("turn_exit_hold_sec", 3.0);
    // 표지판 쪽으로 turn_exit_entry_deg 꺾어 들어갈 때까지 좌/우 기억을 이 시간 [s] 까지 유지 (0 = turn_hold_sec 만)
    turn_entry_max_ = param("turn_entry_max_sec", 0.0);
    // 진입 고정: 표지판 쪽으로 들어가기 전 (위 대기 중) 표지판 쪽 길을 한 번 찾으면 그 뒤로는 반대쪽 목표를 안 고름.
    //   표지판 쪽 길이 안 보이는 프레임은 turn_entry_seek_linear [m/s 실제] 로 가며 표지판 쪽으로 turn_entry_seek_angular
    //   [rad/s 실제] 로 꺾음 (drive37 59s: 섬 정면으로 와서 섬 왼쪽 / 오른쪽 목표가 프레임마다 바뀌어 흔들리다 왼쪽으로 돎)
    turn_entry_commit_ = param("turn_entry_commit", true);
    turn_entry_seek_linear_ = param("turn_entry_seek_linear", 0.08);
    turn_entry_seek_angular_ = param("turn_entry_seek_angular", 1.2);
    // 들어감 판정 뒤에도 이 시간 [s] 동안은 길이 안 보이면 같은 방식으로 표지판 쪽으로 꺾으며 천천히 (10-30: 왼쪽
    //   25도 들어감 직후 경로 없음 -> 그 자리에 섬). 0 = 예전 (들어가면 바로 끝)
    turn_entry_after_seek_sec_ = param("turn_entry_after_seek_sec", 0.0);
    // 교차로 (좌/우 기억 / U턴) 중 planner.soft_line_block_mm 대신 이 값 (음수 = planner 값, 0 = 교차로에선 끔).
    //   10-30 drive43: soft 선 막기 뒤로 교차로 진입 / 출구에서 "목표 없음" 이 거의 매번 나 멈춤 (drive42 까진 없었음)
    turn_soft_line_block_ = param("turn_soft_line_block_mm", -1.0);
    // 왼쪽 표지판: 표지판을 본 순간부터 반대쪽 (오른쪽) 목표를 안 고르고 (길이 없으면 위처럼 왼쪽으로 꺾으며 천천히),
    //   진입 각도 (turn_exit_entry_deg) 는 왼쪽 길을 찾은 뒤부터 셈. 교차로 바로 앞 왼쪽 코너 끝에서 표지판을 보면
    //   그 코너 회전이 진입으로 세어져 진입 고정이 풀리고, 경로가 섬 오른쪽 / 표지판 앞으로 직진했다
    //   (2026-10-29 drive38 34.7s / 117.6s, 10-30 02:3x 3번). 오른쪽 표지판은 그 코너에서 왼쪽 목표가 필요해 안 함
    turn_entry_left_early_ = param("turn_entry_left_early", false);
    turn_exit_max_ = param("turn_exit_max_sec", 20.0);   // 표지판 뒤 이 시간 안에 출구가 없으면 잊음
    // 표지판 쪽으로 이만큼 [odom 도] 꺾어 들어간 뒤부터 U턴 양을 셈 (0 = 바로). 교차로 앞 왼쪽 코너를 도는 중에
    //   오른쪽 표지판을 보면 그 코너 회전이 U턴으로 세어져 진입하자마자 출구로 봤다 (2026-10-29 drive28 60.5s)
    turn_exit_entry_ = param("turn_exit_entry_deg", 0.0) * M_PI / 180.0;
    // 출구에서 표지판 쪽으로 이만큼 [odom 도] 돌면 turn_exit_hold_sec 전이라도 좌/우 기억 끝 (0 = 시간만).
    //   속도를 올리자 1.5초 동안 계속 꺾어 출구 바깥 노란 선을 넘었다 (drive28 146s)
    turn_exit_turn_ = param("turn_exit_turn_deg", 0.0) * M_PI / 180.0;
    // 좌/우 기억 중 선 기억 프레임 수 (0 이하 = planner.memory_max_frames). U턴 중 안쪽 고리 선이 화면 오른쪽 아래로
    //   빠진 뒤 0.5초 만에 잊어서 그 위로 경로를 냈다 (2026-10-28 drive19)
    turn_memory_frames_ = static_cast<int>(param("turn_memory_max_frames", 0));
    // 좌/우 기억 중 가장 급한 원호 반지름 [mm 실제] (0 = 기본 150). 교차로 U턴이 150 으로 못 돌아 바깥 선을 넘었다
    turn_min_radius_ = param("turn_min_radius_mm", 0.0);
    // 좌/우 기억 중 곡률 바꿈 벌점 (0 = planner.change_weight). 진입 좌회전 -> U턴 우회전으로 빨리 바꾸게 낮춤
    turn_change_weight_ = param("turn_change_weight", 0.0);
    // 좌/우 기억 중 가운데 따라가기 비용 (planner.center_cost, 0 = 끔) 과 가운데를 찾는 반경 [mm]
    turn_center_cost_ = param("turn_center_cost", 0.0);
    p_.center_radius_mm = param("turn_center_radius_mm", p_.center_radius_mm);
    barrier_hold_ = param("barrier_hold_sec", 1.5);
    // 닫힌 차단바 앞에서 경로가 끊기면 (빨간 줄무늬가 박스로 잡혀 멀리서 막힘) 이만큼 [mm] 더 직진한 뒤 정지 (0 = 끔).
    //   차단바 화면 세로 위치 (y_ratio) 가 barrier_stop_y 이상이면 그 전에 정지 (2026-10-29 drive32: 멀리서 멈춤)
    barrier_creep_mm_ = param("barrier_creep_mm", 0.0);
    barrier_stop_y_ = param("barrier_stop_y", 1.0);
    // 닫힌 차단바가 보이는 동안 선 여유 [mm] (planner.line_margin_mm 보다 작을 때만, 아래 값이 없으면 끔)
    barrier_line_margin_ = param("barrier_line_margin_mm", 1000.0);
    barrier_line_margin_on_ = barrier_line_margin_ < 999.0;
    // 닫힌 차단바 앞 감속 거리 [mm] (정지 거리보다 이만큼 멀 때부터, 음수 = planner.dead_end_slow_mm)
    barrier_slow_mm_ = param("barrier_slow_mm", -1.0);
    // 열린 차단바 인식 최소 확신도 (이 이상이면 바로 출발)
    barrier_open_conf_ = param("barrier_open_conf", 0.5);
    construction_track_outer_ = param("construction_track_outer_mm", -1.0);
    // 공사 모드 동작 (회전 제한 / 바깥 비킴 끔 / 먼 선 비용 끔 / 막히면 천천히 꺾기) 은 박스가 가까울 때만 (바퀴 축 앞 이 거리 안에
    //   박스가 보이거나 박스 기억이 있음, 그 뒤 construction_box_near_hold_sec 유지). 표지판 직후 박스 전 코너 / 박스 뒤 코너에서도
    //   켜져 있어 선 넘고 멈칫했다 (2026-10-30 drive40 41.0s 오른쪽 코너에서 0.8초 왼쪽으로 돎). 0 = 예전 (공사 모드 내내)
    construction_box_near_ = param("construction_box_near_mm", 0.0);
    construction_box_near_hold_ = param("construction_box_near_hold_sec", 0.5);
    // 교차로 U턴 (표지판 쪽으로 들어간 뒤 출구 전, 좌/우 기억이 끝나도) 선 여유 [mm] 와 선 기억 (안쪽 고리 선이 화면 밖으로
    //   빠져 잊고 안쪽 밟음, drive40 오른쪽 U턴 86~87s). 음수 = 끔
    turn_uturn_margin_ = param("turn_uturn_margin_mm", -1.0);
    turn_uturn_memory_ = param("turn_uturn_memory", false);
    // 왼쪽 표지판 U턴 여유 (음수 = turn_uturn_margin_mm). 왼쪽 진입 여유 70 이 U턴까지 이어져 고리를 바깥 노란선에 붙어 돎 (drive41 276s)
    turn_uturn_margin_left_ = param("turn_uturn_margin_left_mm", -1.0);
    // 왼쪽 교차로 바깥 비킴 [mm] (planner.track_outer_mm 대신, 음수 = planner 값)
    turn_track_outer_left_ = param("turn_track_outer_left_mm", -1.0);
    // 오른쪽 교차로 (좌/우 기억 RIGHT) 바깥 비킴 [mm] (planner.track_outer_mm 대신, 음수 = planner 값). 왼쪽은 planner 값.
    //   planner 90 이면 오른쪽 진입이 크게 돌아 U턴에서 선 밟음 (2026-10-29, 왼쪽은 잘 들어감)
    turn_track_outer_right_ = param("turn_track_outer_right_mm", -1.0);
    // 닫힌 차단바로 다가가는 속도 [m/s 실제] (barrier_stop_y < 1 일 때, 가까워지면 dead_end_creep_linear 까지)
    barrier_approach_linear_ = param("barrier_approach_linear", 0.25);
  }

  double uturnMargin() const
  {
    return exit_dir_ == grid_drive::Turn::LEFT && turn_uturn_margin_left_ >= 0 ? turn_uturn_margin_left_ : turn_uturn_margin_;
  }
  bool leftEarly() const {return turn_entry_left_early_ && exit_dir_ == grid_drive::Turn::LEFT;}
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
      // 지난 주행의 표지판 / 출구 기다림을 버림 (drive28: GUI 로 멈췄다 다시 시작한 뒤 교차로 앞 코너에서 출구로 봄)
      turn_ = grid_drive::Turn::NONE;
      exit_armed_ = exit_leaving_ = false;
      corner_armed_ = true;
      corner_until_ = 0;
      // 주차 노드 등에서 넘겨받을 때 이미 달리는 중이면 그 속도에서 이어감 (0 부터 가속하면 한 번 멈칫함)
      v_f_ = have_odom_ ? std::max(0.0, odom_.twist.twist.linear.x) : 0.0;
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
          const bool right = exit_dir_ == grid_drive::Turn::RIGHT;
          // 진입: 표지판 뒤 가장 반대쪽으로 돌았던 곳에서 표지판 쪽으로 turn_exit_entry_deg (drive32: 앞 왼쪽 코너를
          //   도는 중 오른쪽 표지판을 봐서 표지판 본 방향 기준으로는 오른쪽 진입이 25도가 안 됐음)
          const bool entered = exit_entered_ || ((!leftEarly() || entry_side_seen_) &&
            (right ? exit_hi_ - exit_acc_ : exit_acc_ - exit_lo_) >= turn_exit_entry_);
          if (entered && !exit_entered_) {
            exit_entered_ = true;
            entered_t_ = now();
            RCLCPP_INFO(get_logger(), "교차로 %s 쪽으로 %.0f도 들어감", right ? "오른" : "왼", turn_exit_entry_ * 180.0 / M_PI);
          }
          const double prog = !entered ? 0.0 : right ? exit_acc_ - exit_lo_ : exit_hi_ - exit_acc_;
          if (prog >= turn_exit_yaw_) {
            exit_armed_ = false;
            turn_ = exit_dir_;
            turn_until_ = now() + rclcpp::Duration::from_seconds(turn_exit_hold_);
            exit_leaving_ = turn_exit_turn_ > 0;
            exit_leave_acc_ = exit_leave_lo_ = 0;
            RCLCPP_INFO(get_logger(), "교차로 U턴 %.0f도 -> 출구, %s 쪽 %.1f초 다시 우선", prog * 180.0 / M_PI,
              exit_dir_ == grid_drive::Turn::RIGHT ? "오른" : "왼", turn_exit_hold_);
          } else if (now() > exit_deadline_) {
            exit_armed_ = false;
          }
        } else if (exit_leaving_) {
          // 출구: 표지판 쪽으로 turn_exit_turn_deg 돌았으면 좌/우 기억 끝
          //   (출구 뒤에도 U턴 쪽으로 더 돌 수 있어 가장 U턴 쪽으로 돌았던 곳부터 셈, drive28 145s +75도)
          exit_leave_acc_ += std::remainder(yaw - prev_yaw_, 2 * M_PI) * (exit_dir_ == grid_drive::Turn::RIGHT ? -1.0 : 1.0);
          exit_leave_lo_ = std::min(exit_leave_lo_, exit_leave_acc_);
          if (turn_ != exit_dir_) {
            exit_leaving_ = false;
          } else if (exit_leave_acc_ - exit_leave_lo_ >= turn_exit_turn_) {
            exit_leaving_ = false;
            turn_until_ = now();
            RCLCPP_INFO(get_logger(), "출구로 %.0f도 돎 -> 좌/우 기억 끝", (exit_leave_acc_ - exit_leave_lo_) * 180.0 / M_PI);
          }
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
    // 표지판 쪽으로 turn_exit_entry_deg 만큼 꺾어 들어가기 전에는 turn_hold_sec 가 지나도 (turn_entry_max_sec 까지) 유지.
    //   drive32 53s: 앞 왼쪽 코너를 도는 중 오른쪽 표지판을 봤고 1.0s 가 진입 전에 끝나 고리를 왼쪽으로 돎
    const bool entry_wait = exit_armed_ && !exit_entered_ && turn_ == exit_dir_ &&
      (now() - turn_sign_t_).seconds() < turn_entry_max_;
    if (turn_ != grid_drive::Turn::NONE && now() > turn_until_ && !entry_wait) {
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
    if (in_construction && construction_box_near_ > 0) {
      bool near = last_box_memory_ > 0;
      if (!near && !obs.box.empty()) {
        const int r0 = std::clamp(static_cast<int>((p_.far_mm - construction_box_near_) / p_.cell_mm), 0, obs.box.rows);
        near = cv::countNonZero(obs.box.rowRange(r0, obs.box.rows)) > 0;
      }
      if (near) {
        box_near_t_ = now();
      }
    }
    // 공사 모드 동작을 쓸지 (박스 가까움)
    const bool con_active = in_construction &&
      (construction_box_near_ <= 0 || (now() - box_near_t_).seconds() < construction_box_near_hold_);
    con_active_ = con_active;
    if (!rs.mask.empty()) {
      grid_drive::SegMasks sm;
      sm.road = rs.mask;
      grid_drive::merge(obs, grid_drive::observeSegmentation(sm, p_));
    }
    // 모든 계획은 도로 모델 기준: 도로 모델이 주행 가능 구역을 하나도 못 찾으면
    // 색 (선) 만으로 계획하지 않고 경로 없음 -> 정지. 도로가 지도 칸 밖에만 있으면 지도 전체가 주행 불가 -> 경로 없음
    const bool no_road = haveRoad() && (rs.mask.empty() || rs.detections == 0 || obs.offroad.empty());
    grid_drive::Params pp = p_;
    if (lead_sec_ > 0 && v_f_ > 0.02) {
      pp.lead_mm = v_f_ * 1000.0 * lead_sec_;
      pp.lead_k = -w_f_ / (v_f_ * 1000.0);   // w = -v k (오른쪽 k +)
    }
    if (in_turn || con_active) {
      pp.far_line_cost = 0.0;   // 교차로 / 공사 구간은 넓은 바닥을 가로지름
    }
    // 표지판 진입 여유: 표지판을 마지막으로 본 뒤 turn_line_margin_sec 동안 (turn_hold_sec 보다 길어도 유지)
    const bool sign_margin = turn_line_margin_sec_ > 0 &&
      (now() - turn_sign_t_).seconds() < turn_line_margin_sec_;
    if (sign_margin || (in_turn && turn_line_margin_sec_ <= 0)) {
      const auto dir = sign_margin ? turn_sign_dir_ : turn_;
      pp.line_margin_mm = dir == grid_drive::Turn::LEFT ? turn_line_margin_left_ : turn_line_margin_;
      // turn_entry_margin_until_entered: 표지판 쪽으로 turn_exit_entry_deg 들어간 뒤 (exit_entered_) 는 U턴 여유보다
      //   작게는 안 함 (drive36: 오른쪽 진입 10mm 가 섬 끝을 지날 때까지 이어져 경로가 표지판 섬 노란선에 붙어 밟음.
      //   왼쪽 진입 70 은 그대로)
      if (turn_entry_margin_until_entered_ && exit_armed_ && exit_entered_ && dir == exit_dir_) {
        // 들어간 뒤: U턴 여유가 있으면 그것 (왼쪽 진입 70 이 U턴까지 이어지지 않게), 없으면 예전처럼 큰 쪽
        const double um = uturnMargin();
        pp.line_margin_mm = um >= 0 ? um : std::max(pp.line_margin_mm, turn_after_margin_);
      }
    } else if (in_turn) {
      pp.line_margin_mm = turn_after_margin_;
    }
    // 교차로 U턴 중 (들어간 뒤 출구 전): 좌/우 기억이 끝나도 U턴 여유
    const bool in_uturn = exit_armed_ && exit_entered_;
    if (in_uturn && uturnMargin() >= 0) {
      pp.line_margin_mm = uturnMargin();
    }
    // 닫힌 차단바를 봄: 차단바 앞은 길이 좁아 양쪽 선 빨간 띠가 겹쳐 경로가 안 생기고 멀리서 멈췄다 (2026-10-29)
    //   -> 선 여유를 barrier_line_margin_mm 로 (공사 구간 / 좌우 기억 중에는 안 함)
    if (barrier_line_margin_on_ && !in_turn && !in_construction &&
      (now() - bar_closed_t_).seconds() < barrier_hold_)
    {
      pp.line_margin_mm = std::min(pp.line_margin_mm, barrier_line_margin_);
    }
    margin_used_ = pp.line_margin_mm;
    if (in_turn) {
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
    if (turn_ == grid_drive::Turn::RIGHT && turn_track_outer_right_ >= 0) {
      pp.track_outer_mm = turn_track_outer_right_;
    }
    // U턴 중 (좌/우 기억이 끝나도) 도 교차로 방향의 바깥 비킴
    const auto tdir = in_turn ? turn_ : in_uturn ? exit_dir_ : grid_drive::Turn::NONE;
    if (tdir == grid_drive::Turn::RIGHT && turn_track_outer_right_ >= 0) {
      pp.track_outer_mm = turn_track_outer_right_;
    } else if (tdir == grid_drive::Turn::LEFT && turn_track_outer_left_ >= 0) {
      pp.track_outer_mm = turn_track_outer_left_;
    }
    // 공사 구간: 바깥 비킴은 박스 쪽으로 밀 수 있어 construction_track_outer_mm 로 (음수 = 그대로)
    if (con_active && !in_turn && construction_track_outer_ >= 0) {
      pp.track_outer_mm = construction_track_outer_;
    }
    if (con_active && !in_turn) {
      if (construction_max_angular_ > 0) {
        pp.max_angular = construction_max_angular_;
      }
      if (construction_change_weight_ > 0) {
        pp.change_weight = construction_change_weight_;
      }
    }
    if ((in_turn && turn_memory_) || (in_uturn && turn_uturn_memory_) || (in_construction && construction_memory_)) {
      pp.memory = true;
      // 공사 구간: 박스만 기억 (선은 계속 보여서 기억할 필요가 적고, 어긋난 선 기억이 경로를 뒤집음)
      pp.memory_lines = in_turn || (in_uturn && turn_uturn_memory_) || p_.memory_lines;
      if (in_construction && !in_turn) {
        pp.memory_lines = construction_memory_lines_;
      }
    }
    // 비전이 닫힌 차단바를 barrier_hold_sec 안에 봄: 막다른 길을 차단바로 (색으로는 흰 줄무늬가 선으로 잡힘)
    const bool bar_near = (now() - bar_closed_t_).seconds() < barrier_hold_;
    pp.dead_end_force_box = bar_near;
    pp.dead_end_line_mm += 1000.0 * std::max(0.0, v_f_) * dead_end_lag_;
    // 차단바 앞 감속 거리 (planner.dead_end_slow_mm 500 이면 차단바를 처음 본 먼 곳부터 30% 까지 느려졌음)
    if (bar_near && barrier_slow_mm_ >= 0) {
      pp.dead_end_slow_mm = barrier_slow_mm_;
    }
    const bool entry_commit = turn_entry_commit_ && entry_wait && (entry_side_seen_ || leftEarly());
    pp.turn_block_wrong = entry_commit;
    if ((in_turn || in_uturn) && turn_soft_line_block_ >= 0) {
      pp.soft_line_block_mm = turn_soft_line_block_;
    }
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
    // 공사 구간 / 교차로에서는 안 함: 박스 / 선을 코스 끝으로 잘못 보고 경로가 끊기면 앞만 보고 직진해 차선을 넘었음 (2026-10-29)
    const bool end_creep_ok = turn_ == grid_drive::Turn::NONE && now() >= construction_until_;
    if (!end_creep_ok) {
      end_seen_ = false;
    } else if (r.dead_end && r.path_ok && !r.dead_end_box) {
      end_seen_ = true;
      end_left_mm_ = r.dead_end_mm - r.dead_end_target_mm;
      end_x_ = odom_x;
      end_y_ = odom_y;
      end_t_ = now();
    } else if (r.path_ok || (r.dead_end && !r.path_ok)) {
      end_seen_ = false;   // 보통 경로 / 정지 거리에 이미 옴
    }
    // 차단바 앞에서 멈췄으면 차단바가 barrier_hold_sec 동안 안 보일 때까지 (올라감) 계속 정지.
    //   barrier_creep_mm > 0: 경로가 끊긴 곳에서 바로 서지 않고 odom 으로 그만큼 직진한 뒤 정지
    //   공사 구간 중에는 안 함 (박스를 차단바로 잘못 보면 박스로 직진)
    //   barrier_stop_y < 1: 위 대신 차단바 화면 세로 위치 (y_ratio) 가 barrier_stop_y 가 될 때까지 차선을 따라 다가가서 정지
    //   (경로가 빨간 띠 / 박스로 멀리서 끊겨 BAR WAIT 로 멀리 섰음, 2026-10-29). 멀면 barrier_approach_linear, 가까우면 creep 속도
    bool bar_approach = false;
    double bar_v = 0, bar_w = 0;
    const bool bar_by_y = barrier_stop_y_ < 1.0;
    if (bar_by_y && bar_near && !bar_stopped_ && driving() && !no_road && now() >= construction_until_) {
      if (bar_y_ >= barrier_stop_y_) {
        bar_stopped_ = true;
        RCLCPP_INFO(get_logger(), "차단바 앞 정지 (y %.2f >= %.2f)", bar_y_, barrier_stop_y_);
      } else {
        bar_approach = true;
        // 감속 없이 일정 속도로 다가가서 바로 정지 (2026-10-29 사용자 요청). 0 이하 = 평소 직선 속도 (planner.linear)
        bar_v = barrier_approach_linear_ > 0 ? barrier_approach_linear_ : p_.linear;
        bar_w = r.follow_ok && !r.pivot ? -bar_v * r.curvature * 1000.0 : 0.0;
        r.reason = "닫힌 차단바로 다가감";
      }
    }
    const bool bar_blocked = !bar_by_y && bar_near && !bar_stopped_ && (!r.path_ok || r.pivot) &&
      now() >= construction_until_;
    if (bar_blocked && barrier_creep_mm_ > 0 && !bar_creeping_ && have_odom_) {
      bar_creeping_ = true;
      bar_creep_x_ = odom_x;
      bar_creep_y_ = odom_y;
      RCLCPP_INFO(get_logger(), "닫힌 차단바 앞 경로 끊김 (y %.2f) -> %.0fmm 더 직진 후 정지", bar_y_, barrier_creep_mm_);
    }
    bool bar_creep = false;
    if (bar_creeping_ && !bar_stopped_) {
      const double gone = 1000.0 * std::hypot(odom_x - bar_creep_x_, odom_y - bar_creep_y_);
      if (gone >= barrier_creep_mm_ || bar_y_ >= barrier_stop_y_) {
        bar_stopped_ = true;
        RCLCPP_INFO(get_logger(), "차단바 앞 정지: %.0fmm 직진, y %.2f", gone, bar_y_);
      } else {
        bar_creep = true;
      }
    } else if (!bar_by_y && r.dead_end && r.dead_end_box && !r.path_ok && bar_near && barrier_creep_mm_ <= 0) {
      bar_stopped_ = true;
    }
    if (!bar_near || !driving()) {
      bar_stopped_ = false;
      bar_creeping_ = false;
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
      // odom 거리와 (속도 x 시간) 중 큰 쪽 (odom 이 덜 재도 넘어가지 않게)
      const double moved = std::max(1000.0 * std::hypot(odom_x - end_x_, odom_y - end_y_),
        1000.0 * std::max(v_f_, creep_linear_) * (now() - end_t_).seconds());
      creep_left = end_left_mm_ - moved;
      if (creep_left <= 0) {
        end_seen_ = false;
        end_stopped_ = true;   // 남은 거리 다 감: 아래 road_hold 로 직전 속도 이어 보내지 않음
      }
    }
    const bool creep = driving() && !bar_stopped_ && creep_linear_ > 0 && (creep_left > 0 || bar_creep);
    // 남은 거리 직진은 감속 없이 직전 속도 그대로 (최소 dead_end_creep_linear)
    if (creep && !creeping_) {
      creep_v_ = std::max(v_f_, creep_linear_);
    }
    creeping_ = creep;
    if (creep) {
      r.v = creep_v_;
      r.w = 0;
      r.reason = "막는 것 앞까지 남은 거리 직진";
    }

    // 도로 / 경로를 잠깐 (road_hold_sec 이하) 놓치면 직전 명령을 계속 보냄 (U턴 중 1~2 프레임 놓침).
    // 막다른 길 정지 (차단바 앞) 는 바로 멈춤
    // 원호가 전부 막혀 제자리 회전 (pivot) 도 한두 프레임이면 직전 명령 유지 (drive26: 교차로 진입 왼쪽 회전 중 한 프레임
    //   pivot -> 속도 0.27 -> 0.14 로 멈칫). road_hold_sec 넘게 계속되면 그때 제자리 회전
    const bool lost = !creep && !bar_approach && (no_road || (!r.path_ok && !r.dead_end) || r.pivot);
    if (!lost) {
      last_ok_ = now();
      end_stopped_ = false;
    }
    // 공사 구간 원호 막힘 (construction_pivot_linear) 은 직전 명령 유지 없이 바로 천천히 가며 꺾음
    //   (유지 0.3s 동안 0.48 m/s 로 14cm 더 나가 첫 박스를 크게 돌고 두 번째 박스에 박을 뻔, 2026-10-29)
    if (turn_entry_commit_ && entry_wait && !entry_side_seen_ && r.path_ok &&
      (turn_ == grid_drive::Turn::RIGHT ? r.goal.x > 0 : r.goal.x < 0))
    {
      entry_side_seen_ = true;
      if (leftEarly()) {
        exit_lo_ = exit_hi_ = exit_acc_;   // 진입 각도는 여기부터 (앞 코너 회전 빼고)
      }
      RCLCPP_INFO(get_logger(), "교차로 %s 쪽 길 찾음 -> 반대쪽 목표 안 고름", turn_ == grid_drive::Turn::RIGHT ? "오른" : "왼");
    }
    // 진입 고정 중 표지판 쪽 길이 안 보임: 직전 명령 유지 없이 천천히 가며 표지판 쪽으로 꺾음
    const bool after_seek = exit_armed_ && exit_entered_ && turn_entry_after_seek_sec_ > 0 &&
      (now() - entered_t_).seconds() < turn_entry_after_seek_sec_;
    // 길이 road_hold_sec 넘게 안 보일 때만 (한두 프레임 놓침은 아래 hold 로 직전 명령 유지). 10-30 drive43: 한 프레임
    //   놓쳐도 바로 0.08 m/s 로 떨어져 진입마다 0.3~1초 "뚝" 멈춤, 교차로 구간 평균 0.35 -> 0.27 m/s
    const bool entry_seek = (entry_commit || after_seek) && !no_road && !r.path_ok && !r.dead_end &&
      turn_entry_seek_linear_ > 0 && !(sent_moving_ && (now() - last_ok_).seconds() <= road_hold_sec_);
    if (entry_seek) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "교차로 진입: 표지판 쪽 길 안 보임 -> 그쪽으로 꺾으며 천천히");
    }
    last_box_memory_ = r.box_memory;
    const bool con_pivot = r.pivot && construction_pivot_linear_ > 0 &&
      turn_ == grid_drive::Turn::NONE && con_active_;
    const bool hold = driving() && !bar_stopped_ && !end_stopped_ && !con_pivot && !entry_seek && lost && sent_moving_ &&
      (now() - last_ok_).seconds() <= road_hold_sec_;
    if (hold) {
      publishCmd(v_f_, w_f_);
      stop_sent_ = false;
    } else if (bar_approach) {
      v_f_ += smooth_ * (bar_v - v_f_);
      w_f_ += smooth_ * (bar_w - w_f_);
      publishCmd(v_f_, w_f_);
      stop_sent_ = false;
    } else if (creep) {
      v_f_ = creep_v_;
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
      if (entry_seek) {
        v = turn_entry_seek_linear_;
        w = (exit_dir_ == grid_drive::Turn::RIGHT ? -1.0 : 1.0) * turn_entry_seek_angular_;
      }
      const bool in_con = turn_ == grid_drive::Turn::NONE && now() < construction_until_;
      if (con_pivot) {
        v = construction_pivot_linear_;
        // 꺾는 방향은 한 번 정하면 0.5s 동안 유지 (매 프레임 정하면 박스 사이에서 좌우로 뒤집혀 느리게 흔들렸음, 2026-10-29)
        //   정한 시각부터 0.5s (예전엔 막힌 프레임마다 시각을 새로 해서 계속 같은 쪽으로 돎, drive40 41.0s 0.8초 반대로)
        if ((now() - con_pivot_dec_t_).seconds() > 0.5 || con_pivot_dir_ == 0) {
          con_pivot_dir_ = r.w >= 0 ? 1 : -1;
          con_pivot_dec_t_ = now();
        }
        con_pivot_t_ = now();
        w = con_pivot_dir_ * (construction_max_angular_ > 0 ? construction_max_angular_ : p_.max_angular);
      }
      // 공사 구간 (좌/우 기억 아닐 때) 은 construction_linear 까지 감속. 곡률은 그대로
      if (in_con && construction_linear_ > 0 && v > construction_linear_) {
        w *= construction_linear_ / v;
        v = construction_linear_;
      }
      cornerDelay(r, w);
      const double w_raw = w;
      w = medianW(w);
      const double v_prev = v_f_;
      v_f_ += smooth_ * (v - v_f_);
      w_f_ += smooth_ * (w - w_f_);
      if (con_pivot || entry_seek) {   // 중간값 / 부드럽게 하면 한두 프레임 꺾음이 -0.2 정도로 묻혀 거의 직진했음
        v_f_ = std::min(v_f_, v);
        w_f_ = w_raw;
      }
      const double acc_lim = in_con && construction_accel_limit_ >= 0 ? construction_accel_limit_ : accel_limit_;
      if (acc_lim > 0 && v_f_ > v_prev) {
        const double dt = std::clamp((now() - last_cmd_t_).seconds(), 0.0, 0.1);
        v_f_ = std::min(v_f_, v_prev + acc_lim * dt);
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
    RCLCPP_DEBUG(get_logger(), "v %.3f w %.3f k %.4f out %.3f %.3f ok %d piv %d end %d soft %d road %d C %d [%s] (%.1fms) 전체 %.1fms",
      r.v, r.w, r.curvature, v_f_, w_f_, r.path_ok, r.pivot, r.dead_end, r.soft, rs.detections,
      now() < construction_until_, r.reason.c_str(), ms_road, ms);

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
    std::snprintf(text, sizeof(text), "%s v%.2f w%.2f %s%s turn %s m%.0f road %d %.2f  %.0f/%.0fms",
      !driving() ? "STOP" : bar_stopped_ ? "BAR WAIT" : no_road ? "NO ROAD" : "RUN", r.v, r.w, r.dead_end ? "END " : r.soft ? "SOFT " : "", r.pivot ? "PIVOT " : "", turn,
      margin_used_, rs.detections, rs.best_conf, ms_road, ms);
    cv::putText(left, text, {6, 20}, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2);

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
  grid_drive::ColorParams colors_;
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
  double construction_max_angular_ = 0, construction_change_weight_ = 0, construction_linear_ = 0;
  double construction_pivot_linear_ = 0, construction_accel_limit_ = -1, turn_track_outer_right_ = -1;
  int con_pivot_dir_ = 0;
  bool con_active_ = false;
  rclcpp::Time con_pivot_t_{0, 0, RCL_ROS_TIME};
  double lead_sec_ = 0;
  double turn_line_margin_ = 10.0, turn_line_margin_sec_ = 0.0, turn_after_margin_ = 10.0;
  double turn_line_margin_left_ = 10.0;
  bool turn_entry_margin_until_entered_ = true;
  bool turn_entry_commit_ = true, entry_side_seen_ = false, turn_entry_left_early_ = false;
  double turn_entry_seek_linear_ = 0.08, turn_entry_seek_angular_ = 1.2, turn_entry_after_seek_sec_ = 0.0;
  double turn_soft_line_block_ = -1.0;
  rclcpp::Time entered_t_{0, 0, RCL_ROS_TIME};
  int turn_memory_frames_ = 0;
  double turn_exit_yaw_ = 0, turn_exit_hold_ = 3.0, turn_exit_max_ = 20.0;
  bool exit_armed_ = false, exit_leaving_ = false;
  double turn_exit_entry_ = 0, turn_exit_turn_ = 0, exit_leave_acc_ = 0, exit_leave_lo_ = 0;
  grid_drive::Turn exit_dir_ = grid_drive::Turn::NONE;
  double exit_acc_ = 0, exit_lo_ = 0, exit_hi_ = 0;
  rclcpp::Time exit_deadline_{0, 0, RCL_ROS_TIME};
  double turn_min_radius_ = 0, turn_change_weight_ = 0, turn_center_cost_ = 0;
  rclcpp::Time turn_sign_t_{0, 0, RCL_ROS_TIME};
  grid_drive::Turn turn_sign_dir_ = grid_drive::Turn::NONE;   // 마지막으로 본 좌/우 표지판 (진입 선 여유용)
  double margin_used_ = 0;   // 이번 프레임 선 여유 [mm] (디버그 화면)
  double road_hold_sec_ = 0.15;
  double creep_linear_ = 0.05, creep_v_ = 0.05;
  bool creeping_ = false, end_stopped_ = false;
  double dead_end_lag_ = 0.25;
  bool end_seen_ = false;
  double end_left_mm_ = 0, end_x_ = 0, end_y_ = 0;
  rclcpp::Time end_t_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_ok_{0, 0, RCL_ROS_TIME};
  std::deque<double> w_hist_;
  bool corner_armed_ = true;
  double corner_until_ = 0, last_curve_t_ = 0;
  double turn_flip_delay_ = 0;
  int last_curve_sign_ = 0;   // 좌/우 기억 중 마지막 곡선 방향 (+1 오른쪽, -1 왼쪽, 0 없음)
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
  rclcpp::Time bar_closed_t_{0, 0, RCL_ROS_TIME}, bar_open_t_{0, 0, RCL_ROS_TIME};
  double bar_y_ = 0, barrier_creep_mm_ = 0, barrier_stop_y_ = 1.0, bar_creep_x_ = 0, bar_creep_y_ = 0;
  bool bar_creeping_ = false, exit_entered_ = false;
  double turn_entry_max_ = 0, barrier_line_margin_ = 1000.0;
  bool barrier_line_margin_on_ = false;
  double construction_box_near_ = 0.0, construction_box_near_hold_ = 0.5, turn_uturn_margin_ = -1.0;
  bool turn_uturn_memory_ = false;
  double turn_uturn_margin_left_ = -1.0, turn_track_outer_left_ = -1.0;
  int last_box_memory_ = 0;
  rclcpp::Time box_near_t_{0, 0, RCL_ROS_TIME}, con_pivot_dec_t_{0, 0, RCL_ROS_TIME};
  double barrier_slow_mm_ = -1.0, barrier_open_conf_ = 0.5, construction_track_outer_ = -1.0,
    barrier_approach_linear_ = 0.25;
  bool bar_stopped_ = false;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GridDriveNode>());
  rclcpp::shutdown();
  return 0;
}
