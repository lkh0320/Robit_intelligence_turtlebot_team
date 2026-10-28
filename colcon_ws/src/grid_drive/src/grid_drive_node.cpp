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
    p_.memory_max_frames = static_cast<int>(declare_parameter("planner.memory_max_frames", 15));
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
    };
    for (const auto & [name, ptr] : dparams) {
      *ptr = declare_parameter(std::string("planner.") + name, *ptr);
    }
    p_.memory = declare_parameter("planner.memory", p_.memory);

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
    sign_sub_ = create_subscription<Sign>(
      "sign", rclcpp::SensorDataQoS(), [this](Sign::ConstSharedPtr m) {
        if (m->type == Sign::CONSTRUCTION && m->confidence >= sign_conf_) {
          if (now() > construction_until_) {
            RCLCPP_INFO(get_logger(), "공사 구간 표지판 -> 공사 모드 %.0fs", construction_hold_);
          }
          construction_until_ = now() + rclcpp::Duration::from_seconds(construction_hold_);
        }
        if ((m->type == Sign::LEFT || m->type == Sign::RIGHT) && m->confidence >= sign_conf_) {
          turn_ = m->type == Sign::LEFT ? grid_drive::Turn::LEFT : grid_drive::Turn::RIGHT;
          turn_until_ = now() + rclcpp::Duration::from_seconds(turn_hold_);
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
    if (corner_delay_ <= 0) {
      return;
    }
    const double t = now().seconds();
    const bool curve = r.follow_ok && !r.pivot && std::abs(r.curvature) * 1000.0 > corner_k_;
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
    const bool lost = !creep && (no_road || (!r.path_ok && !r.dead_end));
    if (!lost) {
      last_ok_ = now();
    }
    const bool hold = driving() && !bar_stopped_ && lost && sent_moving_ && (now() - last_ok_).seconds() <= road_hold_sec_;
    if (hold) {
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
  double road_hold_sec_ = 0.15;
  double creep_linear_ = 0.05;
  bool end_seen_ = false;
  double end_left_mm_ = 0, end_x_ = 0, end_y_ = 0;
  rclcpp::Time end_t_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_ok_{0, 0, RCL_ROS_TIME};
  std::deque<double> w_hist_;
  bool corner_armed_ = true;
  double corner_until_ = 0, last_curve_t_ = 0;
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
