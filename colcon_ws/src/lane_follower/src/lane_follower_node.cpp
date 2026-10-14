// 라인 추적 주행: lane_info -> cmd_vel
//   구독: lane_info (interfaces/LaneInfo)        lane_detection (슬라이딩 윈도우)
//         control_mode (interfaces/ControlMode)  task_planner / GUI 모드 버튼
//         psd (interfaces/PsdArray)              정면 비상 정지, 공사 구간 박스 통과 세기
//   발행: cmd_vel (geometry_msgs/Twist)          stm_bridge 가 바퀴로 보낸다
//
// 운행 모드가 LANE / AVOID 일 때만 주행한다.
//   그 외 모드(STOP, PARKING, MANUAL)가 되면 정지 명령을 한 번 보내고 조용히 있는다
//   (GUI 수동 주행이나 주차 기동 노드의 cmd_vel 과 싸우지 않도록)
// autostart: true 면 control_mode 를 받기 전에도 LANE 으로 시작 (task_planner 없이 주행 시험).
//   GUI STOP 버튼(control_mode STOP)을 누르면 그대로 멈춘다.
// 안전: lane_info 가 lane_timeout 동안 안 오거나 detected = false 면 정지,
//       정면 PSD 가 front_stop_distance 보다 가까우면 정지
// 제어 법칙은 include/lane_follower/controller.hpp (2026-10-14 path_planner / map_planner / hybrid 는 지움)
//
// 교차로 회전 (include/lane_follower/intersection_turn.hpp, intersection.* 파라미터):
//   LEFT/RIGHT 표지판을 본 뒤 차선을 놓치면 (교차로 정면의 가로 선) 정지 대신 조금 직진한 뒤 표지판 방향으로 돌고,
//   차선을 다시 잡으면 원래 주행으로. 왼쪽으로 돌았으면 고리 출구에서 한 번 더 왼쪽으로.
//   추가 구독: sign, dxl_state (이동 거리 / 회전량), cross_line (lane_detection 의 T자 가로 선)
// 공사 구간 (include/lane_follower/construction_zone.hpp, construction.* 파라미터):
//   공사 표지판을 본 뒤 박스가 가까워지면 wall_detection 이 고른 경로 방향(free_offset)으로 조향하고,
//   옆 PSD 로 박스 통과를 세어 3개 지나면 차선 추종으로 돌아간다. 로그 "[공사] ..."
//   추가 구독: wall_obstacle (interfaces/WallObstacle, obstacle_detection wall_detection_node)
//   추가 발행: lane_follower/construction (std_msgs/String, 공사 구간 상태가 바뀔 때)
#include <algorithm>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "interfaces/msg/control_mode.hpp"
#include "interfaces/msg/dxl_state.hpp"
#include "interfaces/msg/lane_info.hpp"
#include "interfaces/msg/psd_array.hpp"
#include "interfaces/msg/sign.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "interfaces/msg/wall_obstacle.hpp"
#include "lane_follower/construction_zone.hpp"
#include "lane_follower/controller.hpp"
#include "lane_follower/intersection_turn.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class LaneFollowerNode : public rclcpp::Node
{
  using ControlMode = interfaces::msg::ControlMode;

public:
  LaneFollowerNode()
  : Node("lane_follower")
  {
    const lane_follower::Gains g;
    declare_parameter("linear_speed", g.linear_speed);
    declare_parameter("min_linear_speed", g.min_linear_speed);
    declare_parameter("max_angular", g.max_angular);
    declare_parameter("low_confidence", g.low_confidence);
    declare_parameter("d_filter_alpha", g.d_filter_alpha);
    // 조향 게인 기본값은 config/lane_follower.yaml 과 같게
    declare_parameter("kp", 0.674);
    declare_parameter("kd", 0.0042);
    declare_parameter("ka", 0.253);
    declare_parameter("slow_down_angle", 0.8);
    declare_parameter("wheel_separation", 0.190);   // stm_bridge 와 같게 [m] (바퀴 속도 -> 이동 거리 / 회전)
    declare_parameter("lane_timeout", 0.7);
    declare_parameter("front_stop_distance", 0.10);   // 0 이하면 끔
    declare_parameter("psd_timeout", 0.5);
    const bool autostart = declare_parameter("autostart", false);
    const double rate = declare_parameter("control_rate", 20.0);
    declareTurnParams();
    declareConstructionParams();

    mode_ = autostart ? ControlMode::LANE : ControlMode::STOP;

    const auto qos = rclcpp::SensorDataQoS();
    auto lane_cb = [this](Lane & l) {
        return [this, &l](interfaces::msg::LaneInfo::ConstSharedPtr msg) {
                 l.msg = *msg;
                 l.stamp = now();
                 l.received = true;
               };
      };
    lane_sub_ = create_subscription<interfaces::msg::LaneInfo>("lane_info", qos, lane_cb(lane_));
    sign_sub_ = create_subscription<interfaces::msg::Sign>(
      "sign", qos, [this](interfaces::msg::Sign::ConstSharedPtr m) {
        // 멈춰 있을 때 (수동 조종 / STOP) 본 표지판은 안 쓴다 (손으로 옮기는 동안 본 표지판이 남지 않게)
        if (mode_ != ControlMode::LANE && mode_ != ControlMode::AVOID) {
          return;
        }
        turn_.onSign(m->type, m->confidence, m->area_ratio, readTurnParams());
        logTurn();
        zone_.onSign(m->type, m->confidence, m->area_ratio, now().seconds(), readConstructionParams());
        logZoneState();
      });
    wall_sub_ = create_subscription<interfaces::msg::WallObstacle>(
      "wall_obstacle", qos, [this](interfaces::msg::WallObstacle::ConstSharedPtr m) {
        zone_.onWall(m->detected, m->bottom_ratio, m->free_offset, m->confidence, now().seconds(),
        readConstructionParams());
      });
    // 상태가 바뀔 때만 발행 -> 늦게 켠 GUI 도 마지막 상태를 받게 transient_local
    zone_pub_ = create_publisher<std_msgs::msg::String>(
      "lane_follower/construction", rclcpp::QoS(1).reliable().transient_local());
    dxl_sub_ = create_subscription<interfaces::msg::DxlState>(
      "dxl_state", qos, [this](interfaces::msg::DxlState::ConstSharedPtr m) {onDxl(*m);});
    // lane_detection 의 교차로 T자 가로 선: x 세로 위치, y 기울기 [rad], z 길이 (0 = 없음)
    cross_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "cross_line", qos, [this](geometry_msgs::msg::Vector3Stamped::ConstSharedPtr m) {
        turn_.onCrossLine(m->vector.z > 0, m->vector.y);
      });
    mode_sub_ = create_subscription<ControlMode>(
      "control_mode", 10, [this](ControlMode::ConstSharedPtr msg) {
        if (msg->mode != mode_) {
          RCLCPP_INFO(get_logger(), "control_mode %u -> %u (%s)", mode_, msg->mode,
            msg->mission_state.c_str());
        }
        mode_ = msg->mode;
      });
    psd_sub_ = create_subscription<interfaces::msg::PsdArray>(
      "psd", qos, [this](interfaces::msg::PsdArray::ConstSharedPtr msg) {
        psd_ = *msg;
        psd_stamp_ = now();
        psd_received_ = true;
        zone_.onPsd(msg->left, msg->right, psd_stamp_.seconds(), readConstructionParams());
        logZoneState();
      });
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this] {tick();});

    RCLCPP_INFO(get_logger(), "lane_follower 시작 (%s)",
      autostart ? "autostart: 바로 LANE 주행" : "control_mode LANE/AVOID 대기");
  }

private:
  struct Lane
  {
    interfaces::msg::LaneInfo msg;
    rclcpp::Time stamp;
    bool received = false;
  };

  double dbl(const std::string & name) {return get_parameter(name).as_double();}
  int integer(const std::string & name) {return static_cast<int>(get_parameter(name).as_int());}

  void declareTurnParams()
  {
    const lane_follower::TurnParams d;
    declare_parameter("intersection.enabled", d.enabled);
    declare_parameter("intersection.sign_min_confidence", d.sign_min_confidence);
    declare_parameter("intersection.sign_min_area", d.sign_min_area);
    declare_parameter("intersection.sign_confirm_count", d.sign_confirm_count);
    declare_parameter("intersection.sign_confirm_window", d.sign_confirm_window);
    declare_parameter("intersection.sign_instant_confidence", d.sign_instant_confidence);
    declare_parameter("intersection.arm_max_dist", d.arm_max_dist);
    declare_parameter("intersection.require_cross_line", d.require_cross_line);
    declare_parameter("intersection.cross_recent_time", d.cross_recent_time);
    declare_parameter("intersection.approach_align", d.approach_align);
    declare_parameter("intersection.approach_align_kp", d.approach_align_kp);
    declare_parameter("intersection.approach_align_max", d.approach_align_max);
    declare_parameter("intersection.approach_align_fresh", d.approach_align_fresh);
    declare_parameter("intersection.approach_slow_linear", d.approach_slow_linear);
    declare_parameter("intersection.align_enabled", d.align_enabled);
    declare_parameter("intersection.align_kp", d.align_kp);
    declare_parameter("intersection.align_max_angular", d.align_max_angular);
    declare_parameter("intersection.align_min_angular", d.align_min_angular);
    declare_parameter("intersection.align_tol", d.align_tol);
    declare_parameter("intersection.align_ok_count", d.align_ok_count);
    declare_parameter("intersection.align_max_time", d.align_max_time);
    declare_parameter("intersection.align_lost_time", d.align_lost_time);
    declare_parameter("intersection.approach_time", d.approach_time);
    declare_parameter("intersection.approach_linear", d.approach_linear);
    declare_parameter("intersection.turn_pivot", d.turn_pivot);
    declare_parameter("intersection.turn_linear", d.turn_linear);
    declare_parameter("intersection.turn_angular", d.turn_angular);
    declare_parameter("intersection.turn_search_angular", d.turn_search_angular);
    declare_parameter("intersection.turn_min_yaw", d.turn_min_yaw);
    declare_parameter("intersection.turn_max_yaw", d.turn_max_yaw);
    declare_parameter("intersection.turn_switch_max_yaw", d.turn_switch_max_yaw);
    declare_parameter("intersection.exit_ok_count", d.exit_ok_count);
    declare_parameter("intersection.exit_min_confidence", d.exit_min_confidence);
    declare_parameter("intersection.exit_max_angle", d.exit_max_angle);
    declare_parameter("intersection.exit_max_offset", d.exit_max_offset);
    declare_parameter("intersection.exit_lead_angle", d.exit_lead_angle);
    declare_parameter("intersection.exit_turn_side_only", d.exit_turn_side_only);
    declare_parameter("intersection.rearm_dist", d.rearm_dist);
    declare_parameter("intersection.after_enabled", d.after_enabled);
    declare_parameter("intersection.after_dist", d.after_dist);
    declare_parameter("intersection.after_max_yaw", d.after_max_yaw);
    declare_parameter("intersection.after_max_retries", d.after_max_retries);
    declare_parameter("intersection.out_enabled", d.out_enabled);
    declare_parameter("intersection.out_trigger_yaw", d.out_trigger_yaw);
    declare_parameter("intersection.out_island_offset", d.out_island_offset);
    declare_parameter("intersection.out_island_min_yaw", d.out_island_min_yaw);
    declare_parameter("intersection.out_min_dist", d.out_min_dist);
    declare_parameter("intersection.out_max_dist", d.out_max_dist);
    declare_parameter("intersection.out_approach_time", d.out_approach_time);
    declare_parameter("intersection.out_turn_min_yaw", d.out_turn_min_yaw);
    declare_parameter("intersection.out_turn_max_yaw", d.out_turn_max_yaw);
    declare_parameter("intersection.out_follow", d.out_follow);
    declare_parameter("intersection.out_guard_dist", d.out_guard_dist);
    declare_parameter("intersection.out_guard_max_offset", d.out_guard_max_offset);
    declare_parameter("intersection.out_arc_linear", d.out_arc_linear);
    declare_parameter("intersection.out_arc_angular", d.out_arc_angular);
    declare_parameter("intersection.out_arc_max_dist", d.out_arc_max_dist);
    declare_parameter("intersection.out_hold_time", d.out_hold_time);
  }

  void declareConstructionParams()
  {
    const lane_follower::ConstructionParams d;
    declare_parameter("construction.enabled", d.enabled);
    declare_parameter("construction.sign_min_confidence", d.sign_min_confidence);
    declare_parameter("construction.sign_min_area", d.sign_min_area);
    declare_parameter("construction.sign_confirm_count", d.sign_confirm_count);
    declare_parameter("construction.sign_confirm_window", d.sign_confirm_window);
    declare_parameter("construction.sign_instant_confidence", d.sign_instant_confidence);
    declare_parameter("construction.arm_max_time", d.arm_max_time);
    declare_parameter("construction.enter_bottom", d.enter_bottom);
    declare_parameter("construction.enter_count", d.enter_count);
    declare_parameter("construction.wall_timeout", d.wall_timeout);
    declare_parameter("construction.linear", d.linear);
    declare_parameter("construction.kp", d.kp);
    declare_parameter("construction.max_angular", d.max_angular);
    declare_parameter("construction.low_confidence", d.low_confidence);
    declare_parameter("construction.slow_ratio", d.slow_ratio);
    declare_parameter("construction.boxes", d.boxes);
    declare_parameter("construction.pass_low", d.pass_low);
    declare_parameter("construction.pass_high", d.pass_high);
    declare_parameter("construction.pass_low_time", d.pass_low_time);
    declare_parameter("construction.pass_high_time", d.pass_high_time);
    declare_parameter("construction.same_side_gap", d.same_side_gap);
    declare_parameter("construction.no_box_time", d.no_box_time);
    declare_parameter("construction.return_ok_count", d.return_ok_count);
    declare_parameter("construction.return_max_time", d.return_max_time);
    declare_parameter("construction.avoid_max_time", d.avoid_max_time);
    declare_parameter("construction.rearm_time", d.rearm_time);
  }

  lane_follower::ConstructionParams readConstructionParams()
  {
    lane_follower::ConstructionParams p;
    p.enabled = get_parameter("construction.enabled").as_bool();
    p.sign_min_confidence = dbl("construction.sign_min_confidence");
    p.sign_min_area = dbl("construction.sign_min_area");
    p.sign_confirm_count = integer("construction.sign_confirm_count");
    p.sign_confirm_window = integer("construction.sign_confirm_window");
    p.sign_instant_confidence = dbl("construction.sign_instant_confidence");
    p.arm_max_time = dbl("construction.arm_max_time");
    p.enter_bottom = dbl("construction.enter_bottom");
    p.enter_count = integer("construction.enter_count");
    p.wall_timeout = dbl("construction.wall_timeout");
    p.linear = dbl("construction.linear");
    p.kp = dbl("construction.kp");
    p.max_angular = dbl("construction.max_angular");
    p.low_confidence = dbl("construction.low_confidence");
    p.slow_ratio = dbl("construction.slow_ratio");
    p.boxes = integer("construction.boxes");
    p.pass_low = dbl("construction.pass_low");
    p.pass_high = dbl("construction.pass_high");
    p.pass_low_time = dbl("construction.pass_low_time");
    p.pass_high_time = dbl("construction.pass_high_time");
    p.same_side_gap = dbl("construction.same_side_gap");
    p.no_box_time = dbl("construction.no_box_time");
    p.return_ok_count = integer("construction.return_ok_count");
    p.return_max_time = dbl("construction.return_max_time");
    p.avoid_max_time = dbl("construction.avoid_max_time");
    p.rearm_time = dbl("construction.rearm_time");
    return p;
  }

  void logZoneState()
  {
    if (zone_.takeChanged()) {
      RCLCPP_INFO(get_logger(), "[공사] %s -> %s", zone_.lastReason().c_str(), zone_.describe().c_str());
      std_msgs::msg::String s;
      s.data = zone_.describe() + " | " + zone_.lastReason();
      zone_pub_->publish(s);
    }
  }

  lane_follower::TurnParams readTurnParams()
  {
    lane_follower::TurnParams p;
    p.enabled = get_parameter("intersection.enabled").as_bool();
    p.sign_min_confidence = dbl("intersection.sign_min_confidence");
    p.sign_min_area = dbl("intersection.sign_min_area");
    p.sign_confirm_count = integer("intersection.sign_confirm_count");
    p.sign_confirm_window = integer("intersection.sign_confirm_window");
    p.sign_instant_confidence = dbl("intersection.sign_instant_confidence");
    p.arm_max_dist = dbl("intersection.arm_max_dist");
    p.require_cross_line = get_parameter("intersection.require_cross_line").as_bool();
    p.cross_recent_time = dbl("intersection.cross_recent_time");
    p.approach_align = get_parameter("intersection.approach_align").as_bool();
    p.approach_align_kp = dbl("intersection.approach_align_kp");
    p.approach_align_max = dbl("intersection.approach_align_max");
    p.approach_align_fresh = dbl("intersection.approach_align_fresh");
    p.approach_slow_linear = dbl("intersection.approach_slow_linear");
    p.align_enabled = get_parameter("intersection.align_enabled").as_bool();
    p.align_kp = dbl("intersection.align_kp");
    p.align_max_angular = dbl("intersection.align_max_angular");
    p.align_min_angular = dbl("intersection.align_min_angular");
    p.align_tol = dbl("intersection.align_tol");
    p.align_ok_count = integer("intersection.align_ok_count");
    p.align_max_time = dbl("intersection.align_max_time");
    p.align_lost_time = dbl("intersection.align_lost_time");
    p.approach_time = dbl("intersection.approach_time");
    p.approach_linear = dbl("intersection.approach_linear");
    p.turn_pivot = get_parameter("intersection.turn_pivot").as_bool();
    p.wheel_separation = dbl("wheel_separation");
    p.turn_linear = dbl("intersection.turn_linear");
    p.turn_angular = dbl("intersection.turn_angular");
    p.turn_search_angular = dbl("intersection.turn_search_angular");
    p.turn_min_yaw = dbl("intersection.turn_min_yaw");
    p.turn_max_yaw = dbl("intersection.turn_max_yaw");
    p.turn_switch_max_yaw = dbl("intersection.turn_switch_max_yaw");
    p.exit_ok_count = integer("intersection.exit_ok_count");
    p.exit_min_confidence = dbl("intersection.exit_min_confidence");
    p.exit_max_angle = dbl("intersection.exit_max_angle");
    p.exit_max_offset = dbl("intersection.exit_max_offset");
    p.exit_lead_angle = dbl("intersection.exit_lead_angle");
    p.exit_turn_side_only = get_parameter("intersection.exit_turn_side_only").as_bool();
    p.rearm_dist = dbl("intersection.rearm_dist");
    p.after_enabled = get_parameter("intersection.after_enabled").as_bool();
    p.after_dist = dbl("intersection.after_dist");
    p.after_max_yaw = dbl("intersection.after_max_yaw");
    p.after_max_retries = integer("intersection.after_max_retries");
    p.out_enabled = get_parameter("intersection.out_enabled").as_bool();
    p.out_trigger_yaw = dbl("intersection.out_trigger_yaw");
    p.out_island_offset = dbl("intersection.out_island_offset");
    p.out_island_min_yaw = dbl("intersection.out_island_min_yaw");
    p.out_min_dist = dbl("intersection.out_min_dist");
    p.out_max_dist = dbl("intersection.out_max_dist");
    p.out_approach_time = dbl("intersection.out_approach_time");
    p.out_turn_min_yaw = dbl("intersection.out_turn_min_yaw");
    p.out_turn_max_yaw = dbl("intersection.out_turn_max_yaw");
    p.out_follow = get_parameter("intersection.out_follow").as_bool();
    p.out_guard_dist = dbl("intersection.out_guard_dist");
    p.out_guard_max_offset = dbl("intersection.out_guard_max_offset");
    p.out_arc_linear = dbl("intersection.out_arc_linear");
    p.out_arc_angular = dbl("intersection.out_arc_angular");
    p.out_arc_max_dist = dbl("intersection.out_arc_max_dist");
    p.out_hold_time = dbl("intersection.out_hold_time");
    return p;
  }

  void logTurn()
  {
    if (turn_.takeChanged()) {
      RCLCPP_INFO(get_logger(), "[교차로] %s -> %s", turn_.lastReason().c_str(), turn_.describe().c_str());
    }
  }

  // 바퀴 속도를 적분해 이동 거리 / 회전을 교차로 회전에 넘긴다
  void onDxl(const interfaces::msg::DxlState & d)
  {
    const auto t = now();
    if (dxl_received_) {
      const double dt = std::min(0.3, (t - dxl_stamp_).seconds());
      const double v = (last_dxl_.left_velocity + last_dxl_.right_velocity) / 2;
      const double w = (last_dxl_.right_velocity - last_dxl_.left_velocity) / dbl("wheel_separation");
      turn_.onMove(v * dt, w * dt, readTurnParams());
      logTurn();
    }
    last_dxl_ = d;
    dxl_stamp_ = t;
    dxl_received_ = true;
  }

  bool fresh(const Lane & l, const rclcpp::Time & t)
  {
    return l.received && l.msg.detected && (t - l.stamp).seconds() <= dbl("lane_timeout");
  }

  // 파라미터는 매 tick 다시 읽는다 (GUI 슬라이더 / ros2 param set 으로 주행 중 튜닝 가능)
  lane_follower::Gains readGains()
  {
    lane_follower::Gains g;
    g.linear_speed = dbl("linear_speed");
    g.min_linear_speed = dbl("min_linear_speed");
    g.kp = dbl("kp");
    g.kd = dbl("kd");
    g.ka = dbl("ka");
    g.max_angular = dbl("max_angular");
    g.slow_down_angle = dbl("slow_down_angle");
    g.low_confidence = dbl("low_confidence");
    g.d_filter_alpha = dbl("d_filter_alpha");
    return g;
  }

  void tick()
  {
    const auto t = now();
    const Lane & lane = lane_;
    const bool driving_mode = mode_ == ControlMode::LANE || mode_ == ControlMode::AVOID;
    if (!driving_mode) {
      // 주행 모드에서 막 빠져나왔으면 정지 한 번, 그 뒤로는 발행하지 않는다
      if (was_driving_) {
        cmd_pub_->publish(geometry_msgs::msg::Twist());
        last_cmd_ = geometry_msgs::msg::Twist();
        controller_.reset();
        turn_.cancelManeuver();
        logTurn();
        zone_.cancel();
        logZoneState();
        was_driving_ = false;
      }
      return;
    }
    was_driving_ = true;

    geometry_msgs::msg::Twist cmd;   // 기본 = 정지
    const bool lane_ok = fresh(lane, t);
    const double stop_dist = dbl("front_stop_distance");
    const bool front_blocked = stop_dist > 0 && psd_received_ &&
      (t - psd_stamp_).seconds() <= dbl("psd_timeout") && psd_.front > 0 && psd_.front < stop_dist;

    // 교차로 회전: 표지판 방향을 기억해 두었다가 차선을 놓치면 직진 후 그쪽으로 돈다
    lane_follower::IntersectionTurn::Cmd turn_cmd;
    {
      const double tick_dt = last_tick_.nanoseconds() > 0 ? std::min(0.3, (t - last_tick_).seconds()) : 0.0;
      // dxl_state 가 안 오면 (stm 없이 시험) 지난 틱에 보낸 속도로 회전량 / 거리 추정
      if (!dxl_received_ || (t - dxl_stamp_).seconds() > 0.5) {
        turn_.onMove(last_cmd_.linear.x * tick_dt, last_cmd_.angular.z * tick_dt, readTurnParams());
      }
      lane_follower::IntersectionTurn::LaneObs obs;
      obs.lost = lane.received && !lane.msg.detected && (t - lane.stamp).seconds() <= dbl("lane_timeout");
      obs.ok = lane_ok;
      obs.fresh = lane.received && lane.stamp != last_turn_lane_;
      if (lane.received) {   // 안 받았으면 stamp 시계 종류가 달라 비교하면 예외
        last_turn_lane_ = lane.stamp;
      }
      obs.confidence = lane.msg.confidence;
      obs.angle = lane.msg.angle;
      obs.offset = lane.msg.offset;
      obs.left = lane.msg.left_detected;
      obs.right = lane.msg.right_detected;
      turn_cmd = turn_.update(obs, readTurnParams(), tick_dt);
      logTurn();
      if (turn_.phase() == lane_follower::IntersectionTurn::Phase::LOOP) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "[교차로] %s", turn_.describe().c_str());
      }
    }

    // 공사 구간: 박스를 피하는 동안은 교차로 회전 / 차선 추종 대신 wall_detection 이 고른 경로로 조향
    const auto zone_cmd = zone_.update(lane_ok, t.seconds(), readConstructionParams());
    logZoneState();

    if (zone_cmd.active && !front_blocked) {
      controller_.reset();
      last_tick_ok_ = false;
      cmd.linear.x = zone_cmd.v;
      cmd.angular.z = zone_cmd.w;
    } else if (turn_cmd.active && !front_blocked) {
      controller_.reset();
      last_tick_ok_ = false;
      cmd.linear.x = turn_cmd.v;
      cmd.angular.z = turn_cmd.w;
      if (turn_cmd.hold && have_follow_cmd_) {   // 고리 출구에서 차선을 잠깐 놓침 -> 휘던 대로
        cmd = last_follow_cmd_;
      }
    } else if (lane_ok && !front_blocked) {
      const double dt = last_tick_ok_ ? (t - last_tick_).seconds() : 0.0;
      const auto c = controller_.step(lane.msg.offset, lane.msg.angle, lane.msg.confidence, dt,
          readGains());
      cmd.linear.x = c.v;
      cmd.angular.z = c.w;
      last_tick_ok_ = true;
      last_follow_cmd_ = cmd;
      have_follow_cmd_ = true;
      // 교차로 표지판을 본 뒤 T자 가로 선이 보이면 다가가면서 가로 선에 수평이 되게 조향을 더한다
      double w_add = 0, v_max = 0;
      const bool assist = turn_.crossAssist(readTurnParams(), w_add, v_max);
      if (assist) {
        const double max_w = dbl("max_angular");
        cmd.angular.z = std::clamp(cmd.angular.z + w_add, -max_w, max_w);
        if (v_max > 0) {
          cmd.linear.x = std::min(cmd.linear.x, v_max);
        }
        if (!cross_assist_) {
          RCLCPP_INFO(get_logger(), "[교차로] 가로 선 보며 다가가며 정렬 (기울기 %.1f°, 조향 %+.2f)",
            turn_.crossAngle() * 57.2958, w_add);
        }
      }
      cross_assist_ = assist;
    } else {
      controller_.reset();
      last_tick_ok_ = false;
      const char * why = front_blocked ? "정면 PSD 가까움" :
        !lane.received ? "lane_info 없음" : !lane.msg.detected ? "차선 못 찾음" : "lane_info 끊김";
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "정지: %s", why);
    }
    last_tick_ = t;
    last_cmd_ = cmd;
    cmd_pub_->publish(cmd);
  }

  lane_follower::Controller controller_;
  uint8_t mode_;
  bool was_driving_ = false;

  Lane lane_;
  lane_follower::IntersectionTurn turn_;   // 교차로 회전
  lane_follower::ConstructionZone zone_;   // 공사 구간
  bool cross_assist_ = false;   // 다가가며 정렬 중 (로그 한 번만)
  geometry_msgs::msg::Twist last_follow_cmd_;   // 마지막 차선 추종 명령 (고리 출구에서 차선 놓쳤을 때 유지)
  bool have_follow_cmd_ = false;
  rclcpp::Time last_turn_lane_{0, 0, RCL_ROS_TIME};   // 교차로 회전에 마지막으로 넘긴 lane_info
  interfaces::msg::DxlState last_dxl_;
  rclcpp::Time dxl_stamp_;
  bool dxl_received_ = false;
  interfaces::msg::PsdArray psd_;
  rclcpp::Time psd_stamp_;
  bool psd_received_ = false;
  rclcpp::Time last_tick_;
  geometry_msgs::msg::Twist last_cmd_;   // dxl_state 없을 때 이동량 추정용
  bool last_tick_ok_ = false;

  rclcpp::Subscription<interfaces::msg::LaneInfo>::SharedPtr lane_sub_;
  rclcpp::Subscription<interfaces::msg::Sign>::SharedPtr sign_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr cross_sub_;
  rclcpp::Subscription<interfaces::msg::WallObstacle>::SharedPtr wall_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr zone_pub_;
  rclcpp::Subscription<interfaces::msg::DxlState>::SharedPtr dxl_sub_;
  rclcpp::Subscription<ControlMode>::SharedPtr mode_sub_;
  rclcpp::Subscription<interfaces::msg::PsdArray>::SharedPtr psd_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaneFollowerNode>());
  rclcpp::shutdown();
  return 0;
}
