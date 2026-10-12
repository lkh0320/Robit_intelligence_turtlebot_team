// 라인 추적 주행: lane_info -> cmd_vel
//   구독: lane_info (interfaces/LaneInfo)        path_planner / lane_detection / map_planner
//         control_mode (interfaces/ControlMode)  task_planner / GUI 모드 버튼
//         psd (interfaces/PsdArray)              정면 비상 정지
//   발행: cmd_vel (geometry_msgs/Twist)          stm_bridge 가 바퀴로 보낸다
//
// 운행 모드가 LANE / AVOID 일 때만 주행한다 (AVOID 도 주행: path_planner 는 회피를 경로로 처리).
//   그 외 모드(STOP, PARKING, MANUAL)가 되면 정지 명령을 한 번 보내고 조용히 있는다
//   (GUI 수동 주행이나 주차 기동 노드의 cmd_vel 과 싸우지 않도록)
// autostart: true 면 control_mode 를 받기 전에도 LANE 으로 시작 (task_planner 없이 주행 시험).
//   GUI STOP 버튼(control_mode STOP)을 누르면 그대로 멈춘다.
// 안전: lane_info 가 lane_timeout 동안 안 오거나 detected = false 면 정지,
//       정면 PSD 가 front_stop_distance 보다 가까우면 정지
// 조향 게인은 lane_method(path / sliding / map) 이름 묶음에서 고른다 (config/lane_follower.yaml)
// 제어 법칙은 include/lane_follower/controller.hpp
//
// lane_method:=hybrid: lane_detection(lane_info/sliding) 과 path_planner(lane_info/path) 를 같이 받아
//   평소엔 sliding, 갈림길 / 공사 구간에서만 path 로 주행한다. 고르는 규칙은 include/lane_follower/drive_selector.hpp
//   추가 구독: sign, box_obstacle (path_planner), dxl_state (바퀴 속도 -> 이동 거리 / 회전)
//   추가 발행: lane_info (고른 쪽을 그대로, GUI 표시용), lane_follower/drive_state (std_msgs/String, 구간 상태)
//             drive_hint (interfaces/Sign)  지금 구간 -> path_planner (갈림길 방향 / 공사 구간, 그 외 NONE)
//             vision/lane_debug/compressed  GUI 화면: 평소엔 lane_detection 화면(vision/sliding_debug),
//                                           갈림길 / 공사 구간(복귀 중 포함)엔 path_planner 화면(vision/path_debug)을 중계
#include <algorithm>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "interfaces/msg/control_mode.hpp"
#include "interfaces/msg/dxl_state.hpp"
#include "interfaces/msg/lane_info.hpp"
#include "interfaces/msg/psd_array.hpp"
#include "interfaces/msg/sign.hpp"
#include "interfaces/msg/wall_obstacle.hpp"
#include "lane_follower/controller.hpp"
#include "lane_follower/drive_selector.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/string.hpp"

class LaneFollowerNode : public rclcpp::Node
{
  using ControlMode = interfaces::msg::ControlMode;

public:
  LaneFollowerNode()
  : Node("lane_follower")
  {
    const lane_follower::Gains g;
    declare_parameter("lane_method", std::string("path"));   // path / sliding / map / hybrid
    declare_parameter("linear_speed", g.linear_speed);
    declare_parameter("min_linear_speed", g.min_linear_speed);
    declare_parameter("max_angular", g.max_angular);
    declare_parameter("low_confidence", g.low_confidence);
    declare_parameter("d_filter_alpha", g.d_filter_alpha);
    // 조향 게인은 선 검출 방식마다 따로 (offset/angle 뜻이 달라서). 기본값은 config/lane_follower.yaml 과 같게
    struct Steer {const char * method; double pp, kp, kd, ka, slow;};
    for (const auto & s : {Steer{"path", 0.20, 0, 0, 0, 0.35}, Steer{"sliding", 0, 0.674, 0.0042, 0.253, 0.5},
        Steer{"map", 0.25, 0, 0, 0, 0.5}})
    {
      const std::string m = s.method;
      declare_parameter(m + ".pp_lookahead", s.pp);
      declare_parameter(m + ".kp", s.kp);
      declare_parameter(m + ".kd", s.kd);
      declare_parameter(m + ".ka", s.ka);
      declare_parameter(m + ".slow_down_angle", s.slow);
    }
    declare_parameter("lane_timeout", 0.7);
    declare_parameter("front_stop_distance", 0.10);   // 0 이하면 끔
    declare_parameter("psd_timeout", 0.5);
    const bool autostart = declare_parameter("autostart", false);
    const double rate = declare_parameter("control_rate", 20.0);
    declareHybridParams();

    mode_ = autostart ? ControlMode::LANE : ControlMode::STOP;
    // hybrid 는 시작할 때 정한다 (구독 토픽이 달라서 주행 중 전환 불가)
    hybrid_ = get_parameter("lane_method").as_string() == "hybrid";
    sel_params_ = readSelectorParams();

    const auto qos = rclcpp::SensorDataQoS();
    auto lane_cb = [this](Lane & l) {
        return [this, &l](interfaces::msg::LaneInfo::ConstSharedPtr msg) {
                 l.msg = *msg;
                 l.stamp = now();
                 l.received = true;
               };
      };
    if (hybrid_) {
      sliding_sub_ = create_subscription<interfaces::msg::LaneInfo>(
        "lane_info/sliding", qos, lane_cb(sliding_));
      path_sub_ = create_subscription<interfaces::msg::LaneInfo>(
        "lane_info/path", qos, lane_cb(path_));
      sign_sub_ = create_subscription<interfaces::msg::Sign>(
        "sign", qos, [this](interfaces::msg::Sign::ConstSharedPtr m) {
          selector_.onSign(m->type, m->confidence, m->area_ratio, sel_params_);
          logZone();
        });
      box_sub_ = create_subscription<interfaces::msg::WallObstacle>(
        "box_obstacle", qos, [this](interfaces::msg::WallObstacle::ConstSharedPtr m) {
          selector_.onBox(m->detected, m->distance, m->bottom_ratio, sel_params_);
          logZone();
        });
      dxl_sub_ = create_subscription<interfaces::msg::DxlState>(
        "dxl_state", qos, [this](interfaces::msg::DxlState::ConstSharedPtr m) {onDxl(*m);});
      selected_pub_ = create_publisher<interfaces::msg::LaneInfo>("lane_info", qos);
      state_pub_ = create_publisher<std_msgs::msg::String>("lane_follower/drive_state", 10);
      hint_pub_ = create_publisher<interfaces::msg::Sign>("drive_hint", 10);
      debug_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
        "vision/lane_debug/compressed", qos);
    } else {
      lane_sub_ = create_subscription<interfaces::msg::LaneInfo>("lane_info", qos, lane_cb(lane_));
    }
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
      });
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this] {tick();});

    RCLCPP_INFO(get_logger(), "lane_follower 시작 (lane_method: %s, %s)",
      hybrid_ ? "hybrid (sliding + 갈림길/공사 구간 path)" : method().c_str(),
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

  void declareHybridParams()
  {
    const lane_follower::SelectorParams d;
    declare_parameter("wheel_separation", 0.190);   // stm_bridge 와 같게 [m]
    declare_parameter("hybrid.enabled", d.enabled);
    declare_parameter("hybrid.path_fallback", d.path_fallback);
    declare_parameter("hybrid.sign_min_confidence", d.sign_min_confidence);
    declare_parameter("hybrid.sign_min_area", d.sign_min_area);
    declare_parameter("hybrid.sign_confirm_count", d.sign_confirm_count);
    declare_parameter("hybrid.sign_confirm_window", d.sign_confirm_window);
    declare_parameter("hybrid.fork_exit_yaw", d.fork_exit_yaw);
    declare_parameter("hybrid.fork_max_dist", d.fork_max_dist);
    declare_parameter("hybrid.box_confirm_count", d.box_confirm_count);
    declare_parameter("hybrid.box_enter_dist", d.box_enter_dist);
    declare_parameter("hybrid.construction_boxes", d.construction_boxes);
    declare_parameter("hybrid.box_match_tol", d.box_match_tol);
    declare_parameter("hybrid.box_pass_margin", d.box_pass_margin);
    declare_parameter("hybrid.box_min_hits", d.box_min_hits);
    declare_parameter("hybrid.box_clip_ratio", d.box_clip_ratio);
    declare_parameter("hybrid.construction_lost_dist", d.construction_lost_dist);
    declare_parameter("hybrid.construction_no_box_dist", d.construction_no_box_dist);
    declare_parameter("hybrid.construction_max_dist", d.construction_max_dist);
    declare_parameter("hybrid.return_ok_count", d.return_ok_count);
    declare_parameter("hybrid.return_min_confidence", d.return_min_confidence);
    declare_parameter("hybrid.return_max_offset", d.return_max_offset);
    declare_parameter("hybrid.return_max_dist", d.return_max_dist);
    declare_parameter("hybrid.rearm_dist", d.rearm_dist);
  }

  lane_follower::SelectorParams readSelectorParams()
  {
    lane_follower::SelectorParams p;
    p.enabled = get_parameter("hybrid.enabled").as_bool();
    p.path_fallback = get_parameter("hybrid.path_fallback").as_bool();
    p.sign_min_confidence = dbl("hybrid.sign_min_confidence");
    p.sign_min_area = dbl("hybrid.sign_min_area");
    p.sign_confirm_count = integer("hybrid.sign_confirm_count");
    p.sign_confirm_window = integer("hybrid.sign_confirm_window");
    p.fork_exit_yaw = dbl("hybrid.fork_exit_yaw");
    p.fork_max_dist = dbl("hybrid.fork_max_dist");
    p.box_confirm_count = integer("hybrid.box_confirm_count");
    p.box_enter_dist = dbl("hybrid.box_enter_dist");
    p.construction_boxes = integer("hybrid.construction_boxes");
    p.box_match_tol = dbl("hybrid.box_match_tol");
    p.box_pass_margin = dbl("hybrid.box_pass_margin");
    p.box_min_hits = integer("hybrid.box_min_hits");
    p.box_clip_ratio = dbl("hybrid.box_clip_ratio");
    p.construction_lost_dist = dbl("hybrid.construction_lost_dist");
    p.construction_no_box_dist = dbl("hybrid.construction_no_box_dist");
    p.construction_max_dist = dbl("hybrid.construction_max_dist");
    p.return_ok_count = integer("hybrid.return_ok_count");
    p.return_min_confidence = dbl("hybrid.return_min_confidence");
    p.return_max_offset = dbl("hybrid.return_max_offset");
    p.return_max_dist = dbl("hybrid.return_max_dist");
    p.rearm_dist = dbl("hybrid.rearm_dist");
    return p;
  }

  // 바퀴 속도를 적분해 이동 거리 / 회전을 selector 에 넘긴다 (path_planner 와 같은 방식)
  void onDxl(const interfaces::msg::DxlState & d)
  {
    const auto t = now();
    if (dxl_received_) {
      const double dt = std::min(0.3, (t - dxl_stamp_).seconds());
      const double v = (last_dxl_.left_velocity + last_dxl_.right_velocity) / 2;
      const double w = (last_dxl_.right_velocity - last_dxl_.left_velocity) / dbl("wheel_separation");
      selector_.onMove(v * dt, w * dt, sel_params_);
    }
    last_dxl_ = d;
    dxl_stamp_ = t;
    dxl_received_ = true;
  }

  // 구간이 바뀌면 로그 + drive_state 발행
  void logZone()
  {
    const auto z = selector_.zone();
    if (z == last_logged_zone_) {
      return;
    }
    last_logged_zone_ = z;
    RCLCPP_INFO(get_logger(), "[hybrid] %s -> %s", selector_.lastReason().c_str(),
      selector_.describe().c_str());
    publishState();
  }

  void publishState()
  {
    std_msgs::msg::String s;
    s.data = selector_.describe() + (source_ == lane_follower::Source::PATH ? " | path" : " | sliding");
    state_pub_->publish(s);
  }

  bool fresh(const Lane & l, const rclcpp::Time & t)
  {
    return l.received && l.msg.detected && (t - l.stamp).seconds() <= dbl("lane_timeout");
  }

  // 알 수 없는 값이면 path 게인으로 (한 번만 경고). hybrid 는 지금 고른 쪽 게인
  std::string method()
  {
    if (hybrid_) {
      return source_ == lane_follower::Source::PATH ? "path" : "sliding";
    }
    const auto m = get_parameter("lane_method").as_string();
    if (m == "path" || m == "sliding" || m == "map") {
      return m;
    }
    RCLCPP_WARN_ONCE(get_logger(), "lane_method '%s' 모름 -> path 게인 사용", m.c_str());
    return "path";
  }

  // 파라미터는 매 tick 다시 읽는다 (GUI 슬라이더 / ros2 param set 으로 주행 중 튜닝 가능)
  lane_follower::Gains readGains()
  {
    const auto m = method() + ".";
    lane_follower::Gains g;
    g.linear_speed = dbl("linear_speed");
    g.min_linear_speed = dbl("min_linear_speed");
    g.kp = dbl(m + "kp");
    g.kd = dbl(m + "kd");
    g.ka = dbl(m + "ka");
    g.pp_lookahead = dbl(m + "pp_lookahead");
    g.max_angular = dbl("max_angular");
    g.slow_down_angle = dbl(m + "slow_down_angle");
    g.low_confidence = dbl("low_confidence");
    g.d_filter_alpha = dbl("d_filter_alpha");
    return g;
  }

  void tick()
  {
    const auto t = now();
    // hybrid 는 멈춰 있어도 고르기 / lane_info 중계는 계속 (GUI 표시, 구간 상태)
    const Lane & lane = hybrid_ ? selectLane(t) : lane_;
    const bool driving_mode = mode_ == ControlMode::LANE || mode_ == ControlMode::AVOID;
    if (!driving_mode) {
      // 주행 모드에서 막 빠져나왔으면 정지 한 번, 그 뒤로는 발행하지 않는다
      if (was_driving_) {
        cmd_pub_->publish(geometry_msgs::msg::Twist());
        controller_.reset();
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

    if (lane_ok && !front_blocked) {
      const double dt = last_tick_ok_ ? (t - last_tick_).seconds() : 0.0;
      const auto c = controller_.step(lane.msg.offset, lane.msg.angle, lane.msg.confidence, dt,
          readGains());
      cmd.linear.x = c.v;
      cmd.angular.z = c.w;
      last_tick_ok_ = true;
    } else {
      controller_.reset();
      last_tick_ok_ = false;
      const char * why = front_blocked ? "정면 PSD 가까움" :
        !lane.received ? "lane_info 없음" : !lane.msg.detected ? "차선 못 찾음" : "lane_info 끊김";
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "정지: %s", why);
    }
    last_tick_ = t;
    cmd_pub_->publish(cmd);
  }

  // hybrid: 이번 틱에 쓸 lane_info. 바뀌면 미분항이 튀지 않게 제어기를 초기화
  const Lane & selectLane(const rclcpp::Time & t)
  {
    sel_params_ = readSelectorParams();
    const auto & s = sliding_.msg;
    const lane_follower::LaneStatus st{fresh(sliding_, t), s.offset, s.confidence};
    const auto src = selector_.update(st, fresh(path_, t), sel_params_);
    logZone();
    if (src != source_) {
      source_ = src;
      controller_.reset();
      last_tick_ok_ = false;
      RCLCPP_INFO(get_logger(), "[hybrid] 주행 입력 -> %s (%s)",
        src == lane_follower::Source::PATH ? "path" : "sliding", selector_.describe().c_str());
      publishState();
    }
    // path_planner 는 이 구간 정보로 갈림길 가점 / 공사 구간 선 넘기 허용을 정한다 (매 틱)
    interfaces::msg::Sign hint;
    hint.header.stamp = t;
    hint.type = static_cast<uint8_t>(selector_.hint());
    hint.confidence = 1.0f;
    hint.area_ratio = 1.0f;
    hint_pub_->publish(hint);

    const Lane & l = src == lane_follower::Source::PATH ? path_ : sliding_;
    if (l.received && l.stamp != last_relayed_) {
      selected_pub_->publish(l.msg);   // GUI 는 lane_info 를 본다
      last_relayed_ = l.stamp;
    }
    if (++state_tick_ % 10 == 0) {
      publishState();   // 0.5초마다 (20Hz 기준) 상태 문자열
    }
    updateDebugRelay();
    return l;
  }

  // GUI 화면 중계: 구간에 맞는 쪽 화면 하나만, 보는 사람이 있을 때만 구독한다
  // (lane_detection / path_planner 는 구독자가 있을 때만 화면을 그리므로 안 보는 쪽은 그리지도 않음).
  // 틱마다 바뀌는 path_fallback 이 아니라 구간으로 고른다 (구독을 자주 다시 만들지 않게)
  void updateDebugRelay()
  {
    const int want = debug_pub_->get_subscription_count() == 0 ? -1 :
      selector_.zone() == lane_follower::Zone::NORMAL ? 0 : 1;
    if (want == debug_view_) {
      return;
    }
    debug_view_ = want;
    debug_sub_.reset();
    if (want < 0) {
      return;
    }
    debug_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      want == 0 ? "vision/sliding_debug/compressed" : "vision/path_debug/compressed",
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr m) {debug_pub_->publish(*m);});
  }

  lane_follower::Controller controller_;
  uint8_t mode_;
  bool was_driving_ = false;

  Lane lane_;                 // hybrid 가 아닐 때
  Lane sliding_, path_;       // hybrid
  bool hybrid_ = false;
  lane_follower::DriveSelector selector_;
  lane_follower::SelectorParams sel_params_;
  lane_follower::Source source_ = lane_follower::Source::SLIDING;
  lane_follower::Zone last_logged_zone_ = lane_follower::Zone::NORMAL;
  rclcpp::Time last_relayed_{0, 0, RCL_ROS_TIME};   // now() 와 같은 시계 종류여야 비교 가능
  int state_tick_ = 0;
  interfaces::msg::DxlState last_dxl_;
  rclcpp::Time dxl_stamp_;
  bool dxl_received_ = false;
  interfaces::msg::PsdArray psd_;
  rclcpp::Time psd_stamp_;
  bool psd_received_ = false;
  rclcpp::Time last_tick_;
  bool last_tick_ok_ = false;

  rclcpp::Subscription<interfaces::msg::LaneInfo>::SharedPtr lane_sub_, sliding_sub_, path_sub_;
  rclcpp::Subscription<interfaces::msg::Sign>::SharedPtr sign_sub_;
  rclcpp::Subscription<interfaces::msg::WallObstacle>::SharedPtr box_sub_;
  rclcpp::Subscription<interfaces::msg::DxlState>::SharedPtr dxl_sub_;
  rclcpp::Publisher<interfaces::msg::LaneInfo>::SharedPtr selected_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<interfaces::msg::Sign>::SharedPtr hint_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debug_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr debug_sub_;
  int debug_view_ = -1;   // 중계 중인 화면: -1 없음, 0 sliding, 1 path
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
