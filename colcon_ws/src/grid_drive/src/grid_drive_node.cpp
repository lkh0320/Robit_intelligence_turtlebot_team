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

    // 계획기 파라미터 (나머지는 grid_planner.hpp 기본값)
    const std::vector<std::pair<const char *, double *>> dparams = {
      {"linear", &p_.linear}, {"min_linear", &p_.min_linear},
      {"max_angular", &p_.max_angular}, {"pivot_angular", &p_.pivot_angular},
      {"soft_speed_ratio", &p_.soft_speed_ratio}, {"follow_mm", &p_.follow_mm},
      {"robot_half_width_mm", &p_.robot_half_width_mm}, {"line_margin_mm", &p_.line_margin_mm},
      {"box_margin_mm", &p_.box_margin_mm}, {"map_y_scale", &p_.map_y_scale},
      {"road_min_ratio", &p_.road_min_ratio},
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
    sign_sub_ = create_subscription<Sign>(
      "sign", rclcpp::SensorDataQoS(), [this](Sign::ConstSharedPtr m) {
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
    if (!rs.mask.empty()) {
      grid_drive::SegMasks sm;
      sm.road = rs.mask;
      grid_drive::merge(obs, grid_drive::observeSegmentation(sm, p_));
    }
    if (turn_ != grid_drive::Turn::NONE && now() > turn_until_) {
      turn_ = grid_drive::Turn::NONE;
    }
    const grid_drive::Result r = planner_.plan(std::move(obs), p_, turn_);
    const double ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();

    if (driving()) {
      publishCmd(r.v, r.w);
      stop_sent_ = false;
    } else if (!stop_sent_) {
      publishCmd(0, 0);
      stop_sent_ = true;
    }
    if (!r.path_ok || r.pivot) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "%s: %s",
        r.pivot ? "제자리 회전" : "경로 없음", r.reason.c_str());
    }
    RCLCPP_DEBUG(get_logger(), "v %.3f w %.3f soft %d road %d (%.1fms) 전체 %.1fms",
      r.v, r.w, r.soft, rs.detections, ms_road, ms);

    if (debug_pub_->get_subscription_count() > 0) {
      publishDebug(img, rs, r, ms_road, ms, msg->header);
    }
  }

  // 왼쪽: 원본 + 도로 마스크 (초록), 오른쪽: 계획 그림 (GridPlanner::draw)
  void publishDebug(const cv::Mat & img, const grid_drive::RoadSeg & rs, const grid_drive::Result & r,
    double ms_road, double ms, const std_msgs::msg::Header & header)
  {
    cv::Mat left = img.clone();
    if (!rs.mask.empty()) {
      cv::Mat tint(img.size(), CV_8UC3, cv::Scalar(0, 200, 0));
      cv::Mat mixed;
      cv::addWeighted(img, 0.6, tint, 0.4, 0, mixed);
      mixed.copyTo(left, rs.mask);
    }
    const char * turn = turn_ == grid_drive::Turn::LEFT ? "L" : turn_ == grid_drive::Turn::RIGHT ? "R" : "-";
    char text[160];
    std::snprintf(text, sizeof(text), "%s v%.2f w%.2f %s%s turn %s road %d %.2f  %.0f/%.0fms",
      driving() ? "RUN" : "STOP", r.v, r.w, r.soft ? "SOFT " : "", r.pivot ? "PIVOT " : "", turn,
      rs.detections, rs.best_conf, ms_road, ms);
    cv::putText(left, text, {6, 20}, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2);

    cv::Mat right = planner_.draw(r, p_), out;
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
