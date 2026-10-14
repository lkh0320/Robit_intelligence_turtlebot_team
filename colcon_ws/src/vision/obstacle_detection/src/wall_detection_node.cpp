// 장애물 인식: 공사 구간 갈색 박스(벽) + 피해 갈 방향
//   mode: planner (기본) = include/obstacle_detection/box_planner.hpp: 원본을 넓게 펴서 로봇 폭으로 지나갈 경로를
//         고르고, 카메라 아래로 사라진 박스를 바퀴 이동량으로 기억 (2026-10-14 실주행: free_space 방식은
//         첫 박스에서 너무 일찍 꺾고, 나머지는 지나가며 옆을 긁음)
//         free_space = 예전 방식 (화면에서 가장 멀리 빈 열 방향)
//   구독: image_raw (sensor_msgs/Image, 원본 카메라)
//         dxl_state (interfaces/DxlState, planner: 바퀴 속도 -> 박스 기억 옮기기)
//         psd (interfaces/PsdArray, planner: 옆이 가까우면 그쪽으로 붙는 경로 탈락)
//   발행: wall_obstacle (interfaces/WallObstacle)
//         planner 모드에서 free_offset / confidence / distance 뜻이 다르다:
//           free_offset  고른 경로 위 목표점 방향 [rad] (+ 오른쪽, ±1 로 자름) -> lane_follower w = -kp * 이 값
//           confidence   1 = 지나갈 경로 있음, fallback_confidence = 전부 막혀 가장 멀리 가는 경로 (천천히)
//           distance     가장 가까운 박스 (기억 포함) 까지 [지도 m, 엔코더의 약 odom_scale 배]
//           detected / side / x_ratio / bottom_ratio 는 지금 화면에 보이는 가장 가까운 박스 (free_space 와 같은 뜻)
//         free_space 모드:
//           detected     박스가 보이나 (box_min_area 넘는 덩어리)
//           side         가장 가까운 박스가 화면 왼쪽(LEFT) / 오른쪽(RIGHT)
//           x_ratio      가장 가까운 박스 가로 중심 [-1, 1] (+ 오른쪽)
//           bottom_ratio 가장 가까운 박스 아래 끝 [0 위, 1 아래], 클수록 가까움
//           area_ratio   박스 덩어리 면적 합 / 화면 면적
//           free_offset  박스 / 선 없이 가장 멀리 비어 있는 방향 [-1, 1] (+ 오른쪽) -> lane_follower 공사 구간 조향
//           confidence   고른 방향이 얼마나 멀리 비었나 [0, 1] (1 = 화면 맨 위까지)
//           distance     0 (미터 보정 없음)
//         vision/wall_debug/compressed (보는 사람이 있을 때만)
//   계산은 include/obstacle_detection/free_space.hpp. 원본 30Hz 를 process_rate 로 솎아서 처리
//   BEV 대신 원본을 쓰는 이유: BEV 는 앞 18~58cm, 좌우 ±20cm 만 보여 박스를 늦게 본다 (원본은 1.2m 넘게)
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "cv_bridge/cv_bridge.hpp"
#include "interfaces/msg/dxl_state.hpp"
#include "interfaces/msg/psd_array.hpp"
#include "interfaces/msg/wall_obstacle.hpp"
#include "obstacle_detection/box_planner.hpp"
#include "obstacle_detection/free_space.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

class WallDetectionNode : public rclcpp::Node
{
public:
  WallDetectionNode()
  : Node("wall_detection")
  {
    const obstacle_detection::FreeSpaceParams d;
    const auto image_topic = declare_parameter("image_topic", std::string("image_raw"));
    declare_parameter("mode", std::string("planner"));   // planner | free_space
    declare_parameter("process_rate", 10.0);   // [Hz] 0 이하 = 모든 프레임
    declare_parameter("work_width", d.work_width);
    declare_parameter("work_height", d.work_height);
    declare_parameter("box_l_min", d.box_l_min);
    declare_parameter("box_l_max", d.box_l_max);
    declare_parameter("box_a_min", d.box_a_min);
    declare_parameter("box_a_max", d.box_a_max);
    declare_parameter("box_b_min", d.box_b_min);
    declare_parameter("box_b_max", d.box_b_max);
    declare_parameter("box_min_area", d.box_min_area);
    declare_parameter("yellow_l_min", d.yellow_l_min);
    declare_parameter("yellow_a_max", d.yellow_a_max);
    declare_parameter("yellow_b_min", d.yellow_b_min);
    declare_parameter("white_l_min", d.white_l_min);
    declare_parameter("white_ab_dev", d.white_ab_dev);
    declare_parameter("robot_width_px", d.robot_width_px);
    declare_parameter("center_bias", d.center_bias);
    declarePlannerParams();
    declare_parameter("debug_jpeg_quality", 70);
    last_box_t_ = now();   // 기본 생성 Time 은 시계 종류가 달라 빼면 예외

    const auto qos = rclcpp::SensorDataQoS();
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, qos, [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {onImage(msg);});
    dxl_sub_ = create_subscription<interfaces::msg::DxlState>(
      "dxl_state", qos, [this](interfaces::msg::DxlState::ConstSharedPtr msg) {onDxl(*msg);});
    psd_sub_ = create_subscription<interfaces::msg::PsdArray>(
      "psd", qos, [this](interfaces::msg::PsdArray::ConstSharedPtr msg) {
        psd_ = *msg;
        psd_t_ = now();
        have_psd_ = true;
      });
    wall_pub_ = create_publisher<interfaces::msg::WallObstacle>("wall_obstacle", qos);
    debug_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      "vision/wall_debug/compressed", qos);

    RCLCPP_INFO(get_logger(), "wall_detection 시작 (구독: %s, 방식: %s)", image_topic.c_str(),
      get_parameter("mode").as_string().c_str());
  }

private:
  int intParam(const std::string & n) {return static_cast<int>(get_parameter(n).as_int());}
  double dbl(const std::string & n) {return get_parameter(n).as_double();}

  obstacle_detection::FreeSpaceParams readParams()
  {
    obstacle_detection::FreeSpaceParams p;
    p.work_width = intParam("work_width");
    p.work_height = intParam("work_height");
    p.box_l_min = intParam("box_l_min");
    p.box_l_max = intParam("box_l_max");
    p.box_a_min = intParam("box_a_min");
    p.box_a_max = intParam("box_a_max");
    p.box_b_min = intParam("box_b_min");
    p.box_b_max = intParam("box_b_max");
    p.box_min_area = intParam("box_min_area");
    p.yellow_l_min = intParam("yellow_l_min");
    p.yellow_a_max = intParam("yellow_a_max");
    p.yellow_b_min = intParam("yellow_b_min");
    p.white_l_min = intParam("white_l_min");
    p.white_ab_dev = intParam("white_ab_dev");
    p.robot_width_px = intParam("robot_width_px");
    p.center_bias = get_parameter("center_bias").as_double();
    return p;
  }

  // planner.* : 바닥 지도 / 후보 / 점수 (box_planner.hpp PlannerParams), planner.color.* : 색 (color_maps.hpp)
  void declarePlannerParams()
  {
    const obstacle_detection::PlannerParams p;
    declare_parameter("planner.src_top_y", p.src_top_y);
    declare_parameter("planner.src_top_left_x", p.src_top_left_x);
    declare_parameter("planner.src_top_right_x", p.src_top_right_x);
    declare_parameter("planner.src_bottom_y", p.src_bottom_y);
    declare_parameter("planner.src_bottom_left_x", p.src_bottom_left_x);
    declare_parameter("planner.src_bottom_right_x", p.src_bottom_right_x);
    declare_parameter("planner.bev_near_mm", p.bev_near_mm);
    declare_parameter("planner.cell_mm", p.cell_mm);
    declare_parameter("planner.half_width_mm", p.half_width_mm);
    declare_parameter("planner.far_mm", p.far_mm);
    declare_parameter("planner.robot_half_width_mm", p.robot_half_width_mm);
    declare_parameter("planner.box_margin_mm", p.box_margin_mm);
    declare_parameter("planner.box_depth_mm", p.box_depth_mm);
    declare_parameter("planner.react_mm", p.react_mm);
    declare_parameter("planner.odom_scale", p.odom_scale);
    declare_parameter("planner.memory_back_mm", p.memory_back_mm);
    declare_parameter("planner.psd_keep_mm", p.psd_keep_mm);
    declare_parameter("planner.psd_len_mm", p.psd_len_mm);
    declare_parameter("planner.psd_close", 0.25);          // [m] 옆 PSD 가 이보다 가까우면 박스 옆
    declare_parameter("planner.psd_timeout", 0.5);         // [s]
    declare_parameter("planner.dxl_timeout", 0.5);         // [s] 이보다 오래 끊긴 구간은 이동량 버림
    declare_parameter("planner.wheel_separation", 0.190);  // [m] stm_bridge 와 같게
    declare_parameter("planner.max_shift_mm", p.max_shift_mm);
    declare_parameter("planner.line_weight", p.line_weight);
    declare_parameter("planner.cross_blocks", p.cross_blocks);
    declare_parameter("planner.cross_weight", p.cross_weight);
    declare_parameter("planner.line_horizon_mm", p.line_horizon_mm);
    declare_parameter("planner.clearance_weight", p.clearance_weight);
    declare_parameter("planner.shift_weight", p.shift_weight);
    declare_parameter("planner.curve_weight", p.curve_weight);
    declare_parameter("planner.smooth_weight", p.smooth_weight);
    declare_parameter("planner.lookahead_mm", p.lookahead_mm);
    declare_parameter("planner.fallback_confidence", 0.3);
    declare_parameter("planner.reset_after", 3.0);         // [s] 박스가 이만큼 안 보이면 기억 지움
    const obstacle_detection::ColorParams c;
    declare_parameter("planner.color.yellow_l_min", c.yellow_l_min);
    declare_parameter("planner.color.yellow_b_min", c.yellow_b_min);
    declare_parameter("planner.color.yellow_a_max", c.yellow_a_max);
    declare_parameter("planner.color.white_l_min", c.white_l_min);
    declare_parameter("planner.color.white_ab_dev", c.white_ab_dev);
    declare_parameter("planner.color.box_l_min", c.box_l_min);
    declare_parameter("planner.color.box_l_max", c.box_l_max);
    declare_parameter("planner.color.box_a_min", c.box_a_min);
    declare_parameter("planner.color.box_a_max", c.box_a_max);
    declare_parameter("planner.color.box_b_min", c.box_b_min);
    declare_parameter("planner.color.box_b_max", c.box_b_max);
    declare_parameter("planner.color.box_min_b_minus_a", c.box_min_b_minus_a);
    declare_parameter("planner.color.box_min_area", c.box_min_area);
  }

  obstacle_detection::PlannerParams readPlannerParams()
  {
    obstacle_detection::PlannerParams p;
    p.src_top_y = dbl("planner.src_top_y");
    p.src_top_left_x = dbl("planner.src_top_left_x");
    p.src_top_right_x = dbl("planner.src_top_right_x");
    p.src_bottom_y = dbl("planner.src_bottom_y");
    p.src_bottom_left_x = dbl("planner.src_bottom_left_x");
    p.src_bottom_right_x = dbl("planner.src_bottom_right_x");
    p.bev_near_mm = dbl("planner.bev_near_mm");
    p.cell_mm = dbl("planner.cell_mm");
    p.half_width_mm = dbl("planner.half_width_mm");
    p.far_mm = dbl("planner.far_mm");
    p.robot_half_width_mm = dbl("planner.robot_half_width_mm");
    p.box_margin_mm = dbl("planner.box_margin_mm");
    p.box_depth_mm = dbl("planner.box_depth_mm");
    p.react_mm = dbl("planner.react_mm");
    p.odom_scale = dbl("planner.odom_scale");
    p.memory_back_mm = dbl("planner.memory_back_mm");
    p.psd_keep_mm = dbl("planner.psd_keep_mm");
    p.psd_len_mm = dbl("planner.psd_len_mm");
    p.max_shift_mm = dbl("planner.max_shift_mm");
    p.line_weight = dbl("planner.line_weight");
    p.cross_blocks = get_parameter("planner.cross_blocks").as_bool();
    p.cross_weight = dbl("planner.cross_weight");
    p.line_horizon_mm = dbl("planner.line_horizon_mm");
    p.clearance_weight = dbl("planner.clearance_weight");
    p.shift_weight = dbl("planner.shift_weight");
    p.curve_weight = dbl("planner.curve_weight");
    p.smooth_weight = dbl("planner.smooth_weight");
    p.lookahead_mm = dbl("planner.lookahead_mm");
    return p;
  }

  obstacle_detection::ColorParams readColorParams()
  {
    obstacle_detection::ColorParams c;
    c.yellow_l_min = intParam("planner.color.yellow_l_min");
    c.yellow_b_min = intParam("planner.color.yellow_b_min");
    c.yellow_a_max = intParam("planner.color.yellow_a_max");
    c.white_l_min = intParam("planner.color.white_l_min");
    c.white_ab_dev = intParam("planner.color.white_ab_dev");
    c.box_l_min = intParam("planner.color.box_l_min");
    c.box_l_max = intParam("planner.color.box_l_max");
    c.box_a_min = intParam("planner.color.box_a_min");
    c.box_a_max = intParam("planner.color.box_a_max");
    c.box_b_min = intParam("planner.color.box_b_min");
    c.box_b_max = intParam("planner.color.box_b_max");
    c.box_min_b_minus_a = intParam("planner.color.box_min_b_minus_a");
    c.box_min_area = intParam("planner.color.box_min_area");
    return c;
  }

  // 바퀴 속도(엔코더)를 적분해 직전 영상 이후 이동량을 모은다
  void onDxl(const interfaces::msg::DxlState & d)
  {
    const auto t = now();
    if (have_dxl_) {
      const double dt = (t - dxl_t_).seconds();
      if (dt > 0 && dt <= dbl("planner.dxl_timeout")) {
        const double v = (last_dxl_.left_velocity + last_dxl_.right_velocity) / 2;
        const double w = (last_dxl_.right_velocity - last_dxl_.left_velocity) /
          dbl("planner.wheel_separation");
        move_ds_mm_ += v * dt * 1000.0;
        move_dyaw_ += w * dt;
      }
    }
    last_dxl_ = d;
    dxl_t_ = t;
    have_dxl_ = true;
  }

  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    // 이미지 시각으로 솎는다 (bag 재생 속도를 바꿔도 같은 프레임을 고르게)
    const rclcpp::Time stamp(msg->header.stamp);
    const double rate = get_parameter("process_rate").as_double();
    if (rate > 0 && have_last_ && (stamp - last_).seconds() < 0.9 / rate &&
      (stamp - last_).seconds() >= 0)
    {
      return;
    }
    cv::Mat bgr;
    try {
      bgr = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "cv_bridge: %s", e.what());
      return;
    }
    last_ = stamp;
    have_last_ = true;

    if (get_parameter("mode").as_string() == "free_space") {
      onImageFreeSpace(bgr, msg->header);
    } else {
      onImagePlanner(bgr, msg->header);
    }
  }

  void onImagePlanner(const cv::Mat & bgr, const std_msgs::msg::Header & header)
  {
    const auto pp = readPlannerParams();
    const auto t = now();
    planner_.onMove(move_ds_mm_, move_dyaw_, pp);
    move_ds_mm_ = 0;
    move_dyaw_ = 0;
    const bool psd_fresh = have_psd_ && (t - psd_t_).seconds() <= dbl("planner.psd_timeout");
    const double close = dbl("planner.psd_close");
    const bool left_close = psd_fresh && psd_.left < close;
    const bool right_close = psd_fresh && psd_.right < close;
    const auto t0 = std::chrono::steady_clock::now();
    const auto r = planner_.plan(bgr, pp, readColorParams(), left_close, right_close);
    const double ms_plan = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();
    // 공사 구간이 끝난 뒤 남은 기억 / 경로가 다음 구간에 끼어들지 않게
    if (r.box_visible) {
      last_box_t_ = t;
    } else if ((t - last_box_t_).seconds() > dbl("planner.reset_after")) {
      planner_.reset();
    }

    interfaces::msg::WallObstacle out;
    out.header = header;
    out.detected = r.box_visible;
    out.free_offset = static_cast<float>(std::clamp(r.angle, -1.0, 1.0));
    out.confidence = static_cast<float>(r.ok ? 1.0 : r.fallback ? dbl("planner.fallback_confidence") : 0.0);
    const auto ms = obstacle_detection::BoxPlanner::mapSize(pp);
    out.area_ratio = static_cast<float>(r.box_pixels / static_cast<double>(ms.area()));
    out.distance = r.box_ahead ? static_cast<float>(std::max(0.0, r.box_dist_mm) / 1000.0) : 0.0f;
    if (r.box_visible) {
      out.x_ratio = static_cast<float>(2 * r.box_raw_x - 1);
      out.bottom_ratio = static_cast<float>(r.box_raw_bottom);
      out.side = out.x_ratio < 0 ? interfaces::msg::WallObstacle::LEFT :
        interfaces::msg::WallObstacle::RIGHT;
    }
    wall_pub_->publish(out);

    if (debug_pub_->get_subscription_count() > 0) {
      cv::Mat vis = planner_.draw(r, pp);
      char text[128];
      std::snprintf(text, sizeof(text), "%s ang %+.2f box %s %.2f mem %d %.0fms %s%s",
        r.ok ? "ok" : r.fallback ? "FALLBACK" : "none", r.angle,
        !out.detected ? "-" : out.side == interfaces::msg::WallObstacle::LEFT ? "L" : "R",
        out.bottom_ratio, r.memory_points, ms_plan, left_close ? "PL " : "", right_close ? "PR" : "");
      cv::putText(vis, text, {4, 14}, cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(0, 255, 255), 1);
      publishJpeg(vis, header);
    }
  }

  void onImageFreeSpace(const cv::Mat & bgr, const std_msgs::msg::Header & header)
  {
    const auto r = obstacle_detection::findFreeSpace(bgr, readParams());
    const double w = r.blocked.cols, h = r.blocked.rows;
    interfaces::msg::WallObstacle out;
    out.header = header;
    out.detected = r.box_found;
    out.free_offset = static_cast<float>(r.offset());
    out.confidence = static_cast<float>(r.clearance());
    out.area_ratio = static_cast<float>(r.box_pixels / (w * h));
    if (r.box_found) {
      const auto & b = r.nearest.rect;
      out.x_ratio = static_cast<float>((b.x + b.width / 2.0) / (w / 2) - 1.0);
      out.bottom_ratio = static_cast<float>(b.br().y / h);
      out.side = out.x_ratio < 0 ? interfaces::msg::WallObstacle::LEFT :
        interfaces::msg::WallObstacle::RIGHT;
    }
    wall_pub_->publish(out);

    if (debug_pub_->get_subscription_count() > 0) {
      publishDebug(r, out);
    }
  }

  // 막힘 = 빨강, 열마다 빈 높이 = 초록 막대, 고른 통로 = 노란 선, 가장 가까운 박스 = 하늘색 상자
  void publishDebug(const obstacle_detection::FreeSpaceResult & r,
    const interfaces::msg::WallObstacle & o)
  {
    cv::Mat vis = r.small.clone();
    const int h = vis.rows;
    for (int x = 0; x < vis.cols; ++x) {
      cv::line(vis, {x, h - 1}, {x, h - 1 - static_cast<int>(r.free[x])}, cv::Scalar(0, 90, 0), 1);
    }
    cv::addWeighted(vis, 0.6, r.small, 0.4, 0, vis);
    vis.setTo(cv::Scalar(0, 0, 200), r.blocked);
    vis.setTo(cv::Scalar(0, 0, 120), r.box);
    if (r.box_found) {
      cv::rectangle(vis, r.nearest.rect, cv::Scalar(255, 255, 0), 1);
    }
    cv::line(vis, {vis.cols / 2, h - 1}, {r.column, h - 1 - static_cast<int>(r.corridor[r.column])},
      cv::Scalar(0, 255, 255), 2);
    char text[96];
    std::snprintf(text, sizeof(text), "free %+.2f clr %.2f box %s %.2f", o.free_offset, o.confidence,
      !o.detected ? "-" : o.side == interfaces::msg::WallObstacle::LEFT ? "L" : "R", o.bottom_ratio);
    cv::putText(vis, text, {4, 14}, cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(0, 255, 255), 1);
    publishJpeg(vis, o.header);
  }

  void publishJpeg(const cv::Mat & vis, const std_msgs::msg::Header & header)
  {
    sensor_msgs::msg::CompressedImage jpeg;
    jpeg.header = header;
    jpeg.format = "jpeg";
    cv::imencode(".jpg", vis, jpeg.data,
      {cv::IMWRITE_JPEG_QUALITY, intParam("debug_jpeg_quality")});
    debug_pub_->publish(jpeg);
  }

  rclcpp::Time last_;
  bool have_last_ = false;
  obstacle_detection::BoxPlanner planner_;
  double move_ds_mm_ = 0, move_dyaw_ = 0;   // 직전 영상 이후 바퀴 이동 [mm], [rad] (+ 왼쪽)
  interfaces::msg::DxlState last_dxl_;
  rclcpp::Time dxl_t_;
  bool have_dxl_ = false;
  interfaces::msg::PsdArray psd_;
  rclcpp::Time psd_t_;
  bool have_psd_ = false;
  rclcpp::Time last_box_t_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<interfaces::msg::DxlState>::SharedPtr dxl_sub_;
  rclcpp::Subscription<interfaces::msg::PsdArray>::SharedPtr psd_sub_;
  rclcpp::Publisher<interfaces::msg::WallObstacle>::SharedPtr wall_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debug_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WallDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
