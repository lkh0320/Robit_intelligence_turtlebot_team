// 2D 지역 지도 기반 경로 계획 (launch 인자 lane_method:=map)
//   구독: image_raw (sensor_msgs/Image)   원본 카메라 영상. BEV 는 이 노드가 같은 변환으로 직접 만든다
//         dxl_state (interfaces/DxlState)  바퀴 속도 -> 로봇 이동량 (지도 위 로봇 자세)
//         sign (interfaces/Sign)      공사 구간 / 갈림길 방향
//         psd (interfaces/PsdArray)   카메라에 안 보이는 가까운 옆 장애물
//   발행: lane_info (interfaces/LaneInfo)  lane_detection / path_planner 와 같은 토픽, 같은 규약
//         vision/lane_debug/compressed     GUI 선·벡터 검출 화면: 지도 + 경로 (구독자가 있을 때만)
//
// 알고리즘은 include/map_planner/map_core.hpp (ROS 없이 테스트 가능)
// 이 노드는 파라미터 읽기, 표지판/PSD 상태 관리, 바퀴 이동량 적분, 메시지 변환, 디버그 화면만 맡는다.
//
// 공사 구간 / 갈림길 / PSD 처리는 path_planner 노드와 같다.
#include <cmath>
#include <memory>
#include <string>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "cv_bridge/cv_bridge.hpp"
#include "interfaces/msg/dxl_state.hpp"
#include "interfaces/msg/lane_info.hpp"
#include "interfaces/msg/psd_array.hpp"
#include "interfaces/msg/sign.hpp"
#include "map_planner/map_core.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

class MapPlannerNode : public rclcpp::Node
{
  using Sign = interfaces::msg::Sign;

public:
  MapPlannerNode()
  : Node("map_planner")
  {
    const map_planner::Params d;      // 기본값은 map_core.hpp 와 같게
    const path_planner::Params dm;    // 색 기준 기본값은 path_planner 와 같게
    const auto image_topic = declare_parameter("image_topic", std::string("image_raw"));
    // 색 마스크
    declare_parameter("yellow_l_min", dm.yellow_l_min);
    declare_parameter("yellow_b_min", dm.yellow_b_min);
    declare_parameter("yellow_a_max", dm.yellow_a_max);
    declare_parameter("white_l_min", dm.white_l_min);
    declare_parameter("white_ab_dev", dm.white_ab_dev);
    declare_parameter("box_l_min", dm.box_l_min);
    declare_parameter("box_l_max", dm.box_l_max);
    declare_parameter("box_a_min", dm.box_a_min);
    declare_parameter("box_a_max", dm.box_a_max);
    declare_parameter("box_b_min", dm.box_b_min);
    declare_parameter("box_b_max", dm.box_b_max);
    declare_parameter("box_min_b_minus_a", dm.box_min_b_minus_a);
    declare_parameter("box_min_area", dm.box_min_area);
    declare_parameter("line_gap_close_px", dm.line_gap_close_px);
    // BEV 변환 (bird_eye_view.yaml 과 같게)
    declare_parameter("src_top_y", d.src_top_y);
    declare_parameter("src_top_left_x", d.src_top_left_x);
    declare_parameter("src_top_right_x", d.src_top_right_x);
    declare_parameter("src_bottom_y", d.src_bottom_y);
    declare_parameter("src_bottom_left_x", d.src_bottom_left_x);
    declare_parameter("src_bottom_right_x", d.src_bottom_right_x);
    declare_parameter("bev_width", d.bev_width);
    declare_parameter("bev_height", d.bev_height);
    declare_parameter("raw_max_range_px", d.raw_max_range_px);
    declare_parameter("raw_max_lateral_px", d.raw_max_lateral_px);
    declare_parameter("raw_scale", d.raw_scale);
    // 지도
    declare_parameter("bev_center_x", d.bev_center_x);
    declare_parameter("robot_y_offset_px", d.robot_y_offset_px);
    declare_parameter("cell_px", d.cell_px);
    declare_parameter("map_size_px", d.map_size_px);
    declare_parameter("map_recenter_px", d.map_recenter_px);
    declare_parameter("memory_px", d.memory_px);
    declare_parameter("raw_memory_px", d.raw_memory_px);
    declare_parameter("view_half_width_px", d.view_half_width_px);
    declare_parameter("view_front_px", d.view_front_px);
    declare_parameter("view_back_px", d.view_back_px);
    // 후보 경로 / 점수 / 출력
    declare_parameter("n_paths", d.n_paths);
    declare_parameter("max_curvature", d.max_curvature);
    declare_parameter("n_curv_rates", d.n_curv_rates);
    declare_parameter("max_curvature_rate", d.max_curvature_rate);
    declare_parameter("path_length_px", d.path_length_px);
    declare_parameter("sample_step_px", d.sample_step_px);
    declare_parameter("min_known_ratio", d.min_known_ratio);
    declare_parameter("robot_half_width_px", d.robot_half_width_px);
    declare_parameter("clearance_cap_px", d.clearance_cap_px);
    declare_parameter("max_lane_width_px", d.max_lane_width_px);
    declare_parameter("line_weight", d.line_weight);
    declare_parameter("clearance_weight", d.clearance_weight);
    declare_parameter("align_weight", d.align_weight);
    declare_parameter("box_margin_px", d.box_margin_px);
    declare_parameter("box_weight", d.box_weight);
    declare_parameter("smooth_weight", d.smooth_weight);
    declare_parameter("straight_weight", d.straight_weight);
    declare_parameter("fork_weight", d.fork_weight);
    declare_parameter("psd_weight", d.psd_weight);
    declare_parameter("lane_width_px", d.lane_width_px);
    declare_parameter("lookahead_px", d.lookahead_px);
    declare_parameter("follower_lookahead_px", d.follower_lookahead_px);
    declare_parameter("smooth_alpha", d.smooth_alpha);
    declare_parameter("hold_frames", d.hold_frames);
    declare_parameter("min_wall_pixels", d.min_wall_pixels);
    // 바퀴 속도 -> 이동량
    declare_parameter("bev_px_per_m", 1008.0);     // BEV 축척 (차선 폭 267px / 0.265m)
    declare_parameter("wheel_separation", 0.160);  // stm_bridge 와 같게 [m]
    declare_parameter("dxl_timeout_sec", 0.5);     // 이보다 오래 dxl_state 가 없으면 멈춰 있다고 본다
    // 표지판 / PSD
    declare_parameter("sign_min_confidence", 0.5);
    declare_parameter("sign_min_area_ratio", 0.01);
    declare_parameter("construction_line_weight", 1.0);
    declare_parameter("construction_exit_sec", 3.0);
    declare_parameter("construction_timeout_sec", 30.0);
    declare_parameter("fork_hold_sec", 3.0);
    declare_parameter("psd_side_threshold", 0.30);
    declare_parameter("psd_timeout_sec", 0.5);
    // 디버그 화면
    declare_parameter("debug_scale", 2);
    declare_parameter("debug_candidates", true);
    declare_parameter("debug_jpeg_quality", 70);

    const auto qos = rclcpp::SensorDataQoS();
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, qos, [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {onImage(msg);});
    sign_sub_ = create_subscription<Sign>(
      "sign", qos, [this](Sign::ConstSharedPtr msg) {onSign(*msg);});
    psd_sub_ = create_subscription<interfaces::msg::PsdArray>(
      "psd", qos, [this](interfaces::msg::PsdArray::ConstSharedPtr msg) {
        psd_ = *msg;
        psd_stamp_ = now();
        psd_received_ = true;
      });
    dxl_sub_ = create_subscription<interfaces::msg::DxlState>(
      "dxl_state", qos, [this](interfaces::msg::DxlState::ConstSharedPtr msg) {onDxl(*msg);});
    lane_pub_ = create_publisher<interfaces::msg::LaneInfo>("lane_info", qos);
    debug_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      "vision/lane_debug/compressed", qos);

    RCLCPP_INFO(get_logger(), "map_planner 시작 (구독: %s)", image_topic.c_str());
  }

private:
  int intParam(const std::string & name) {return static_cast<int>(get_parameter(name).as_int());}
  double dblParam(const std::string & name) {return get_parameter(name).as_double();}

  // 파라미터는 매 프레임 다시 읽는다 (ros2 param set 으로 주행 중 튜닝 가능)
  path_planner::Params readMaskParams()
  {
    path_planner::Params m;
    m.yellow_l_min = intParam("yellow_l_min");
    m.yellow_b_min = intParam("yellow_b_min");
    m.yellow_a_max = intParam("yellow_a_max");
    m.white_l_min = intParam("white_l_min");
    m.white_ab_dev = intParam("white_ab_dev");
    m.box_l_min = intParam("box_l_min");
    m.box_l_max = intParam("box_l_max");
    m.box_a_min = intParam("box_a_min");
    m.box_a_max = intParam("box_a_max");
    m.box_b_min = intParam("box_b_min");
    m.box_b_max = intParam("box_b_max");
    m.box_min_b_minus_a = intParam("box_min_b_minus_a");
    m.box_min_area = intParam("box_min_area");
    m.line_gap_close_px = intParam("line_gap_close_px");
    return m;
  }

  map_planner::Params readParams()
  {
    map_planner::Params p;
    p.src_top_y = dblParam("src_top_y");
    p.src_top_left_x = dblParam("src_top_left_x");
    p.src_top_right_x = dblParam("src_top_right_x");
    p.src_bottom_y = dblParam("src_bottom_y");
    p.src_bottom_left_x = dblParam("src_bottom_left_x");
    p.src_bottom_right_x = dblParam("src_bottom_right_x");
    p.bev_width = std::max(1, intParam("bev_width"));
    p.bev_height = std::max(1, intParam("bev_height"));
    p.raw_max_range_px = dblParam("raw_max_range_px");
    p.raw_max_lateral_px = dblParam("raw_max_lateral_px");
    p.raw_scale = dblParam("raw_scale");
    p.bev_center_x = dblParam("bev_center_x");
    p.robot_y_offset_px = dblParam("robot_y_offset_px");
    p.cell_px = std::max(1.0, dblParam("cell_px"));
    p.map_size_px = dblParam("map_size_px");
    p.map_recenter_px = dblParam("map_recenter_px");
    p.memory_px = dblParam("memory_px");
    p.raw_memory_px = dblParam("raw_memory_px");
    p.view_half_width_px = dblParam("view_half_width_px");
    p.view_front_px = dblParam("view_front_px");
    p.view_back_px = dblParam("view_back_px");
    p.n_paths = intParam("n_paths");
    p.max_curvature = dblParam("max_curvature");
    p.n_curv_rates = intParam("n_curv_rates");
    p.max_curvature_rate = dblParam("max_curvature_rate");
    p.path_length_px = dblParam("path_length_px");
    p.sample_step_px = std::max(1.0, dblParam("sample_step_px"));
    p.min_known_ratio = dblParam("min_known_ratio");
    p.robot_half_width_px = dblParam("robot_half_width_px");
    p.clearance_cap_px = dblParam("clearance_cap_px");
    p.max_lane_width_px = dblParam("max_lane_width_px");
    p.line_weight = dblParam("line_weight");   // 공사 구간이면 onImage 에서 바꾼다
    p.clearance_weight = dblParam("clearance_weight");
    p.align_weight = dblParam("align_weight");
    p.box_margin_px = dblParam("box_margin_px");
    p.box_weight = dblParam("box_weight");
    p.smooth_weight = dblParam("smooth_weight");
    p.straight_weight = dblParam("straight_weight");
    p.fork_weight = dblParam("fork_weight");
    p.psd_weight = dblParam("psd_weight");
    p.lane_width_px = dblParam("lane_width_px");
    p.lookahead_px = dblParam("lookahead_px");
    p.follower_lookahead_px = dblParam("follower_lookahead_px");
    p.smooth_alpha = dblParam("smooth_alpha");
    p.hold_frames = intParam("hold_frames");
    p.min_wall_pixels = intParam("min_wall_pixels");
    return p;
  }

  // 바퀴 속도를 적분해 직전 영상 이후 이동량을 모은다
  // (dxl_state 의 속도는 엔코더가 아니라 STM 이 마지막으로 보낸 바퀴 명령 값)
  void onDxl(const interfaces::msg::DxlState & d)
  {
    const auto t = now();
    if (dxl_received_) {
      // 직전 속도로 dt 동안 움직였다고 본다. 너무 오래 끊겼으면 그동안은 모른다고 보고 버린다
      const double dt = (t - dxl_stamp_).seconds();
      if (dt > 0 && dt <= dblParam("dxl_timeout_sec")) {
        const double v = (last_dxl_.left_velocity + last_dxl_.right_velocity) / 2;
        const double w = (last_dxl_.right_velocity - last_dxl_.left_velocity) /
          dblParam("wheel_separation");
        odom_.ds_px += v * dt * dblParam("bev_px_per_m");
        odom_.dtheta += w * dt;
      }
    }
    last_dxl_ = d;
    dxl_stamp_ = t;
    dxl_received_ = true;
  }

  void onSign(const Sign & s)
  {
    if (s.confidence < dblParam("sign_min_confidence") ||
      s.area_ratio < dblParam("sign_min_area_ratio"))
    {
      return;
    }
    const auto t = now();
    if (s.type == Sign::CONSTRUCTION) {
      if (!construction_) {
        RCLCPP_INFO(get_logger(), "공사 구간 진입: 선 넘기 벌점 완화");
      }
      construction_ = true;
      construction_start_ = t;
      box_seen_ = false;
    } else if (s.type == Sign::LEFT || s.type == Sign::RIGHT) {
      fork_dir_ = s.type == Sign::LEFT ? -1 : 1;
      fork_stamp_ = t;
    }
  }

  // 표지판 / PSD 상태 -> 이번 프레임 Bias. 공사 구간 종료도 여기서 판단
  map_planner::Bias makeBias(bool box_now)
  {
    const auto t = now();
    if (construction_) {
      if (box_now) {
        box_seen_ = true;
        box_stamp_ = t;
      }
      const bool cleared = box_seen_ && (t - box_stamp_).seconds() > dblParam("construction_exit_sec");
      const bool timeout = (t - construction_start_).seconds() > dblParam("construction_timeout_sec");
      if (cleared || timeout) {
        construction_ = false;
        RCLCPP_INFO(get_logger(), "공사 구간 종료 (%s)", cleared ? "박스 안 보임" : "시간 초과");
      }
    }
    map_planner::Bias b;
    if (fork_dir_ != 0 && (t - fork_stamp_).seconds() <= dblParam("fork_hold_sec")) {
      b.fork_dir = fork_dir_;
    } else {
      fork_dir_ = 0;
    }
    if (psd_received_ && (t - psd_stamp_).seconds() <= dblParam("psd_timeout_sec")) {
      const double th = dblParam("psd_side_threshold");
      // PSD 가 범위 밖이면 0 이나 음수를 줄 수 있으므로 양수일 때만 본다
      b.psd_left_close = psd_.left > 0 && psd_.left < th;
      b.psd_right_close = psd_.right > 0 && psd_.right < th;
    }
    return b;
  }

  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    cv::Mat raw;
    try {
      raw = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "cv_bridge: %s", e.what());
      return;
    }
    // 공사 구간 판단에 이번 프레임 박스 여부가 필요하므로 마스크를 먼저 만들고,
    // 선 넘기 벌점(점수에만 쓰임)은 그 뒤에 정한다
    auto p = readParams();
    const auto obs = map_planner::observe(raw, readMaskParams(), p);
    const auto bias = makeBias(obs.bev_maps.box_pixels > 0 || obs.raw_maps.box_pixels > 0);
    if (construction_) {
      p.line_weight = dblParam("construction_line_weight");
    }
    const auto r = planner_.step(obs, p, bias, odom_);
    odom_ = {};

    interfaces::msg::LaneInfo lane;
    lane.header = msg->header;
    lane.detected = r.detected;
    lane.offset = r.offset;
    lane.angle = r.angle;
    lane.confidence = r.confidence;
    lane_pub_->publish(lane);

    if (debug_pub_->get_subscription_count() > 0) {
      publishDebug(obs, r, p, bias, msg->header);
    }
  }

  // GUI 화면: 왼쪽 = 지도 (map_core.hpp drawMap 설명 참고), 오른쪽 위 = 이번 BEV
  void publishDebug(const map_planner::Observation & obs, const map_planner::Result & r,
    const map_planner::Params & p, const map_planner::Bias & bias,
    const std_msgs::msg::Header & header)
  {
    cv::Mat map_img = map_planner::drawMap(planner_.map(), r, p, std::max(1, intParam("debug_scale")),
        get_parameter("debug_candidates").as_bool());
    cv::Mat bev = obs.bev.clone();
    bev.setTo(cv::Scalar(0, 220, 255), obs.bev_maps.yellow);
    bev.setTo(cv::Scalar(255, 255, 255), obs.bev_maps.white);
    bev.setTo(cv::Scalar(0, 0, 200), obs.bev_maps.box_blocked);
    const int bh = map_img.rows / 2;
    cv::resize(bev, bev, {cvRound(bev.cols * static_cast<double>(bh) / bev.rows), bh});
    cv::Mat out(map_img.rows, map_img.cols + bev.cols, CV_8UC3, cv::Scalar(30, 30, 30));
    map_img.copyTo(out(cv::Rect(0, 0, map_img.cols, map_img.rows)));
    bev.copyTo(out(cv::Rect(map_img.cols, 0, bev.cols, bev.rows)));

    char text[128];
    std::snprintf(text, sizeof(text), "%s off %+.2f ang %+.2f c %.1f", r.detected ? "o" : "x",
      r.offset, r.angle, r.confidence);
    cv::putText(out, text, {6, 20}, cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 255), 2);
    std::snprintf(text, sizeof(text), "%s%s%s%s", construction_ ? "CONSTR " : "",
      bias.fork_dir < 0 ? "FORK_L " : bias.fork_dir > 0 ? "FORK_R " : "",
      bias.psd_left_close ? "PSD_L " : "", bias.psd_right_close ? "PSD_R" : "");
    cv::putText(out, text, {6, 42}, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 165, 255), 2);

    sensor_msgs::msg::CompressedImage jpeg;
    jpeg.header = header;
    jpeg.format = "jpeg";
    cv::imencode(".jpg", out, jpeg.data,
      {cv::IMWRITE_JPEG_QUALITY, intParam("debug_jpeg_quality")});
    debug_pub_->publish(jpeg);
  }

  map_planner::MapPlanner planner_;
  map_planner::Odom odom_;   // 직전 영상 이후 이동량
  interfaces::msg::DxlState last_dxl_;
  rclcpp::Time dxl_stamp_;
  bool dxl_received_ = false;

  bool construction_ = false, box_seen_ = false;
  rclcpp::Time construction_start_, box_stamp_;
  int fork_dir_ = 0;
  rclcpp::Time fork_stamp_;
  interfaces::msg::PsdArray psd_;
  rclcpp::Time psd_stamp_;
  bool psd_received_ = false;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<Sign>::SharedPtr sign_sub_;
  rclcpp::Subscription<interfaces::msg::PsdArray>::SharedPtr psd_sub_;
  rclcpp::Subscription<interfaces::msg::DxlState>::SharedPtr dxl_sub_;
  rclcpp::Publisher<interfaces::msg::LaneInfo>::SharedPtr lane_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debug_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
