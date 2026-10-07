// 주행 가능 영역 기반 경로 선택 (lane_detection 대신 쓰는 방식, launch 인자 lane_method:=path)
//   구독: image_bev (sensor_msgs/Image, bird_eye_view 노드)
//         sign (interfaces/Sign)      공사 구간 / 갈림길 방향
//         psd (interfaces/PsdArray)   카메라에 안 보이는 가까운 옆 장애물
//         dxl_state (interfaces/DxlState)  바퀴 속도 -> 로봇 이동량 (안 보이는 구간의 경로 기억을 옮기는 데 씀)
//   발행: lane_info (interfaces/LaneInfo)  lane_detection 과 같은 토픽, 같은 규약
//         vision/lane_debug/compressed     GUI 선·벡터 검출 화면 (구독자가 있을 때만)
//
// 알고리즘은 include/path_planner/planner_core.hpp (ROS 없이 테스트 가능)
// 이 노드는 파라미터 읽기, 표지판/PSD 상태 관리, 메시지 변환, 디버그 화면만 맡는다.
//
// 공사 구간: CONSTRUCTION 표지판을 보면 선 넘기 벌점을 line_weight -> construction_line_weight 로 낮춘다.
//   (박스를 피하려고 선을 넘어가는 것을 허용) 박스를 본 뒤 construction_exit_sec 동안 박스가 안 보이거나,
//   construction_timeout_sec 가 지나면 원래대로 돌아온다.
// 갈림길: LEFT/RIGHT 표지판을 보면 fork_hold_sec 동안 그 방향으로 도는 경로에 가점.
// PSD: 좌/우 PSD 가 psd_side_threshold 보다 가까우면 그쪽으로 도는 경로에 벌점.
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
#include "path_planner/planner_core.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

class PathPlannerNode : public rclcpp::Node
{
  using Sign = interfaces::msg::Sign;

public:
  PathPlannerNode()
  : Node("path_planner")
  {
    const path_planner::Params d;   // 기본값은 planner_core.hpp 와 같게
    const auto image_topic = declare_parameter("image_topic", std::string("image_bev"));
    declare_parameter("yellow_l_min", d.yellow_l_min);
    declare_parameter("yellow_b_min", d.yellow_b_min);
    declare_parameter("yellow_a_max", d.yellow_a_max);
    declare_parameter("white_l_min", d.white_l_min);
    declare_parameter("white_ab_dev", d.white_ab_dev);
    declare_parameter("box_l_min", d.box_l_min);
    declare_parameter("box_l_max", d.box_l_max);
    declare_parameter("box_a_min", d.box_a_min);
    declare_parameter("box_a_max", d.box_a_max);
    declare_parameter("box_b_min", d.box_b_min);
    declare_parameter("box_b_max", d.box_b_max);
    declare_parameter("box_min_b_minus_a", d.box_min_b_minus_a);
    declare_parameter("box_min_area", d.box_min_area);
    declare_parameter("line_gap_close_px", d.line_gap_close_px);
    declare_parameter("bev_center_x", d.bev_center_x);
    declare_parameter("robot_y_offset_px", d.robot_y_offset_px);
    declare_parameter("n_paths", d.n_paths);
    declare_parameter("max_curvature", d.max_curvature);
    declare_parameter("n_curv_rates", d.n_curv_rates);
    declare_parameter("max_curvature_rate", d.max_curvature_rate);
    declare_parameter("n_hidden", d.n_hidden);
    declare_parameter("max_hidden_offset_px", d.max_hidden_offset_px);
    declare_parameter("max_hidden_heading", d.max_hidden_heading);
    declare_parameter("continuous_curvature", d.continuous_curvature);
    declare_parameter("path_length_px", d.path_length_px);
    declare_parameter("sample_step_px", d.sample_step_px);
    declare_parameter("robot_half_width_px", d.robot_half_width_px);
    declare_parameter("line_soft_px", d.line_soft_px);
    declare_parameter("clearance_cap_px", d.clearance_cap_px);
    declare_parameter("line_weight", d.line_weight);
    declare_parameter("clearance_weight", d.clearance_weight);
    declare_parameter("smooth_weight", d.smooth_weight);
    declare_parameter("straight_weight", d.straight_weight);
    declare_parameter("fork_weight", d.fork_weight);
    declare_parameter("psd_weight", d.psd_weight);
    declare_parameter("lane_width_px", d.lane_width_px);
    declare_parameter("lookahead_px", d.lookahead_px);
    declare_parameter("smooth_alpha", d.smooth_alpha);
    declare_parameter("hold_frames", d.hold_frames);
    declare_parameter("min_wall_pixels", d.min_wall_pixels);
    // 바퀴 속도 -> BEV 이동량
    declare_parameter("bev_px_per_m", 1008.0);     // BEV 축척 (차선 폭 267px / 0.265m)
    declare_parameter("wheel_separation", 0.160);  // stm_bridge 와 같게 [m]
    // 표지판 / PSD
    declare_parameter("sign_min_confidence", 0.5);
    declare_parameter("sign_min_area_ratio", 0.01);
    declare_parameter("construction_line_weight", 1.0);
    declare_parameter("construction_exit_sec", 3.0);
    declare_parameter("construction_timeout_sec", 30.0);
    declare_parameter("fork_hold_sec", 3.0);
    declare_parameter("psd_side_threshold", 0.30);
    declare_parameter("psd_timeout_sec", 0.5);
    // 디버그: final (경로) / mask (막힌 곳 지도)
    declare_parameter("debug_view", std::string("final"));
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

    RCLCPP_INFO(get_logger(), "path_planner 시작 (구독: %s)", image_topic.c_str());
  }

private:
  int intParam(const std::string & name) {return static_cast<int>(get_parameter(name).as_int());}
  double dblParam(const std::string & name) {return get_parameter(name).as_double();}

  // 파라미터는 매 프레임 다시 읽는다 (ros2 param set 으로 주행 중 튜닝 가능)
  path_planner::Params readParams()
  {
    path_planner::Params p;
    p.yellow_l_min = intParam("yellow_l_min");
    p.yellow_b_min = intParam("yellow_b_min");
    p.yellow_a_max = intParam("yellow_a_max");
    p.white_l_min = intParam("white_l_min");
    p.white_ab_dev = intParam("white_ab_dev");
    p.box_l_min = intParam("box_l_min");
    p.box_l_max = intParam("box_l_max");
    p.box_a_min = intParam("box_a_min");
    p.box_a_max = intParam("box_a_max");
    p.box_b_min = intParam("box_b_min");
    p.box_b_max = intParam("box_b_max");
    p.box_min_b_minus_a = intParam("box_min_b_minus_a");
    p.box_min_area = intParam("box_min_area");
    p.line_gap_close_px = intParam("line_gap_close_px");
    p.bev_center_x = dblParam("bev_center_x");
    p.robot_y_offset_px = dblParam("robot_y_offset_px");
    p.n_paths = intParam("n_paths");
    p.max_curvature = dblParam("max_curvature");
    p.n_curv_rates = intParam("n_curv_rates");
    p.max_curvature_rate = dblParam("max_curvature_rate");
    p.n_hidden = intParam("n_hidden");
    p.max_hidden_offset_px = dblParam("max_hidden_offset_px");
    p.max_hidden_heading = dblParam("max_hidden_heading");
    p.continuous_curvature = get_parameter("continuous_curvature").as_bool();
    p.path_length_px = dblParam("path_length_px");
    p.sample_step_px = std::max(1.0, dblParam("sample_step_px"));
    p.robot_half_width_px = dblParam("robot_half_width_px");
    p.line_soft_px = dblParam("line_soft_px");
    p.clearance_cap_px = dblParam("clearance_cap_px");
    p.line_weight = dblParam("line_weight");   // 공사 구간이면 onImage 에서 바꾼다
    p.clearance_weight = dblParam("clearance_weight");
    p.smooth_weight = dblParam("smooth_weight");
    p.straight_weight = dblParam("straight_weight");
    p.fork_weight = dblParam("fork_weight");
    p.psd_weight = dblParam("psd_weight");
    p.lane_width_px = dblParam("lane_width_px");
    p.lookahead_px = dblParam("lookahead_px");
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
      const double dt = std::min(0.3, (t - dxl_stamp_).seconds());   // 직전 속도로 dt 동안 움직였다고 봄
      const double v = (last_dxl_.left_velocity + last_dxl_.right_velocity) / 2;
      const double w = (last_dxl_.right_velocity - last_dxl_.left_velocity) /
        dblParam("wheel_separation");
      odom_.ds_px += v * dt * dblParam("bev_px_per_m");
      odom_.dtheta += w * dt;
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
  path_planner::Bias makeBias(bool box_now)
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
    path_planner::Bias b;
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
    cv::Mat bev;
    try {
      bev = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "cv_bridge: %s", e.what());
      return;
    }
    // 공사 구간 판단에 이번 프레임 박스 여부가 필요하므로 지도를 먼저 만들고,
    // 선 넘기 벌점(점수에만 쓰임)은 그 뒤에 정한다
    auto p = readParams();
    const path_planner::Maps maps = path_planner::buildMaps(bev, p);
    const auto bias = makeBias(maps.box_pixels > 0);
    if (construction_) {
      p.line_weight = dblParam("construction_line_weight");
    }
    const auto r = planner_.plan(maps, p, bias, odom_);
    odom_ = {};

    interfaces::msg::LaneInfo lane;
    lane.header = msg->header;
    lane.detected = r.detected;
    lane.offset = r.offset;
    lane.angle = r.angle;
    lane.confidence = r.confidence;
    lane_pub_->publish(lane);

    if (debug_pub_->get_subscription_count() > 0) {
      publishDebug(bev, maps, r, bias, msg->header);
    }
  }

  // GUI 화면 (BEV 아래에 바퀴 축까지 안 보이는 구간을 붙여 그림)
  //   노란/흰 칠: 색 마스크, 어두운 빨강 칠: 박스로 막힌 곳, 회색 선: 후보, 어두운 빨강 선: 박스에 막힌 후보
  //   초록 굵은 선: 기억한 경로(로봇 -> 고른 원호), 분홍 점: 목표점, 하늘색 점: 바퀴 축
  void publishDebug(const cv::Mat & bev, const path_planner::Maps & m,
    const path_planner::Result & r, const path_planner::Bias & bias,
    const std_msgs::msg::Header & header)
  {
    const bool mask_only = get_parameter("debug_view").as_string() == "mask";
    cv::Mat view = mask_only ? cv::Mat(bev.size(), CV_8UC3, cv::Scalar(0, 0, 0)) : bev * 0.6;
    if (mask_only) {
      view.setTo(cv::Scalar(90, 90, 90), m.line_wall);   // 빈틈을 메운 선
    }
    view.setTo(cv::Scalar(0, 0, 110), m.box_blocked);
    view.setTo(cv::Scalar(19, 69, 139), m.box);
    view.setTo(cv::Scalar(0, 220, 255), m.yellow);
    view.setTo(cv::Scalar(255, 255, 255), m.white);
    // BEV 아래로 바퀴 축까지 안 보이는 구간을 붙여 그린다 (어두운 남색)
    const int below = std::max(0, cvRound(r.robot.y) - (bev.rows - 1) + 20);
    cv::Mat out(bev.rows + below, bev.cols, CV_8UC3, cv::Scalar(50, 30, 30));
    view.copyTo(out(cv::Rect(0, 0, bev.cols, bev.rows)));
    if (below > 0) {
      cv::line(out, {0, bev.rows}, {bev.cols, bev.rows}, cv::Scalar(120, 120, 120), 1);
    }

    auto toInt = [](const std::vector<cv::Point2d> & pts) {
        std::vector<cv::Point> v;
        for (const auto & q : pts) {
          v.emplace_back(cvRound(q.x), cvRound(q.y));
        }
        return v;
      };
    for (const auto & c : r.cands) {
      cv::polylines(out, toInt(c.pts), false,
        c.blocked ? cv::Scalar(0, 0, 140) : cv::Scalar(150, 150, 150), 1);
    }
    if (r.detected) {
      cv::polylines(out, toInt(r.memory), false,
        r.holding ? cv::Scalar(0, 120, 0) : cv::Scalar(0, 255, 0), 3);
      cv::circle(out, {cvRound(r.target.x), cvRound(r.target.y)}, 6, cv::Scalar(255, 0, 255), -1);
    }
    // 로봇(바퀴 축) 위치와 앞 방향
    const cv::Point rp(cvRound(r.robot.x), cvRound(r.robot.y));
    cv::circle(out, rp, 6, cv::Scalar(255, 255, 0), -1);
    cv::line(out, rp, rp - cv::Point(0, 30), cv::Scalar(255, 255, 0), 2);

    char text[128];
    std::snprintf(text, sizeof(text), "%s off %+.2f ang %+.2f", r.detected ? "o" : "x",
      r.offset, r.angle);
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

  path_planner::Planner planner_;
  path_planner::Odom odom_;   // 직전 영상 이후 이동량
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
  rclcpp::spin(std::make_shared<PathPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
