// 주행용 선 검출
//   구독: image_raw (sensor_msgs/Image)
//   발행: lane_info (LaneInfo), stop_line (StopLine),
//         vision/lane_debug/compressed (검출 결과를 그린 화면, 구독자가 있을 때만)
// 처리: 아래쪽 ROI -> 노란색/흰색 HSV 마스크 -> Canny -> HoughLinesP
//       노란 선 = 왼쪽 차선, 흰 선 = 오른쪽 차선 (색으로 좌우를 나눠 커브에서도 섞이지 않게)
//       노란 선 가운데가 빛에 하얗게 날아가 흰색으로도 잡히므로, 왼쪽 차선보다 왼쪽의 흰 선분은 버린다.
//       흰색 중 거의 수평이고 긴 선은 정지선으로 분류한다.
#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "interfaces/msg/lane_info.hpp"
#include "interfaces/msg/stop_line.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace
{
// sensor_msgs/Image -> BGR cv::Mat
// cv_bridge 는 쓰지 않는다: Jetson 은 OpenCV 4.8(JetPack)인데 ROS cv_bridge 는 4.6 에 링크돼 있어
// 한 프로세스에 두 버전이 섞이기 때문
bool toBgr(const sensor_msgs::msg::Image & msg, cv::Mat & out)
{
  const int w = static_cast<int>(msg.width), h = static_cast<int>(msg.height);
  const auto step = static_cast<size_t>(msg.step);
  auto * data = const_cast<uint8_t *>(msg.data.data());
  if (msg.encoding == "bgr8") {
    out = cv::Mat(h, w, CV_8UC3, data, step).clone();
  } else if (msg.encoding == "rgb8") {
    cv::cvtColor(cv::Mat(h, w, CV_8UC3, data, step), out, cv::COLOR_RGB2BGR);
  } else if (msg.encoding == "mono8") {
    cv::cvtColor(cv::Mat(h, w, CV_8UC1, data, step), out, cv::COLOR_GRAY2BGR);
  } else if (msg.encoding == "yuv422_yuy2" || msg.encoding == "yuyv") {
    cv::cvtColor(cv::Mat(h, w, CV_8UC2, data, step), out, cv::COLOR_YUV2BGR_YUY2);
  } else if (msg.encoding == "bgra8") {
    cv::cvtColor(cv::Mat(h, w, CV_8UC4, data, step), out, cv::COLOR_BGRA2BGR);
  } else if (msg.encoding == "rgba8") {
    cv::cvtColor(cv::Mat(h, w, CV_8UC4, data, step), out, cv::COLOR_RGBA2BGR);
  } else {
    return false;
  }
  return true;
}

// 직선 x = a*y + b (이미지 좌표, y 아래로 증가)
struct LineXY
{
  double a;
  double b;
  double x(double y) const {return a * y + b;}
};

// 선분 여러 개를 길이 가중 평균으로 합쳐 직선 하나로 (선분이 없으면 nullopt)
class LineAccumulator
{
public:
  void add(const cv::Vec4i & l)
  {
    const double dx = l[2] - l[0], dy = l[3] - l[1];
    const double len = std::hypot(dx, dy);
    const double a = dx / dy;   // 호출 전에 수평에 가까운 선은 걸러져 있어야 한다
    sum_a_ += a * len;
    sum_b_ += (l[0] - a * l[1]) * len;
    sum_w_ += len;
  }
  std::optional<LineXY> line() const
  {
    if (sum_w_ <= 0.0) {
      return std::nullopt;
    }
    return LineXY{sum_a_ / sum_w_, sum_b_ / sum_w_};
  }

private:
  double sum_a_ = 0.0, sum_b_ = 0.0, sum_w_ = 0.0;
};

// 선분의 수평으로부터의 각도 [deg, 0~90]
double fromHorizontal(const cv::Vec4i & l)
{
  const double deg = std::abs(std::atan2(l[3] - l[1], l[2] - l[0])) * 180.0 / CV_PI;
  return std::min(deg, 180.0 - deg);
}
}  // namespace

class LaneDetectionNode : public rclcpp::Node
{
public:
  LaneDetectionNode()
  : Node("lane_detection")
  {
    const auto image_topic = declare_parameter("image_topic", std::string("image_raw"));
    declare_parameter("roi_top", 0.5);              // 이 비율 아래쪽만 처리 [0~1]
    // 노란 선 (왼쪽 차선), OpenCV HSV: H 0~180
    declare_parameter("yellow_h_min", 15);
    declare_parameter("yellow_h_max", 40);
    declare_parameter("yellow_s_min", 60);
    declare_parameter("yellow_v_min", 100);
    // 흰 선 (오른쪽 차선, 정지선)
    declare_parameter("white_s_max", 50);
    declare_parameter("white_v_min", 200);   // 바닥 반사광(V≈170)은 제외
    declare_parameter("canny_low", 50);
    declare_parameter("canny_high", 150);
    declare_parameter("hough_threshold", 30);
    declare_parameter("hough_min_length", 20);      // [px]
    declare_parameter("hough_max_gap", 20);         // [px]
    declare_parameter("right_margin", 0.1);         // 왼쪽 차선에서 이만큼(이미지 폭 비율) 오른쪽부터 흰 선 인정
    declare_parameter("lane_min_angle", 20.0);      // 수평에서 이 각도[deg] 이상이면 차선 후보
    declare_parameter("stop_max_angle", 10.0);      // 수평에서 이 각도[deg] 이하면 정지선 후보
    declare_parameter("stop_min_length", 0.4);      // 정지선 최소 길이 (이미지 폭 비율)
    declare_parameter("lane_width", 0.6);           // 한쪽 차선만 보일 때 가정할 차선 폭 (ROI 아래쪽 이미지 폭 비율)
    declare_parameter("debug_jpeg_quality", 70);

    const auto qos = rclcpp::SensorDataQoS();
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, qos, [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {onImage(msg);});
    lane_pub_ = create_publisher<interfaces::msg::LaneInfo>("lane_info", qos);
    stop_pub_ = create_publisher<interfaces::msg::StopLine>("stop_line", qos);
    debug_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      "vision/lane_debug/compressed", qos);

    RCLCPP_INFO(get_logger(), "lane_detection 시작 (구독: %s)", image_topic.c_str());
  }

private:
  int intParam(const std::string & name) {return static_cast<int>(get_parameter(name).as_int());}

  // 마스크 -> 엣지 -> 허프 선분
  std::vector<cv::Vec4i> houghLines(const cv::Mat & mask)
  {
    cv::Mat edges;
    cv::Canny(mask, edges, intParam("canny_low"), intParam("canny_high"));
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(edges, lines, 1, CV_PI / 180, intParam("hough_threshold"),
      intParam("hough_min_length"), intParam("hough_max_gap"));
    return lines;
  }

  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    cv::Mat frame;
    if (!toBgr(*msg, frame)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "지원하지 않는 영상 인코딩: %s", msg->encoding.c_str());
      return;
    }
    const int w = frame.cols, h = frame.rows;
    const int roi_y = std::clamp(static_cast<int>(h * get_parameter("roi_top").as_double()), 0, h - 2);
    const cv::Mat roi = frame(cv::Rect(0, roi_y, w, h - roi_y));

    // --- 색 마스크 ---
    cv::Mat hsv, yellow, white;
    cv::GaussianBlur(roi, hsv, cv::Size(5, 5), 0);
    cv::cvtColor(hsv, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv,
      cv::Scalar(intParam("yellow_h_min"), intParam("yellow_s_min"), intParam("yellow_v_min")),
      cv::Scalar(intParam("yellow_h_max"), 255, 255), yellow);
    cv::inRange(hsv, cv::Scalar(0, 0, intParam("white_v_min")),
      cv::Scalar(180, intParam("white_s_max"), 255), white);
    cv::Mat yellow_near;
    cv::dilate(yellow, yellow_near, cv::Mat(), cv::Point(-1, -1), 3);
    white.setTo(0, yellow_near);

    const auto yellow_lines = houghLines(yellow);
    const auto white_lines = houghLines(white);

    // --- 선 분류 (좌표는 ROI 기준) ---
    const double lane_min = get_parameter("lane_min_angle").as_double();
    const double stop_max = get_parameter("stop_max_angle").as_double();
    const double stop_min_len = get_parameter("stop_min_length").as_double() * w;
    LineAccumulator left_acc, right_acc;
    std::optional<cv::Vec4i> stop_line;
    double stop_len = 0.0;

    for (const auto & l : yellow_lines) {
      if (fromHorizontal(l) >= lane_min) {
        left_acc.add(l);
      }
    }
    const auto left = left_acc.line();
    const double margin = get_parameter("right_margin").as_double() * w;
    for (const auto & l : white_lines) {
      const double ang = fromHorizontal(l);
      const double len = std::hypot(l[2] - l[0], l[3] - l[1]);
      if (ang >= lane_min) {
        // 왼쪽 차선보다 오른쪽에 있는 흰 선분만 오른쪽 차선으로
        const double mid_x = (l[0] + l[2]) / 2.0, mid_y = (l[1] + l[3]) / 2.0;
        if (!left || mid_x > left->x(mid_y) + margin) {
          right_acc.add(l);
        }
      } else if (ang <= stop_max && len >= stop_min_len) {
        // 가장 아래(가까운) 정지선 후보
        if (!stop_line || std::max(l[1], l[3]) > std::max((*stop_line)[1], (*stop_line)[3])) {
          stop_line = l;
          stop_len = len;
        }
      }
    }

    // --- 차선 중심 계산 ---
    const auto right = right_acc.line();
    const double y_bot = roi.rows - 1, y_top = 0.0;
    const double half_lane = get_parameter("lane_width").as_double() * w / 2.0;

    interfaces::msg::LaneInfo lane;
    lane.header = msg->header;
    double cx_bot = w / 2.0, cx_top = w / 2.0;
    if (left && right) {
      cx_bot = (left->x(y_bot) + right->x(y_bot)) / 2.0;
      cx_top = (left->x(y_top) + right->x(y_top)) / 2.0;
      lane.confidence = 0.9f;
    } else if (left) {
      cx_bot = left->x(y_bot) + half_lane;
      cx_top = left->x(y_top) + half_lane;
      lane.confidence = 0.5f;
    } else if (right) {
      cx_bot = right->x(y_bot) - half_lane;
      cx_top = right->x(y_top) - half_lane;
      lane.confidence = 0.5f;
    }
    lane.detected = left || right;
    if (lane.detected) {
      // offset +: 차선 중심이 화면 오른쪽 / angle +: 진행 방향이 오른쪽으로 기울어짐
      lane.offset = static_cast<float>(std::clamp((cx_bot - w / 2.0) / (w / 2.0), -1.0, 1.0));
      lane.angle = static_cast<float>(std::atan2(cx_top - cx_bot, y_bot - y_top));
    }
    lane_pub_->publish(lane);

    interfaces::msg::StopLine stop;
    stop.header = msg->header;
    stop.detected = stop_line.has_value();
    if (stop_line) {
      const int y = std::max((*stop_line)[1], (*stop_line)[3]) + roi_y;
      stop.y_ratio = static_cast<float>(static_cast<double>(y) / h);
      stop.confidence = static_cast<float>(std::min(1.0, stop_len / w));
    }
    stop_pub_->publish(stop);

    // --- 디버그 화면 (GUI 선·벡터 검출 창) ---
    if (debug_pub_->get_subscription_count() == 0) {
      return;
    }
    cv::Mat dbg = frame.clone();
    const cv::Point off(0, roi_y);
    cv::rectangle(dbg, cv::Rect(0, roi_y, w, h - roi_y), cv::Scalar(255, 200, 0), 1);
    for (const auto & l : yellow_lines) {
      cv::line(dbg, cv::Point(l[0], l[1]) + off, cv::Point(l[2], l[3]) + off,
        cv::Scalar(0, 140, 255), 1);
    }
    for (const auto & l : white_lines) {
      cv::line(dbg, cv::Point(l[0], l[1]) + off, cv::Point(l[2], l[3]) + off,
        cv::Scalar(255, 150, 150), 1);
    }
    auto draw_fit = [&](const std::optional<LineXY> & f) {
        if (f) {
          cv::line(dbg, cv::Point(cvRound(f->x(y_bot)), cvRound(y_bot)) + off,
            cv::Point(cvRound(f->x(y_top)), cvRound(y_top)) + off, cv::Scalar(0, 255, 0), 3);
        }
      };
    draw_fit(left);
    draw_fit(right);
    if (lane.detected) {
      cv::arrowedLine(dbg, cv::Point(cvRound(cx_bot), cvRound(y_bot)) + off,
        cv::Point(cvRound(cx_top), cvRound(y_top)) + off, cv::Scalar(255, 0, 255), 3,
        cv::LINE_8, 0, 0.15);
    }
    if (stop_line) {
      const auto & l = *stop_line;
      cv::line(dbg, cv::Point(l[0], l[1]) + off, cv::Point(l[2], l[3]) + off,
        cv::Scalar(0, 0, 255), 4);
    }
    cv::line(dbg, cv::Point(w / 2, roi_y), cv::Point(w / 2, h), cv::Scalar(255, 255, 255), 1);
    char text[96];
    std::snprintf(text, sizeof(text), "L%s R%s  offset %+.2f  angle %+.2f %s",
      left ? "o" : "x", right ? "o" : "x", lane.offset, lane.angle, stop.detected ? " STOP" : "");
    cv::putText(dbg, text, cv::Point(8, 22), cv::FONT_HERSHEY_SIMPLEX, 0.6,
      cv::Scalar(0, 255, 255), 2);
    publishDebug(dbg, msg->header);
  }

  void publishDebug(const cv::Mat & img, const std_msgs::msg::Header & header)
  {
    sensor_msgs::msg::CompressedImage out;
    out.header = header;
    out.format = "jpeg";
    const std::vector<int> opts{cv::IMWRITE_JPEG_QUALITY,
      static_cast<int>(get_parameter("debug_jpeg_quality").as_int())};
    cv::imencode(".jpg", img, out.data, opts);
    debug_pub_->publish(out);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<interfaces::msg::LaneInfo>::SharedPtr lane_pub_;
  rclcpp::Publisher<interfaces::msg::StopLine>::SharedPtr stop_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debug_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaneDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
