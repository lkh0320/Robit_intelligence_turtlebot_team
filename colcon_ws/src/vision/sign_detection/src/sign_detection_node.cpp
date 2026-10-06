// 표지판용 객체 인식 (YOLOv8 ONNX + OpenCV DNN, CPU)
//   구독: image_raw (sensor_msgs/Image)
//   발행: sign (interfaces/Sign)                   매 프레임, 표지판이 없으면 type = NONE
//         vision/object_debug/compressed           GUI 객체 인식 화면 (구독자가 있을 때만)
//
// 모델: 노트북에서 best.pt 를 ONNX 로 바꿔 젯슨에 복사한다 (config/sign_detection.yaml 의 model_path)
//   yolo export model=best.pt format=onnx imgsz=640 opset=12 simplify=True
//   ONNX 는 기기와 상관없이 돌아가므로 노트북에서 만들어도 된다 (.engine 과 다름)
//   젯슨의 JetPack OpenCV 4.8 은 CUDA 없이 빌드되어 CPU 로 돈다. 느리면 imgsz=416/320 으로 export 하고
//   input_size 를 같이 바꾼다
//
// 처리 순서
//   1. letterbox: 비율 유지하며 input_size 정사각형으로 (남는 곳 회색)
//   2. 추론 -> 1 x (4 + 9) x N 출력
//   3. 신뢰도 conf_threshold 이상 + 클래스별 NMS
//   4. 표지판 클래스 중 신뢰도가 가장 높은 박스 하나만 sign 으로 발행
//      (한 프레임에 박스가 2개 잡혀도 하나만 쓴다)
//
// area_ratio: 박스 면적 / 이미지 면적. 표지판에 가까워질수록 커진다 (task_planner 가 접근 판단에 사용)
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/dnn.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "cv_bridge/cv_bridge.hpp"
#include "interfaces/msg/sign.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sign_detection/yolo_classes.hpp"
#include "sign_detection/yolo_detector.hpp"

namespace sd = sign_detection;

namespace
{
// "~/..." -> "$HOME/..." (ROS 파라미터는 셸을 거치지 않아 ~ 가 풀리지 않는다)
std::string expandHome(const std::string & path)
{
  const char * home = std::getenv("HOME");
  if (home && path.size() >= 1 && path[0] == '~') {
    return home + path.substr(1);
  }
  return path;
}
}  // namespace

class SignDetectionNode : public rclcpp::Node
{
public:
  SignDetectionNode()
  : Node("sign_detection")
  {
    const auto image_topic = declare_parameter("image_topic", std::string("image_raw"));
    const auto model_path = expandHome(
      declare_parameter("model_path", std::string("~/models/signs_v1.onnx")));
    input_size_ = static_cast<int>(declare_parameter("input_size", 640));   // export 때 imgsz 와 같게
    declare_parameter("conf_threshold", 0.5);   // 이보다 낮은 박스는 버림
    declare_parameter("nms_threshold", 0.45);   // 같은 클래스 박스가 이 IoU 넘게 겹치면 하나만
    declare_parameter("debug_jpeg_quality", 70);

    const auto qos = rclcpp::SensorDataQoS();
    sign_pub_ = create_publisher<interfaces::msg::Sign>("sign", qos);
    debug_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      "vision/object_debug/compressed", qos);

    // 모델이 없거나 못 읽으면 노드는 켜 둔 채 아무것도 안 한다 (launch 의 다른 노드는 계속 돈다)
    try {
      net_ = cv::dnn::readNetFromONNX(model_path);
      net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
      net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    } catch (const cv::Exception & e) {
      RCLCPP_ERROR(get_logger(), "모델을 읽지 못함: %s\n  %s\n  model_path 파라미터 확인 "
        "(config/sign_detection.yaml). 표지판 인식 꺼짐", model_path.c_str(), e.what());
      return;
    }

    // 추론이 카메라보다 느리므로 대기열은 1: 밀린 옛 프레임 대신 항상 최신 프레임을 처리한다
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {onImage(msg);});

    RCLCPP_INFO(get_logger(), "sign_detection 시작 (구독: %s, 모델: %s, 입력 %d)",
      image_topic.c_str(), model_path.c_str(), input_size_);
  }

private:
  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    cv::Mat img;
    try {
      img = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "cv_bridge: %s", e.what());
      return;
    }

    // 1~2. letterbox + 추론
    const auto t0 = std::chrono::steady_clock::now();
    cv::Mat input;
    const sd::Letterbox lb = sd::letterbox(img, input_size_, input);
    // 0~255 -> 0~1, BGR -> RGB (학습 이미지는 RGB), HWC -> NCHW
    net_.setInput(cv::dnn::blobFromImage(input, 1.0 / 255.0, cv::Size(), cv::Scalar(), true, false));
    const cv::Mat out = net_.forward();
    const double ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();

    if (!sd::validOutput(out, sd::NUM_CLASSES)) {
      // 클래스 수가 다르거나 NMS 가 들어간 형식(1 x 300 x 6 등)으로 export 한 모델
      std::string shape;
      for (int i = 0; i < out.dims; ++i) {
        shape += (i ? " x " : "") + std::to_string(out.size[i]);
      }
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
        "모델 출력 모양 %s 이 1 x %d x N 이 아님. yolov8 모델을 nms=False 로 export 했는지, "
        "클래스가 %d개(yolo/data.yaml)인지 확인", shape.c_str(), 4 + sd::NUM_CLASSES,
        sd::NUM_CLASSES);
      return;
    }

    // 3~4. 박스 -> 표지판 하나
    const auto dets = sd::decode(out, sd::NUM_CLASSES, lb, img.size(),
      static_cast<float>(get_parameter("conf_threshold").as_double()),
      static_cast<float>(get_parameter("nms_threshold").as_double()));
    const auto best = sd::bestSign(dets);

    interfaces::msg::Sign sign;
    sign.header = msg->header;
    if (best) {
      sign.type = sd::to_sign_type(best->cls);
      sign.confidence = best->conf;
      sign.area_ratio = static_cast<float>(best->box.area() / (img.cols * img.rows));
    }   // 없으면 기본값 NONE, 0, 0
    sign_pub_->publish(sign);

    RCLCPP_DEBUG(get_logger(), "추론 %.1f ms, 박스 %zu개", ms, dets.size());

    // 디버그 화면은 GUI 가 보고 있을 때만 만든다
    if (debug_pub_->get_subscription_count() > 0) {
      publishDebug(img, dets, best, ms, msg->header);
    }
  }

  // GUI "객체 인식" 화면
  //   초록 굵은 상자: sign 으로 발행한 박스, 노란 얇은 상자: 나머지 박스
  //   왼쪽 위: 발행한 표지판, 추론 시간
  void publishDebug(const cv::Mat & img, const std::vector<sd::Detection> & dets,
    const std::optional<sd::Detection> & best, double ms, const std_msgs::msg::Header & header)
  {
    cv::Mat out = img.clone();
    for (const auto & d : dets) {
      const bool chosen = best && d.cls == best->cls && d.box == best->box;
      const cv::Scalar color = chosen ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 255, 255);
      cv::rectangle(out, d.box, color, chosen ? 3 : 1);
      char label[48];
      std::snprintf(label, sizeof(label), "%s %.2f", sd::NAMES[d.cls], d.conf);
      const cv::Point org(static_cast<int>(d.box.x), std::max(14, static_cast<int>(d.box.y) - 4));
      cv::putText(out, label, org, cv::FONT_HERSHEY_SIMPLEX, 0.5, color, chosen ? 2 : 1);
    }
    char text[96];
    std::snprintf(text, sizeof(text), "sign %s  %.0f ms",
      best ? sd::NAMES[best->cls] : "-", ms);
    cv::putText(out, text, {6, 20}, cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 255), 2);

    sensor_msgs::msg::CompressedImage jpeg;
    jpeg.header = header;
    jpeg.format = "jpeg";
    cv::imencode(".jpg", out, jpeg.data,
      {cv::IMWRITE_JPEG_QUALITY, static_cast<int>(get_parameter("debug_jpeg_quality").as_int())});
    debug_pub_->publish(jpeg);
  }

  cv::dnn::Net net_;
  int input_size_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<interfaces::msg::Sign>::SharedPtr sign_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debug_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SignDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
