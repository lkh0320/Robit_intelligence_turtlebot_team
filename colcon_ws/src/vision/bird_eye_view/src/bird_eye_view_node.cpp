// 카메라 영상 -> Bird's Eye View (Perspective Transform)
//   구독: image_raw (sensor_msgs/Image)
//   발행: image_bev (sensor_msgs/Image, bgr8)          -> 선 검출 등 비전 노드용
//         image_bev/compressed (CompressedImage, jpeg) -> GUI 표시용 (draw_grid 면 격자 표시)
// 원본에서 바닥의 사다리꼴(src_*, 이미지 크기 비율)을 출력 영상 전체 직사각형으로 편다.
// 파라미터는 실행 중에 ros2 param set 으로 바꾸면 다음 프레임부터 바로 반영된다.
// 보정 방법: 직선 차선 위에 로봇을 두고, BEV 에서 두 차선이 격자 세로선과 평행해질 때까지 src_* 조절
//
// 왜 BEV 를 쓰나: 카메라가 비스듬히 바닥을 보기 때문에 원본에서는 평행한 두 차선이 멀어질수록
// 가운데로 모여 보인다(원근). 바닥을 위에서 내려다본 것처럼 펴면 차선이 다시 평행해지고,
// 화면 속 거리가 실제 바닥 거리에 비례하므로 차선 폭/위치/기울기를 계산하기 쉬워진다.
//
//   원본 (src 사다리꼴)            BEV (dst 직사각형)
//        TL ---- TR                TL -------- TR
//       /          \       ->      |            |
//     BL ---------- BR             BL -------- BR
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "cv_bridge/cv_bridge.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

class BirdEyeViewNode : public rclcpp::Node
{
public:
  BirdEyeViewNode()
  : Node("bird_eye_view")
  {
    const auto image_topic = declare_parameter("image_topic", std::string("image_raw"));
    // 원본에서 펼 사다리꼴 네 점 (이미지 폭/높이 비율, 0~1 밖도 가능)
    declare_parameter("src_top_y", 0.442);
    declare_parameter("src_top_left_x", 0.178);
    declare_parameter("src_top_right_x", 0.822);
    declare_parameter("src_bottom_y", 1.0);
    declare_parameter("src_bottom_left_x", -0.276);
    declare_parameter("src_bottom_right_x", 1.276);
    declare_parameter("bev_width", 400);        // 출력 크기 [px]
    declare_parameter("bev_height", 400);
    declare_parameter("draw_grid", true);       // GUI용 압축 영상에만 격자 표시
    declare_parameter("jpeg_quality", 70);

    // 카메라 영상은 최신 프레임만 중요하므로 SensorDataQoS (best effort, 오래된 프레임은 버림)
    const auto qos = rclcpp::SensorDataQoS();
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, qos, [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {onImage(msg);});
    bev_pub_ = create_publisher<sensor_msgs::msg::Image>("image_bev", qos);
    compressed_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      "image_bev/compressed", qos);

    RCLCPP_INFO(get_logger(), "bird_eye_view 시작 (구독: %s)", image_topic.c_str());
  }

private:
  // 파라미터 읽기 단축 함수. 매 프레임 새로 읽으므로 ros2 param set 이 바로 반영된다
  double d(const std::string & name) {return get_parameter(name).as_double();}
  int i(const std::string & name) {return static_cast<int>(get_parameter(name).as_int());}

  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    // 구독자가 아무도 없으면 변환 자체를 건너뛴다 (Jetson CPU 절약)
    const bool want_raw = bev_pub_->get_subscription_count() > 0;
    const bool want_jpeg = compressed_pub_->get_subscription_count() > 0;
    if (!want_raw && !want_jpeg) {
      return;
    }

    // ROS 이미지 -> OpenCV Mat (bgr8). toCvShare 는 가능하면 복사 없이 메시지 메모리를 그대로 쓴다
    cv::Mat frame;
    try {
      frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "cv_bridge: %s", e.what());
      return;
    }

    // 비율로 저장된 사다리꼴 네 점을 실제 픽셀 좌표로 바꾼다 (순서: 좌상, 우상, 우하, 좌하)
    const float w = static_cast<float>(frame.cols), h = static_cast<float>(frame.rows);
    const int bw = std::max(1, i("bev_width")), bh = std::max(1, i("bev_height"));
    const cv::Point2f src[4] = {
      {static_cast<float>(d("src_top_left_x")) * w, static_cast<float>(d("src_top_y")) * h},
      {static_cast<float>(d("src_top_right_x")) * w, static_cast<float>(d("src_top_y")) * h},
      {static_cast<float>(d("src_bottom_right_x")) * w, static_cast<float>(d("src_bottom_y")) * h},
      {static_cast<float>(d("src_bottom_left_x")) * w, static_cast<float>(d("src_bottom_y")) * h},
    };
    // 사다리꼴을 펼 목표: 출력 영상의 네 모서리 (src 와 같은 순서)
    const cv::Point2f dst[4] = {
      {0.0f, 0.0f}, {static_cast<float>(bw), 0.0f},
      {static_cast<float>(bw), static_cast<float>(bh)}, {0.0f, static_cast<float>(bh)},
    };
    // BEV -> 원본 방향 행렬로 구해 역변환한다. 원본 -> BEV 방향은 사다리꼴 옆변의 소실점이
    // 원본 맨 윗줄(y=0)에 오면 행렬의 h33 이 0 이 되어 getPerspectiveTransform 이 풀지 못한다.
    cv::Mat bev;
    cv::warpPerspective(frame, bev, cv::getPerspectiveTransform(dst, src), cv::Size(bw, bh),
      cv::INTER_LINEAR | cv::WARP_INVERSE_MAP, cv::BORDER_CONSTANT);

    // 비전 노드용 원본 BEV (격자 없음)
    if (want_raw) {
      bev_pub_->publish(*cv_bridge::CvImage(msg->header, "bgr8", bev).toImageMsg());
    }
    // GUI 용 JPEG (보정할 때 보기 쉽게 격자를 그림)
    if (want_jpeg) {
      cv::Mat shown = bev;
      if (get_parameter("draw_grid").as_bool()) {
        shown = bev.clone();   // 비전 노드로 보낸 bev 에 격자가 섞이지 않도록 복사본에 그림
        // 8x8 격자. 세로 가운데 선(k == 4)만 노란색 -> 화면 중앙 기준선
        for (int k = 1; k < 8; ++k) {
          const cv::Scalar color = k == 4 ? cv::Scalar(0, 255, 255) : cv::Scalar(0, 180, 0);
          cv::line(shown, {bw * k / 8, 0}, {bw * k / 8, bh}, color, 1);
          cv::line(shown, {0, bh * k / 8}, {bw, bh * k / 8}, cv::Scalar(0, 180, 0), 1);
        }
      }
      sensor_msgs::msg::CompressedImage out;
      out.header = msg->header;
      out.format = "jpeg";
      cv::imencode(".jpg", shown, out.data, {cv::IMWRITE_JPEG_QUALITY, i("jpeg_quality")});
      compressed_pub_->publish(out);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr bev_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BirdEyeViewNode>());
  rclcpp::shutdown();
  return 0;
}
