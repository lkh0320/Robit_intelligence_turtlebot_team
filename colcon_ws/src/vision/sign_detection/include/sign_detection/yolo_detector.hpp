// YOLOv8 ONNX 전처리 / 후처리 (OpenCV DNN 용, ROS 의존 없음 -> test/test_yolo_detector.cpp 에서 검사)
//
// 모델 출력 모양: 1 x (4 + 클래스 수) x 후보 수   (640 입력이면 후보 8400개)
//   행 0~3: 박스 중심 cx, cy, 폭 w, 높이 h (letterbox 된 입력 이미지 픽셀 좌표)
//   행 4~ : 클래스별 점수 (이미 0~1, 클래스 중 가장 높은 값이 그 후보의 신뢰도)
// NMS 는 모델 밖에서 해야 한다 (ultralytics 기본 ONNX export 는 NMS 를 넣지 않는다)
#pragma once

#include <algorithm>
#include <optional>
#include <vector>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

#include "sign_detection/yolo_classes.hpp"

namespace sign_detection
{

struct Detection
{
  int cls;          // YOLO 클래스 번호 (yolo_classes.hpp)
  float conf;       // 0 ~ 1
  cv::Rect2d box;   // 원본 이미지 픽셀 좌표
};

// 원본 -> 모델 입력 변환 정보 (모델 좌표 = 원본 좌표 * scale + pad)
struct Letterbox
{
  double scale = 1.0;
  int pad_x = 0, pad_y = 0;
};

// 비율을 유지한 채 size x size 정사각형에 넣고 남는 곳은 회색(114)으로 채운다 (학습 때와 같은 방식)
// 640x480 카메라 -> 640x640 이면 scale 1, 위아래 80px 씩 회색
inline Letterbox letterbox(const cv::Mat & src, int size, cv::Mat & dst)
{
  Letterbox lb;
  lb.scale = std::min(static_cast<double>(size) / src.cols, static_cast<double>(size) / src.rows);
  const int w = cvRound(src.cols * lb.scale), h = cvRound(src.rows * lb.scale);
  lb.pad_x = (size - w) / 2;
  lb.pad_y = (size - h) / 2;
  cv::Mat resized;
  if (w == src.cols && h == src.rows) {
    resized = src;
  } else {
    cv::resize(src, resized, {w, h}, 0, 0, cv::INTER_LINEAR);
  }
  cv::copyMakeBorder(resized, dst, lb.pad_y, size - h - lb.pad_y, lb.pad_x, size - w - lb.pad_x,
    cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
  return lb;
}

// 모델 출력이 위 모양(1 x (4 + num_classes) x N)인지 확인
inline bool validOutput(const cv::Mat & out, int num_classes)
{
  return out.dims == 3 && out.size[0] == 1 && out.size[1] == 4 + num_classes &&
         out.type() == CV_32F;
}

// 출력 -> 신뢰도 conf_th 이상인 박스, 클래스별 NMS 후 (신뢰도 높은 순)
//   NMS: 같은 클래스 박스끼리 IoU 가 nms_th 보다 많이 겹치면 신뢰도 높은 것만 남긴다
//   img: 원본 크기 (박스를 이 안으로 자른다)
inline std::vector<Detection> decode(
  const cv::Mat & out, int num_classes, const Letterbox & lb, cv::Size img,
  float conf_th, float nms_th)
{
  const int n = out.size[2];
  // (4 + nc) x N 을 N x (4 + nc) 로 돌려서 후보 하나 = 한 행으로 읽는다
  const cv::Mat rows = cv::Mat(4 + num_classes, n, CV_32F, const_cast<float *>(out.ptr<float>())).t();
  const cv::Rect2d bounds(0, 0, img.width, img.height);

  std::vector<cv::Rect2d> boxes;
  std::vector<float> scores;
  std::vector<int> classes;
  for (int i = 0; i < n; ++i) {
    const float * r = rows.ptr<float>(i);
    const float * best = std::max_element(r + 4, r + 4 + num_classes);
    if (*best < conf_th) {
      continue;
    }
    // 모델 좌표 (중심, 크기) -> 원본 좌표 (왼쪽 위, 크기)
    const double w = r[2] / lb.scale, h = r[3] / lb.scale;
    const double x = (r[0] - lb.pad_x) / lb.scale - w / 2, y = (r[1] - lb.pad_y) / lb.scale - h / 2;
    const cv::Rect2d box = cv::Rect2d(x, y, w, h) & bounds;
    if (box.area() <= 0) {
      continue;
    }
    boxes.push_back(box);
    scores.push_back(*best);
    classes.push_back(static_cast<int>(best - (r + 4)));
  }

  std::vector<int> keep;
  cv::dnn::NMSBoxesBatched(boxes, scores, classes, conf_th, nms_th, keep);
  std::vector<Detection> dets;
  for (int k : keep) {
    dets.push_back({classes[k], scores[k], boxes[k]});
  }
  std::sort(dets.begin(), dets.end(),
    [](const Detection & a, const Detection & b) {return a.conf > b.conf;});
  return dets;
}

// 표지판 클래스 중 신뢰도가 가장 높은 박스 하나 (없으면 nullopt)
// 한 표지판에 좌/우회전처럼 다른 클래스 박스가 겹쳐 잡혀도 NMS 는 클래스별이라 둘 다 남으므로 여기서 하나만 고른다
inline std::optional<Detection> bestSign(const std::vector<Detection> & dets)
{
  std::optional<Detection> best;
  for (const auto & d : dets) {
    if (is_sign(d.cls) && (!best || d.conf > best->conf)) {
      best = d;
    }
  }
  return best;
}

}  // namespace sign_detection
