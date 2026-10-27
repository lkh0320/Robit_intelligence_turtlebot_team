// 도로 (내 차선) 세그멘테이션 출력 -> 원본 화면 마스크 (ROS / TensorRT 없이 테스트 가능)
//
//   모델: YOLO26n-seg 한 클래스 road (2026-10-27 r2, 노란 선 ~ 흰 선 사이 내 차선만), nms 없이 export
//     output0: 1 x (4 + 1 + nm) x N   박스 cx cy w h (입력 픽셀) / 점수 (sigmoid 끝남) / 마스크 계수 nm 개
//     output1: 1 x nm x ph x pw       proto (입력 1/4 크기)
//   처리: 점수 >= conf -> NMS -> 계수 x proto -> sigmoid -> 박스 밖은 0 -> 모든 박스의 최댓값 (합집합)
//     -> letterbox 회색 띠를 잘라 원본 크기로 (bilinear) -> mask_thr 넘으면 255
//   ultralytics predict (retina_masks=False) 와 같은 순서 (proto 크기에서 자르고 올려서 0.5)
#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace grid_drive
{

struct Letterbox
{
  double scale = 1.0;
  int pad_x = 0, pad_y = 0;
};

// 비율 유지하며 size x size 에 넣고 남는 곳은 회색 114 (학습 때와 같음). 640x480 -> 320: 0.5 배, 위아래 40px
inline Letterbox letterbox(const cv::Mat & src, int size, cv::Mat & dst)
{
  Letterbox lb;
  lb.scale = std::min(static_cast<double>(size) / src.cols, static_cast<double>(size) / src.rows);
  const int w = cvRound(src.cols * lb.scale), h = cvRound(src.rows * lb.scale);
  lb.pad_x = (size - w) / 2;
  lb.pad_y = (size - h) / 2;
  cv::Mat resized;
  cv::resize(src, resized, {w, h}, 0, 0, cv::INTER_LINEAR);
  cv::copyMakeBorder(resized, dst, lb.pad_y, size - h - lb.pad_y, lb.pad_x, size - w - lb.pad_x,
    cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
  return lb;
}

// letterbox 결과 (BGR 8bit) -> 1 x 3 x S x S 입력 (RGB, 0~1). cv::dnn::blobFromImage(.., 1/255, swapRB) 와 같음
inline cv::Mat toBlob(const cv::Mat & input)
{
  cv::Mat rgb, f;
  cv::cvtColor(input, rgb, cv::COLOR_BGR2RGB);
  rgb.convertTo(f, CV_32FC3, 1.0 / 255.0);
  const int sz[4] = {1, 3, input.rows, input.cols};
  cv::Mat blob(4, sz, CV_32F);
  std::vector<cv::Mat> ch;
  for (int c = 0; c < 3; ++c) {
    ch.emplace_back(input.rows, input.cols, CV_32F, blob.ptr<float>(0, c));
  }
  cv::split(f, ch);
  return blob;
}

struct RoadSegParams
{
  float conf = 0.5f;        // 이보다 낮은 박스는 버림
  float nms = 0.45f;        // 이 IoU 넘게 겹치면 하나만 (합집합이라 결과는 거의 같고 계산만 줄임)
  int max_det = 10;
  float mask_thr = 0.5f;
};

struct RoadSeg
{
  cv::Mat mask;             // raw_size, 0/255. 박스가 없으면 전부 0
  int detections = 0;
  float best_conf = 0;
};

inline float iou(const cv::Rect2f & a, const cv::Rect2f & b)
{
  const float inter = (a & b).area();
  return inter <= 0 ? 0.f : inter / (a.area() + b.area() - inter);
}

// out0 / proto: 엔진 출력 그대로 (CV_32F, 연속). input_size: 엔진 입력 한 변
inline RoadSeg decodeRoad(
  const cv::Mat & out0, const cv::Mat & proto, int input_size, const Letterbox & lb,
  cv::Size raw_size, const RoadSegParams & rp = {})
{
  CV_Assert(out0.dims == 3 && proto.dims == 4 && out0.type() == CV_32F && proto.type() == CV_32F);
  const int C = out0.size[1], N = out0.size[2];
  const int nm = proto.size[1], ph = proto.size[2], pw = proto.size[3];
  CV_Assert(C == 5 + nm);
  const float * d = out0.ptr<float>();

  // 1. 점수 순 + NMS
  std::vector<int> cand;
  for (int i = 0; i < N; ++i) {
    if (d[4 * N + i] >= rp.conf) {
      cand.push_back(i);
    }
  }
  std::sort(cand.begin(), cand.end(), [&](int a, int b) {return d[4 * N + a] > d[4 * N + b];});
  auto boxOf = [&](int i) {
      return cv::Rect2f(d[i] - d[2 * N + i] / 2, d[N + i] - d[3 * N + i] / 2, d[2 * N + i], d[3 * N + i]);
    };
  std::vector<int> keep;
  for (int i : cand) {
    if (static_cast<int>(keep.size()) >= rp.max_det) {
      break;
    }
    const cv::Rect2f b = boxOf(i);
    if (std::none_of(keep.begin(), keep.end(), [&](int k) {return iou(boxOf(k), b) > rp.nms;})) {
      keep.push_back(i);
    }
  }

  RoadSeg rs;
  rs.detections = static_cast<int>(keep.size());
  rs.mask = cv::Mat::zeros(raw_size, CV_8U);
  if (keep.empty()) {
    return rs;
  }
  rs.best_conf = d[4 * N + keep[0]];

  // 2. 계수 x proto -> sigmoid, 박스 밖 0, 합집합 (최댓값)
  cv::Mat coef(static_cast<int>(keep.size()), nm, CV_32F);
  for (int k = 0; k < coef.rows; ++k) {
    for (int j = 0; j < nm; ++j) {
      coef.at<float>(k, j) = d[(5 + j) * N + keep[k]];
    }
  }
  const cv::Mat P(nm, ph * pw, CV_32F, const_cast<float *>(proto.ptr<float>()));
  const cv::Mat logits = coef * P;
  cv::Mat prob = cv::Mat::zeros(ph, pw, CV_32F);
  const float sx = static_cast<float>(pw) / input_size, sy = static_cast<float>(ph) / input_size;
  for (int k = 0; k < coef.rows; ++k) {
    const cv::Rect2f b = boxOf(keep[k]);
    const float x1 = b.x * sx, x2 = (b.x + b.width) * sx, y1 = b.y * sy, y2 = (b.y + b.height) * sy;
    const float * l = logits.ptr<float>(k);
    for (int r = std::max(0, static_cast<int>(std::ceil(y1))); r < ph && r < y2; ++r) {
      float * pr = prob.ptr<float>(r);
      for (int c = std::max(0, static_cast<int>(std::ceil(x1))); c < pw && c < x2; ++c) {
        pr[c] = std::max(pr[c], 1.f / (1.f + std::exp(-l[r * pw + c])));
      }
    }
  }

  // 3. letterbox 띠 자르기 -> 원본 크기 -> 문턱
  const double w = raw_size.width * lb.scale, h = raw_size.height * lb.scale;
  cv::Rect roi(cvRound(lb.pad_x * sx), cvRound(lb.pad_y * sy), cvRound(w * sx), cvRound(h * sy));
  roi &= cv::Rect(0, 0, pw, ph);
  cv::Mat up;
  cv::resize(prob(roi), up, raw_size, 0, 0, cv::INTER_LINEAR);
  rs.mask = up > rp.mask_thr;
  return rs;
}

}  // namespace grid_drive
