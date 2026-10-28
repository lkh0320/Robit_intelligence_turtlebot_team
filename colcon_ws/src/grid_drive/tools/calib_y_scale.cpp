// 바닥 지도 앞뒤 축척 (map_y_scale = 지도 앞뒤 mm / 실제 mm) 을 녹화로 맞춘다 (ROS 없이)
//   연속 두 프레임 a, b: a 에서 본 선 칸을 바퀴 이동량 (odom) 과 축척 s 로 옮겨 b 의 선 거리 변환에서 평균 거리를 잰다.
//   b 에서도 보이는 곳에 떨어진 점만 센다. 평균 거리가 가장 작은 s 가 맞는 축척
//   사용: calib_y_scale <extract_drive.py 폴더>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "grid_drive/floor_obs.hpp"

int main(int argc, char ** argv)
{
  const std::string in = argv[1];
  grid_drive::FloorParams p;
  grid_drive::ColorParams colors;
  colors.white_l_min = 170;
  std::vector<double> scales;
  for (double s = 0.6; s <= 3.01; s += 0.2) {
    scales.push_back(s);
  }
  std::vector<double> sum(scales.size(), 0);
  std::vector<long> cnt(scales.size(), 0);
  int pairs = 0;
  for (auto & e : std::filesystem::directory_iterator(in)) {
    if (e.path().extension() != ".csv") {
      continue;
    }
    std::ifstream f(e.path());
    std::string line;
    std::getline(f, line);
    struct Row {std::string name; bool mv; double ds, dyaw;};
    std::vector<Row> rows;
    while (std::getline(f, line)) {
      std::stringstream ss(line);
      std::string n, mv, ds, dy;
      std::getline(ss, n, ',');
      std::getline(ss, mv, ',');
      std::getline(ss, ds, ',');
      std::getline(ss, dy, ',');
      rows.push_back({n, mv == "1", std::stod(ds), std::stod(dy)});
    }
    const int GAP = argc > 2 ? std::atoi(argv[2]) : 5;   // 이 프레임 수 떨어진 쌍 (0.1s 간격 녹화)
    for (size_t i = GAP; i < rows.size(); ++i) {
      bool all = true;
      double dsum = 0, ysum = 0;
      for (size_t j = i - GAP; j <= i; ++j) {
        all = all && rows[j].mv;
        if (j > i - GAP) {
          dsum += rows[j].ds;
          ysum += rows[j].dyaw;
        }
      }
      if (!all || dsum < 30 || std::abs(ysum) > 0.12) {
        continue;
      }
      const cv::Mat raw_a = cv::imread((std::filesystem::path(in) / (rows[i - GAP].name + ".jpg")).string());
      const cv::Mat raw = cv::imread((std::filesystem::path(in) / (rows[i].name + ".jpg")).string());
      if (raw.empty() || raw_a.empty()) {
        continue;
      }
      const auto prev = grid_drive::observeColor(raw_a, p, colors);
      auto cur = grid_drive::observeColor(raw, p, colors);
      rows[i].ds = dsum;
      rows[i].dyaw = ysum;
      if (
        cv::countNonZero(prev.line) > 200 && cv::countNonZero(cur.line) > 200)
      {
        cv::Mat dist, inv;
        cv::bitwise_not(cur.line, inv);
        cv::distanceTransform(inv, dist, cv::DIST_L2, 3);
        std::vector<cv::Point> pts;
        cv::findNonZero(prev.line, pts);
        const double ds = rows[i].ds, dyaw = rows[i].dyaw;
        const double half = dyaw / 2, c = std::cos(dyaw), sn = std::sin(dyaw);
        const double tx = -ds * std::sin(half), ty = ds * std::cos(half);
        for (size_t k = 0; k < scales.size(); ++k) {
          const double s = scales[k];
          double acc = 0;
          long n = 0;
          for (size_t j = 0; j < pts.size(); j += 3) {
            const double x = pts[j].x * p.cell_mm - p.half_width_mm;
            const double y = p.far_mm - pts[j].y * p.cell_mm;
            const double xr = x - tx, yr = y / s - ty;
            const double nx = xr * c + yr * sn, ny = (-xr * sn + yr * c) * s;
            const int u = cvRound((nx + p.half_width_mm) / p.cell_mm);
            const int v = cvRound((p.far_mm - ny) / p.cell_mm);
            if (u < 0 || v < 0 || u >= cur.valid.cols || v >= cur.valid.rows || !cur.valid.at<uint8_t>(v, u)) {
              continue;
            }
            acc += std::min(40.0f, dist.at<float>(v, u)) * p.cell_mm;
            ++n;
          }
          if (n > 100) {
            sum[k] += acc / n;
            ++cnt[k];
          }
        }
        ++pairs;
      }
      i += GAP - 1;
    }
  }
  std::printf("pairs %d\n", pairs);
  for (size_t k = 0; k < scales.size(); ++k) {
    std::printf("scale %.1f  mean line distance %.1f mm  (%ld)\n", scales[k], cnt[k] ? sum[k] / cnt[k] : -1, cnt[k]);
  }
  return 0;
}
