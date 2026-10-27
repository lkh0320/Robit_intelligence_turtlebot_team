// 도로 엔진 + decodeRoad 를 jpg 폴더에 돌려 마스크 png 를 남긴다 (ROS 없이, ultralytics 결과와 비교용)
//   사용: road_seg_frames <엔진> <프레임 폴더> <출력 폴더> [conf]
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "grid_drive/road_seg.hpp"
#include "grid_drive/trt_engine.hpp"

int main(int argc, char ** argv)
{
  if (argc < 4) {
    std::fprintf(stderr, "usage: %s engine frames_dir out_dir [conf]\n", argv[0]);
    return 1;
  }
  grid_drive::TrtEngine eng(argv[1]);
  grid_drive::RoadSegParams rp;
  if (argc > 4) {
    rp.conf = static_cast<float>(std::atof(argv[4]));
  }
  std::filesystem::create_directories(argv[3]);
  std::vector<std::filesystem::path> jpgs;
  for (auto & e : std::filesystem::directory_iterator(argv[2])) {
    if (e.path().extension() == ".jpg" || e.path().extension() == ".png") {
      jpgs.push_back(e.path());
    }
  }
  std::sort(jpgs.begin(), jpgs.end());
  double total = 0;
  for (const auto & j : jpgs) {
    const cv::Mat img = cv::imread(j.string());
    const auto t0 = std::chrono::steady_clock::now();
    cv::Mat input;
    const auto lb = grid_drive::letterbox(img, eng.inputSize(), input);
    const auto outs = eng.infer(grid_drive::toBlob(input));
    const auto rs = grid_drive::decodeRoad(outs[0], outs[1], eng.inputSize(), lb, img.size(), rp);
    total += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    cv::imwrite((std::filesystem::path(argv[3]) / (j.stem().string() + ".png")).string(), rs.mask);
  }
  std::printf("%zu frames, %.2f ms/frame (letterbox + infer + decode)\n", jpgs.size(),
    jpgs.empty() ? 0.0 : total / jpgs.size());
  return 0;
}
