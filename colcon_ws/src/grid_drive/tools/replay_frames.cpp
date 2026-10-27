// 녹화에서 뽑은 원본 프레임 (jpg 폴더) 으로 grid_planner 를 돌려 디버그 그림 / 시간을 남긴다 (ROS 없이)
//   사용: replay_frames <프레임 폴더> <출력 폴더> [매 N 장마다 그림 저장] [turn: none|left|right]
//   폴더에 <세션>.csv (name,moving,ds_mm,dyaw,v,w: extract_drive.py) 가 있으면 그 순서로 바퀴 이동량을 넣고 기억을 켠다.
//   없으면 jpg 이름 순서로, 기억은 끈다
//   환경 변수 MAP_Y_SCALE / LINE_MARGIN 으로 파라미터 바꿔 보기
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "grid_drive/grid_planner.hpp"

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::fprintf(stderr, "usage: %s frames_dir out_dir [every] [none|left|right]\n", argv[0]);
    return 1;
  }
  const std::string in = argv[1], out = argv[2];
  const int every = argc > 3 ? std::atoi(argv[3]) : 10;
  const std::string t = argc > 4 ? argv[4] : "none";
  const grid_drive::Turn turn = t == "left" ? grid_drive::Turn::LEFT :
    t == "right" ? grid_drive::Turn::RIGHT : grid_drive::Turn::NONE;
  std::filesystem::create_directories(out);
  // 재생 순서: (이름, 이동 ds [mm], dyaw [rad], 계획할지)
  struct Step {std::string file; double ds = 0, dyaw = 0; bool plan = true;};
  std::vector<Step> steps;
  bool odom = false;
  std::vector<std::filesystem::path> csvs, jpgs;
  for (auto & e : std::filesystem::directory_iterator(in)) {
    const auto ext = e.path().extension();
    if (ext == ".csv") {
      csvs.push_back(e.path());
    } else if (ext == ".jpg" || ext == ".png") {
      jpgs.push_back(e.path());
    }
  }
  std::sort(csvs.begin(), csvs.end());
  std::sort(jpgs.begin(), jpgs.end());
  for (const auto & c : csvs) {
    odom = true;
    std::ifstream f(c);
    std::string line;
    std::getline(f, line);
    bool first = true;
    while (std::getline(f, line)) {
      std::stringstream ss(line);
      std::string name, mv, ds, dy;
      std::getline(ss, name, ',');
      std::getline(ss, mv, ',');
      std::getline(ss, ds, ',');
      std::getline(ss, dy, ',');
      Step s{(std::filesystem::path(in) / (name + ".jpg")).string(), std::stod(ds), std::stod(dy), mv == "1"};
      if (first) {
        s.file = "RESET:" + s.file;   // 세션 시작
        first = false;
      }
      steps.push_back(s);
    }
  }
  if (!odom) {
    for (const auto & j : jpgs) {
      steps.push_back({j.string(), 0, 0, true});
    }
  }

  grid_drive::Params p;
  p.memory = odom;
  if (const char * e = std::getenv("MAP_Y_SCALE")) {
    p.map_y_scale = std::atof(e);
  }
  if (const char * e = std::getenv("LINE_MARGIN")) {
    p.line_margin_mm = std::atof(e);
  }
  obstacle_detection::ColorParams colors;
  colors.white_l_min = 170;   // lane_detection.yaml 2026-10-26
  grid_drive::GridPlanner planner;
  std::vector<double> t_map, t_search, t_follow, t_all;
  int no_path = 0, pivot = 0, soft = 0, n = 0;
  std::string prev_session;
  for (size_t i = 0; i < steps.size(); ++i) {
    std::string file = steps[i].file;
    if (file.rfind("RESET:", 0) == 0) {
      file = file.substr(6);
      planner.reset();
    }
    const std::string name = std::filesystem::path(file).stem();
    const std::string session = name.substr(0, name.rfind('_'));
    if (!odom && session != prev_session) {
      planner.reset();
      prev_session = session;
    }
    planner.onMove(steps[i].ds, steps[i].dyaw, p);
    if (!steps[i].plan) {
      continue;
    }
    const cv::Mat raw = cv::imread(file);
    const auto r = planner.plan(raw, p, colors, turn);
    t_map.push_back(r.ms_map);
    t_search.push_back(r.ms_search);
    t_follow.push_back(r.ms_follow);
    t_all.push_back(r.ms_map + r.ms_search + r.ms_follow);
    no_path += !r.path_ok;
    pivot += r.pivot;
    soft += r.soft;
    ++n;
    if (every > 0 && (i % every == 0 || !r.path_ok || r.pivot || (r.soft && i % 5 == 0))) {
      cv::Mat vis = planner.draw(r, p), raw_s;
      cv::resize(vis, vis, cv::Size(), 2.0 * 0 + 1.0, 1.0);
      cv::resize(raw, raw_s, cv::Size(vis.rows * raw.cols / raw.rows, vis.rows));
      cv::Mat both;
      cv::hconcat(raw_s, vis, both);
      char buf[256];
      std::snprintf(buf, sizeof(buf), "%s  v %.2f w %+.2f k %+.4f goal %d %s  %.1f+%.1f+%.1f ms",
        name.c_str(), r.v, r.w, r.curvature, r.goal_kind, r.path_ok ? (r.pivot ? "PIVOT" : (r.soft ? "SOFT" : "")) : "NOPATH",
        r.ms_map, r.ms_search, r.ms_follow);
      cv::putText(both, buf, {8, 20}, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
      cv::imwrite(out + "/" + name + ".jpg", both, {cv::IMWRITE_JPEG_QUALITY, 85});
    }
  }
  auto pct = [](std::vector<double> v, double q) {
      std::sort(v.begin(), v.end());
      return v[std::min(v.size() - 1, static_cast<size_t>(v.size() * q))];
    };
  std::printf("frames %d  no path %d  soft %d  pivot %d\n", n, no_path, soft, pivot);
  std::printf("ms median / p99: map %.2f / %.2f  search %.2f / %.2f  follow %.2f / %.2f  total %.2f / %.2f\n",
    pct(t_map, 0.5), pct(t_map, 0.99), pct(t_search, 0.5), pct(t_search, 0.99),
    pct(t_follow, 0.5), pct(t_follow, 0.99), pct(t_all, 0.5), pct(t_all, 0.99));
  return 0;
}
