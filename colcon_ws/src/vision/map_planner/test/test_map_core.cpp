// 합성 트랙을 로봇 자세에 맞춰 BEV 로 그려 map_planner 를 확인한다
//   단위 테스트: 지도 옮기기 / 덮어쓰기
//   폐루프 시뮬레이션: BEV -> map_planner -> pure pursuit (lane_follower 와 같은 식) -> 이동 -> Odom
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdio>
#include <cstdlib>

#include <sstream>
#include <string>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "map_planner/map_core.hpp"

namespace mp = map_planner;

namespace
{
constexpr int W = 400, H = 400, LINE_W = 10;
const cv::Scalar FLOOR(80, 80, 80);
const cv::Vec3b YELLOW(0, 200, 230), WHITE(240, 240, 240), BROWN(10, 15, 49);

// 실제 BEV 처럼 아래 양 모서리는 찍히지 않는다 (사다리꼴 아랫변이 화면보다 넓음)
cv::Mat validMask()
{
  cv::Mat v(H, W, CV_8U, cv::Scalar(255));
  std::vector<cv::Point> left{{0, 310}, {45, H}, {0, H}}, right{{W, 310}, {W - 45, H}, {W, H}};
  cv::fillPoly(v, std::vector<std::vector<cv::Point>>{left, right}, cv::Scalar(0));
  return v;
}

// 세계 좌표 (x 오른쪽, y 앞) [px]. 차선 가운데 = lateral 0, 선은 가운데에서 ±133
//   curve_r > 0 이면 y >= curve_y0 부터 반지름 curve_r 로 curve_angle 만큼 돈 뒤 다시 직선
//   curve_dir = -1 왼쪽, +1 오른쪽 커브
struct Track
{
  double curve_y0 = 1e9, curve_r = 0, curve_angle = CV_PI / 2;
  int curve_dir = -1;
  double half_width = 133;   // 차선 가운데에서 선까지
  // 차선 가운데에서 오른쪽으로 벗어난 거리
  double lateral(double wx, double wy) const
  {
    if (curve_r <= 0) {
      return wx;
    }
    const double sx = curve_dir < 0 ? wx : -wx;   // 오른쪽 커브는 좌우를 뒤집어 왼쪽 커브로 계산
    const double vx = sx + curve_r, vy = wy - curve_y0;
    const double a = std::atan2(vy, vx);
    double d;
    if (a < 0 || wy < curve_y0) {
      d = sx;
    } else if (a <= curve_angle) {
      d = std::hypot(vx, vy) - curve_r;
    } else {
      const double ex = -curve_r + curve_r * std::cos(curve_angle);
      const double ey = curve_y0 + curve_r * std::sin(curve_angle);
      d = (sx - ex) * std::cos(curve_angle) + (wy - ey) * std::sin(curve_angle);
    }
    return curve_dir < 0 ? d : -d;
  }
};

struct Pose
{
  double x = 0, y = 0, psi = 0;   // psi + = 오른쪽
};

// 로봇 자세에서 본 BEV (노란 선 왼쪽, 흰 선 오른쪽). box: 세계 좌표 직사각형 (선택)
cv::Mat render(const Track & tr, const Pose & ps, const mp::Params & p,
  const cv::Rect2d & box = {})
{
  cv::Mat img(H, W, CV_8UC3, FLOOR);
  const double cx = p.bev_center_x * W, c = std::cos(ps.psi), s = std::sin(ps.psi);
  for (int y = 0; y < H; ++y) {
    const double ry = p.robot_y_offset_px + (H - 1 - y);
    auto * row = img.ptr<cv::Vec3b>(y);
    for (int x = 0; x < W; ++x) {
      const double rx = x - cx;
      const double wx = ps.x + rx * c + ry * s, wy = ps.y - rx * s + ry * c;
      const double d = tr.lateral(wx, wy);
      if (box.area() > 0 && box.contains(cv::Point2d(wx, wy))) {
        row[x] = BROWN;
      } else if (std::abs(d + tr.half_width) < LINE_W / 2.0) {
        row[x] = YELLOW;
      } else if (std::abs(d - tr.half_width) < LINE_W / 2.0) {
        row[x] = WHITE;
      }
    }
  }
  return img;
}

// 로봇 자세에서 본 원본 카메라 영상 (640x480, BEV 와 같은 투시 변환의 역방향)
constexpr int RW = 640, RH = 480;
cv::Mat renderRaw(const Track & tr, const Pose & ps, const mp::Params & p,
  const cv::Rect2d & box = {})
{
  cv::Mat img(RH, RW, CV_8UC3, FLOOR);
  const cv::Matx33d G = mp::bevToRaw(p, {RW, RH}), Hm = G.inv();
  const double cx = p.bev_center_x * p.bev_width, c = std::cos(ps.psi), s = std::sin(ps.psi);
  for (int v = 0; v < RH; ++v) {
    auto * row = img.ptr<cv::Vec3b>(v);
    for (int u = 0; u < RW; ++u) {
      const cv::Vec3d b = Hm * cv::Vec3d(u, v, 1);
      if (b[2] <= 1e-9) {
        continue;   // 지평선 위
      }
      const double bx = b[0] / b[2], by = b[1] / b[2];
      const double rx = bx - cx, ry = p.robot_y_offset_px + (p.bev_height - 1 - by);
      if (ry > 3000 || ry < 0) {
        continue;
      }
      const double wx = ps.x + rx * c + ry * s, wy = ps.y - rx * s + ry * c;
      const double d = tr.lateral(wx, wy);
      if (box.area() > 0 && box.contains(cv::Point2d(wx, wy))) {
        row[u] = BROWN;
      } else if (std::abs(d + tr.half_width) < LINE_W / 2.0) {
        row[u] = YELLOW;
      } else if (std::abs(d - tr.half_width) < LINE_W / 2.0) {
        row[u] = WHITE;
      }
    }
  }
  return img;
}

bool simRaw() {return !std::getenv("SIM_BEV");}   // 기본: 원본 영상 입력 (노드 기본값과 같게)

mp::Result stepImage(mp::MapPlanner & pl, const cv::Mat & img, const mp::Params & p,
  const mp::Odom & od = {}, const mp::Bias & b = {})
{
  const path_planner::Params mask;   // 색 기준은 path_planner 기본값 (실측값)
  return pl.step(path_planner::buildMaps(img, mask), validMask(), p, b, od);
}

struct SimResult
{
  std::vector<double> lat;   // 프레임마다 차선 가운데에서 벗어난 거리 [px]
  std::vector<Pose> poses;
  bool lost = false;
};

// odom_scale: 실제 이동 대비 Odom 이 알려 주는 이동 비율 (바퀴 미끄러짐 흉내)
// 실험용: SIM_SET="이름=값,..." 으로 파라미터 바꾸기
mp::Params tweak(mp::Params p)
{
  if (const char * e = std::getenv("SIM_SET")) {
    std::stringstream ss(e);
    std::string kv;
    while (std::getline(ss, kv, ',')) {
      const auto eq = kv.find('=');
      const std::string k = kv.substr(0, eq);
      const double v = std::stod(kv.substr(eq + 1));
      if (k == "len") {p.path_length_px = v;}
      if (k == "lw") {p.line_weight = v;}
      if (k == "aw") {p.align_weight = v;}
      if (k == "cw") {p.clearance_weight = v;}
      if (k == "sw") {p.smooth_weight = v;}
      if (k == "stw") {p.straight_weight = v;}
      if (k == "n") {p.n_paths = static_cast<int>(v);}
      if (k == "nr") {p.n_curv_rates = static_cast<int>(v);}
      if (k == "alpha") {p.smooth_alpha = v;}
      if (k == "dkmax") {p.max_curvature_rate = v;}
      if (k == "la") {p.lookahead_px = v;}
      if (k == "rr") {p.raw_max_range_px = v;}
      if (k == "rl") {p.raw_max_lateral_px = v;}
      if (k == "mem") {p.memory_px = v;}
      if (k == "rmem") {p.raw_memory_px = v;}
      if (k == "bw") {p.box_weight = v;}
      if (k == "bm") {p.box_margin_px = v;}
    }
  }
  return p;
}

SimResult simulate(const Track & tr, Pose ps, int frames, double odom_scale = 1.0,
  const cv::Rect2d & box = {}, const mp::Params & p0 = {}, double ds = 4.0)
{
  const mp::Params p = tweak(p0);
  mp::MapPlanner pl;
  mp::Odom od;
  SimResult out;
  for (int f = 0; f < frames; ++f) {
    mp::Result r;
    if (simRaw()) {
      // 노드와 같이: 원본 한 장에서 BEV 도 만들어 둘 다 쓴다
      const auto o = mp::observe(renderRaw(tr, ps, p, box), path_planner::Params{}, p);
      if (std::getenv("SIM_BEVONLY")) {   // 실험: 원본에서 만든 BEV 만 쓰기
        r = pl.step(o.bev_maps, o.bev_valid, p, {}, od);
      } else {
        r = pl.step(o, p, {}, od);
      }
    } else {
      r = stepImage(pl, render(tr, ps, p, box), p, od);
    }
    if (!r.detected) {
      out.lost = true;
      return out;
    }
    // lane_follower 의 pure pursuit: 곡률 = 2 sin(angle) / lookahead (+ 오른쪽)
    const double kappa = 2 * std::sin(r.angle) / p.follower_lookahead_px;
    const double dpsi = kappa * ds;
    ps.x += ds * std::sin(ps.psi + dpsi / 2);
    ps.y += ds * std::cos(ps.psi + dpsi / 2);
    ps.psi += dpsi;
    od.ds_px = ds * odom_scale;
    od.dtheta = -dpsi * odom_scale;   // Odom 은 왼쪽이 +
    out.lat.push_back(tr.lateral(ps.x, ps.y));
    out.poses.push_back(ps);
    if (const char * snap = std::getenv("SIM_SNAP")) {
      // SIM_SNAP=폴더 : 10 프레임마다 지도 그림 저장
      if (f % 10 == 0) {
        cv::imwrite(std::string(snap) + "/f" + std::to_string(1000 + f) + ".png",
          mp::drawMap(pl.map(), r, p, 1));
      }
    }
    if (const char * fr = std::getenv("SIM_SCORES")) {
      if (std::atoi(fr) == f) {
        std::vector<const mp::Candidate *> v;
        for (const auto & c : r.cands) {
          v.push_back(&c);
        }
        std::sort(v.begin(), v.end(), [](auto a, auto b) {return a->score > b->score;});
        for (size_t i = 0; i < v.size() && i < 12; ++i) {
          std::printf("k0 %+.5f dk %+.7f score %.3f clear %.3f mis %.3f hit %.3f known %.2f blk %d\n",
            v[i]->k0, v[i]->dk, v[i]->score, v[i]->clearance, v[i]->misalign, v[i]->line_hit,
            v[i]->known, v[i]->blocked);
        }
        for (const auto & c : r.cands) {
          if (std::abs(c.k0 - 0.00067) < 1e-5 && std::abs(c.dk + 0.0000111) < 1e-6) {
            std::printf("corr k0 %+.5f dk %+.7f score %.3f clear %.3f mis %.3f hit %.3f\n", c.k0, c.dk,
              c.score, c.clearance, c.misalign, c.line_hit);
          }
        }
      }
    }
    if (std::getenv("SIM_DUMP")) {
      std::printf("%d lat %+.1f psi %+.3f k0 %+.5f dk %+.7f ang %+.3f conf %.1f\n", f,
        out.lat.back(), ps.psi, r.k0, r.dk, r.angle, r.confidence);
    }
  }
  return out;
}

double maxAbs(const std::vector<double> & v, size_t from = 0)
{
  double m = 0;
  for (size_t i = from; i < v.size(); ++i) {
    m = std::max(m, std::abs(v[i]));
  }
  if (std::getenv("SIM_SUMMARY")) {
    std::printf("maxAbs(from %zu) = %.1f\n", from, m);
  }
  return m;
}
}  // namespace

// BEV 에서 로봇 좌표 q 에 점 하나를 본 것처럼 지도에 기록
void observeDot(mp::LocalMap & m, const mp::Params & p, const cv::Point2d & q)
{
  cv::Mat line = cv::Mat::zeros(H, W, CV_8U), box = cv::Mat::zeros(H, W, CV_8U);
  const cv::Point b(cvRound(p.bev_center_x * W + q.x), cvRound((H - 1) - (q.y - p.robot_y_offset_px)));
  cv::circle(line, b, 8, cv::Scalar(255), -1);
  m.integrate(line, box, cv::Mat(H, W, CV_8U, cv::Scalar(255)), p.bev_center_x,
    p.robot_y_offset_px);
}

TEST(LocalMap, IntegrateThenMoveForward)
{
  // 로봇 앞 500px 에 점을 보고, 200px 앞으로 가면 그 점은 300px 앞에 있어야 한다 (안 보이는 구간)
  const mp::Params p;
  mp::LocalMap m;
  m.configure(p);
  observeDot(m, p, {0, 500});
  EXPECT_GT(m.at(m.line, {0, 500}), 0.5f);
  m.move({200, 0});
  EXPECT_GT(m.at(m.line, {0, 300}), 0.5f);
  EXPECT_LT(m.at(m.line, {0, 500}), 0.5f);
  EXPECT_GT(m.at(m.known, {0, 300}), 0.5f);   // 지금은 안 보이는 곳이지만 본 적 있음
  EXPECT_LT(m.at(m.known, {0, 100}), 0.5f);   // 처음부터 안 보이던 곳
}

TEST(LocalMap, RotateLeftMovesPointsRight)
{
  // 로봇이 왼쪽으로 돌면 정면에 있던 점은 오른쪽에 보인다
  const mp::Params p;
  mp::LocalMap m;
  m.configure(p);
  observeDot(m, p, {0, 400});
  m.move({0, 0.3});
  EXPECT_GT(m.at(m.line, {400 * std::sin(0.3), 400 * std::cos(0.3)}), 0.5f);
}

TEST(LocalMap, RecenterKeepsLines)
{
  // 지도를 옮겨도 (정수 칸) 기록이 그대로 따라온다
  mp::Params p;
  p.map_recenter_px = 100;
  mp::LocalMap m;
  m.configure(p);
  observeDot(m, p, {40, 600});
  for (int i = 0; i < 10; ++i) {
    m.move({50, 0});
  }
  EXPECT_GT(m.at(m.line, {40, 100}), 0.5f);
}

TEST(MapPlanner, StraightCentered)
{
  mp::Params p;
  mp::MapPlanner pl;
  mp::Result r;
  for (int i = 0; i < 5; ++i) {
    r = stepImage(pl, render({}, {}, p), p);
  }
  ASSERT_TRUE(r.detected);
  EXPECT_NEAR(r.offset, 0.0, 0.05);
  EXPECT_NEAR(r.angle, 0.0, 0.05);
  EXPECT_GT(r.confidence, 0.8f);
}

TEST(MapPlanner, NothingVisible)
{
  mp::Params p;
  mp::MapPlanner pl;
  EXPECT_FALSE(stepImage(pl, cv::Mat(H, W, CV_8UC3, FLOOR), p).detected);
}

TEST(MapPlanner, CurveAheadDoesNotTurnYet)
{
  // 왼쪽 커브가 바퀴 축 45cm 앞에서 시작 (BEV 에 보임) -> 바퀴 축 앞 20cm 목표점은 아직 거의 정면
  Track tr;
  tr.curve_y0 = 450;
  tr.curve_r = 450;
  mp::Params p;
  mp::MapPlanner pl;
  mp::Result r;
  for (int i = 0; i < 5; ++i) {
    r = stepImage(pl, render(tr, {}, p), p);
  }
  ASSERT_TRUE(r.detected);
  EXPECT_GT(r.angle, -0.08);     // 목표점은 아직 거의 정면 (안쪽을 미리 파고들지 않음)
}

TEST(PlannerSim, StraightRecoversWithoutWeaving)
{
  // 차선 가운데에서 4cm 오른쪽, 5도 비스듬히 출발 -> 가운데로 돌아와 흔들리지 않는다
  Pose ps;
  ps.x = 40;
  ps.psi = 0.09;
  const auto s = simulate({}, ps, 300);
  ASSERT_FALSE(s.lost);
  EXPECT_LT(maxAbs(s.lat), 60);
  EXPECT_LT(maxAbs(s.lat, 200), 10);     // 마지막 40cm 는 가운데 ±1cm
}

TEST(PlannerSim, LeftCurveStaysInLane)
{
  // 50cm 직진 뒤 반지름 45cm 왼쪽 90도 커브, 다시 직선
  Track tr;
  tr.curve_y0 = 500;
  tr.curve_r = 450;
  const auto s = simulate(tr, {}, 350);
  ASSERT_FALSE(s.lost);
  EXPECT_LT(maxAbs(s.lat), 20);          // 로봇 반폭 103px 이면 선까지 여유가 약 25px
}

TEST(PlannerSim, RightCurveStaysInLane)
{
  Track tr;
  tr.curve_y0 = 500;
  tr.curve_r = 450;
  tr.curve_dir = 1;
  const auto s = simulate(tr, {}, 350);
  ASSERT_FALSE(s.lost);
  EXPECT_LT(maxAbs(s.lat), 20);
}

TEST(PlannerSim, WideLaneStaysInside)
{
  // 폭 46cm 차선 (공사 구간처럼 넓은 곳), 10cm 오른쪽에서 출발.
  // BEV 가 로봇 좌우 약 20cm 까지만 보여 양쪽 선이 한꺼번에 안 보이므로 가운데까지는 못 가고,
  // 보이는 선에서 반 차선 폭(133px) 떨어져 따라간다 -> 선에 닿지 않으면 된다
  Track tr;
  tr.half_width = 230;
  Pose ps;
  ps.x = 100;
  const auto s = simulate(tr, ps, 300);
  ASSERT_FALSE(s.lost);
  EXPECT_LT(maxAbs(s.lat), tr.half_width - LINE_W / 2.0 - 103);
}

TEST(PlannerSim, OdometryErrorTolerated)
{
  // 바퀴 속도로 계산한 이동이 실제와 10% 달라도 (미끄러짐, 명령값 사용) 커브를 돈다
  // (시뮬레이션에서 15% 작게 알려 주면 커브 끝에서 놓친다 -> 엔코더 값으로 바꾸면 나아질 것)
  Track tr;
  tr.curve_y0 = 500;
  tr.curve_r = 450;
  for (const double scale : {0.9, 1.1}) {
    const auto s = simulate(tr, {}, 350, scale);
    ASSERT_FALSE(s.lost) << "scale " << scale;
    EXPECT_LT(maxAbs(s.lat), 25) << "scale " << scale;
  }
}

TEST(PlannerSim, AvoidsBoxInWideLane)
{
  // 폭 46cm 차선 (공사 구간) 오른쪽 15cm 를 막은 박스 (1m 앞) -> 왼쪽으로 비켜 지나간다
  // (실제 공사 구간 폭 / 박스 위치는 실측 후 맞출 것)
  Track tr;
  tr.half_width = 230;
  const cv::Rect2d box(80, 1000, 150, 120);
  const auto s = simulate(tr, {}, 320, 1.0, box);
  ASSERT_FALSE(s.lost);
  for (size_t f = 0; f < s.poses.size(); ++f) {
    const auto & ps = s.poses[f];
    // 박스 옆을 지날 때 로봇 몸통(반폭 103px)이 박스와 겹치면 안 된다
    if (ps.y > box.y - 30 && ps.y < box.y + box.height + 30) {
      EXPECT_LT(ps.x + 103, box.x) << "frame " << f;
    }
    // 선은 넘지 않는다
    EXPECT_LT(std::abs(s.lat[f]), tr.half_width - LINE_W / 2.0 - 103 + 5) << "frame " << f;
  }
}
