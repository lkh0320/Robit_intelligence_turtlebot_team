// lane_info(offset, angle) -> cmd_vel(v, w) 제어 법칙 (ROS 없이 테스트 가능)
//
//   w = -(v * 2 sin(angle) / pp_lookahead + kp * offset + kd * d(offset)/dt + ka * angle)
//     offset/angle 은 + 가 "목표가 오른쪽" 이고, ROS 각속도는 + 가 왼쪽(반시계) 이라 부호를 뒤집는다
//     첫 항은 pure pursuit: 바퀴 축에서 pp_lookahead [m] 떨어진 목표점(방향 angle)을 지나는 원호의 곡률 x v
//     (path_planner 용. 0 이면 끔, lane_detection 은 offset/angle 이 다른 뜻이라 kp/ka 만 쓴다)
//   v = linear_speed 에서 커브(|angle|)가 클수록, confidence 가 낮을수록 줄인다 (min_linear_speed 까지)
#pragma once

#include <algorithm>
#include <cmath>

namespace lane_follower
{

struct Gains
{
  double linear_speed = 0.10;      // 직선 속도 [m/s]
  double min_linear_speed = 0.04;  // 커브/저신뢰에서 최소 속도 [m/s]
  double kp = 0.8;                 // offset -> 각속도 [rad/s]
  double kd = 0.05;                // offset 변화율 -> 각속도
  double ka = 1.0;                 // angle [rad] -> 각속도
  double pp_lookahead = 0.0;       // pure pursuit 목표점 거리 [m] (0 = 끔)
  double max_angular = 1.5;        // [rad/s]
  double slow_down_angle = 0.5;    // |angle| 이 이 값 [rad] 이면 min_linear_speed 까지 감속
  double low_confidence = 0.5;     // confidence 가 이보다 낮으면 min_linear_speed
  double d_filter_alpha = 0.3;     // 미분항 저역 필터 (새 값 비중)
};

struct Command
{
  double v = 0, w = 0;
};

class Controller
{
public:
  // dt: 직전 호출 이후 시간 [s] (0 이하면 미분항 생략)
  Command step(double offset, double angle, double confidence, double dt, const Gains & g)
  {
    double d_offset = 0;
    if (has_prev_ && dt > 1e-3) {
      const double raw = (offset - prev_offset_) / dt;
      d_filt_ = g.d_filter_alpha * raw + (1 - g.d_filter_alpha) * d_filt_;
      d_offset = d_filt_;
    }
    prev_offset_ = offset;
    has_prev_ = true;

    Command c;
    const double turn = g.slow_down_angle > 0 ?
      std::min(1.0, std::abs(angle) / g.slow_down_angle) : 0.0;
    c.v = g.linear_speed - (g.linear_speed - g.min_linear_speed) * turn;
    if (confidence < g.low_confidence) {
      c.v = std::min(c.v, g.min_linear_speed);
    }
    c.v = std::max(0.0, c.v);
    const double pp = g.pp_lookahead > 0 ? c.v * 2 * std::sin(angle) / g.pp_lookahead : 0.0;
    c.w = std::clamp(-(pp + g.kp * offset + g.kd * d_offset + g.ka * angle), -g.max_angular,
        g.max_angular);
    return c;
  }

  void reset() {has_prev_ = false; d_filt_ = 0;}

private:
  bool has_prev_ = false;
  double prev_offset_ = 0, d_filt_ = 0;
};

}  // namespace lane_follower
