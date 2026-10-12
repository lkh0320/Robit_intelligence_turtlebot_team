// 바퀴 엔코더 위치 -> 로봇 위치 (차동구동 추측 항법)
//
//   엔코더 프레임마다 update() 에 두 바퀴 위치 [tick] 를 넣으면
//   이전 값과의 차이로 바퀴가 굴러간 거리를 구해 x, y, yaw 를 누적한다.
//   - 첫 프레임은 기준값만 저장 (이동 0)
//   - 한 프레임에 max_jump_ticks 보다 많이 변한 바퀴는 버린다 (모터 재부팅으로 위치가 0 근처로 돌아간 경우)
//
//   왼쪽 거리 dl, 오른쪽 거리 dr, 바퀴 간격 L 일 때
//     ds = (dl + dr) / 2,  dyaw = (dr - dl) / L
//     중간 방향(yaw + dyaw/2)으로 ds 만큼 이동
#pragma once

#include <cmath>
#include <cstdint>
#include <cstdlib>

namespace stm
{

class WheelOdometry
{
public:
  WheelOdometry(double wheel_radius, double wheel_separation, double ticks_per_rev,
    int32_t max_jump_ticks)
  : radius_(wheel_radius), separation_(wheel_separation), ticks_per_rev_(ticks_per_rev),
    max_jump_(max_jump_ticks) {}

  // 새 위치 [tick] ([0] 왼쪽, [1] 오른쪽, 앞 = +). 버린 바퀴가 있으면 false
  bool update(const int32_t pos[2])
  {
    if (!has_last_) {
      last_[0] = pos[0];
      last_[1] = pos[1];
      has_last_ = true;
      return true;
    }
    bool ok = true;
    double d[2] = {0.0, 0.0};
    for (int k = 0; k < 2; ++k) {
      // int64 로 빼서 int32 넘침 방지
      const int64_t diff = static_cast<int64_t>(pos[k]) - last_[k];
      if (std::llabs(diff) <= max_jump_) {
        d[k] = static_cast<double>(diff) / ticks_per_rev_ * 2.0 * M_PI * radius_;
      } else {
        ok = false;   // 재부팅 등: 이번 차이는 버리고 새 값을 기준으로 삼는다
      }
      last_[k] = pos[k];
    }
    distance_[0] += d[0];
    distance_[1] += d[1];
    const double ds = (d[0] + d[1]) / 2.0;
    const double dyaw = (d[1] - d[0]) / separation_;
    x_ += ds * std::cos(yaw_ + dyaw / 2.0);
    y_ += ds * std::sin(yaw_ + dyaw / 2.0);
    yaw_ = std::atan2(std::sin(yaw_ + dyaw), std::cos(yaw_ + dyaw));   // -pi ~ pi
    return ok;
  }

  // 위치를 원점으로 (엔코더 기준값은 유지)
  void reset()
  {
    x_ = y_ = yaw_ = 0.0;
    distance_[0] = distance_[1] = 0.0;
  }

  double x() const {return x_;}
  double y() const {return y_;}
  double yaw() const {return yaw_;}
  // 시작부터 각 바퀴가 굴러간 거리 [m] (앞 = +)
  double distance(int k) const {return distance_[k];}

private:
  double radius_, separation_, ticks_per_rev_;
  int64_t max_jump_;
  bool has_last_ = false;
  int64_t last_[2] = {0, 0};
  double x_ = 0.0, y_ = 0.0, yaw_ = 0.0;
  double distance_[2] = {0.0, 0.0};
};

}  // namespace stm
