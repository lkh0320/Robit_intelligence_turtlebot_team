// sliding(lane_detection) 주행에서 공사 구간 통과: 갈색 박스 3개를 빈 공간 쪽으로 비켜 간다 (ROS 없이 테스트 가능)
//
//   트랙 (2026-10-13 사용자): 맵 왼쪽 세로 구간을 위 -> 아래로. 박스 3개 좌우는 매번 랜덤, 구간은 커브 바로 뒤.
//   방법 (~/construction_test_20261013/NOTES.md): 미터 보정(카메라 모델 / odom)을 못 믿어서 픽셀 + PSD 만 쓴다
//     - 조향: wall_detection 의 free_offset (원본 카메라에서 박스 / 선 없이 가장 멀리 비어 있는 방향).
//       수동 주행 녹화에서 사람 조향과 뚜렷한 조향 39번 중 38번 같은 방향
//     - 박스 통과 세기: 옆 PSD. 평소 0.80(최대), 박스 옆 0.10~0.13. 녹화 RLR 이 그대로 보임
//   동작
//     1. ARMED:  CONSTRUCTION 표지판 확인 (sign_confirm_window 프레임 중 sign_confirm_count 번, 또는
//                sign_instant_confidence 이상 한 번). 차선 추종은 그대로. arm_max_time 안에 박스가 안 오면 잊음
//                (박스만으로는 진입 안 함: test_021825 출발선 오인식, 녹화에서 앞 도로에서도 주차장 너머 박스가 보임)
//     2. AVOID:  박스 아래 끝이 enter_bottom 아래로 (가까이) enter_count 번 연속 보이거나, 박스가 보이는데 차선을
//                놓치면 시작. v = linear, w = -kp * free_offset (± max_angular)
//                옆 PSD 가 pass_low 아래로 pass_low_time 넘게 있다가 pass_high 위로 pass_high_time 넘게 있으면 1개 통과
//                (녹화: 표지판 받침대가 왼쪽 PSD 를 0.8초 막음 -> AVOID 전이라 안 셈. 박스를 지나며 0.1~0.2초 튀는 값 무시)
//                AVOID 를 시작할 때 이미 막혀 있던 쪽은 한 번 열린 뒤부터 센다
//     3. RETURN: boxes 개 통과 (또는 하나 이상 통과한 뒤 no_box_time 동안 박스가 안 보임) -> 차선을 return_ok_count 틱
//                연속 잡을 때까지 빈 공간 조향 계속, 잡으면 끝 (return_max_time 지나면 그냥 끝)
//     안전: AVOID 가 avoid_max_time 을 넘으면 끝. wall_obstacle 이 wall_timeout 넘게 안 오면 AVOID / RETURN 중 정지.
//           정면 PSD 비상 정지는 lane_follower 가 그대로 한다. 주행 모드에서 빠지면 (수동 / STOP) 전부 잊음
//   끝난 뒤 rearm_time 동안 공사 표지판 무시
//   시간은 초 단위 now 를 받는다 (lane_follower 가 노드 시계로)
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <string>

namespace lane_follower
{

constexpr int ZONE_SIGN_CONSTRUCTION = 3;   // interfaces/Sign CONSTRUCTION

struct ConstructionParams
{
  bool enabled = true;
  double sign_min_confidence = 0.6;
  double sign_min_area = 0.005;        // bbox 면적 / 이미지 면적 (녹화: 처음 볼 때 0.009)
  int sign_confirm_count = 2;
  int sign_confirm_window = 6;
  double sign_instant_confidence = 0.85;
  double arm_max_time = 15.0;          // [s] 표지판 뒤 이만큼 박스가 안 오면 잊음
  double enter_bottom = 0.3;           // 박스 아래 끝 (화면 높이 비율, 1 = 맨 아래) 이 이 이상이면 가까움
  int enter_count = 2;                 // 연속 wall_obstacle 수
  double wall_timeout = 0.5;           // [s]
  double linear = 0.06;                // [m/s] 녹화 수동 주행 속도
  double kp = 1.0;                     // [rad/s] w = -kp * free_offset (offset 1 = 화면 끝 방향, 약 40°)
  double max_angular = 0.6;            // [rad/s] (녹화 수동 조향 키 0.6)
  int boxes = 3;
  double pass_low = 0.25;              // [m] 옆 PSD 가 이보다 가까우면 박스 옆
  double pass_high = 0.6;              // [m] 이보다 멀면 비었음
  double pass_low_time = 0.2;          // [s]
  double pass_high_time = 0.5;         // [s]
  double no_box_time = 3.0;            // [s] 하나 이상 통과한 뒤 박스가 이만큼 안 보이면 RETURN
  int return_ok_count = 3;             // 차선 잡은 틱 연속 수
  double return_max_time = 3.0;        // [s]
  double avoid_max_time = 30.0;        // [s]
  double rearm_time = 5.0;             // [s]
};

class ConstructionZone
{
public:
  enum class Phase {IDLE, ARMED, AVOID, RETURN};

  struct Cmd
  {
    bool active = false;   // true 면 lane_follower 는 이 명령으로 (차선 추종 대신)
    double v = 0, w = 0;
  };

  void onSign(int type, double confidence, double area, double now, const ConstructionParams & p)
  {
    const bool ok = type == ZONE_SIGN_CONSTRUCTION && confidence >= p.sign_min_confidence &&
      area >= p.sign_min_area;
    window_.push_back(ok);
    while (static_cast<int>(window_.size()) > std::max(1, p.sign_confirm_window)) {
      window_.pop_front();
    }
    if (!p.enabled || !ok || phase_ != Phase::IDLE || now - done_t_ < p.rearm_time) {
      return;
    }
    const int seen = static_cast<int>(std::count(window_.begin(), window_.end(), true));
    if (seen < p.sign_confirm_count && confidence < p.sign_instant_confidence) {
      return;
    }
    set(Phase::ARMED, now, "공사 표지판 확인 (" + fmt(confidence) + ")");
    near_count_ = 0;
  }

  // wall_detection 결과 (wall_obstacle)
  void onWall(bool detected, double bottom, double free_offset, double now, const ConstructionParams & p)
  {
    wall_t_ = now;
    have_wall_ = true;
    free_offset_ = free_offset;
    box_ = detected;
    if (detected) {
      last_box_t_ = now;
    }
    near_count_ = detected && bottom >= p.enter_bottom ? near_count_ + 1 : 0;
    bottom_ = detected ? bottom : 0;
  }

  void onPsd(double left, double right, double now, const ConstructionParams & p)
  {
    if (phase_ != Phase::AVOID) {
      return;
    }
    for (int i = 0; i < 2; ++i) {
      if (side_[i].update(i == 0 ? left : right, now, p)) {
        ++passes_;
        pass_sides_ += i == 0 ? 'L' : 'R';
        set(Phase::AVOID, now, std::string(i == 0 ? "왼쪽" : "오른쪽") + " 박스 통과 " +
          std::to_string(passes_) + "/" + std::to_string(p.boxes) + " (" + pass_sides_ + ")");
      }
    }
  }

  // 매 제어 틱. lane_ok: 차선 추종할 lane_info 가 있음
  Cmd update(bool lane_ok, double now, const ConstructionParams & p)
  {
    Cmd c;
    if (!p.enabled) {
      if (phase_ != Phase::IDLE) {
        finish(now, "끔");
      }
      return c;
    }
    const bool wall_fresh = have_wall_ && now - wall_t_ <= p.wall_timeout;
    switch (phase_) {
      case Phase::IDLE:
        return c;
      case Phase::ARMED:
        if (now - t_ > p.arm_max_time) {
          finish(now, fmt(p.arm_max_time, 0) + "s 동안 박스 없음 -> 표지판 잊음");
          return c;
        }
        if (wall_fresh && near_count_ >= p.enter_count) {
          startAvoid(now, "박스 가까움 (아래 끝 " + fmt(bottom_) + ")");
        } else if (wall_fresh && box_ && !lane_ok) {
          startAvoid(now, "박스 보이고 차선 놓침");
        } else {
          return c;
        }
        break;
      case Phase::AVOID:
        if (passes_ >= p.boxes) {
          set(Phase::RETURN, now, "박스 " + std::to_string(passes_) + "개 통과 (" + pass_sides_ + ") -> 차선 찾기");
          ok_count_ = 0;
        } else if (passes_ > 0 && now - last_box_t_ > p.no_box_time) {
          set(Phase::RETURN, now, "박스 " + std::to_string(passes_) + "개 통과 뒤 " + fmt(p.no_box_time, 1) +
            "s 박스 없음 -> 차선 찾기");
          ok_count_ = 0;
        } else if (now - t_ > p.avoid_max_time) {
          finish(now, fmt(p.avoid_max_time, 0) + "s 넘음 (통과 " + std::to_string(passes_) + "개) -> 차선 추종");
          return c;
        }
        break;
      case Phase::RETURN:
        ok_count_ = lane_ok ? ok_count_ + 1 : 0;
        if (ok_count_ >= p.return_ok_count) {
          finish(now, "차선 잡음 -> 차선 추종");
          return c;
        }
        if (now - t_ > p.return_max_time) {
          finish(now, fmt(p.return_max_time, 1) + "s 지나도 차선 없음 -> 차선 추종");
          return c;
        }
        break;
    }
    c.active = true;
    if (!wall_fresh) {   // 빈 공간을 모르면 정지 (wall_detection 꺼짐 / 카메라 끊김)
      if (!stale_logged_) {
        reason_ = "wall_obstacle 끊김 -> 정지";
        changed_ = true;
        stale_logged_ = true;
      }
      return c;
    }
    stale_logged_ = false;
    c.v = p.linear;
    c.w = std::clamp(-p.kp * free_offset_, -p.max_angular, p.max_angular);
    return c;
  }

  // 주행 모드에서 빠짐 (수동 / STOP): 손으로 옮기는 동안의 상태를 남기지 않는다
  void cancel()
  {
    if (phase_ != Phase::IDLE) {
      phase_ = Phase::IDLE;
      reason_ = "주행 모드 아님 -> 취소";
      changed_ = true;
    }
  }

  Phase phase() const {return phase_;}
  bool active() const {return phase_ == Phase::AVOID || phase_ == Phase::RETURN;}
  int passes() const {return passes_;}
  const std::string & lastReason() const {return reason_;}
  bool takeChanged()
  {
    const bool c = changed_;
    changed_ = false;
    return c;
  }
  std::string describe() const
  {
    switch (phase_) {
      case Phase::IDLE: return "IDLE";
      case Phase::ARMED: return "ARMED";
      case Phase::AVOID: return "AVOID " + std::to_string(passes_) + " (" + pass_sides_ + ")";
      case Phase::RETURN: return "RETURN " + std::to_string(passes_) + " (" + pass_sides_ + ")";
    }
    return "?";
  }

private:
  // 옆 PSD 하나: 가까움(pass_low 아래)이 pass_low_time 넘게 -> 박스 옆, 그 뒤 멂(pass_high 위)이 pass_high_time 넘게 -> 통과
  //   그 사이 값(pass_low ~ pass_high)은 상태를 안 바꾼다 (박스 옆에서 0.25~0.5 로 흔들림)
  struct Side
  {
    bool ready = false;    // 한 번 멀어진 뒤부터 센다 (AVOID 시작할 때 이미 막혀 있던 것 무시)
    bool beside = false;
    double low_since = -1, high_since = -1;

    bool update(double d, double now, const ConstructionParams & p)
    {
      if (d < p.pass_low) {
        high_since = -1;
        if (low_since < 0) {
          low_since = now;
        }
        if (ready && now - low_since >= p.pass_low_time) {
          beside = true;
        }
      } else if (d > p.pass_high) {
        low_since = -1;
        ready = true;
        if (high_since < 0) {
          high_since = now;
        }
        if (beside && now - high_since >= p.pass_high_time) {
          beside = false;
          return true;
        }
      } else {
        low_since = -1;
        high_since = -1;
      }
      return false;
    }
  };

  void startAvoid(double now, const std::string & why)
  {
    set(Phase::AVOID, now, why + " -> 빈 공간 조향");
    passes_ = 0;
    pass_sides_.clear();
    side_[0] = Side();
    side_[1] = Side();
  }

  void set(Phase ph, double now, const std::string & why)
  {
    if (ph != phase_) {
      t_ = now;
    }
    phase_ = ph;
    reason_ = why;
    changed_ = true;
  }

  void finish(double now, const std::string & why)
  {
    phase_ = Phase::IDLE;
    done_t_ = now;
    reason_ = why;
    changed_ = true;
  }

  static std::string fmt(double v, int digits = 2)
  {
    char b[32];
    std::snprintf(b, sizeof(b), "%.*f", digits, v);
    return b;
  }

  Phase phase_ = Phase::IDLE;
  std::deque<bool> window_;
  double t_ = 0;               // 지금 단계 시작 시각
  double done_t_ = -1e9;       // 마지막으로 끝난 시각
  bool have_wall_ = false;
  double wall_t_ = 0, last_box_t_ = 0;
  bool box_ = false;
  double bottom_ = 0, free_offset_ = 0;
  int near_count_ = 0;
  Side side_[2];               // 왼쪽, 오른쪽
  int passes_ = 0;
  std::string pass_sides_;     // 통과한 순서 (예: "RLR")
  int ok_count_ = 0;
  bool stale_logged_ = false;
  std::string reason_;
  bool changed_ = false;
};

}  // namespace lane_follower
