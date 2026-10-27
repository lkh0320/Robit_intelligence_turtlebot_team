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
//                + 차선이 보이면 길 방향 맞추기 w += -heading_gain * lane angle (2026-10-15 001420_test: 차선은 길이 왼쪽으로
//                꺾인다고 (angle -0.5) 계속 보는데 경로 계획은 박스만 피해서 길 바깥을 향한 채 다음 박스에 붙음)
//                |free_offset| 이 크면 (제자리 꺾기 후보) v 를 줄임 (turn_slow_angle 에서 0)
//                wall_detection 이 stuck (박스에 바짝 붙어 전부 막힘, confidence <= stuck_confidence) 이면
//                stuck_reverse_time 동안 후진 -> stuck_turn_time 동안 제자리에서 고른 쪽으로 -> 정지
//                wall_detection planner 모드 (기본): free_offset = 로봇 폭으로 지나갈 경로 위 목표점 방향 [rad].
//                confidence 가 low_confidence 아래 (지나갈 경로 없음 -> 가장 멀리 가는 경로) 면 v x slow_ratio
//                옆 PSD 가 pass_low 아래로 pass_low_time 넘게 있다가 pass_high 위로 pass_high_time 넘게 있으면 1개 통과
//                (녹화: 표지판 받침대가 왼쪽 PSD 를 0.8초 막음 -> AVOID 전이라 안 셈. 박스를 지나며 0.1~0.2초 튀는 값 무시)
//                AVOID 를 시작할 때 이미 막혀 있던 쪽은 한 번 열린 뒤부터 센다
//                같은 쪽 통과가 same_side_gap 안에 또 나오면 안 셈 (2026-10-14 실주행: 박스를 긁으며 PSD 가 흔들려
//                왼-오-왼 을 왼-왼-오 로 세고 3개로 끝내 마지막 박스에 직진)
//     3. RETURN: boxes 개 통과 (또는 하나 이상 통과한 뒤 no_box_time 동안 박스가 안 보임) -> 차선을 return_ok_count 틱
//                연속 잡을 때까지 빈 공간 조향 계속, 잡으면 끝 (return_max_time 지나면 그냥 끝)
//                단 가까운 박스(enter_bottom)가 보이는 동안은 RETURN 으로 안 가고, RETURN 중에 보이면 AVOID 로 돌아감
//     안전: AVOID 가 avoid_max_time 을 넘으면 끝. wall_obstacle 이 wall_timeout 넘게 안 오면 AVOID / RETURN 중 정지.
//           정면 PSD 비상 정지는 lane_follower 가 그대로 한다. 주행 모드에서 빠지면 (수동 / STOP) 전부 잊음
//   line 방식 (method = line, 2026-10-26 사용자: 빈 공간 조향이 좌우로 흔들려 바꿈):
//     AVOID 동안 조향은 평소 차선 추종 제어기 그대로, 목표만 차선 중심에서 옆으로 옮긴다 (lane_follower 가 offset - bias 로 추종).
//     가장 가까운 박스가 바닥 지도에서 흰 선(오른쪽) / 노란 선(왼쪽) 중 어디에 붙었나로 박스 쪽을 정하고
//     (side_by_line, wall_detection 의 side. false 면 원본 화면 가로 위치 x_ratio 부호), 반대쪽이 side_votes 번 연속 보여야 바꾼다
//     (화면 맨 아래에 걸린 지금 지나가는 박스는 뺌) -> 반대쪽 선으로 line_shift 만큼 (shift_rate 로 천천히) 붙어 간다.
//     옆 PSD 는 통과 개수 세기에만 쓴다.
//     박스 왼쪽 -> 오른쪽(흰 선) 으로 bias -line_shift, 박스 오른쪽 -> 왼쪽(노란 선) 으로 +line_shift.
//     선을 밟으면 실격이라 line_shift 는 로봇이 선 안에 남는 값으로 (1 = 차선 중심이 선 위).
//     차선을 놓치면 천천히, wall_detection 경로 방향으로 (line_lost_steer, 끄면 직진).
//     RETURN 은 bias 를 0 으로 되돌리며 차선 추종.
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
  bool line_mode = true;               // true: 차선 추종 + 옆으로 옮기기 (method line), false: 빈 공간 조향 (method planner)
  double line_shift = 0.35;            // 박스 반대쪽으로 옮기는 차선 중심 [반 차선 폭 = 1]
  double shift_rate = 1.5;             // [1/s] bias 바뀌는 속도
  int side_votes = 3;                  // 박스 쪽을 정하는 연속 같은 판단 수 (wall_obstacle 10Hz)
  bool side_by_line = true;            // true: wall_obstacle side (바닥 지도에서 박스가 붙은 선) 로, 모르면 판단 안 함
                                       // false: 원본 화면 가로 위치 x_ratio (아래 side_dead / side_angle_gain)
  double side_dead = 0.1;              // |x_ratio| 이 이보다 작으면 (가운데) 판단 안 함
  double line_lost_ratio = 0.5;        // 차선을 놓치면 linear 에 곱함
  bool line_lost_steer = true;         // 차선을 놓치면 직진 대신 wall_detection 경로 방향으로 (w = -kp * free_offset)
  double side_bottom = 0.1;            // 박스 아래 끝이 이 이상일 때만 쪽을 판단 (너무 먼 박스 빼기)
  double side_angle_gain = 1.8;        // 박스 쪽 판단 전에 x_ratio -= 이 값 x 차선 방향 [rad] (커브에서 길이 휜 만큼 박스가 옆으로 보임)
  double side_bottom_max = 0.85;       // 이보다 아래(화면 맨 아래에 걸림 = 지금 옆을 지나는 박스)는 판단에서 뺌
  double lane_kp = 1.2;                // 공사 구간 차선 추종 kp (옆으로 옮기는 반응, 평소 kp 보다 크게)
  double lane_ka = 0.6;                // 공사 구간 차선 추종 ka (옆으로 옮길 때 너무 틀어지지 않게)
  double sign_min_confidence = 0.6;
  double sign_min_area = 0.005;        // bbox 면적 / 이미지 면적 (녹화: 처음 볼 때 0.009)
  int sign_confirm_count = 2;
  int sign_confirm_window = 6;
  double sign_instant_confidence = 0.85;
  double arm_max_time = 15.0;          // [s] 표지판 뒤 이만큼 박스가 안 오면 잊음
  double enter_bottom = 0.3;           // 박스 아래 끝 (화면 높이 비율, 1 = 맨 아래) 이 이 이상이면 가까움
  int enter_count = 2;                 // 연속 wall_obstacle 수
  double wall_timeout = 0.5;           // [s]
  double linear = 0.10;                // [m/s] (10-14 0.06 = 녹화 수동 주행 속도 -> 10-15 사용자 "더 빠르게")
  double kp = 1.0;                     // [rad/s] w = -kp * free_offset (offset 1 = 화면 끝 방향, 약 40°)
  double max_angular = 0.8;            // [rad/s]
  double heading_gain = 0.5;           // [rad/s per rad] 차선이 보이면 w += -heading_gain * lane angle (0 = 끔)
  double offset_alpha = 0.4;           // free_offset 새 값 비중 (1 = 필터 없음, wall_obstacle 10Hz)
  double turn_slow_angle = 1.0;        // [rad] |free_offset| 이 이만큼이면 v = 0 (제자리 꺾기)
  double low_confidence = 0.5;         // wall_obstacle confidence 가 이보다 낮으면 천천히
  double slow_ratio = 0.6;             // 그때 linear 에 곱함
  double stuck_confidence = 0.15;      // confidence 가 이 이하 = stuck (wall_detection planner.stuck_confidence)
  double stuck_reverse_speed = 0.05;   // [m/s]
  double stuck_reverse_time = 1.0;     // [s]
  double stuck_turn_time = 2.0;        // [s] 후진 뒤 제자리에서 고른 쪽으로 도는 시간 (그 뒤 정지)
  int boxes = 3;
  double pass_low = 0.25;              // [m] 옆 PSD 가 이보다 가까우면 박스 옆
  double pass_high = 0.6;              // [m] 이보다 멀면 비었음
  double pass_low_time = 0.2;          // [s]
  double pass_high_time = 0.5;         // [s]
  double same_side_gap = 3.0;          // [s] 같은 쪽 통과가 이보다 빨리 또 나오면 안 셈
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
    bool lane_bias = false;   // true 면 v, w 대신 차선 추종 (offset - bias, 속도는 v 이하)
    double bias = 0;
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
    votes_ = 0;
    vote_side_ = 0;
    box_side_ = 0;
  }

  // wall_detection 결과 (wall_obstacle)
  void onWall(
    bool detected, double bottom, double free_offset, double confidence, double now,
    const ConstructionParams & p, double x_ratio = 0.0, char line_side = 0)
  {
    // line 방식: 가장 가까운 박스 쪽. 화면 맨 아래에 걸린 박스(side_bottom_max 넘음 = 지금 옆을 지나는 것)는 빼고,
    //   지금과 다른 쪽이 side_votes 번 연속 보여야 바꾼다 (같은 쪽 박스가 이어지면 그대로)
    //   커브에서는 길이 휜 쪽으로 박스가 보이므로 차선 방향만큼 빼고 본다 (2026-10-27 시뮬: 왼쪽 코너 직후 오른쪽 박스가 화면 왼쪽)
    //   2026-10-27 005115_line: 그래도 왼쪽 커브 끝 오른쪽 박스를 왼쪽으로 봄 (아직 커브 전이라 차선 방향 ~0)
    //   -> side_by_line: 바닥 지도에서 박스가 흰 선(R) / 노란 선(L) 중 어디에 붙었나 (wall_detection)
    const double x = x_ratio - p.side_angle_gain * lane_angle_;
    const char s = p.side_by_line ? line_side : std::abs(x) >= p.side_dead ? (x < 0 ? 'L' : 'R') : 0;
    if ((phase_ == Phase::ARMED || phase_ == Phase::AVOID) && detected && bottom >= p.side_bottom &&
      bottom <= p.side_bottom_max && s != 0)
    {
      votes_ = s != box_side_ && s == vote_side_ ? votes_ + 1 : (s != box_side_ ? 1 : 0);
      vote_side_ = s;
      if (votes_ >= std::max(1, p.side_votes)) {
        box_side_ = s;
        votes_ = 0;
        if (p.line_mode) {
          set(phase_, now, std::string("박스 ") + (s == 'L' ? "왼쪽 -> 오른쪽 선으로" : "오른쪽 -> 왼쪽 선으로") +
            (p.side_by_line ? std::string(" (붙은 선") : " (x " + fmt(x_ratio) + " 보정 " + fmt(x)) +
            ", 아래 끝 " + fmt(bottom) + ")");
        }
      }
    }
    // 경로 방향을 걸러서 쓴다 (2026-10-26 215056_box_test: 영상마다 평균 0.21 씩 튀어 w 가 ±max 에서 초당 2번 넘게 뒤집힘)
    const bool fresh = have_wall_ && now - wall_t_ <= p.wall_timeout;
    const double a = std::clamp(p.offset_alpha, 0.0, 1.0);
    free_offset_ = fresh ? a * free_offset + (1.0 - a) * free_offset_ : free_offset;
    wall_t_ = now;
    have_wall_ = true;
    confidence_ = confidence;
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
        if (now - last_pass_t_[i] < p.same_side_gap) {
          set(Phase::AVOID, now, std::string(i == 0 ? "왼쪽" : "오른쪽") + " 통과 " +
            fmt(now - last_pass_t_[i], 1) + "s 만에 또 -> 같은 박스로 보고 안 셈");
          last_pass_t_[i] = now;
          continue;
        }
        last_pass_t_[i] = now;
        ++passes_;
        pass_sides_ += i == 0 ? 'L' : 'R';
        set(Phase::AVOID, now, std::string(i == 0 ? "왼쪽" : "오른쪽") + " 박스 통과 " +
          std::to_string(passes_) + "/" + std::to_string(p.boxes) + " (" + pass_sides_ + ")");
      }
    }
  }

  // 매 제어 틱. lane_ok: 차선 추종할 lane_info 가 있음, lane_angle: 그 차선 방향 (lane_info angle)
  Cmd update(bool lane_ok, double now, const ConstructionParams & p, double lane_angle = 0.0)
  {
    Cmd c;
    const double dt = last_update_ > 0 ? std::clamp(now - last_update_, 0.0, 0.3) : 0.0;
    last_update_ = now;
    lane_angle_ = lane_ok ? lane_angle : 0.0;
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
        if (passes_ >= p.boxes && !(wall_fresh && near_count_ >= p.enter_count)) {   // 가까운 박스가 보이면 아직
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
        if (wall_fresh && near_count_ >= p.enter_count) {
          set(Phase::AVOID, now, "차선 찾는 중 가까운 박스 (아래 끝 " + fmt(bottom_) + ") -> 다시 피하기");
          break;
        }
        ok_count_ = lane_ok ? ok_count_ + 1 : 0;
        if (ok_count_ >= p.return_ok_count && (!p.line_mode || std::abs(bias_) < 0.05)) {
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
    if (p.line_mode) {
      // 차선 추종 + 박스 반대쪽으로 bias (RETURN 은 0 으로)
      double target = 0;
      if (phase_ == Phase::AVOID && box_side_ != 0) {
        target = box_side_ == 'L' ? -p.line_shift : p.line_shift;
      }
      const double step = p.shift_rate * dt;
      bias_ += std::clamp(target - bias_, -step, step);
      c.v = p.linear;
      if (lane_ok) {
        c.lane_bias = true;
        c.bias = bias_;
      } else {
        // 차선을 놓치면 천천히, 경로 계획 방향으로 (2026-10-27 011237_side: 박스를 비켜 오른쪽으로 꺾은 뒤 선을 놓치고
        //   0.05m/s 직진 5초로 트랙 바깥 쪽으로 감. 그동안 경로 계획은 계속 왼쪽(-0.4 ~ -1.0) = 길 안쪽을 가리킴)
        c.v = p.linear * p.line_lost_ratio;
        if (p.line_lost_steer && wall_fresh) {
          c.w = std::clamp(-p.kp * free_offset_, -p.max_angular, p.max_angular);
        }
      }
      return c;
    }
    if (!wall_fresh) {   // 빈 공간을 모르면 정지 (wall_detection 꺼짐 / 카메라 끊김)
      if (!stale_logged_) {
        reason_ = "wall_obstacle 끊김 -> 정지";
        changed_ = true;
        stale_logged_ = true;
      }
      return c;
    }
    stale_logged_ = false;
    if (confidence_ <= p.stuck_confidence) {
      // 박스에 바짝 붙어 갈 경로가 없음: 후진 -> 제자리에서 고른 쪽으로 -> 정지
      stuck_t_ += dt;
      unstuck_t_ = 0;
      int step = 0;
      if (stuck_t_ <= p.stuck_reverse_time) {
        c.v = -p.stuck_reverse_speed;
      } else if (stuck_t_ <= p.stuck_reverse_time + p.stuck_turn_time) {
        c.w = std::clamp(-p.kp * free_offset_, -p.max_angular, p.max_angular);
        step = 1;
      } else {
        step = 2;
      }
      if (step != stuck_step_) {
        static const char * what[] = {"후진", "제자리에서 고른 쪽으로", "정지"};
        reason_ = std::string("박스에 붙어 갈 경로 없음 -> ") + what[step];
        changed_ = true;
        stuck_step_ = step;
      }
      return c;
    }
    if (stuck_t_ > 0) {   // 한동안 안 막히면 다음 막힘은 처음부터 (후진부터)
      unstuck_t_ += dt;
      if (unstuck_t_ > 1.0) {
        stuck_t_ = 0;
        stuck_step_ = -1;
      }
    }
    const double turn = std::min(1.0, std::abs(free_offset_) / std::max(1e-3, p.turn_slow_angle));
    c.v = (confidence_ < p.low_confidence ? p.linear * p.slow_ratio : p.linear) * (1.0 - turn);
    double w = -p.kp * free_offset_;
    if (lane_ok) {
      w -= p.heading_gain * lane_angle;
    }
    c.w = std::clamp(w, -p.max_angular, p.max_angular);
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
      case Phase::AVOID: return "AVOID " + std::to_string(passes_) + " (" + pass_sides_ + ")" +
               (box_side_ ? std::string(" next ") + box_side_ : std::string());
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
    last_pass_t_[0] = last_pass_t_[1] = -1e9;
    stuck_t_ = 0;
    stuck_step_ = -1;
    bias_ = 0;
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
    stuck_t_ = 0;
    stuck_step_ = -1;
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
  double bottom_ = 0, free_offset_ = 0, confidence_ = 1;
  double last_update_ = 0;
  double stuck_t_ = 0, unstuck_t_ = 0;   // 막힘 동작 시간 / 막힘이 풀린 시간 [s]
  int stuck_step_ = -1;                  // 로그용 (0 후진, 1 돌기, 2 정지)
  int near_count_ = 0;
  Side side_[2];               // 왼쪽, 오른쪽
  double last_pass_t_[2] = {-1e9, -1e9};
  int passes_ = 0;
  std::string pass_sides_;     // 통과한 순서 (예: "RLR")
  int ok_count_ = 0;
  bool stale_logged_ = false;
  // line 방식
  char vote_side_ = 0, box_side_ = 0;   // 'L' / 'R' (0 = 아직)
  int votes_ = 0;
  double bias_ = 0;            // 지금 차선 중심 옮김 [반 차선 폭]
  double lane_angle_ = 0;      // 마지막 차선 방향 (박스 쪽 판단 보정)
  std::string reason_;
  bool changed_ = false;
};

}  // namespace lane_follower
