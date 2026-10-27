// sliding(lane_detection) 주행에서 교차로(T자) 통과: 표지판 방향으로 돈다 (ROS 없이 테스트 가능)
//
//   문제 (2026-10-13 test_024032): 교차로에 들어서면 정면을 가로지르는 노란 / 흰 선만 보여서
//     lane_detection 이 차선을 못 찾고 (detected = false) lane_follower 가 "정지: 차선 못 찾음" 으로 멈춘다.
//   동작
//     1. ARMED:    LEFT / RIGHT 표지판을 확인하면 그 방향을 기억한다 (나중에 본 표지판이 이긴다).
//                  sign_confirm_window 프레임 중 같은 방향이 sign_confirm_count 번, 또는 confidence 가
//                  sign_instant_confidence 이상이면 한 번만 봐도 (표지판 인식이 5Hz 정도라 빨리 지나가면 1~2번)
//                  기억한 뒤 arm_max_dist 를 달려도 교차로를 못 만나면 잊는다
//     2. APPROACH: 기억한 방향이 있는데 차선을 못 찾고, 최근 cross_recent_time 안에 T자 가로 선(lane_detection
//                  cross_line)을 봤으면 (교차로 진입) approach_time 동안 approach_linear 로 직진
//                  (test_052007: 진입 곡선을 차선 추종으로 이미 58° 돈 뒤 차선을 한 프레임 놓친 걸 교차로로 보고
//                  66° 더 돌아 섬으로 감. 그때는 가로 선이 안 보였다 -> require_cross_line)
//                  (2026-10-13 사용자 요청: 교차로 안으로 조금 들어간 뒤 돌게. 0 이면 바로 TURN)
//     1b. 다가가며 정렬 (approach_align): ARMED 인데 가로 선이 보이면 차선 추종 조향에 -kp * 기울기를 더하고
//                  속도를 approach_slow_linear 로 줄여, 멈추지 않고 가로 선에 수평으로 다가간다 (crossAssist).
//                  (2026-10-13 사용자: "교차로 들어가면서 자연스럽게 정렬". 녹화에서 T자 앞 기울기 6~14°)
//     2b. ALIGN:   (align_enabled) 가로 선 기울기(lane_detection cross_line)가 수평이 될 때까지 제자리에서 돈다
//                  (2026-10-13 사용자 요청: 0.5초 직진 대신 표지판 있는 가로 선 보고 수평 맞춘 뒤 한쪽 바퀴로 회전).
//                  |기울기| <= align_tol 이 align_ok_count 프레임 연속이거나, 가로 선이 align_lost_time 넘게 안 보이거나,
//                  align_max_time 이 지나면 APPROACH (approach_time 0 이면 바로 TURN)
//     3. TURN:     표지판 방향으로 돈다. turn_pivot 이면 안쪽 바퀴는 멈추고 바깥 바퀴만 돌려 (안쪽 바퀴 중심으로)
//                  조금씩 나아가며 돌고 (2026-10-13 사용자 요청: 제자리 회전보다 자연스럽게), 아니면 turn_linear 로.
//                  turn_min_yaw 까지는 turn_angular, 그 뒤 차선을 찾는 동안은
//                  turn_search_angular 로 천천히. turn_min_yaw 이상 돈 뒤 차선을 exit_ok_count 프레임 연속 제대로
//                  (confidence, |angle| 작음) 잡으면 끝 -> sliding 주행으로 돌아감.
//                  제대로 = 두 선이 다 보이거나 (lane_detection 이 차선 폭까지 확인), 한 선만 보이면 도는 쪽 선
//                  (왼쪽 = 노란, 오른쪽 = 흰, exit_turn_side_only). test_041619: turn_min_yaw 75° 였을 때
//                  왼쪽 44° / 오른쪽 38° 에서 보인 올바른 차선을 무시하고 더 돌아 엉뚱한 선을 잡음.
//                  test_030522: 왼쪽으로 돌며 흰 선만 보고 56° 에서 끝내 이탈 -> 한 선은 도는 쪽 선만 믿는다
//                  turn_max_yaw 를 넘게 돌아도 차선이 없으면 포기 (정지, 원래대로)
//                  직진 / 도는 중에 반대 방향 표지판이 확정되면 (test_030522: 멀티탭 RIGHT 오인식 뒤 진짜 LEFT 가
//                  차선 놓친 직후 확정) turn_switch_max_yaw 안이면 그 방향으로 바꿔 처음부터 돈다
//     3b. AFTER:   (오른쪽 등 LOOP 로 안 가는 회전, after_enabled) 돈 뒤 after_dist 동안 차선을 놓치면 같은 쪽으로
//                  더 돌며 다시 찾는다 (최대 after_max_yaw, after_max_retries 번). exit_lead_angle 로 일찍 끝내서
//                  나머지를 차선 추종이 도는데, 그 사이 차선을 놓치면 멈추던 것 (07:37 오른쪽 5번 중 1번, 30° 에서 끝낸 뒤)
//     4. LOOP:     (out_enabled, 오른쪽은 아래 4b) 교차로 고리를 시계 방향으로 돌아 출구를 찾는다.
//                  트랙 맵: 왼쪽 길은 고리 위쪽을 돌아 오른쪽 아래로 내려오고, 출구는 진행 방향 왼쪽(동쪽)으로 갈라진다.
//                  그런데 출구에서 오른쪽에 섬의 노란 선이 곧게 이어져 lane_detection 이 그걸 왼쪽 선으로 잡고
//                  고리를 계속 돈다 (사용자: "왼쪽 표지판 보고 들어가서는 교차로 나가지지 않아").
//                  -> 돈 뒤 차선 추종으로 고리를 따라가면서 회전량을 재다가, 오른쪽으로 out_trigger_yaw 만큼
//                  꺾였으면 (위쪽을 돌아 남쪽을 보면), 차선을 놓치면, 또는 노란 선만 로봇 오른쪽에 보이면 (섬 끝,
//                  out_island_offset) 출구에서 다시 왼쪽으로 돈다 (APPROACH -> TURN, out_* 값).
//                  test_044608: 고리 위쪽을 따라 오른쪽으로 60~70° 꺾인 곳 (0.45m) 에서 노란 선이 왼쪽으로 휘어
//                  화면 밖으로 나가고, 섬 끝 노란 선을 왼쪽 선으로 잡아 (offset +1) 섬 쪽으로 꺾음.
//                  out_max_dist 를 가도 못 찾으면 잊는다. 이 동안 표지판은 무시 (섬 안 표지판이 또 보임)
//     5. EXIT:     (out_follow) 출구에서 제자리로 돌지 않고 차선을 계속 따라가되, 섬 쪽으로 치우친 차선
//                  (offset >= out_guard_max_offset, 섬 끝 노란 선) 이나 차선 없음이면 왼쪽으로 완만한 호
//                  (out_arc_linear / out_arc_angular). out_guard_dist 를 가면 끝.
//                  test_054541: 출구(고리 60°)에서 로봇이 이미 S자 노란 선의 왼쪽 굽이에 붙어 있어 제자리로
//                  34° 넘게 돌면 노란 선을 넘어 트랙 바깥만 보였다. 그 순간 흰 ∩ 선을 오른쪽 선으로 한 차선은
//                  거의 정면 (사용자: "거기서 왼쪽으로 가야 해" = S자 길을 따라 왼쪽으로)
//                  오른쪽 길은 출구에서 오른쪽 흰 선이 출구로 이어져 그냥 나간다 (사용자 확인)
//     4b. 오른쪽 길도 LOOP / EXIT (out_right_enabled, 2026-10-27 사용자 "오른쪽 표지판 보고 들어가서 탈출하는 것도
//                  왼쪽처럼"): 182547_run 2회차에서 오른쪽으로 돈 뒤 고리를 따라 왼쪽으로 62~66° 돈 곳(출구)에서
//                  흰 선 없는 노란 S자만 보여 lane angle +1.1 / offset ±0.6 뒤집힘 -> 직진하다 차선 놓쳐 정지.
//                  성공한 1·3회차도 같은 곳에서 오른쪽으로 꺾여 나감 -> 왼쪽의 거울: 고리 쪽(왼쪽)으로 out_trigger_yaw
//                  꺾이면 EXIT, 섬 쪽(왼쪽) 차선은 무시하고 오른쪽 호. 섬 끝 노란 선 조건은 왼쪽 길만.
//                  LOOP 처음 out_min_dist 안에서 차선을 놓치면 AFTER 처럼 같은 쪽으로 더 돈 뒤 LOOP 로 돌아감 (양쪽)
//   끝난 뒤 rearm_dist 동안은 표지판을 무시한다 (돌고 나서 같은 표지판을 다시 보는 경우)
//
//   회전량 / 이동 거리는 onMove 로 받는다 (lane_follower 가 dxl_state 바퀴 속도를 적분해서 넘김)
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <string>

namespace lane_follower
{

struct TurnParams
{
  bool enabled = true;
  double sign_min_confidence = 0.7;
  double sign_min_area = 0.01;         // bbox 면적 / 이미지 면적
  int sign_confirm_count = 2;          // 최근 sign_confirm_window 프레임 중 같은 방향이 이만큼
  int sign_confirm_window = 6;
  double sign_instant_confidence = 0.9;   // 이 이상이면 한 번만 봐도 확정 (1 이상 = 끔)
  double arm_max_dist = 1.5;           // [m] 표지판을 본 뒤 이만큼 가도 교차로가 없으면 잊음
  bool require_cross_line = true;      // 차선을 놓쳐도 최근에 T자 가로 선을 봤을 때만 교차로로 봄
  // 가로 선이 없어도(헤어핀처럼 가로지르는 선이 아예 없는 급커브), 표지판을 본 뒤 차선을 이 시간
  // 넘게 계속 놓치고 있으면 그냥 교차로로 보고 돈다. 너무 짧으면 진입 곡선 중 한 프레임만 놓친 것도
  // (test_052007) 교차로로 오인하니 어느 정도 여유를 둔다 (require_cross_line 이 false 면 안 씀)
  double require_cross_line_grace = 0.4;   // [s]
  bool approach_align = true;          // ARMED 중 가로 선이 보이면 다가가며 수평 맞춤 (crossAssist)
  double approach_align_kp = 2.0;      // [rad/s per rad] 차선 추종 조향에 더함
  double approach_align_max = 0.4;     // [rad/s]
  double approach_align_fresh = 0.3;   // [s] 이 안에 본 가로 선만 씀
  double approach_slow_linear = 0.06;  // [m/s] 가로 선이 보이면 이 속도 이하로 (0 = 그대로)
  bool align_enabled = true;           // 돌기 전에 가로 선이 수평이 되게 정렬
  double align_kp = 1.5;               // [rad/s per rad] w = -align_kp * 기울기
  double align_max_angular = 0.3;      // [rad/s]
  double align_min_angular = 0.12;     // [rad/s] 이보다 느리면 바퀴가 안 돌 수 있어서
  double align_tol = 0.05;             // [rad] (약 3°) 이 안이면 수평
  int align_ok_count = 2;              // 수평 프레임 연속 수
  double align_max_time = 3.0;         // [s]
  double align_lost_time = 0.5;        // [s] 가로 선이 이만큼 안 보이면 정렬 포기하고 돎
  double cross_recent_time = 1.0;      // [s] 이 안에 본 가로 선만 "최근"
  double approach_time = 0.5;          // [s] 차선 놓친 뒤 돌기 전에 직진하는 시간 (0 = 바로 돎)
  double approach_linear = 0.10;       // [m/s]
  bool turn_pivot = false;             // true: 안쪽 바퀴 0, 바깥 바퀴만 (v = |w| * wheel_separation / 2)
  double wheel_separation = 0.190;     // [m] turn_pivot 용 (stm_bridge 와 같게)
  double turn_linear = 0.0;            // [m/s] turn_pivot 이 아닐 때 도는 동안 전진 속도 (0 = 제자리)
  double turn_angular = 0.5;           // [rad/s] turn_min_yaw 까지
  double turn_search_angular = 0.3;    // [rad/s] turn_min_yaw 뒤 차선을 찾는 동안 (천천히)
  double turn_min_yaw = 0.35;          // [rad] 이만큼은 돌아야 끝낼 수 있음 (약 20°, 놓친 차선 / 가로 선 무시)
  double turn_switch_max_yaw = 0.6;    // [rad] 이만큼 돌기 전에 반대 표지판이 확정되면 방향을 바꿈
  double turn_max_yaw = 2.2;           // [rad] 이만큼 돌아도 차선이 없으면 포기 (약 126°)
  int exit_ok_count = 3;
  double exit_min_confidence = 0.5;
  double exit_max_angle = 0.5;         // [rad] 차선 방향이 이 안이면 그 차선에 들어선 것
  double exit_max_offset = 0.9;        // |offset| 이 이보다 크면 (로봇이 차선 밖 / 선이 반대쪽) 안 끝냄
  // 도는 쪽으로 아직 이만큼 [rad] 남았어도 끝냄 (나머지는 차선 추종이 앞으로 가며 돎). 지나친 쪽은 exit_max_angle
  double exit_lead_angle = 0.5;
  bool exit_turn_side_only = true;     // 한 선만 보이면 도는 쪽 선(왼쪽 = 노란, 오른쪽 = 흰)일 때만 끝냄
  double rearm_dist = 0.5;             // [m]
  // 왼쪽으로 돈 뒤 고리 출구 (LOOP)
  bool after_enabled = true;           // 돈 뒤 after_dist 안에서 차선 놓치면 같은 쪽으로 더 돎
  double after_dist = 0.4;             // [m]
  double after_max_yaw = 1.0;          // [rad] 더 도는 최대 각도 (약 57°)
  int after_max_retries = 2;
  bool out_enabled = true;
  bool out_right_enabled = true;       // 오른쪽으로 돈 뒤에도 LOOP / EXIT (false = 예전처럼 AFTER 만)
  double out_trigger_yaw = 1.05;       // [rad] (약 60°) 돈 뒤 오른쪽으로 이만큼 꺾였으면 출구
  double out_island_offset = 0.9;      // 노란 선만 보이는데 offset 이 이 이상 (노란 선이 로봇 오른쪽) 이면 섬 끝 = 출구
  double out_island_min_yaw = 0.6;     // [rad] 섬 끝 판단은 고리를 이만큼 돈 뒤부터 (선을 넘어 밖으로 나간 경우와 구분)
  double out_min_dist = 0.3;           // [m] 이만큼은 간 뒤부터 출구 판단 (돈 직후 차선 흔들림 무시)
  double out_max_dist = 3.0;           // [m] 이만큼 가도 출구가 없으면 잊음
  double out_approach_time = 0.0;      // [s] 출구에서 돌기 전 직진
  double out_turn_min_yaw = 0.6;       // [rad] (약 34°, exit_max_angle 보다 커야 지금 차선을 다시 안 잡음)
  double out_turn_max_yaw = 2.0;       // [rad] (약 115°)
  bool out_follow = true;              // 출구에서 제자리 회전 대신 EXIT (차선 따라가며 섬 쪽 차선 무시)
  double out_guard_dist = 0.5;         // [m] EXIT 유지 거리
  double out_guard_max_offset = 0.6;   // 출구 쪽 반대(섬 쪽)로 offset 이 이 이상이면 그 차선은 안 따라감
  double out_arc_linear = 0.08;        // [m/s] 안 따라갈 때 호 (반지름 = linear / angular = 20cm)
  double out_arc_angular = 0.4;        // [rad/s]
  // 호는 EXIT 시작 뒤 이 거리 안에서만 (섬 끝 노란 선은 0.02~0.07m 에서 보임). 그 뒤 섬 쪽 차선 / 차선 없음은
  // 직전 차선 추종 명령 유지 (out_hold_time 까지, 그 뒤 직진)
  //   (2026-10-13 06:46~ 로그: 출구 차선에 들어간 뒤 0.27~0.29m 에서 차선을 잠깐 놓쳐 왼쪽 호 -> 갑자기 왼쪽으로 꺾임.
  //    그래서 직진으로 바꿨더니 07:26 S자 굽이에서 0.27~0.38m 곧게 가 선을 넘음 -> 휘던 대로 유지)
  double out_arc_max_dist = 0.2;       // [m]
  double out_hold_time = 0.8;          // [s] 차선을 놓쳐도 직전 조향을 유지하는 시간
};

// interfaces/msg/Sign 과 같은 값
constexpr int TURN_SIGN_LEFT = 1, TURN_SIGN_RIGHT = 2;

class IntersectionTurn
{
public:
  enum class Phase {IDLE, ARMED, APPROACH, TURN, LOOP, EXIT, ALIGN, AFTER};

  struct Cmd
  {
    bool active = false;   // true 면 lane_follower 는 이 명령으로 (차선 추종 대신)
    double v = 0, w = 0;
    bool hold = false;     // active 이고 hold 면 직전 차선 추종 명령을 그대로 (없으면 v, w)
  };

  // 이번 틱의 lane_info
  struct LaneObs
  {
    bool lost = false;     // lane_info 는 오는데 차선을 못 찾음 (detected = false)
    bool ok = false;       // 차선 잡음 (detected, lane_timeout 안)
    bool fresh = true;     // 지난 틱 뒤에 새로 온 lane_info (같은 프레임을 여러 틱 세지 않게)
    double confidence = 0, angle = 0;
    double offset = 0;     // 차선 중심 - 로봇 (반 차선 폭 = 1, + 오른쪽, -1 ~ 1 로 잘림)
    bool left = false, right = false;   // 노란(왼쪽) / 흰(오른쪽) 선을 이 프레임에서 직접 찾음
  };

  void onSign(int type, double confidence, double area, const TurnParams & p)
  {
    const bool ok = (type == TURN_SIGN_LEFT || type == TURN_SIGN_RIGHT) &&
      confidence >= p.sign_min_confidence && area >= p.sign_min_area;
    window_.push_back(ok ? type : 0);
    while (static_cast<int>(window_.size()) > std::max(1, p.sign_confirm_window)) {
      window_.pop_front();
    }
    if (!p.enabled || !ok || since_done_ < p.rearm_dist || phase_ == Phase::LOOP || exiting_) {
      return;
    }
    int same = 0;
    for (int t : window_) {
      same += t == type;
    }
    if (same < p.sign_confirm_count && confidence < p.sign_instant_confidence) {
      return;
    }
    const int dir = type == TURN_SIGN_LEFT ? 1 : -1;
    if (phase_ == Phase::APPROACH || phase_ == Phase::TURN || phase_ == Phase::ALIGN) {
      if (dir != dir_ && yaw_ < p.turn_switch_max_yaw) {
        reason_ = std::string(phase_ == Phase::TURN ? "도는" : "직진") + " 중 표지판 " +
          (dir > 0 ? "LEFT" : "RIGHT") + " 확인 -> 방향 바꿈";
        changed_ = true;
        dir_ = dir;
        yaw_ = 0;
        ok_count_ = 0;
      }
      return;
    }
    if (phase_ != Phase::ARMED || dir != dir_) {
      reason_ = std::string("표지판 ") + (dir > 0 ? "LEFT" : "RIGHT") + " 확인";
      changed_ = true;
    }
    phase_ = Phase::ARMED;
    dir_ = dir;
    dist_ = 0;
    armed_lost_t_ = 0;
  }

  // ds: 이동 거리 [m], dyaw: 회전 [rad] (반시계 = 왼쪽 +)
  void onMove(double ds, double dyaw, const TurnParams & p)
  {
    since_done_ += std::abs(ds);
    switch (phase_) {
      case Phase::ARMED:
        dist_ += std::abs(ds);
        if (dist_ > p.arm_max_dist) {
          finish("교차로 없이 " + fmt(p.arm_max_dist) + "m -> 표지판 잊음", false);
        }
        break;
      case Phase::TURN:
        yaw_ += dyaw * dir_;
        if (loop_retry_) {
          loop_yaw_ -= dyaw * dir_;   // 고리에서 차선 놓쳐 더 도는 것도 고리 회전량에 넣음 (출구 쪽 반대라 -)
        }
        break;
      case Phase::LOOP:
        dist_ += std::abs(ds);
        loop_yaw_ -= dyaw * dir_;   // 왼쪽으로 돌았으면 고리는 오른쪽(시계)으로 돈다 -> + 가 출구 쪽
        if (dist_ > p.out_max_dist) {
          // 출구 신호(각도/차선 놓침/섬 끝)를 하나도 못 봤어도, 돌던 방향(dir_)을 아니까 완전히 포기하는 대신
          // EXIT 단계로 넘겨 그 방향 기준 탈출 동작(호 -> 직전 조향 유지 -> 직진)을 그대로 시도한다.
          // (예전엔 finish(false)로 완전히 포기해서, 그 뒤 lane_follower 의 lostHold 도 last_lane_ok_ 가
          //  오래전(회전 시작 전) 값이라 작동 못 하고 그대로 정지했다)
          startExit(fmt(p.out_max_dist) + "m 가도 출구 못 찾음 -> 그래도 " +
            (dir_ > 0 ? "왼쪽" : "오른쪽") + " 기준 탈출 시도 (" + fmt(loop_yaw_ * 57.2958, 0) + "°)", p);
        }
        break;
      case Phase::EXIT:
      case Phase::AFTER:
        dist_ += std::abs(ds);
        break;
      case Phase::IDLE:
      case Phase::APPROACH:
      case Phase::ALIGN:
        break;
    }
  }

  // 매 제어 틱. dt: 지난 틱부터 [s]
  Cmd update(const LaneObs & lane, const TurnParams & p, double dt)
  {
    const bool lane_lost = lane.lost;
    Cmd c;
    if (!p.enabled) {
      if (phase_ != Phase::IDLE) {
        finish("끔", false);
      }
      return c;
    }
    since_cross_ += dt;
    if (phase_ == Phase::ARMED) {
      armed_lost_t_ = lane_lost ? armed_lost_t_ + dt : 0;
    }
    if (phase_ == Phase::ARMED && lane_lost) {
      const bool cross_ok = !p.require_cross_line || since_cross_ <= p.cross_recent_time;
      const bool sustained = p.require_cross_line && armed_lost_t_ >= p.require_cross_line_grace;
      if (cross_ok || sustained) {
        const std::string why = cross_ok ?
          (p.require_cross_line ? "차선 놓침 + 가로 선 (교차로)" : "차선 놓침 (교차로)") :
          "차선 놓침 " + fmt(armed_lost_t_, 1) + "s 지속, 가로 선 없어도 교차로로 판단";
        if (p.align_enabled) {
          phase_ = Phase::ALIGN;
          t_ = 0;
          align_ok_ = 0;
          reason_ = why + " -> 가로 선 수평 맞춤";
          changed_ = true;
        } else {
          startApproach(false, why);
        }
      } else if (!no_cross_logged_) {
        reason_ = "차선 놓쳤지만 가로 선 없음 -> 회전 안 함";
        changed_ = true;
        no_cross_logged_ = true;
      }
    } else if (!lane_lost) {
      no_cross_logged_ = false;
    }
    if (phase_ == Phase::LOOP && dist_ < p.out_min_dist && lane_lost && p.after_enabled &&
      retries_ < p.after_max_retries)
    {
      // 돈 직후 차선을 놓침 (덜 돎) -> 같은 쪽으로 더 돌며 찾은 뒤 고리로 돌아감 (아래 TURN 이 이번 틱에 돎)
      ++retries_;
      phase_ = Phase::TURN;
      yaw_ = p.turn_min_yaw;
      ok_count_ = 0;
      after_turn_ = true;
      loop_retry_ = true;
      reason_ = "고리 " + fmt(dist_) + "m 에서 차선 놓침 -> 같은 쪽으로 더 돎 (" + std::to_string(retries_) + "번째)";
      changed_ = true;
    }
    if (phase_ == Phase::LOOP && dist_ >= p.out_min_dist) {
      const std::string at = fmt(dist_) + "m " + fmt(loop_yaw_ * 57.2958, 0) + "°";
      if (loop_yaw_ >= p.out_trigger_yaw) {
        startExit("고리 " + at + " 돎 (출구)", p);
      } else if (lane_lost) {
        startExit("고리에서 차선 놓침 (출구, " + at + ")", p);
      } else if (dir_ > 0 && loop_yaw_ >= p.out_island_min_yaw && lane.fresh && lane.ok && lane.left && !lane.right &&
        lane.offset >= p.out_island_offset)
      {
        startExit("노란 선이 오른쪽 = 섬 끝 (출구, " + at + ")", p);
      }
    }
    if (phase_ == Phase::EXIT) {
      if (dist_ >= p.out_guard_dist) {
        finish("출구 " + fmt(dist_) + "m 지남", true);
        return c;
      }
      // 차선이 있고 섬 쪽으로 치우치지 않았으면 차선 추종. 아니면 처음 out_arc_max_dist 안에서는 출구 쪽으로
      // 완만한 호 (이때 직전 조향은 고리를 따라 반대로 돌던 것이라 못 씀), 그 뒤로는 직전 차선 추종 명령 유지
      // (휘는 길에서 잠깐 놓친 것), out_hold_time 넘게 못 찾으면 직진
      const bool follow = lane.ok && dir_ * lane.offset < p.out_guard_max_offset;
      int mode = 0;   // 0 추종, 1 호, 2 직진, 3 직전 조향 유지
      if (follow) {
        hold_t_ = 0;
        exit_followed_ = true;
      } else if (dist_ < p.out_arc_max_dist || !exit_followed_) {
        // 출구에서 아직 차선을 한 번도 안 따라갔으면 직전 조향 = 고리를 따라 반대로 돌던 것 -> 계속 호
        mode = 1;
        hold_t_ = 0;
      } else {
        hold_t_ += dt;
        mode = hold_t_ <= p.out_hold_time ? 3 : 2;
      }
      if (mode != exit_mode_) {
        const std::string what = lane.ok ? "섬 쪽 차선 (offset " + fmt(lane.offset) + ")" : "차선 없음";
        reason_ = mode == 0 ? "차선 다시 따라감" :
          mode == 1 ? what + " -> 완만하게 " + (dir_ > 0 ? "왼쪽" : "오른쪽") :
          mode == 3 ? what + " -> 직전 조향 유지" : what + " -> " + fmt(p.out_hold_time, 1) + "s 지나 직진";
        changed_ = true;
        exit_mode_ = mode;
      }
      if (mode != 0) {
        c.active = true;
        c.v = p.out_arc_linear;
        c.w = mode == 1 ? dir_ * p.out_arc_angular : 0.0;
        c.hold = mode == 3;
      }
      return c;
    }
    if (phase_ == Phase::ALIGN) {
      t_ += dt;
      if (cross_fresh_) {
        align_ok_ = std::abs(cross_angle_) <= p.align_tol ? align_ok_ + 1 : 0;
        cross_fresh_ = false;
      }
      std::string done;
      if (align_ok_ >= p.align_ok_count) {
        done = "가로 선 수평 (" + fmt(cross_angle_ * 57.2958, 1) + "°, " + fmt(t_, 1) + "s)";
      } else if (since_cross_ > p.align_lost_time) {
        done = "가로 선 안 보임 (" + fmt(t_, 1) + "s)";
      } else if (t_ > p.align_max_time) {
        done = fmt(p.align_max_time, 1) + "s 지나도 수평 아님 (" + fmt(cross_angle_ * 57.2958, 1) + "°)";
      }
      if (done.empty()) {
        // 기울기 + = 오른쪽 끝이 가까움 = 로봇이 왼쪽으로 틀어짐 -> 오른쪽(-)으로 돈다
        // 이미 수평이면 멈춰서 확인 프레임을 기다린다
        double w = 0;
        if (std::abs(cross_angle_) > p.align_tol) {
          w = std::clamp(-p.align_kp * cross_angle_, -p.align_max_angular, p.align_max_angular);
          if (std::abs(w) < p.align_min_angular) {
            w = w < 0 ? -p.align_min_angular : p.align_min_angular;
          }
        }
        c.active = true;
        c.w = w;
        return c;
      }
      startApproach(false, done);
    }
    if (phase_ == Phase::APPROACH) {
      t_ += dt;
      if (t_ < (exiting_ ? p.out_approach_time : p.approach_time)) {
        c.active = true;
        c.v = p.approach_linear;
        return c;
      }
      phase_ = Phase::TURN;
      yaw_ = 0;
      ok_count_ = 0;
      // 직진 없이 바로 돌면 같은 틱에 로그가 한 번뿐이라 직진을 시작한 이유도 같이 남긴다
      const double approach = exiting_ ? p.out_approach_time : p.approach_time;
      reason_ = (t_ <= dt ? approach_why_ + ", " : std::string()) +
        (approach > 0 ? fmt(t_, 1) + "s 직진 -> " : std::string()) +
        (p.turn_pivot ? "한쪽 바퀴 회전" : "제자리 회전");
      changed_ = true;
    }
    if (phase_ == Phase::AFTER) {
      if (dist_ >= p.after_dist) {
        finish("돈 뒤 " + fmt(dist_) + "m 차선 따라감", true);
        return c;
      }
      if (!lane_lost || retries_ >= p.after_max_retries) {
        return c;   // 차선 추종 (재시도 다 쓰면 놓쳤을 때 lane_follower 가 멈춤)
      }
      ++retries_;
      phase_ = Phase::TURN;
      yaw_ = p.turn_min_yaw;   // 바로 찾는 속도로, 차선 보이면 바로 끝낼 수 있게
      ok_count_ = 0;
      after_turn_ = true;
      reason_ = "돈 뒤 " + fmt(dist_) + "m 에서 차선 놓침 -> 같은 쪽으로 더 돎 (" + std::to_string(retries_) + "번째)";
      changed_ = true;
    }
    if (phase_ == Phase::TURN) {
      const double min_yaw = exiting_ ? p.out_turn_min_yaw : p.turn_min_yaw;
      const double max_yaw = exiting_ ? p.out_turn_max_yaw :
        (after_turn_ ? p.turn_min_yaw + p.after_max_yaw : p.turn_max_yaw);
      if (lane.fresh) {
        ok_count_ = yaw_ >= min_yaw && exitLane(lane, p) ? ok_count_ + 1 : 0;
      }
      if (ok_count_ >= p.exit_ok_count) {
        const std::string why = fmt(yaw_ * 57.2958, 0) + "° 돌고 차선 찾음";
        if (loop_retry_) {
          // 고리에서 차선 놓쳐 더 돈 것: 재던 거리 / 고리 회전량 그대로 고리로 돌아감
          phase_ = Phase::LOOP;
          loop_retry_ = false;
          reason_ = fmt(yaw_ * 57.2958 - p.turn_min_yaw * 57.2958, 0) + "° 더 돌고 차선 찾음 -> 고리 다시 따라감";
          changed_ = true;
        } else if (!exiting_ && p.out_enabled && (dir_ > 0 || p.out_right_enabled)) {
          // 고리를 따라가다 출구에서 다시 도는 쪽으로 (왼쪽 길: 시계 방향 고리 -> 왼쪽 출구, 오른쪽 길: 반대)
          phase_ = Phase::LOOP;
          dist_ = 0;
          loop_yaw_ = 0;
          since_done_ = 0;
          reason_ = why + " -> 고리 따라가며 출구 기다림";
          changed_ = true;
        } else if (!exiting_ && p.after_enabled) {
          // 돈 뒤 한동안 차선을 놓치는지 본다 (놓치면 같은 쪽으로 더 돎)
          phase_ = Phase::AFTER;
          dist_ = 0;
          since_done_ = 0;
          reason_ = (after_turn_ ? fmt(yaw_ * 57.2958 - p.turn_min_yaw * 57.2958, 0) + "° 더 돌고 차선 찾음" : why) +
            " -> 차선 따라가며 확인";
          changed_ = true;
        } else {
          finish(why, true);
        }
        return c;
      }
      if (yaw_ >= max_yaw) {
        finish(fmt(yaw_ * 57.2958, 0) + "° 돌아도 차선 없음 -> 포기", true);
        return c;
      }
      c.active = true;
      c.w = dir_ * (yaw_ < min_yaw ? p.turn_angular : p.turn_search_angular);
      c.v = p.turn_pivot ? std::abs(c.w) * p.wheel_separation / 2 : p.turn_linear;
    }
    return c;
  }

  // lane_detection cross_line: 교차로 T자 가로 선을 이번 프레임에서 봤는지
  // 다가가며 정렬: ARMED 이고 가로 선이 막 보였으면 true. w_add 를 차선 추종 조향에 더하고 속도는 v_max 이하로
  bool crossAssist(const TurnParams & p, double & w_add, double & v_max) const
  {
    if (!p.enabled || !p.approach_align || phase_ != Phase::ARMED || since_cross_ > p.approach_align_fresh) {
      return false;
    }
    w_add = std::clamp(-p.approach_align_kp * cross_angle_, -p.approach_align_max, p.approach_align_max);
    v_max = p.approach_slow_linear;
    return true;
  }
  double crossAngle() const {return cross_angle_;}

  // angle: 가로 선 기울기 [rad] (+ = 오른쪽 끝이 더 가까움 = 로봇이 왼쪽으로 틀어짐)
  void onCrossLine(bool detected, double angle = 0.0)
  {
    if (detected) {
      since_cross_ = 0;
      cross_angle_ = angle;
      cross_fresh_ = true;
    }
  }

  // 주행 모드에서 빠지면 (수동 조종 / STOP) 하던 기동도, 기억한 표지판도 버린다
  // (test_041619: 표지판 기억이 남아 손으로 옮겨 놓고 다시 출발하자 40초 전에 본 LEFT 로 들어감)
  void cancelManeuver()
  {
    window_.clear();
    if (phase_ != Phase::IDLE) {
      finish(phase_ == Phase::ARMED ? "주행 모드 해제 -> 표지판 잊음" : "주행 모드 해제 -> 기동 취소", false);
    }
  }

  Phase phase() const {return phase_;}
  int direction() const {return dir_;}   // +1 왼쪽, -1 오른쪽
  const std::string & lastReason() const {return reason_;}

  // 상태가 바뀌었으면 true (한 번만) -> 로그용
  bool takeChanged()
  {
    const bool c = changed_;
    changed_ = false;
    return c;
  }

  std::string describe() const
  {
    static const char * names[] = {"IDLE", "ARMED", "APPROACH", "TURN", "LOOP", "EXIT", "ALIGN", "AFTER"};
    std::string s = names[static_cast<int>(phase_)];
    if (phase_ != Phase::IDLE) {
      s += dir_ > 0 ? " LEFT" : " RIGHT";
    }
    if (exiting_) {
      s += " (출구)";
    }
    if (phase_ == Phase::LOOP) {
      s += " " + fmt(dist_) + "m " + fmt(loop_yaw_ * 57.2958, 0) + "°";
    }
    if (phase_ == Phase::EXIT || phase_ == Phase::AFTER) {
      s += " " + fmt(dist_) + "m";
    }
    return s;
  }

private:
  // 돌다가 이 차선이면 멈춘다: 곧고, 로봇이 차선 안에 있고, 두 선이 다 보이거나 한 선이면 도는 쪽 선
  //   (test_041619 / test_030522: 왼쪽으로 돌며 흰 선만, 오른쪽으로 돌며 노란 선만 보고 끝내면 엉뚱한 선.
  //    test_044608: 섬 끝 노란 선이 로봇 오른쪽에 있으면 offset +1 -> exit_max_offset 으로 거름)
  bool exitLane(const LaneObs & l, const TurnParams & p) const
  {
    // 남은 회전: + 면 아직 도는 쪽으로 더 돌아야 함 (왼쪽으로 돌 때 차선이 왼쪽 = angle -)
    const double remaining = -dir_ * l.angle;
    if (!l.ok || l.confidence < p.exit_min_confidence || remaining > p.exit_lead_angle ||
      remaining < -p.exit_max_angle || std::abs(l.offset) > p.exit_max_offset)
    {
      return false;
    }
    if (!p.exit_turn_side_only || (l.left && l.right)) {
      return true;
    }
    return dir_ > 0 ? l.left : l.right;
  }

  // 고리 출구: out_follow 면 EXIT (차선 따라가며 섬 쪽 무시), 아니면 예전처럼 제자리 회전
  void startExit(const std::string & why, const TurnParams & p)
  {
    if (!p.out_follow) {
      startApproach(true, why);
      return;
    }
    phase_ = Phase::EXIT;
    exiting_ = true;
    dist_ = 0;
    exit_mode_ = 0;
    hold_t_ = 0;
    exit_followed_ = false;
    reason_ = why + " -> 차선 따라 출구 (섬 쪽 차선 무시)";
    changed_ = true;
  }

  void startApproach(bool exiting, const std::string & why)
  {
    phase_ = Phase::APPROACH;
    exiting_ = exiting;
    t_ = 0;
    yaw_ = 0;
    ok_count_ = 0;
    approach_why_ = why;
    after_turn_ = false;
    loop_retry_ = false;
    if (!exiting) {
      retries_ = 0;
    }
    reason_ = why + " -> 직진";
    changed_ = true;
  }

  void finish(const std::string & why, bool turned)
  {
    phase_ = Phase::IDLE;
    exiting_ = false;
    loop_retry_ = false;
    reason_ = why;
    changed_ = true;
    if (turned) {
      since_done_ = 0;
    }
  }

  static std::string fmt(double x, int prec = 2)
  {
    char b[32];
    std::snprintf(b, sizeof(b), "%.*f", prec, x);
    return b;
  }

  Phase phase_ = Phase::IDLE;
  int dir_ = 1;
  double dist_ = 0, yaw_ = 0, loop_yaw_ = 0, t_ = 0;
  bool exiting_ = false;   // 지금 직진 / 회전 / EXIT 가 고리 출구용
  bool after_turn_ = false;   // AFTER 에서 차선 놓쳐 다시 도는 중
  bool loop_retry_ = false;   // LOOP 처음에 차선 놓쳐 다시 도는 중 (찾으면 LOOP 로)
  int retries_ = 0;           // AFTER 에서 다시 돈 횟수
  int exit_mode_ = 0;      // EXIT 중 0 차선 추종, 1 호, 2 직진, 3 직전 조향 유지
  double hold_t_ = 0;      // EXIT 중 차선 없이 직전 조향을 유지한 시간 [s]
  bool exit_followed_ = false;   // EXIT 에서 차선을 따라간 적이 있음 (직전 조향이 출구 쪽 것)
  double since_done_ = 1e9;
  double since_cross_ = 1e9;   // 마지막으로 가로 선을 본 뒤 [s]
  double armed_lost_t_ = 0;    // ARMED 상태에서 차선을 계속 놓치고 있는 시간 [s] (require_cross_line_grace 용)
  double cross_angle_ = 0;     // 마지막 가로 선 기울기 [rad]
  bool cross_fresh_ = false;   // ALIGN 이 아직 안 쓴 가로 선
  int align_ok_ = 0;
  bool no_cross_logged_ = false;
  int ok_count_ = 0;
  bool changed_ = false;
  std::string reason_;
  std::string approach_why_;   // 직진을 시작한 이유 (로그)
  std::deque<int> window_;
};

}  // namespace lane_follower
