// lane_method:=hybrid 에서 어떤 lane_info 로 주행할지 고르는 상태머신 (ROS 없이 테스트 가능)
//
//   평소(NORMAL)는 sliding (lane_detection), 갈림길(FORK) / 공사 구간(CONSTRUCTION)에서만 path (path_planner).
//   path_planner 는 hybrid 에서도 계속 돌고 있어서 (경로 기억) 바꾸는 순간 바로 쓸 수 있다.
//   지금 구간은 hint() 로 path_planner 에 알려 준다 (갈림길 방향 가점 / 공사 구간 선 넘기 허용을 구간 내내 유지).
//
//   진입
//     FORK:          최근 sign_confirm_window 프레임 중 LEFT/RIGHT 표지판이 sign_confirm_count 번 이상.
//                    방향은 그 안에서 많이 본 쪽 (좌/우 표지판이 번갈아 잡히는 경우가 있어서 둘을 같이 센다),
//                    정해진 방향은 구간이 끝날 때까지 안 바꾼다
//     CONSTRUCTION:  같은 방법으로 CONSTRUCTION 표지판, 또는
//                    표지판을 놓쳐도 box_enter_dist 안 박스가 box_confirm_count 번 연속 보이면
//                    (박스는 RETURNING 중에도, 빠져나온 직후에도 다시 진입시킨다: 박스를 잘못 세서 일찍 나온 경우)
//   탈출 (구간 -> RETURNING: sliding 이 차선을 제대로 잡을 때까지 path 유지 -> NORMAL)
//     FORK:          표지판 방향으로 fork_exit_yaw 만큼 돌았으면 (교차로에서 그 차선으로 들어감)
//                    / fork_max_dist 를 넘게 달리면 (안전장치)
//     CONSTRUCTION:  박스 construction_boxes 개를 지나쳤고 앞에 남은 박스가 없으면
//                    / 박스를 하나 이상 지난 뒤 construction_lost_dist 동안 새 박스가 없으면 (박스를 놓친 경우)
//                    / 박스를 하나도 못 보고 construction_no_box_dist 를 달리면 (표지판 오인식 / 다시 본 경우)
//                    / construction_max_dist 를 넘게 달리면 (안전장치)
//   RETURNING:       sliding 이 return_ok_count 번 연속 정상 (detected, confidence, |offset| 작음) 이면 NORMAL.
//                    return_max_dist 를 넘으면 sliding 상태와 관계없이 NORMAL
//   빠져나온 뒤 rearm_dist 동안은 같은 종류 표지판으로 다시 들어가지 않는다 (지나간 표지판을 다시 보는 경우)
//
//   박스 통과 세기: path_planner 가 매 프레임 가장 가까운 박스까지 거리(바퀴 축 기준)를 준다.
//     거리로 같은 박스를 이어 붙이고 (좌/우는 로봇이 피하며 돌면 바뀔 수 있어 안 씀), 안 보이면 바퀴 이동량으로
//     거리를 줄여 간다. 박스 앞 끝이 바퀴 축 뒤로 box_pass_margin 만큼 지나가면 통과 1개.
//     BEV 맨 아래에 걸친 박스는 실제보다 멀게 나오므로 (잘림) 거리를 고치지 않고 본 것으로만 센다.
//
//   path_fallback: NORMAL 에서 sliding 이 차선을 못 찾고 path 는 찾으면 그 틱만 path 로 (정지 대신)
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <string>
#include <vector>

namespace lane_follower
{

enum class Source {SLIDING, PATH};
enum class Zone {NORMAL, FORK, CONSTRUCTION, RETURNING};

// interfaces/msg/Sign 과 같은 값
constexpr int SIGN_NONE = 0, SIGN_LEFT = 1, SIGN_RIGHT = 2, SIGN_CONSTRUCTION = 3;

struct SelectorParams
{
  bool enabled = true;                 // false: 항상 sliding
  bool path_fallback = true;
  // 표지판
  double sign_min_confidence = 0.5;
  double sign_min_area = 0.01;         // bbox 면적 / 이미지 면적
  int sign_confirm_count = 3;          // 최근 sign_confirm_window 프레임 중 이만큼 보이면 확정
  int sign_confirm_window = 5;         // (인식이 한 프레임씩 끊기는 경우가 있어서 연속이 아니어도 됨)
  // 갈림길
  double fork_exit_yaw = 1.2;          // [rad] 표지판 방향으로 이만큼 돌면 탈출 (약 69°)
  double fork_max_dist = 2.0;          // [m]
  // 공사 구간
  int box_confirm_count = 5;           // 표지판 없이 박스만으로 진입 (0 = 끔)
  double box_enter_dist = 0.45;        // [m] 이보다 가까운 박스만 진입 판단에 씀
  int construction_boxes = 3;
  double box_match_tol = 0.15;         // [m] 직전 추정 거리와 이만큼 안이면 같은 박스
  double box_pass_margin = 0.15;       // [m] 박스 앞 끝이 바퀴 축 뒤로 이만큼 지나가면 통과
  int box_min_hits = 3;                // 이보다 적게 본 박스는 잡음으로 보고 안 센다
  double box_clip_ratio = 0.97;        // bottom_ratio 가 이 이상이면 BEV 아래에 걸쳐 잘린 박스
  double construction_lost_dist = 1.0; // [m]
  double construction_no_box_dist = 2.0;  // [m]
  double construction_max_dist = 4.0;  // [m]
  // 복귀
  int return_ok_count = 5;
  double return_min_confidence = 0.5;
  double return_max_offset = 0.5;      // sliding offset (반 차선 폭 = 1)
  double return_max_dist = 0.6;        // [m]
  double rearm_dist = 1.5;             // [m]
};

// sliding lane_info 상태 (lane_follower 가 신선도까지 따져서 넘긴다)
struct LaneStatus
{
  bool ok = false;        // 받았고, 끊기지 않았고, detected
  double offset = 0, confidence = 0;
};

class DriveSelector
{
public:
  // 표지판 메시지마다 (type: Sign 상수, NONE 포함)
  void onSign(int type, double confidence, double area_ratio, const SelectorParams & p)
  {
    const bool valid = confidence >= p.sign_min_confidence && area_ratio >= p.sign_min_area;
    recent_.push_back(valid ? type : SIGN_NONE);
    while (static_cast<int>(recent_.size()) > std::max(1, p.sign_confirm_window)) {
      recent_.pop_front();
    }
    if (!p.enabled || zone_ != Zone::NORMAL) {
      return;
    }
    int left = 0, right = 0, constr = 0, last_fork = SIGN_NONE;
    for (const int t : recent_) {
      left += t == SIGN_LEFT;
      right += t == SIGN_RIGHT;
      constr += t == SIGN_CONSTRUCTION;
      if (t == SIGN_LEFT || t == SIGN_RIGHT) {
        last_fork = t;
      }
    }
    if (left + right >= p.sign_confirm_count && armed(Zone::FORK, p)) {
      fork_dir_ = left > right ? -1 : right > left ? 1 : last_fork == SIGN_LEFT ? -1 : 1;
      enter(Zone::FORK, "갈림길 표지판");
    } else if (constr >= p.sign_confirm_count && armed(Zone::CONSTRUCTION, p)) {
      enter(Zone::CONSTRUCTION, "공사 표지판");
    }
  }

  // path_planner box_obstacle 메시지마다. distance: 바퀴 축에서 박스 로봇 쪽 끝까지 [m]
  void onBox(bool detected, double distance, double bottom_ratio, const SelectorParams & p)
  {
    if (!detected) {
      box_streak_ = 0;
      return;
    }
    box_streak_ = distance <= p.box_enter_dist ? box_streak_ + 1 : 0;
    // 박스가 눈앞에 있으면 sliding 으로는 못 지나간다 -> 빠져나온 직후여도 다시 공사 구간
    if (p.enabled && (zone_ == Zone::NORMAL || zone_ == Zone::RETURNING) &&
      p.box_confirm_count > 0 && box_streak_ >= p.box_confirm_count)
    {
      enter(Zone::CONSTRUCTION, "박스");
    }
    if (zone_ != Zone::CONSTRUCTION) {
      return;
    }
    const bool clipped = bottom_ratio >= p.box_clip_ratio;
    // 가장 가까운 박스만 오므로 남은 트랙 중 거리가 맞는 것에 붙인다
    Track * best = nullptr;
    for (auto & t : tracks_) {
      const double err = std::abs(t.dist - distance);
      // 잘린 박스는 실제가 보이는 거리보다 가깝다 -> 추정이 더 가까워도 같은 박스로 본다
      const bool match = err <= p.box_match_tol || (clipped && t.dist <= distance);
      if (match && (!best || err < std::abs(best->dist - distance))) {
        best = &t;
      }
    }
    if (best) {
      ++best->hits;
      if (!clipped) {
        best->dist = distance;
      }
    } else {
      tracks_.push_back({distance, 1});
    }
    box_seen_ = true;
    last_box_traveled_ = traveled_;
  }

  // 직전 호출 이후 로봇 이동량: ds 앞으로 간 거리 [m], dyaw 반시계 + [rad]
  void onMove(double ds, double dyaw, const SelectorParams & p)
  {
    traveled_ += ds;
    yaw_ += dyaw;
    for (auto it = tracks_.begin(); it != tracks_.end(); ) {
      it->dist -= ds;
      if (it->dist < -p.box_pass_margin) {
        if (it->hits >= p.box_min_hits) {
          ++boxes_passed_;
        }
        it = tracks_.erase(it);
      } else {
        ++it;
      }
    }
  }

  // 제어 주기마다: 이번 틱에 쓸 lane_info
  Source update(const LaneStatus & sliding, bool path_ok, const SelectorParams & p)
  {
    if (!p.enabled) {
      zone_ = Zone::NORMAL;
      return Source::SLIDING;
    }
    const double dist = traveled_ - zone_start_;
    switch (zone_) {
      case Zone::NORMAL:
        break;
      case Zone::FORK:
        // 왼쪽(fork_dir -1) 은 반시계(+yaw)
        if (-fork_dir_ * (yaw_ - zone_yaw_) >= p.fork_exit_yaw) {
          leave("차선 진입 (회전)");
        } else if (dist > p.fork_max_dist) {
          leave("거리 초과");
        }
        break;
      case Zone::CONSTRUCTION:
        if (boxes_passed_ >= p.construction_boxes && !boxAhead(p)) {
          leave("박스 모두 통과");
        } else if (boxes_passed_ > 0 && tracks_.empty() &&
          traveled_ - last_box_traveled_ > p.construction_lost_dist)
        {
          leave("박스 안 보임");
        } else if (!box_seen_ && dist > p.construction_no_box_dist) {
          leave("박스 못 봄");
        } else if (dist > p.construction_max_dist) {
          leave("거리 초과");
        }
        break;
      case Zone::RETURNING:
        {
          const bool good = sliding.ok && sliding.confidence >= p.return_min_confidence &&
            std::abs(sliding.offset) <= p.return_max_offset;
          ok_streak_ = good ? ok_streak_ + 1 : 0;
          if (ok_streak_ >= p.return_ok_count || traveled_ - return_start_ > p.return_max_dist) {
            zone_ = Zone::NORMAL;
            last_reason_ = ok_streak_ >= p.return_ok_count ? "sliding 복귀" : "sliding 복귀 (거리 초과)";
          }
        }
        break;
    }
    if (zone_ != Zone::NORMAL) {
      return Source::PATH;
    }
    if (!sliding.ok && path_ok && p.path_fallback) {
      return Source::PATH;
    }
    return Source::SLIDING;
  }

  // path_planner 에 알려 줄 구간 (Sign 상수): 갈림길이면 방향, 공사 구간이면 CONSTRUCTION, 그 외 NONE
  int hint() const
  {
    if (zone_ == Zone::FORK) {
      return fork_dir_ < 0 ? SIGN_LEFT : SIGN_RIGHT;
    }
    return zone_ == Zone::CONSTRUCTION ? SIGN_CONSTRUCTION : SIGN_NONE;
  }

  Zone zone() const {return zone_;}
  int forkDir() const {return fork_dir_;}
  int boxesPassed() const {return boxes_passed_;}
  int boxesTracked() const {return static_cast<int>(tracks_.size());}
  const std::string & lastReason() const {return last_reason_;}

  // GUI / 로그용 한 줄
  std::string describe() const
  {
    char buf[96];
    switch (zone_) {
      case Zone::NORMAL: return "NORMAL";
      case Zone::FORK:
        std::snprintf(buf, sizeof(buf), "FORK_%s yaw %+.0fdeg", fork_dir_ < 0 ? "L" : "R",
          (yaw_ - zone_yaw_) * 180 / M_PI);
        return buf;
      case Zone::CONSTRUCTION:
        std::snprintf(buf, sizeof(buf), "CONSTRUCTION box %d (track %zu)", boxes_passed_,
          tracks_.size());
        return buf;
      case Zone::RETURNING:
        std::snprintf(buf, sizeof(buf), "RETURNING from %s",
          last_zone_ == Zone::FORK ? "FORK" : "CONSTRUCTION");
        return buf;
    }
    return "?";
  }

private:
  struct Track
  {
    double dist;   // 바퀴 축에서 박스 로봇 쪽 끝까지 [m]
    int hits;
  };

  // 잡음이 아닌 박스가 아직 앞(통과 전)에 있음
  bool boxAhead(const SelectorParams & p) const
  {
    return std::any_of(tracks_.begin(), tracks_.end(),
             [&p](const Track & t) {return t.hits >= p.box_min_hits;});
  }

  // 같은 구간을 막 빠져나온 직후에는 표지판으로 다시 안 들어간다
  bool armed(Zone z, const SelectorParams & p) const
  {
    return !(has_left_ && last_zone_ == z && traveled_ - left_at_ < p.rearm_dist);
  }

  void enter(Zone z, const char * why)
  {
    zone_ = z;
    zone_start_ = traveled_;
    zone_yaw_ = yaw_;
    boxes_passed_ = 0;
    tracks_.clear();
    box_seen_ = false;
    last_box_traveled_ = traveled_;
    box_streak_ = 0;
    recent_.clear();
    last_reason_ = std::string(z == Zone::FORK ? "갈림길 진입: " : "공사 구간 진입: ") + why;
  }

  void leave(const char * why)
  {
    last_zone_ = zone_;
    has_left_ = true;
    left_at_ = traveled_;
    zone_ = Zone::RETURNING;
    return_start_ = traveled_;
    ok_streak_ = 0;
    last_reason_ = std::string(last_zone_ == Zone::FORK ? "갈림길 탈출: " : "공사 구간 탈출: ") + why;
  }

  Zone zone_ = Zone::NORMAL, last_zone_ = Zone::NORMAL;
  bool has_left_ = false;
  double traveled_ = 0, yaw_ = 0;
  double zone_start_ = 0, zone_yaw_ = 0, return_start_ = 0, left_at_ = 0;
  int fork_dir_ = 0;
  std::deque<int> recent_;   // 최근 표지판 (조건 못 넘은 것은 NONE)
  int box_streak_ = 0, ok_streak_ = 0;
  std::vector<Track> tracks_;
  int boxes_passed_ = 0;
  bool box_seen_ = false;
  double last_box_traveled_ = 0;
  std::string last_reason_;
};

}  // namespace lane_follower
