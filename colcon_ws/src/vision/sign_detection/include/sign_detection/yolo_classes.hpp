// YOLO 클래스 번호 -> ROS 메시지 값 변환 (팀 공용 클래스 규칙: yolo/classes.txt, yolo/data.yaml)
// 번호와 순서는 classes.txt 와 반드시 같아야 한다. 새 클래스는 맨 뒤(9번부터)에만 추가하고
// 여기 enum, NAMES, 아래 변환 함수를 같이 고친다. (test/test_yolo_classes.cpp 가 classes.txt 와 비교한다)
#pragma once

#include <array>
#include <cstdint>

#include "interfaces/msg/barrier.hpp"
#include "interfaces/msg/sign.hpp"
#include "interfaces/msg/traffic_light.hpp"

namespace sign_detection
{

enum YoloClass : int
{
  SIGN_CONSTRUCTION = 0,   // 장애물(공사) 구간 진입 표지판
  SIGN_LEFT = 1,           // 갈림길 좌회전
  SIGN_RIGHT = 2,          // 갈림길 우회전
  SIGN_PARKING = 3,        // 주차 구간
  LIGHT_RED = 4,           // 신호등 빨강 (켜진 불 기준)
  LIGHT_YELLOW = 5,        // 신호등 노랑
  LIGHT_GREEN = 6,         // 신호등 초록
  BAR_CLOSED = 7,          // 차단봉 내려옴 -> 정지
  BAR_OPEN = 8,            // 차단봉 올라감 -> 통과
  NUM_CLASSES = 9
};

// classes.txt 와 같은 순서의 이름 (로그/디버그 화면용)
inline constexpr std::array<const char *, NUM_CLASSES> NAMES{
  "sign_construction", "sign_left", "sign_right", "sign_parking",
  "light_red", "light_yellow", "light_green",
  "bar_closed", "bar_open"};

inline bool is_sign(int cls) {return cls >= SIGN_CONSTRUCTION && cls <= SIGN_PARKING;}
inline bool is_light(int cls) {return cls >= LIGHT_RED && cls <= LIGHT_GREEN;}
inline bool is_bar(int cls) {return cls == BAR_CLOSED || cls == BAR_OPEN;}

// 표지판 클래스 -> Sign.type (표지판이 아니면 NONE)
inline uint8_t to_sign_type(int cls)
{
  using M = interfaces::msg::Sign;
  switch (cls) {
    case SIGN_CONSTRUCTION: return M::CONSTRUCTION;
    case SIGN_LEFT: return M::LEFT;
    case SIGN_RIGHT: return M::RIGHT;
    case SIGN_PARKING: return M::PARKING;
    default: return M::NONE;
  }
}

// 신호등 클래스 -> TrafficLight.state (신호등이 아니면 UNKNOWN)
inline uint8_t to_light_state(int cls)
{
  using M = interfaces::msg::TrafficLight;
  switch (cls) {
    case LIGHT_RED: return M::RED;
    case LIGHT_YELLOW: return M::YELLOW;
    case LIGHT_GREEN: return M::GREEN;
    default: return M::UNKNOWN;
  }
}

// 차단봉 클래스 -> Barrier.state (차단봉이 아니면 UNKNOWN)
inline uint8_t to_barrier_state(int cls)
{
  using M = interfaces::msg::Barrier;
  switch (cls) {
    case BAR_CLOSED: return M::CLOSED;
    case BAR_OPEN: return M::OPEN;
    default: return M::UNKNOWN;
  }
}

}  // namespace sign_detection
