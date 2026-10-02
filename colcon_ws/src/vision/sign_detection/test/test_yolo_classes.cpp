// yolo_classes.hpp 단위 테스트 (gtest)
//   실행: colcon test --packages-select sign_detection && colcon test-result --verbose
#include <gtest/gtest.h>

#include <fstream>
#include <string>
#include <vector>

#include "sign_detection/yolo_classes.hpp"

namespace yc = sign_detection;

// enum / NAMES 가 팀 공용 yolo/classes.txt 와 순서까지 똑같은지
TEST(YoloClasses, MatchClassesTxt)
{
  std::ifstream f(YOLO_CLASSES_TXT);
  ASSERT_TRUE(f.is_open()) << YOLO_CLASSES_TXT << " 를 열 수 없음";
  std::vector<std::string> names;
  for (std::string line; std::getline(f, line); ) {
    while (!line.empty() && (line.back() == '\r' || line.back() == ' ')) {
      line.pop_back();
    }
    if (!line.empty()) {
      names.push_back(line);
    }
  }
  ASSERT_EQ(names.size(), static_cast<size_t>(yc::NUM_CLASSES));
  for (int i = 0; i < yc::NUM_CLASSES; ++i) {
    EXPECT_EQ(names[i], yc::NAMES[i]) << i << "번";
  }
}

TEST(YoloClasses, ToMessages)
{
  using S = interfaces::msg::Sign;
  using L = interfaces::msg::TrafficLight;
  using B = interfaces::msg::Barrier;
  EXPECT_EQ(yc::to_sign_type(yc::SIGN_CONSTRUCTION), S::CONSTRUCTION);
  EXPECT_EQ(yc::to_sign_type(yc::SIGN_LEFT), S::LEFT);
  EXPECT_EQ(yc::to_sign_type(yc::SIGN_RIGHT), S::RIGHT);
  EXPECT_EQ(yc::to_sign_type(yc::SIGN_PARKING), S::PARKING);
  EXPECT_EQ(yc::to_sign_type(yc::LIGHT_RED), S::NONE);

  EXPECT_EQ(yc::to_light_state(yc::LIGHT_RED), L::RED);
  EXPECT_EQ(yc::to_light_state(yc::LIGHT_YELLOW), L::YELLOW);
  EXPECT_EQ(yc::to_light_state(yc::LIGHT_GREEN), L::GREEN);
  EXPECT_EQ(yc::to_light_state(yc::BAR_OPEN), L::UNKNOWN);

  EXPECT_EQ(yc::to_barrier_state(yc::BAR_CLOSED), B::CLOSED);
  EXPECT_EQ(yc::to_barrier_state(yc::BAR_OPEN), B::OPEN);
  EXPECT_EQ(yc::to_barrier_state(yc::SIGN_LEFT), B::UNKNOWN);
}

// 모든 클래스가 정확히 한 그룹(표지판/신호등/차단봉)에 속하는지
TEST(YoloClasses, Groups)
{
  for (int i = 0; i < yc::NUM_CLASSES; ++i) {
    EXPECT_EQ(yc::is_sign(i) + yc::is_light(i) + yc::is_bar(i), 1) << yc::NAMES[i];
  }
  EXPECT_FALSE(yc::is_sign(yc::NUM_CLASSES) || yc::is_light(yc::NUM_CLASSES) ||
    yc::is_bar(yc::NUM_CLASSES));
}
