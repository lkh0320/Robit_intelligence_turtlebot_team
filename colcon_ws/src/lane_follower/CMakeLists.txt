cmake_minimum_required(VERSION 3.8)
project(lane_follower)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(interfaces REQUIRED)

add_executable(lane_follower_node src/lane_follower_node.cpp)
target_include_directories(lane_follower_node PRIVATE include)
ament_target_dependencies(lane_follower_node rclcpp geometry_msgs std_msgs sensor_msgs interfaces)

# 주차 노드: 주차 표지판(또는 PARKING 모드)을 보면 주차하고 다시 라인 주행으로 돌려준다
add_executable(parking_node src/parking_node.cpp)
ament_target_dependencies(parking_node rclcpp geometry_msgs interfaces std_msgs)

install(TARGETS lane_follower_node parking_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_controller test/test_controller.cpp)
  target_include_directories(test_controller PRIVATE include)
  ament_add_gtest(test_drive_selector test/test_drive_selector.cpp)
  target_include_directories(test_drive_selector PRIVATE include)
  ament_add_gtest(test_intersection_turn test/test_intersection_turn.cpp)
  target_include_directories(test_intersection_turn PRIVATE include)
endif()

ament_package()
