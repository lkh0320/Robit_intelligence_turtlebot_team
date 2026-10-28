cmake_minimum_required(VERSION 3.8)
project(task_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(task_planner_node src/task_planner_node.cpp)
ament_target_dependencies(task_planner_node rclcpp interfaces std_msgs)

install(TARGETS task_planner_node
  DESTINATION lib/${PROJECT_NAME})
# turtle_gui "YAML 저장" 을 받아 config yaml 에 쓰는 노드 (grid_drive_bringup 에서 실행)
install(PROGRAMS scripts/param_saver.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
