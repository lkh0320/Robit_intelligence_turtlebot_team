cmake_minimum_required(VERSION 3.8)
project(grid_drive)

# Dijkstra / 원호 후보는 최적화 없이(-O0) 빌드하면 느리다
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(obstacle_detection REQUIRED)   # color_maps.hpp (색 방식 관측)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

# 녹화 프레임 (jpg 폴더) 으로 계획기를 돌려 디버그 그림 / 시간을 남기는 도구 (ROS 없이)
add_executable(replay_frames tools/replay_frames.cpp)
target_include_directories(replay_frames PRIVATE include)
target_link_libraries(replay_frames ${OpenCV_LIBS})
ament_target_dependencies(replay_frames obstacle_detection)

install(TARGETS replay_frames
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_grid_planner test/test_grid_planner.cpp)
  target_include_directories(test_grid_planner PRIVATE include)
  target_link_libraries(test_grid_planner ${OpenCV_LIBS})
  ament_target_dependencies(test_grid_planner obstacle_detection)
endif()

ament_export_include_directories(include)
ament_package()
