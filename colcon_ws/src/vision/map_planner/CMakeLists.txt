cmake_minimum_required(VERSION 3.8)
project(map_planner)

# 후보 경로 평가가 많아 최적화 없이(-O0) 빌드하면 느리다
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)   # third_party/vision_opencv 에서 OpenCV 4.8 로 빌드한 것
find_package(interfaces REQUIRED)
find_package(path_planner REQUIRED)   # buildMaps (색 마스크)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

add_executable(map_planner_node src/map_planner_node.cpp)
target_include_directories(map_planner_node PRIVATE include ${path_planner_INCLUDE_DIRS})
target_link_libraries(map_planner_node ${OpenCV_LIBS})
ament_target_dependencies(map_planner_node rclcpp sensor_msgs cv_bridge interfaces)

install(TARGETS map_planner_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_map_core test/test_map_core.cpp TIMEOUT 300)
  target_include_directories(test_map_core PRIVATE include ${path_planner_INCLUDE_DIRS})
  target_link_libraries(test_map_core ${OpenCV_LIBS})
endif()

ament_package()
