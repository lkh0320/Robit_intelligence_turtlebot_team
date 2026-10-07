cmake_minimum_required(VERSION 3.8)
project(path_planner)

# 후보 경로가 많아 최적화 없이(-O0) 빌드하면 프레임당 계획이 수십 ms 걸린다
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
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

add_executable(path_planner_node src/path_planner_node.cpp)
target_include_directories(path_planner_node PRIVATE include)
target_link_libraries(path_planner_node ${OpenCV_LIBS})
ament_target_dependencies(path_planner_node rclcpp sensor_msgs cv_bridge interfaces)

install(TARGETS path_planner_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})
# map_planner 가 buildMaps (색 마스크) 를 같이 쓴다
install(DIRECTORY include/
  DESTINATION include)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_planner_core test/test_planner_core.cpp)
  target_include_directories(test_planner_core PRIVATE include)
  target_link_libraries(test_planner_core ${OpenCV_LIBS})
endif()

ament_package()
