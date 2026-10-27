cmake_minimum_required(VERSION 3.8)
project(obstacle_detection)

# 공사 구간 플래너는 후보 경로가 많아 최적화 없이(-O0) 빌드하면 느리다
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

# 차단바, 벽(지그재그 구간) 은 따로 켜고 끌 수 있게 노드를 나눈다
foreach(node barrier_detection_node wall_detection_node)
  add_executable(${node} src/${node}.cpp)
  target_include_directories(${node} PRIVATE include)
  target_link_libraries(${node} ${OpenCV_LIBS})
  ament_target_dependencies(${node} rclcpp sensor_msgs cv_bridge interfaces)
endforeach()

install(TARGETS barrier_detection_node wall_detection_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
# color_maps.hpp 등 헤더를 다른 패키지 (grid_drive) 에서도 쓰게 설치
install(DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_free_space test/test_free_space.cpp)
  target_include_directories(test_free_space PRIVATE include)
  target_link_libraries(test_free_space ${OpenCV_LIBS})
  ament_add_gtest(test_box_planner test/test_box_planner.cpp)
  target_include_directories(test_box_planner PRIVATE include)
  target_link_libraries(test_box_planner ${OpenCV_LIBS})
  # 녹화에서 뽑은 원본 프레임 (test/data)
  target_compile_definitions(test_box_planner PRIVATE TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
endif()

ament_export_include_directories(include)
ament_package()
