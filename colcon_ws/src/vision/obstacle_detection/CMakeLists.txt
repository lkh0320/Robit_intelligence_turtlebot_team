cmake_minimum_required(VERSION 3.8)
project(obstacle_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

# 차단바, 벽(지그재그 구간) 은 따로 켜고 끌 수 있게 노드를 나눈다
foreach(node barrier_detection_node wall_detection_node)
  add_executable(${node} src/${node}.cpp)
  target_link_libraries(${node} ${OpenCV_LIBS})
  ament_target_dependencies(${node} rclcpp sensor_msgs interfaces)
endforeach()

install(TARGETS barrier_detection_node wall_detection_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
