cmake_minimum_required(VERSION 3.8)
project(lane_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

add_executable(lane_detection_node src/lane_detection_node.cpp)
target_link_libraries(lane_detection_node ${OpenCV_LIBS})
ament_target_dependencies(lane_detection_node rclcpp sensor_msgs interfaces)

install(TARGETS lane_detection_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
