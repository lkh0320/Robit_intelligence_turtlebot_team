cmake_minimum_required(VERSION 3.8)
project(bird_eye_view)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)   # third_party/vision_opencv 에서 OpenCV 4.8 로 빌드한 것
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

add_executable(bird_eye_view_node src/bird_eye_view_node.cpp)
target_link_libraries(bird_eye_view_node ${OpenCV_LIBS})
ament_target_dependencies(bird_eye_view_node rclcpp sensor_msgs cv_bridge)

install(TARGETS bird_eye_view_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
