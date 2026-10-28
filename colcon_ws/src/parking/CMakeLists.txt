cmake_minimum_required(VERSION 3.8)
project(parking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

# 주차 노드: 주차 표지판을 보면 칸에 들어갔다 나와서 다시 grid_drive 주행으로 돌려준다
add_executable(parking_node src/parking_node.cpp)
target_link_libraries(parking_node ${OpenCV_LIBS})
ament_target_dependencies(parking_node rclcpp geometry_msgs std_msgs sensor_msgs nav_msgs interfaces cv_bridge)

install(TARGETS parking_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_package()
