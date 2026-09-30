cmake_minimum_required(VERSION 3.8)
project(stm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interfaces REQUIRED)

add_executable(stm_bridge_node src/stm_bridge_node.cpp src/serial_port.cpp)
target_include_directories(stm_bridge_node PRIVATE include)
ament_target_dependencies(stm_bridge_node rclcpp geometry_msgs interfaces)

install(TARGETS stm_bridge_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_include_directories(test_protocol PRIVATE include)
endif()

ament_package()
