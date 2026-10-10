cmake_minimum_required(VERSION 3.8)
project(turtle_gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_executable(turtle_gui
  src/main.cpp
  src/main_window.cpp
  include/turtle_gui/main_window.hpp
)
target_include_directories(turtle_gui PRIVATE include)
target_link_libraries(turtle_gui Qt5::Widgets)
ament_target_dependencies(turtle_gui rclcpp sensor_msgs geometry_msgs interfaces)

install(TARGETS turtle_gui
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/fake_robot.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
