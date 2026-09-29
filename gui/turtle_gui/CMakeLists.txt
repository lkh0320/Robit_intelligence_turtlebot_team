cmake_minimum_required(VERSION 3.8)
project(turtle_gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(turtle_interfaces REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_executable(turtle_gui
  src/main.cpp
  src/main_window.cpp
  src/qnode.cpp
  include/turtle_gui/main_window.hpp
  include/turtle_gui/qnode.hpp
  include/turtle_gui/gui_types.hpp
  ui/main_window.ui
)
target_include_directories(turtle_gui PRIVATE include)
target_link_libraries(turtle_gui Qt5::Widgets)
ament_target_dependencies(turtle_gui
  rclcpp
  rcl_interfaces
  sensor_msgs
  geometry_msgs
  std_srvs
  turtle_interfaces
)

install(TARGETS turtle_gui DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/fake_robot.py DESTINATION lib/${PROJECT_NAME})

ament_package()
