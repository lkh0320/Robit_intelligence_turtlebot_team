# turtle_gui: 노트북에서 실행하는 Qt5 GUI 패키지
# 빌드: 레포 루트에서 bash scripts/build_gui.sh
# 실행: ros2 run turtle_gui turtle_gui
cmake_minimum_required(VERSION 3.8)
project(turtle_gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# Qt 자동 처리
#  AUTOMOC: Q_OBJECT 가 있는 클래스(QNode, MainWindow)의 signal/slot 코드를 자동 생성
#  AUTOUIC: ui/main_window.ui (Qt Designer 파일) → ui_main_window.h 자동 생성
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
find_package(Qt5 REQUIRED COMPONENTS Widgets)   # 없으면: sudo apt install qtbase5-dev

# 실행 파일 turtle_gui
# 헤더(.hpp)와 .ui 도 목록에 넣어야 AUTOMOC/AUTOUIC 가 찾아서 처리함
# 새 소스 파일을 추가하면 여기에 한 줄 추가
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

# ros2 run turtle_gui <이름> 으로 실행할 수 있도록 lib/turtle_gui 에 설치
install(TARGETS turtle_gui DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/fake_robot.py DESTINATION lib/${PROJECT_NAME})   # 파이썬 스크립트는 실행 권한 필요

ament_package()
