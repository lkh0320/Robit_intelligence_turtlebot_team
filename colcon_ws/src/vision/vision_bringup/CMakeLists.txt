cmake_minimum_required(VERSION 3.8)
project(vision_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
# launch 에서 카메라 노드가 켜진 뒤 v4l2 설정을 다시 적용할 때 쓴다
install(PROGRAMS camera/turtlebot-camera-apply
  DESTINATION lib/${PROJECT_NAME})

ament_package()
