cmake_minimum_required(VERSION 3.8)
project(vision_bringup)

find_package(ament_cmake REQUIRED)

# config/fastdds_shm.xml: 같은 젯슨 노드끼리 원본 영상을 공유 메모리로 (launch 에서 FASTRTPS_DEFAULT_PROFILES_FILE 로 씀)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})
# launch 에서 카메라 노드가 켜진 뒤 v4l2 설정을 다시 적용할 때 쓴다
install(PROGRAMS camera/turtlebot-camera-apply
  DESTINATION lib/${PROJECT_NAME})

ament_package()
