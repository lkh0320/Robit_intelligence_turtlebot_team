cmake_minimum_required(VERSION 3.8)
project(interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LaneInfo.msg"
  "msg/TrafficLight.msg"
  "msg/StopLine.msg"
  "msg/Sign.msg"
  "msg/Barrier.msg"
  "msg/ParkingSpot.msg"
  "msg/ControlMode.msg"
  "msg/PsdArray.msg"
  "msg/DxlState.msg"
  "msg/WallObstacle.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
