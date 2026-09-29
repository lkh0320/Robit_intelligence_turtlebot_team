cmake_minimum_required(VERSION 3.8)
project(turtle_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PsdArray.msg"
  "msg/MotorCommand.msg"
  "msg/MotorState.msg"
  "msg/RobotState.msg"
  "msg/VisionResult.msg"
  "srv/SetMode.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
