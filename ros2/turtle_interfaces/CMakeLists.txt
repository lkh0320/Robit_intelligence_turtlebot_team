# turtle_interfaces: 팀 공용 커스텀 msg/srv 패키지
# Jetson(ros2 노드들)과 노트북(turtle_gui) 양쪽에서 모두 빌드해서 씁니다.
# msg/srv를 바꾸면 양쪽 모두 git pull 후 다시 빌드해야 통신이 됩니다. (바꾸기 전 팀 합의 필수)
cmake_minimum_required(VERSION 3.8)
project(turtle_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)   # .msg/.srv → C++/Python 코드 생성기
find_package(std_msgs REQUIRED)                    # std_msgs/Header 를 쓰기 때문에 필요

# 새 msg/srv 파일을 만들면 이 목록에 한 줄 추가하고, docs/ros2_interfaces.md 도 같이 수정
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
