# 수동 조작/모니터링 테스트용: 카메라 + 비전 + STM 브릿지 실행 (Jetson 쪽)
# (lane_follower, task_planner 는 안 띄움 — 자율주행 테스트는 lane_follower/autonomous.launch.py 사용)
# turtle_gui 는 노트북에서 따로 실행 (같은 ROS_DOMAIN_ID 면 네트워크로 토픽을 그대로 받음):
#   ros2 run turtle_gui turtle_gui
#   ros2 launch turtle_gui manual_bringup.launch.py
# 옵션 예:
#   sign:=false obstacle:=false        표지판/장애물 인식 노드 끄기
#   lane_method:=path                  차선 검출 방식 변경 (sliding/path/map)
#   stm:=false                         STM 브릿지 없이 (하드웨어 연결 안 됐을 때)
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_file(package, name):
    return PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory(package), 'launch', name))


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sign', default_value='true'),
        DeclareLaunchArgument('obstacle', default_value='true'),
        DeclareLaunchArgument('lane_method', default_value='sliding',
                              description='sliding: lane_detection / path: path_planner / '
                                          'map: map_planner'),
        DeclareLaunchArgument('stm', default_value='true'),

        IncludeLaunchDescription(
            launch_file('vision_bringup', 'camera_vision.launch.py'),
            launch_arguments={
                'sign': LaunchConfiguration('sign'),
                'obstacle': LaunchConfiguration('obstacle'),
                'lane_method': LaunchConfiguration('lane_method'),
            }.items()),
        IncludeLaunchDescription(launch_file('stm', 'stm_bridge.launch.py'),
                                 condition=IfCondition(LaunchConfiguration('stm'))),
    ])
