# 미션 상태머신 실행 (파라미터: config/task_planner.yaml)
#   ros2 launch task_planner task_planner.launch.py
#   start_state:=TO_BARRIER   특정 구간부터 테스트 (WAIT_LIGHT/TO_OBSTACLE/AVOIDING/TO_PARKING/
#                             PARKING/TO_BARRIER/BARRIER_WAIT/FINISHING/DONE, 기본 WAIT_LIGHT)
#   wait_start:=true          GUI "자율주행 시작"(mission_start) 을 받을 때까지 대기
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('task_planner'), 'config', 'task_planner.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('start_state', default_value='WAIT_LIGHT',
                              description='테스트용: 이 상태부터 바로 시작'),
        DeclareLaunchArgument('wait_start', default_value='false',
                              description='true: mission_start 를 받을 때까지 대기'),
        Node(
            package='task_planner',
            executable='task_planner_node',
            name='task_planner',
            parameters=[config, {
                'start_state': LaunchConfiguration('start_state'),
                'wait_start': ParameterValue(LaunchConfiguration('wait_start'), value_type=bool),
            }],
            output='screen',
        ),
    ])
