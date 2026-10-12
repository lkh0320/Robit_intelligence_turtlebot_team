# lane_follower 노드 + 주차 노드 실행 (파라미터: config/lane_follower.yaml, config/parking.yaml)
#   ros2 launch lane_follower lane_follower.launch.py autostart:=true
# 주차 노드는 autostart 에 맞춰 시작 방식을 정한다:
#   autostart:=true  (task_planner 없음) -> 주차 노드가 주차 표지판을 직접 보고 시작
#   autostart:=false (task_planner / GUI) -> PARKING 모드를 받으면 시작
#   parking:=false                        -> 주차 노드를 안 띄움
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory('lane_follower'), 'config',
                          'lane_follower.yaml')
    parking_config = os.path.join(get_package_share_directory('lane_follower'), 'config',
                                  'parking.yaml')
    # autostart 가 true 가 아니면 (= task_planner 나 GUI 가 모드를 정하면) 주차도 그 모드를 따른다
    use_task_planner = ParameterValue(
        PythonExpression(["'", LaunchConfiguration('autostart'), "' != 'true'"]), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('autostart', default_value='false',
                              description='true: control_mode 없이 바로 LANE 주행'),
        DeclareLaunchArgument('parking', default_value='true',
                              description='false: 주차 노드를 띄우지 않음'),
        Node(
            package='lane_follower',
            executable='lane_follower_node',
            name='lane_follower',
            parameters=[config, {'autostart': ParameterValue(LaunchConfiguration('autostart'),
                                                             value_type=bool)}],
            output='screen',
        ),
        Node(
            package='lane_follower',
            executable='parking_node',
            name='parking',
            parameters=[parking_config, {'use_task_planner': use_task_planner}],
            condition=IfCondition(LaunchConfiguration('parking')),
            output='screen',
        ),
    ])
