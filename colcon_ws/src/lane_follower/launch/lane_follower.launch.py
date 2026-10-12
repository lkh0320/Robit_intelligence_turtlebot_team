# lane_follower 노드만 실행 (파라미터: config/lane_follower.yaml)
#   ros2 launch lane_follower lane_follower.launch.py autostart:=true
#   lane_method:=path|sliding|map|hybrid   어떤 선 검출 노드에 맞춘 조향 게인을 쓸지 (기본 path)
#     hybrid: lane_info/sliding + lane_info/path 를 받아 구간에 따라 고른다
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory('lane_follower'), 'config',
                          'lane_follower.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('autostart', default_value='false',
                              description='true: control_mode 없이 바로 LANE 주행'),
        DeclareLaunchArgument('lane_method', default_value='path',
                              description='path / sliding / map / hybrid: lane_follower.yaml 의 같은 이름 게인 사용'),
        Node(
            package='lane_follower',
            executable='lane_follower_node',
            name='lane_follower',
            parameters=[config, {
                'autostart': ParameterValue(LaunchConfiguration('autostart'), value_type=bool),
                'lane_method': LaunchConfiguration('lane_method'),
            }],
            output='screen',
        ),
    ])
