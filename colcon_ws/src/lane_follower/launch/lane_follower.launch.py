# lane_follower 노드만 실행 (파라미터: config/lane_follower.yaml)
#   ros2 launch lane_follower lane_follower.launch.py autostart:=true
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
        Node(
            package='lane_follower',
            executable='lane_follower_node',
            name='lane_follower',
            parameters=[config, {'autostart': ParameterValue(LaunchConfiguration('autostart'),
                                                             value_type=bool)}],
            output='screen',
        ),
    ])
