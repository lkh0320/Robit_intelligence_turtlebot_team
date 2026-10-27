# 격자 계획 주행 노드만 (카메라 / stm_bridge / sign_detection 은 따로)
#   ros2 launch grid_drive grid_drive.launch.py autostart:=true
#   디버그 화면: grid_drive/debug/compressed
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('grid_drive'), 'config', 'grid_drive.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('autostart', default_value='false'),
        DeclareLaunchArgument('use_road', default_value='true'),
        Node(
            package='grid_drive',
            executable='grid_drive_node',
            name='grid_drive',
            output='screen',
            parameters=[config, {
                'autostart': LaunchConfiguration('autostart'),
                'use_road': LaunchConfiguration('use_road'),
            }],
        ),
    ])
