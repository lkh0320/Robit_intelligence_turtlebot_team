import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('lane_detection'), 'config',
                          'lane_detection.yaml')
    return LaunchDescription([
        Node(
            package='lane_detection',
            executable='lane_detection_node',
            name='lane_detection',
            parameters=[config],
            output='screen',
        ),
    ])
