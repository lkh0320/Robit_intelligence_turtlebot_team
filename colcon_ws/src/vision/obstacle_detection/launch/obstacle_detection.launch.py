import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('obstacle_detection'), 'config',
                          'obstacle_detection.yaml')
    return LaunchDescription([
        Node(
            package='obstacle_detection',
            executable='obstacle_detection_node',
            name='obstacle_detection',
            parameters=[config],
            output='screen',
        ),
    ])
