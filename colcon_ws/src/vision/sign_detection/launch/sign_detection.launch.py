import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('sign_detection'), 'config',
                          'sign_detection.yaml')
    return LaunchDescription([
        Node(
            package='sign_detection',
            executable='sign_detection_node',
            name='sign_detection',
            parameters=[config],
            output='screen',
        ),
    ])
