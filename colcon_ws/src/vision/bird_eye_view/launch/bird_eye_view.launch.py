import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('bird_eye_view'), 'config',
                          'bird_eye_view.yaml')
    return LaunchDescription([
        Node(
            package='bird_eye_view',
            executable='bird_eye_view_node',
            name='bird_eye_view',
            parameters=[config],
            output='screen',
        ),
    ])
