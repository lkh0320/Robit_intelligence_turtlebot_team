from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sign_detection',
            executable='sign_detection_node',
            name='sign_detection',
            output='screen',
        ),
    ])
