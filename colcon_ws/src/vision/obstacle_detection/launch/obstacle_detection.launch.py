from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='obstacle_detection',
            executable='barrier_detection_node',
            name='barrier_detection',
            output='screen',
        ),
        Node(
            package='obstacle_detection',
            executable='wall_detection_node',
            name='wall_detection',
            output='screen',
        ),
    ])
