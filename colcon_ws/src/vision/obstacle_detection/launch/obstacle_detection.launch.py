# 장애물 인식 노드 2개 실행: 차단바(barrier_detection) + 벽(wall_detection)
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
