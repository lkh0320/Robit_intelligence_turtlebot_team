# 장애물 인식 노드 실행: 벽(wall_detection)
#   barrier_detection_node 는 미구현 빈 껍데기였고, 차단바 인식은 sign_detection 이 대신 발행하므로 뺌
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='obstacle_detection',
            executable='wall_detection_node',
            name='wall_detection',
            output='screen',
        ),
    ])
