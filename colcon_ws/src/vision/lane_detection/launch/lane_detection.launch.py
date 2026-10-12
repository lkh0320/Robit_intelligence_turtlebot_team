# lane_detection 노드 실행 (파라미터: config/lane_detection.yaml)
# bird_eye_view 노드(image_bev)가 같이 떠 있어야 한다 (vision_bringup/camera_vision.launch.py 는 둘 다 띄움)
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('lane_detection'), 'config',
                          'lane_detection.yaml')
    return LaunchDescription([
        # lane_method:=hybrid 에서 lane_detection 과 path_planner 를 같이 띄울 때 토픽을 나눈다
        DeclareLaunchArgument('lane_topic', default_value='lane_info'),
        DeclareLaunchArgument('debug_topic', default_value='vision/lane_debug/compressed'),
        Node(
            package='lane_detection',
            executable='lane_detection_node',
            name='lane_detection',
            parameters=[config],
            remappings=[('lane_info', LaunchConfiguration('lane_topic')),
                        ('vision/lane_debug/compressed', LaunchConfiguration('debug_topic'))],
            output='screen',
        ),
    ])
