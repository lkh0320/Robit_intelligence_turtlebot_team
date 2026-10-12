# path_planner 노드 실행 (파라미터: config/path_planner.yaml)
# bird_eye_view 노드(image_bev)가 같이 떠 있어야 한다
# (vision_bringup/camera_vision.launch.py lane_method:=path 는 둘 다 띄움)
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory('path_planner'), 'config',
                          'path_planner.yaml')
    return LaunchDescription([
        # lane_method:=hybrid 에서 lane_detection 과 path_planner 를 같이 띄울 때 토픽을 나눈다
        DeclareLaunchArgument('lane_topic', default_value='lane_info'),
        DeclareLaunchArgument('debug_topic', default_value='vision/lane_debug/compressed'),
        # true: 공사 구간 / 갈림길을 lane_follower 의 drive_hint 로 판단 (hybrid)
        DeclareLaunchArgument('use_drive_hint', default_value='false'),
        Node(
            package='path_planner',
            executable='path_planner_node',
            name='path_planner',
            parameters=[config, {'use_drive_hint': ParameterValue(
                LaunchConfiguration('use_drive_hint'), value_type=bool)}],
            remappings=[('lane_info', LaunchConfiguration('lane_topic')),
                        ('vision/lane_debug/compressed', LaunchConfiguration('debug_topic'))],
            output='screen',
        ),
    ])
