# path_planner 노드 실행 (파라미터: config/path_planner.yaml)
# bird_eye_view 노드(image_bev)가 같이 떠 있어야 한다
# (vision_bringup/camera_vision.launch.py lane_method:=path 는 둘 다 띄움)
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('path_planner'), 'config',
                          'path_planner.yaml')
    return LaunchDescription([
        Node(
            package='path_planner',
            executable='path_planner_node',
            name='path_planner',
            parameters=[config],
            output='screen',
        ),
    ])
