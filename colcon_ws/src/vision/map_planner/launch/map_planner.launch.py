# map_planner 노드 실행 (파라미터: config/map_planner.yaml)
# bird_eye_view 노드(image_bev)와 stm_bridge(dxl_state, 바퀴 이동량)가 같이 떠 있어야 한다
# (vision_bringup/camera_vision.launch.py lane_method:=map 은 카메라 + BEV 와 같이 띄움)
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('map_planner'), 'config',
                          'map_planner.yaml')
    return LaunchDescription([
        Node(
            package='map_planner',
            executable='map_planner_node',
            name='map_planner',
            parameters=[config],
            output='screen',
        ),
    ])
