# 미션 상태머신 실행 (파라미터: config/task_planner.yaml)
#   ros2 launch task_planner task_planner.launch.py
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('task_planner'), 'config', 'task_planner.yaml')
    return LaunchDescription([
        Node(
            package='task_planner',
            executable='task_planner_node',
            name='task_planner',
            parameters=[config],
            output='screen',
        ),
    ])
