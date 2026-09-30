# mode:=perspective  원근 영상(image_raw)으로 검출 -> lane_info            (기본)
# mode:=bev          BEV 영상(image_bev)으로 검출  -> lane_info
# mode:=both         둘 다 실행해서 비교. 원근 -> lane_info, BEV -> lane_info_bev,
#                    BEV 화면은 vision/lane_bev_debug/compressed
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    config = os.path.join(get_package_share_directory('lane_detection'), 'config',
                          'lane_detection.yaml')
    bev_params = {'bev': True, 'image_topic': 'image_bev'}
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in ('perspective', 'bev', 'both'):
        raise RuntimeError(f'mode 는 perspective / bev / both 중 하나: {mode}')
    result = []
    if mode in ('perspective', 'both'):
        result.append(Node(
            package='lane_detection', executable='lane_detection_node', name='lane_detection',
            parameters=[config], output='screen'))
    if mode == 'bev':
        result.append(Node(
            package='lane_detection', executable='lane_detection_node', name='lane_detection',
            parameters=[config, bev_params], output='screen'))
    if mode == 'both':
        result.append(Node(
            package='lane_detection', executable='lane_detection_node', name='lane_detection_bev',
            parameters=[config, bev_params], output='screen',
            remappings=[('lane_info', 'lane_info_bev'),
                        ('vision/lane_debug/compressed', 'vision/lane_bev_debug/compressed')]))
    return result


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='perspective',
                              description='perspective / bev / both'),
        OpaqueFunction(function=nodes),
    ])
