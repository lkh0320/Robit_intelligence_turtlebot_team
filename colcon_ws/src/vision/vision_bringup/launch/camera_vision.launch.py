# 카메라 + 비전 노드 전체 실행
#   ros2 launch vision_bringup camera_vision.launch.py
# 옵션 예:
#   video_device:=/dev/video1                        카메라 장치 변경
#   camera_params:=~/.ros/turtle_gui_camera.yaml     GUI에서 저장한 카메라 설정 적용
#   lane:=false sign:=false obstacle:=false          원하는 노드만 끄기
#   lane_mode:=bev / both                            선 검출 입력 (원근 / BEV / 둘 다 비교)
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(package, condition=None, launch_arguments=None):
    path = os.path.join(get_package_share_directory(package), 'launch', f'{package}.launch.py')
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path), condition=condition,
                                    launch_arguments=(launch_arguments or {}).items())


def camera(context):
    params = [{
        'video_device': LaunchConfiguration('video_device').perform(context),
        'image_size': [640, 480],
    }]
    # 파일이 지정됐고 실제로 있을 때만 적용 (없으면 카메라 기본값)
    params_file = os.path.expanduser(LaunchConfiguration('camera_params').perform(context))
    if params_file and os.path.isfile(params_file):
        params.append(params_file)
    return [Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        parameters=params,
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        DeclareLaunchArgument('camera_params', default_value='~/.ros/turtle_gui_camera.yaml',
                              description='카메라 파라미터 YAML (없으면 무시)'),
        DeclareLaunchArgument('lane', default_value='true'),
        DeclareLaunchArgument('lane_mode', default_value='perspective',
                              description='선 검출 입력: perspective / bev / both'),
        DeclareLaunchArgument('sign', default_value='true'),
        DeclareLaunchArgument('obstacle', default_value='true'),

        OpaqueFunction(function=camera),
        include('bird_eye_view'),
        include('lane_detection', IfCondition(LaunchConfiguration('lane')),
                {'mode': LaunchConfiguration('lane_mode')}),
        include('sign_detection', IfCondition(LaunchConfiguration('sign'))),
        include('obstacle_detection', IfCondition(LaunchConfiguration('obstacle'))),
    ])
