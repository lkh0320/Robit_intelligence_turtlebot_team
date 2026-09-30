# 카메라 + 비전 노드 전체 실행
#   ros2 launch vision_bringup camera_vision.launch.py
# 옵션 예:
#   video_device:=/dev/video1                        카메라 장치 변경
#   camera_params:=<yaml>                            카메라 ROS 파라미터 파일 적용 (기본: 없음)
#   camera_conf:=<파일>                               v4l2 카메라 설정 (기본 /etc/turtlebot-camera.conf)
# 카메라 밝기/노출은 v4l2-ctl 로 맞춰 /etc/turtlebot-camera.conf 에 저장한다 (camera/README.md).
# v4l2_camera 노드는 켜질 때 장치 설정을 자기 기본값으로 되돌리므로, 같은 파일을 노드 파라미터로도 넘기고
# (GUI 에 보이는 값), 노드가 적용 순서를 지키지 않아 거부되는 항목(색온도 등)을 위해 3초 뒤 다시 적용한다.
#   lane:=false sign:=false obstacle:=false          원하는 노드만 끄기
#   lane_mode:=bev / both                            선 검출 입력 (원근 / BEV / 둘 다 비교)
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(package, condition=None, launch_arguments=None):
    path = os.path.join(get_package_share_directory(package), 'launch', f'{package}.launch.py')
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path), condition=condition,
                                    launch_arguments=(launch_arguments or {}).items())


# v4l2 bool 컨트롤 (ROS 파라미터로는 true/false)
BOOL_CONTROLS = {'white_balance_automatic', 'exposure_dynamic_framerate'}


def read_camera_conf(path):
    """'항목=값' 형식의 v4l2 설정 파일 -> v4l2_camera 파라미터 dict (파일이 없으면 빈 dict)"""
    params = {}
    if not os.path.isfile(path):
        return params
    with open(path) as f:
        for line in f:
            line = line.split('#', 1)[0].strip()
            if '=' not in line:
                continue
            name, value = (x.strip() for x in line.split('=', 1))
            try:
                number = int(value)
            except ValueError:
                continue
            params[name] = bool(number) if name in BOOL_CONTROLS else number
    return params


def camera(context):
    conf_path = LaunchConfiguration('camera_conf').perform(context)
    conf = read_camera_conf(conf_path)
    device = LaunchConfiguration('video_device').perform(context)
    params = [{
        'video_device': device,
        'image_size': [640, 480],
        **conf,
    }]
    # 파일이 지정됐고 실제로 있을 때만 적용 (없으면 카메라 기본값)
    params_file = os.path.expanduser(LaunchConfiguration('camera_params').perform(context))
    if params_file and os.path.isfile(params_file):
        params.append(params_file)
    actions = [Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        parameters=params,
        output='screen',
    )]
    if conf:
        apply = os.path.join(get_package_prefix('vision_bringup'), 'lib', 'vision_bringup',
                             'turtlebot-camera-apply')
        actions.append(TimerAction(period=3.0, actions=[ExecuteProcess(
            cmd=[apply, device], additional_env={'TURTLEBOT_CAMERA_CONF': conf_path},
            output='screen')]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        DeclareLaunchArgument('camera_conf', default_value='/etc/turtlebot-camera.conf',
                              description='v4l2 카메라 설정 파일 (없으면 카메라 기본값)'),
        DeclareLaunchArgument('camera_params', default_value='',
                              description='카메라 파라미터 YAML (기본: 없음, 장치에 저장된 설정 그대로)'),
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
