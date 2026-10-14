# 수동 조작/모니터링 테스트용: 카메라 + 비전 + STM 브릿지 + lane_follower 실행 (Jetson 쪽)
# (task_planner 는 안 띄움 — lane_follower 는 떠 있어도 control_mode 가 LANE/AVOID 일 때만
#  움직이므로 GUI 로 모드만 바꾸면 그때 바로 반응한다)
# turtle_gui 는 노트북에서 따로 실행 (같은 ROS_DOMAIN_ID 면 네트워크로 토픽을 그대로 받음):
#   ros2 run turtle_gui turtle_gui
#   ros2 launch turtle_gui manual_bringup.launch.py
# 옵션 예:
#   sign:=false obstacle:=false        표지판/장애물 인식 노드 끄기
#   stm:=false                         STM 브릿지 없이 (하드웨어 연결 안 됐을 때)
#   lane_follower:=false               lane_follower 는 안 띄움
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_file(package, name):
    return PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory(package), 'launch', name))


def generate_launch_description():
    return LaunchDescription([
        # 원본 영상(0.9MB)이 Fast DDS 기본 공유 메모리 칸(0.5MB)에 안 들어가 UDP 조각으로 가면서 프레임이 버려짐
        # (녹화까지 켜면 BEV/lane_info 5~9Hz, 1.2s 끊김). 16MB 칸 프로필로 30Hz. 노트북 GUI 는 UDP 로 그대로 통신
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', os.path.join(
            get_package_share_directory('vision_bringup'), 'config', 'fastdds_shm.xml')),
        DeclareLaunchArgument('sign', default_value='true'),
        DeclareLaunchArgument('obstacle', default_value='true'),
        DeclareLaunchArgument('stm', default_value='true'),
        DeclareLaunchArgument('lane_follower', default_value='true'),

        IncludeLaunchDescription(
            launch_file('vision_bringup', 'camera_vision.launch.py'),
            launch_arguments={
                'sign': LaunchConfiguration('sign'),
                'obstacle': LaunchConfiguration('obstacle'),
            }.items()),
        IncludeLaunchDescription(launch_file('stm', 'stm_bridge.launch.py'),
                                 condition=IfCondition(LaunchConfiguration('stm'))),
        IncludeLaunchDescription(launch_file('lane_follower', 'lane_follower.launch.py'),
                                 condition=IfCondition(LaunchConfiguration('lane_follower'))),
    ])
