# 격자 계획 주행 시험 (Jetson): 카메라 + 표지판 인식 + STM 브릿지 + grid_drive
#   ros2 launch grid_drive grid_drive_bringup.launch.py
# lane_follower / task_planner 는 안 띄운다 (cmd_vel 을 grid_drive 혼자 보냄)
# 노트북에서 turtle_gui 를 띄우고 (같은 ROS_DOMAIN_ID) 모드 LANE 을 누르면 출발, STOP 이면 정지.
#   GUI 는 grid_drive 가 떠 있으면 "선 · 벡터 검출" 칸에 grid_drive 화면을 띄운다
# 옵션 예:
#   autostart:=true      GUI 없이 바로 주행
#   use_road:=false      도로 모델 없이 색 방식만
#   lane:=true           lane_detection 도 띄움 (GUI BEV / 선 검출 화면 비교용, CPU 더 씀)
#   obstacle:=true       wall_detection 등 장애물 노드
#   stm:=false           STM 없이 (cmd_vel 만 확인)
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
        # 원본 영상이 Fast DDS 기본 공유 메모리 칸에 안 들어가 프레임이 버려지는 것 (manual_bringup 과 같음)
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', os.path.join(
            get_package_share_directory('vision_bringup'), 'config', 'fastdds_shm.xml')),
        DeclareLaunchArgument('autostart', default_value='false'),
        DeclareLaunchArgument('use_road', default_value='true'),
        DeclareLaunchArgument('lane', default_value='false'),
        DeclareLaunchArgument('sign', default_value='true'),
        DeclareLaunchArgument('obstacle', default_value='false'),
        DeclareLaunchArgument('stm', default_value='true'),

        IncludeLaunchDescription(
            launch_file('vision_bringup', 'camera_vision.launch.py'),
            launch_arguments={
                'lane': LaunchConfiguration('lane'),
                'sign': LaunchConfiguration('sign'),
                'obstacle': LaunchConfiguration('obstacle'),
            }.items()),
        IncludeLaunchDescription(launch_file('stm', 'stm_bridge.launch.py'),
                                 condition=IfCondition(LaunchConfiguration('stm'))),
        IncludeLaunchDescription(
            launch_file('grid_drive', 'grid_drive.launch.py'),
            launch_arguments={
                'autostart': LaunchConfiguration('autostart'),
                'use_road': LaunchConfiguration('use_road'),
            }.items()),
    ])
