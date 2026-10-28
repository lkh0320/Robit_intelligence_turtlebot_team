# 격자 계획 주행 (Jetson): 카메라 + 표지판 인식 + STM 브릿지 + grid_drive + 주차 노드 + task_planner
#   ros2 launch grid_drive grid_drive_bringup.launch.py
# lane_follower 는 안 띄운다: LANE/AVOID 주행은 grid_drive, PARKING 은 parking_node 가 cmd_vel 을 보냄
# 노트북 turtle_gui (같은 ROS_DOMAIN_ID) 의 "자율주행 시작" = task_planner 미션 (신호등 -> 장애물 -> 주차 -> 차단바).
#   모드 LANE 버튼만 누르면 미션 없이 grid_drive 주행만, STOP 이면 정지.
#   GUI 는 grid_drive 가 떠 있으면 "선 · 벡터 검출" 칸에 grid_drive 화면을 띄운다
# 옵션 예:
#   autostart:=true      GUI 없이 바로 주행
#   use_road:=false      도로 모델 없이 색 방식만
#   lane:=true           lane_detection 도 띄움 (GUI BEV / 선 검출 화면 비교용, CPU 더 씀)
#   obstacle:=true       wall_detection 등 장애물 노드
#   stm:=false           STM 없이 (cmd_vel 만 확인)
#   mission:=false       task_planner / 주차 노드 없이 grid_drive 만 (예전 시험 방식)
#   start_state:=TO_PARKING   "자율주행 시작" 때 이 구간부터 (기본 WAIT_LIGHT)
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
        DeclareLaunchArgument('mission', default_value='true'),
        DeclareLaunchArgument('start_state', default_value='WAIT_LIGHT'),

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

        # 주차: parking_node 가 표지판을 보고 칸에 들어갔다 나옴 (그동안 control_mode PARKING -> grid_drive 멈춤)
        Node(
            package='lane_follower',
            executable='parking_node',
            name='parking',
            parameters=[os.path.join(get_package_share_directory('lane_follower'), 'config',
                                     'parking.yaml'),
                        {'use_task_planner': True}],
            condition=IfCondition(LaunchConfiguration('mission')),
            output='screen',
        ),
        # 미션 상태머신: GUI "자율주행 시작" (mission_start) 을 받을 때까지 대기
        IncludeLaunchDescription(
            launch_file('task_planner', 'task_planner.launch.py'),
            launch_arguments={
                'wait_start': 'true',
                'start_state': LaunchConfiguration('start_state'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('mission'))),
    ])
