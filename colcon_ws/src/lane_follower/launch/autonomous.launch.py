# 자율주행 전체 실행: 카메라 + 비전 + STM 브릿지 + 주행 제어 (+ 미션 상태머신)
#   ros2 launch lane_follower autonomous.launch.py
# 옵션 예:
#   task_planner:=true       미션 상태머신 사용 (신호등 GREEN 을 기다렸다가 출발, 모드 전환)
#                            false(기본)면 lane_follower 가 켜지자마자 LANE 주행 (라인 추적 시험용)
#   stm:=false               STM 브릿지 없이 (cmd_vel 만 확인할 때)
# 그 외 인자(video_device, sign, obstacle ...)는 vision_bringup/camera_vision.launch.py 로 넘어간다
# 멈추기: GUI 의 STOP 모드 버튼, 또는 보드 S1 스위치
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def launch_file(package, name):
    return PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory(package), 'launch', name))


def generate_launch_description():
    use_task_planner = LaunchConfiguration('task_planner')
    return LaunchDescription([
        # 원본 영상(0.9MB)이 Fast DDS 기본 공유 메모리 칸(0.5MB)에 안 들어가 UDP 조각으로 가면서 프레임이 버려짐
        # (녹화까지 켜면 BEV/lane_info 5~9Hz, 1.2s 끊김). 16MB 칸 프로필로 30Hz. 노트북 GUI 는 UDP 로 그대로 통신
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', os.path.join(
            get_package_share_directory('vision_bringup'), 'config', 'fastdds_shm.xml')),
        DeclareLaunchArgument('task_planner', default_value='false'),
        DeclareLaunchArgument('stm', default_value='true'),

        IncludeLaunchDescription(launch_file('vision_bringup', 'camera_vision.launch.py')),
        IncludeLaunchDescription(launch_file('stm', 'stm_bridge.launch.py'),
                                 condition=IfCondition(LaunchConfiguration('stm'))),
        IncludeLaunchDescription(launch_file('task_planner', 'task_planner.launch.py'),
                                 condition=IfCondition(use_task_planner)),
        # task_planner 가 없으면 control_mode 를 줄 노드가 없으므로 바로 주행
        IncludeLaunchDescription(
            launch_file('lane_follower', 'lane_follower.launch.py'),
            launch_arguments={
                'autostart': PythonExpression(
                    ["'false' if '", use_task_planner, "' == 'true' else 'true'"]),
                'task_planner': use_task_planner,
            }.items()),
    ])
