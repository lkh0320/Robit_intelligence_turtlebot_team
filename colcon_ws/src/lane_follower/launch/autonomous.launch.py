# 자율주행 전체 실행: 카메라 + 비전 + STM 브릿지 + 주행 제어 (+ 미션 상태머신)
#   ros2 launch lane_follower autonomous.launch.py
# 옵션 예:
#   lane_method:=sliding     선 검출 방식 (기본 path = path_planner, sliding = lane_detection,
#                            map = map_planner 2D 지역 지도)
#   task_planner:=true       미션 상태머신 사용 (신호등 GREEN 을 기다렸다가 출발, 모드 전환)
#                            false(기본)면 lane_follower 가 켜지자마자 LANE 주행 (라인 추적 시험용)
#   stm:=false               STM 브릿지 없이 (cmd_vel 만 확인할 때)
# 그 외 인자(video_device, sign, obstacle ...)는 vision_bringup/camera_vision.launch.py 로 넘어간다
# 멈추기: GUI 의 STOP 모드 버튼, 또는 보드 S1 스위치
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def launch_file(package, name):
    return PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory(package), 'launch', name))


def generate_launch_description():
    use_task_planner = LaunchConfiguration('task_planner')
    return LaunchDescription([
        DeclareLaunchArgument('lane_method', default_value='path',
                              description='path: path_planner / sliding: lane_detection / '
                                          'map: map_planner'),
        DeclareLaunchArgument('task_planner', default_value='false'),
        DeclareLaunchArgument('stm', default_value='true'),

        IncludeLaunchDescription(
            launch_file('vision_bringup', 'camera_vision.launch.py'),
            launch_arguments={'lane_method': LaunchConfiguration('lane_method')}.items()),
        IncludeLaunchDescription(launch_file('stm', 'stm_bridge.launch.py'),
                                 condition=IfCondition(LaunchConfiguration('stm'))),
        IncludeLaunchDescription(launch_file('task_planner', 'task_planner.launch.py'),
                                 condition=IfCondition(use_task_planner)),
        # task_planner 가 없으면 control_mode 를 줄 노드가 없으므로 바로 주행
        IncludeLaunchDescription(
            launch_file('lane_follower', 'lane_follower.launch.py'),
            launch_arguments={'autostart': PythonExpression(
                ["'false' if '", use_task_planner, "' == 'true' else 'true'"])}.items()),
    ])
