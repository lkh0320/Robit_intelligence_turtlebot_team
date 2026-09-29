#!/usr/bin/env bash
# Jetson용 빌드: ros2/ 아래의 모든 패키지 빌드 (GUI는 제외)
# 사용법 (레포 루트 어디서 실행해도 됨):
#   bash scripts/build_robot.sh                                   # 전체 빌드
#   bash scripts/build_robot.sh --packages-select turtle_vision   # 뒤에 붙인 옵션은 colcon build 로 그대로 전달
# 빌드 후: source install/setup.bash
set -e                          # 중간에 실패하면 바로 멈춤
cd "$(dirname "$0")/.."         # 스크립트 위치 기준으로 레포 루트로 이동 (= colcon 워크스페이스)
colcon build --symlink-install --base-paths ros2 "$@"
echo "완료 → source install/setup.bash"
