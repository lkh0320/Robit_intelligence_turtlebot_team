#!/usr/bin/env bash
# 노트북용 빌드: GUI(turtle_gui)와 GUI가 쓰는 msg/srv(turtle_interfaces)만 빌드
# 사용법 (레포 루트 어디서 실행해도 됨):
#   bash scripts/build_gui.sh                      # 기본 빌드
#   bash scripts/build_gui.sh --cmake-clean-cache  # 뒤에 붙인 옵션은 colcon build 로 그대로 전달
# 빌드 후: source install/setup.bash
set -e                          # 중간에 실패하면 바로 멈춤
cd "$(dirname "$0")/.."         # 스크립트 위치 기준으로 레포 루트로 이동 (= colcon 워크스페이스)
colcon build --symlink-install --paths ros2/turtle_interfaces gui/turtle_gui "$@"
echo "완료 → source install/setup.bash"
