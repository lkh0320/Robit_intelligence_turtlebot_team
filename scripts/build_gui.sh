#!/usr/bin/env bash
# 노트북: GUI와 GUI가 쓰는 msg/srv만 빌드
set -e
cd "$(dirname "$0")/.."
colcon build --symlink-install --paths ros2/turtle_interfaces gui/turtle_gui "$@"
echo "완료 → source install/setup.bash"
