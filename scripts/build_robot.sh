#!/usr/bin/env bash
# Jetson: ros2/ 아래 패키지 전체 빌드 (GUI 제외)
set -e
cd "$(dirname "$0")/.."
colcon build --symlink-install --base-paths ros2 "$@"
echo "완료 → source install/setup.bash"
