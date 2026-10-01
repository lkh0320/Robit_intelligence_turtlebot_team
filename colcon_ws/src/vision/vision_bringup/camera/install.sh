#!/bin/sh
# Jetson 에 카메라 자동 설정 설치 (한 번만): sudo ./install.sh
set -e
cd "$(dirname "$0")"
# 스크립트는 /usr/local/bin 에 (udev 규칙이 이 경로를 실행), 규칙은 /etc/udev/rules.d 에 복사
install -m 755 turtlebot-camera-apply turtlebot-camera-save /usr/local/bin/
install -m 644 99-turtlebot-camera.rules /etc/udev/rules.d/
# 규칙을 다시 읽고, 이미 꽂혀 있는 카메라에도 바로 적용 (다시 꽂을 필요 없음)
udevadm control --reload-rules
udevadm trigger --action=add --subsystem-match=video4linux
echo "설치 완료. /dev/turtlebot_camera:"
ls -l /dev/turtlebot_camera || true
