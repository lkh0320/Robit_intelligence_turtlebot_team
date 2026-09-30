#!/bin/sh
# Jetson 에 카메라 자동 설정 설치 (한 번만): sudo ./install.sh
set -e
cd "$(dirname "$0")"
install -m 755 turtlebot-camera-apply turtlebot-camera-save /usr/local/bin/
install -m 644 99-turtlebot-camera.rules /etc/udev/rules.d/
udevadm control --reload-rules
udevadm trigger --action=add --subsystem-match=video4linux
echo "설치 완료. /dev/turtlebot_camera:"
ls -l /dev/turtlebot_camera || true
