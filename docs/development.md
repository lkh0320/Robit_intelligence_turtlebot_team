# 개발 / 디버깅

## 빌드

| 위치 | 명령 |
|---|---|
| Jetson | `bash scripts/build_robot.sh` |
| 노트북 | `bash scripts/build_gui.sh` |
| 패키지 하나만 | `colcon build --symlink-install --base-paths ros2 --packages-select turtle_vision` |
| 깨끗하게 다시 | `rm -rf build install log` 후 다시 빌드 |

빌드 후에는 항상 `source install/setup.bash`.

## 실행

```bash
# Jetson
ros2 launch turtle_bringup robot.launch.py

# 노트북
ros2 run turtle_gui turtle_gui
```

## 자주 쓰는 확인 명령

```bash
ros2 node list                         # 노드가 떠 있는지
ros2 topic list
ros2 topic hz /sensor/psd              # STM32 데이터가 들어오는지 (50 Hz 근처)
ros2 topic echo /robot/state
ros2 param list /vision_node
ros2 param set /control_node max_linear_mps 0.1
ros2 param dump /vision_node           # 튜닝 값 → config/vision.yaml 반영
rqt_graph                              # 노드 연결 그림
ros2 run rqt_image_view rqt_image_view # 영상 확인
```

## 문제별 체크

| 증상 | 확인할 것 |
|---|---|
| 노트북에서 토픽이 안 보임 | 같은 Wi-Fi? `ROS_DOMAIN_ID` 같은지? 방화벽? talker/listener 테스트 |
| `/sensor/psd` 안 들어옴 | `ls -l /dev/stm32`, dialout 권한, 보드레이트, STATUS 패킷의 `protocol_version` |
| 모터가 가다가 멈춤 | STM32 `ERR_CMD_TIMEOUT` (Jetson 명령 주기가 200 ms보다 느린지) |
| 영상이 느림 | GUI가 `compressed` 토픽을 구독하는지, `jpeg_quality`, 해상도 |
| 카메라 안 열림 | `v4l2-ctl --list-devices`, video 그룹 권한 |

## 기록 (rosbag)

```bash
ros2 bag record -o bags/test_01 /sensor/psd /motor/state /robot/state /vision/result
ros2 bag play bags/test_01
```

`bags/`는 git에서 제외됩니다. 공유가 필요하면 Google Drive에 올리세요.
