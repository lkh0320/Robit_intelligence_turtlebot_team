# 카메라 자동 설정 (USB Live camera 0c45:636b)

카메라 밝기/노출은 `v4l2-ctl` 로 직접 맞추고 Jetson 에 저장한다.
저장한 값은 카메라를 꽂을 때와 부팅할 때 udev 가 자동으로 적용한다.
(GUI 카메라 파라미터 패널로는 바꾸지 않는다)

## 설치 (Jetson 에서 한 번만)
```bash
cd ~/Robit_intelligence_turtlebot_team/colcon_ws/src/vision/vision_bringup/camera
sudo ./install.sh
```
- `/usr/local/bin/turtlebot-camera-apply`, `turtlebot-camera-save` 설치
- `/etc/udev/rules.d/99-turtlebot-camera.rules` 설치
- 카메라가 `/dev/turtlebot_camera` 고정 이름으로도 잡힌다 (video 번호가 바뀌어도 같음)

## 값 맞추고 저장
```bash
v4l2-ctl -d /dev/turtlebot_camera --list-ctrls                 # 항목, 범위, 현재값
v4l2-ctl -d /dev/turtlebot_camera -c auto_exposure=1            # 먼저 자동 노출 끄기 (1 수동 / 3 자동)
v4l2-ctl -d /dev/turtlebot_camera -c exposure_time_absolute=157 # 1~5000, 클수록 밝음
v4l2-ctl -d /dev/turtlebot_camera -c white_balance_automatic=0  # 먼저 자동 화이트밸런스 끄기
v4l2-ctl -d /dev/turtlebot_camera -c white_balance_temperature=4600
turtlebot-camera-save                                           # /etc/turtlebot-camera.conf 에 저장
```
- 저장 파일은 `항목=값` 한 줄씩이라 직접 고쳐도 된다: `sudo nano /etc/turtlebot-camera.conf`
- 바로 다시 적용: `turtlebot-camera-apply`
- 적용 기록: `journalctl -t turtlebot-camera`

## 자동으로 되돌리기
```bash
v4l2-ctl -d /dev/turtlebot_camera -c auto_exposure=3,white_balance_automatic=1
turtlebot-camera-save
```

## 카메라 노드(v4l2_camera)와의 관계
`v4l2_camera` 노드는 켜질 때 장치 설정을 자기 기본값(자동 노출 등)으로 되돌린다.
그래서 `ros2 launch vision_bringup camera_vision.launch.py` 는 `/etc/turtlebot-camera.conf` 를
읽어 노드 파라미터로 넘기고, 3초 뒤 `turtlebot-camera-apply` 로 한 번 더 적용한다.
- 다른 파일을 쓰려면 `camera_conf:=<파일>`
- `ros2 run v4l2_camera v4l2_camera_node` 로 따로 켜면 저장값이 덮어써진다. 그때는 켠 뒤
  `turtlebot-camera-apply` 를 실행한다.
