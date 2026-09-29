# jetson/

Jetson Orin Nano **환경 설정 방법**을 모아 둡니다. Jetson에서 도는 코드는 `ros2/`에 있습니다.

## 1. JetPack / ROS 2 버전 확인

```bash
cat /etc/nv_tegra_release        # JetPack(L4T) 버전
lsb_release -a                   # Ubuntu 버전
```

| Ubuntu | 기본 ROS 2 |
|---|---|
| 22.04 (JetPack 6) | Humble |
| 24.04 | Jazzy |

노트북은 Jazzy(Ubuntu 24.04)입니다. Jetson이 22.04라면 둘 중 하나로 맞추세요.

- Jetson에서 Docker로 Jazzy 실행
- 또는 노트북까지 모두 Humble로 통일

버전이 다른 배포판끼리의 통신은 공식 지원이 아니라서, 커스텀 msg에서 문제가 날 수 있습니다. **결정 후 이 문서에 기록해 주세요.**

## 2. 필수 패키지

```bash
sudo apt install ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-image-transport \
                 ros-$ROS_DISTRO-image-transport-plugins ros-$ROS_DISTRO-camera-calibration \
                 python3-colcon-common-extensions v4l-utils
sudo usermod -aG dialout,video $USER      # 시리얼, 카메라 권한 (재로그인 필요)
```

## 3. 장치 이름 고정 (udev)

USB를 꽂는 순서에 따라 `/dev/ttyACM0`, `/dev/ttyUSB0`이 바뀌므로 `/dev/stm32`로 고정합니다.

```bash
lsusb                                   # STM32 쪽 장치의 ID xxxx:yyyy 확인
sudo nano /etc/udev/rules.d/99-turtlebot.rules
```

```text
SUBSYSTEM=="tty", ATTRS{idVendor}=="xxxx", ATTRS{idProduct}=="yyyy", SYMLINK+="stm32", MODE="0666"
```

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/stm32
```

카메라 확인: `v4l2-ctl --list-devices`

## 4. 네트워크

- 노트북과 같은 Wi-Fi, 같은 `ROS_DOMAIN_ID` (`~/.bashrc`)
- 확인: Jetson에서 `ros2 run demo_nodes_cpp talker`, 노트북에서 `ros2 run demo_nodes_cpp listener`
