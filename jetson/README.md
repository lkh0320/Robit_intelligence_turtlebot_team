# jetson/

Jetson Orin Nano **환경 설정 방법**을 모아 둡니다. Jetson에서 도는 코드는 `ros2/`에 있습니다.

> **결정 사항:** Jetson, 노트북 모두 **ROS 2 Jazzy**를 씁니다. (커스텀 msg를 주고받으므로 배포판이 같아야 함)

## 1. 버전 확인

```bash
cat /etc/nv_tegra_release        # JetPack(L4T) 버전
lsb_release -a                   # Ubuntu 버전 (Jazzy 바이너리는 24.04용)
ls /opt/ros                      # jazzy 폴더가 있으면 apt로 설치된 것
```

| 확인 결과 | ROS 2 위치 | `source` 할 파일 |
|---|---|---|
| `/opt/ros/jazzy` 있음 (apt 설치) | `/opt/ros/jazzy` | `/opt/ros/jazzy/setup.bash` |
| `/opt/ros` 없음 (소스 빌드) | 빌드한 폴더 (예: `~/ros2_jazzy`) | `~/ros2_jazzy/install/setup.bash` |
| Docker로 설치 | 컨테이너 안 | 호스트 터미널에서는 `ros2` 없음 → 컨테이너 안에서 실행 |

아래 문서는 apt 설치(`/opt/ros/jazzy`) 기준입니다. 다른 경우 경로만 바꿔 주세요.

## 2. `ros2: command not found` 해결

ROS 2는 설치만 하면 `ros2` 명령이 PATH에 잡히지 않습니다. **터미널마다 setup 파일을 source** 해야 하고, 매번 하기 번거로우니 `~/.bashrc`에 넣습니다.

```bash
source /opt/ros/jazzy/setup.bash
ros2 -h                           # 도움말이 나오면 OK
printenv ROS_DISTRO               # jazzy
```

되면 `~/.bashrc`에 등록 (루트 `README.md` 4.2와 같은 내용):

```bash
nano ~/.bashrc
```

```bash
# ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=30
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

```bash
source ~/.bashrc
```

- `~/.bashrc`에 `humble` 등 다른 배포판을 source 하는 줄이나 `ROS_LOCALHOST_ONLY`가 있으면 지웁니다.
- 빌드 도구가 없으면: `sudo apt install ros-dev-tools` (colcon, rosdep 포함)

## 3. 필수 패키지

```bash
sudo apt update
sudo apt install ros-jazzy-cv-bridge ros-jazzy-image-transport \
                 ros-jazzy-image-transport-plugins ros-jazzy-camera-calibration \
                 ros-jazzy-demo-nodes-cpp ros-jazzy-demo-nodes-py \
                 ros-dev-tools v4l-utils
sudo usermod -aG dialout,video $USER      # 시리얼, 카메라 권한 (재로그인 필요)
```

`demo_nodes_*`는 노트북과의 통신 테스트(talker/listener)용입니다. `ros-jazzy-ros-base`만 설치했다면 따로 설치해야 합니다.

## 4. 노트북과 ROS 2 통신

### 4.1 조건

| 항목 | Jetson / 노트북 |
|---|---|
| 네트워크 | 같은 Wi-Fi(같은 대역, 예: `192.168.0.x`), 서로 `ping` 가능 |
| `ROS_DOMAIN_ID` | 둘 다 `30` |
| `ROS_AUTOMATIC_DISCOVERY_RANGE` | 둘 다 `SUBNET` (`LOCALHOST`/`OFF`면 다른 PC가 안 보임) |
| RMW | 둘 다 기본값(Fast DDS). `RMW_IMPLEMENTATION`을 따로 설정했다면 같게 |
| 방화벽 | `sudo ufw status` → inactive 이거나 상대 대역 허용 |

### 4.2 확인 순서

```bash
hostname -I                              # 각자 IP 확인
ping <상대 IP>                           # 양쪽 방향 모두
ros2 daemon stop                         # 환경 변수 바꾼 뒤 1회

# 멀티캐스트 (ROS 2 자동 발견에 필요)
ros2 multicast receive                   # 한쪽
ros2 multicast send                      # 다른 쪽 → 받는 쪽에 "Received from ..." 출력

# 실제 토픽
ros2 run demo_nodes_cpp talker           # Jetson
ros2 run demo_nodes_cpp listener         # 노트북
```

### 4.3 멀티캐스트가 안 될 때 (학교/공용 Wi-Fi)

`ping`은 되는데 `ros2 multicast`가 안 되면 공유기가 멀티캐스트를 막는 것입니다. 상대 IP를 직접 지정합니다 (`~/.bashrc`):

```bash
# Jetson 쪽
export ROS_STATIC_PEERS=<노트북 IP>
# 노트북 쪽
export ROS_STATIC_PEERS=<Jetson IP>
```

설정 후 `source ~/.bashrc` → `ros2 daemon stop` → talker/listener 다시 확인.
`ping`부터 안 되면 공유기가 기기 간 통신을 막는 것(AP isolation)이라 다른 공유기나 핫스팟을 써야 합니다.
IP가 자주 바뀌면 공유기에서 Jetson IP를 고정(DHCP 예약)해 두세요.

### 4.4 Wi-Fi 절전 끄기 (영상 끊김 방지)

```bash
iw dev                                   # 무선 인터페이스 이름 (예: wlan0, wlP1p1s0)
iw dev wlan0 get power_save
sudo iw dev wlan0 set power_save off     # 재부팅하면 다시 켜짐
```

## 5. 장치 이름 고정 (udev)

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
