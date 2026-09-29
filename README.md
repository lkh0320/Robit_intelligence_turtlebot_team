# Robit Intelligence TurtleBot Team

Jetson Orin Nano + STM32 기반 **TurtleBot형 자율주행 로봇** 프로젝트입니다.
노트북의 Qt GUI에서 ROS 2로 카메라 영상·센서값·로봇 상태를 보고, 파라미터를 바꾸고, 로봇을 조종합니다.

> **ROS 2 버전: Jetson, 노트북 모두 `Jazzy`로 통일합니다.** 모든 명령과 경로는 Jazzy(`/opt/ros/jazzy`) 기준입니다.

> 🚧 **현재 상태:** `turtle_interfaces`(msg/srv)와 Qt GUI(`gui/turtle_gui`, 로봇 없이 테스트용 `fake_robot.py` 포함)까지 작성됨. 나머지 ROS 2 패키지와 펌웨어는 각 폴더 README의 방법대로 생성해 주세요.
>
> 처음 clone 했다면 **1. 시스템 구성 → 2. 폴더 구조 → 5. 내가 수정할 곳**만 먼저 읽어도 충분합니다.

---

## 1. 시스템 구성

```text
 [노트북]   Qt GUI  (영상/센서/상태 보기, 파라미터 변경, 조종)
               │
               │  ROS 2 Jazzy  (같은 Wi-Fi + 같은 ROS_DOMAIN_ID)
               ▼
 [Jetson]   ROS 2 노드
               ├─ turtle_vision        USB 웹캠 + OpenCV 영상처리
               ├─ turtle_control       주행 판단, 수동/자율 모드, 비상정지
               └─ turtle_stm32_bridge  ROS 2 메시지 ↔ 시리얼 패킷 변환
               │
               │  UART / USB 시리얼   (패킷 규약: protocol/)
               ▼
 [STM32]    펌웨어  (Dynamixel 2개, PSD 3개, 저수준 속도 제어)
```

이 프로젝트의 규칙 세 가지:

1. **GUI는 ROS 2(토픽/서비스/파라미터)로만 로봇과 통신합니다.** STM32 통신이나 OpenCV 코드를 GUI에 넣지 않습니다.
2. **STM32와 주고받는 패킷 형식은 `protocol/` 한 곳에만** 정의합니다. STM32와 Jetson이 같은 헤더 파일을 씁니다.
3. **튜닝 값(속도, PID, threshold 등)은 코드에 쓰지 않고 `config/*.yaml`에** 둡니다.

---

## 2. 폴더 구조

```text
.
├── ros2/          Jetson에서 돌아가는 ROS 2 패키지 + 공용 msg/srv
├── gui/           노트북용 Qt GUI (ROS 2 패키지)
├── stm32/         STM32CubeIDE 펌웨어 프로젝트
├── protocol/      Jetson ↔ STM32 통신 규약 (문서 + C 헤더, 유일한 원본)
├── config/        파라미터 기본값 (ROS 2 parameter yaml)
├── jetson/        Jetson 환경 설정 방법 (JetPack, ROS 2, 시리얼/카메라 장치)
├── scripts/       빌드 스크립트
├── docs/          하드웨어, ROS 2 인터페이스, 개발/디버깅 문서
├── README.md      ← 지금 보는 파일
└── CONTRIBUTING.md  Git 협업 규칙 (브랜치, 커밋, PR)
```

폴더마다 README.md가 있습니다. **그 폴더에서 작업하기 전에 먼저 읽어 주세요.**

ROS 2 패키지 구성 (예정):

| 패키지 | 위치 | 실행 장소 | 하는 일 |
|---|---|---|---|
| `turtle_interfaces` | `ros2/` | 둘 다 | 커스텀 msg / srv 정의 |
| `turtle_bringup` | `ros2/` | Jetson | launch 파일, `config/` 설치 |
| `turtle_vision` | `ros2/` | Jetson | 카메라 영상 publish, OpenCV 처리 |
| `turtle_control` | `ros2/` | Jetson | 모드 관리, 주행 명령 생성, 비상정지 |
| `turtle_stm32_bridge` | `ros2/` | Jetson | STM32 시리얼 통신 |
| `turtle_gui` | `gui/` | 노트북 | Qt GUI |

---

## 3. 준비물

**하드웨어**

| 항목 | 내용 |
|---|---|
| 메인 컴퓨터 | NVIDIA Jetson Orin Nano |
| MCU | STM32 (보드 모델: `docs/hardware.md`에 기입) |
| 구동 모터 | Dynamixel × 2 |
| 거리 센서 | PSD × 3 (좌 / 중앙 / 우) |
| 카메라 | USB 웹캠 |
| 개발 PC | Ubuntu 노트북 |

**소프트웨어**

| 장치 | 필요한 것 |
|---|---|
| 노트북 | Ubuntu 24.04, ROS 2 Jazzy, Qt 5, OpenCV, colcon |
| Jetson | JetPack, ROS 2 Jazzy, OpenCV, cv_bridge, image_transport (설정 방법: `jetson/README.md`) |
| STM32 | STM32CubeIDE |

> ⚠️ 커스텀 msg(`turtle_interfaces`)를 주고받으므로 **Jetson과 노트북은 반드시 같은 배포판(Jazzy)** 이어야 합니다. 다른 배포판끼리의 통신은 공식 지원이 아닙니다.

---

## 4. 빌드 & 실행

### 4.1 Clone

```bash
git clone https://github.com/lkh0320/Robit_intelligence_turtlebot_team.git
cd Robit_intelligence_turtlebot_team
```

**레포 루트가 그대로 colcon 워크스페이스입니다.** 빌드하면 루트에 `build/ install/ log/`가 생기고, 이 폴더들은 git에서 제외됩니다.

### 4.2 환경 변수 (Jetson, 노트북 둘 다 · 처음 1회)

`~/.bashrc` 맨 아래에 추가하고 `source ~/.bashrc` (또는 터미널 새로 열기):

```bash
# ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=30                      # 팀 공용 값 (모든 PC 동일)
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET  # 같은 네트워크의 다른 PC와 통신
```

- 터미널에서 `ros2: command not found`가 나오면 첫 줄(`source /opt/ros/jazzy/setup.bash`)이 빠진 것입니다.
- Jazzy에서 `ROS_LOCALHOST_ONLY`는 더 이상 쓰지 않습니다. `~/.bashrc`에 `ROS_LOCALHOST_ONLY=1`이 남아 있으면 **지워 주세요** (다른 PC와 통신이 막힘).
- 환경 변수를 바꾼 뒤에는 `ros2 daemon stop` 한 번 실행 (예전 설정의 데몬이 남아 있으면 토픽 목록이 이상하게 보임).
- 학교 Wi-Fi 등에서 서로 발견이 안 되면 `ROS_STATIC_PEERS` 설정: [`jetson/README.md` 4장](jetson/README.md#4-노트북과-ros-2-통신)

통신 확인 (Jetson ↔ 노트북):

```bash
# Jetson                                # 노트북
ros2 run demo_nodes_cpp talker          ros2 run demo_nodes_cpp listener
```

노트북에 `I heard: [Hello World: N]`이 찍히면 성공. 반대 방향도 한 번 확인하세요.

### 4.3 Jetson (로봇)

```bash
source /opt/ros/jazzy/setup.bash     # ~/.bashrc에 넣었다면 생략
bash scripts/build_robot.sh          # ros2/ 아래 패키지 전체 빌드
source install/setup.bash
ros2 launch turtle_bringup robot.launch.py
```

### 4.4 노트북 (GUI)

```bash
source /opt/ros/jazzy/setup.bash     # ~/.bashrc에 넣었다면 생략
bash scripts/build_gui.sh            # turtle_interfaces + turtle_gui만 빌드
source install/setup.bash
ros2 run turtle_gui turtle_gui
```

### 4.5 STM32

1. STM32CubeIDE → `File > Import > Existing Projects into Workspace`
2. `stm32/turtle_fw` 선택, **"Copy projects into workspace"는 체크 해제**
3. Build(망치 아이콘) → Run 으로 업로드

직접 작성하는 코드는 `stm32/turtle_fw/App/`에만 둡니다. 자세한 건 `stm32/README.md` 참고.

---

## 5. 내가 수정할 곳

| 하고 싶은 작업 | 수정 위치 |
|---|---|
| 카메라, 영상처리 (색/라인/객체 검출) | `ros2/turtle_vision/` (OpenCV 로직은 `core/`) |
| 주행 판단, 모드 전환, 비상정지 | `ros2/turtle_control/` |
| Jetson 쪽 시리얼 통신 | `ros2/turtle_stm32_bridge/` |
| Dynamixel, PSD, 저수준 제어 | `stm32/turtle_fw/App/` |
| GUI 화면, 버튼, 그래프 | `gui/turtle_gui/` |
| 파라미터 기본값 | `config/` |
| launch 구성 | `ros2/turtle_bringup/` |
| ⚠️ 패킷 추가/변경 | `protocol/` — **3명 합의 후** |
| ⚠️ 토픽/서비스 메시지 추가 | `ros2/turtle_interfaces/` — **3명 합의 후** |

⚠️ 표시된 곳은 여러 사람의 코드가 함께 의존하는 "약속" 파일입니다. 바꾸기 전에 팀에 먼저 알려 주세요.

---

## 6. ROS 2 인터페이스 요약

| 이름 | 종류 | 흐름 |
|---|---|---|
| `/camera/image_raw/compressed` | Topic | 카메라 → GUI |
| `/vision/debug_image/compressed` | Topic | 영상처리 결과 → GUI |
| `/sensor/psd` | Topic | STM32 → GUI, 제어 |
| `/motor/cmd` | Topic | 제어 → STM32 |
| `/motor/state` | Topic | STM32 → GUI |
| `/robot/state` | Topic | 제어 → GUI |
| `/cmd_vel_manual` | Topic | GUI → 제어 (수동 조종) |
| `/robot/set_mode`, `/robot/run`, `/robot/estop` | Service | GUI → 제어 |

GUI의 파라미터 변경은 각 노드의 **ROS 2 Parameter**를 직접 바꾸는 방식입니다.
전체 목록과 타입: [`docs/ros2_interfaces.md`](docs/ros2_interfaces.md)

---

## 7. 협업 규칙 (요약)

- `main`에 직접 push 금지 → 작업 브랜치 → Pull Request → 1명 리뷰 후 merge
- 브랜치 이름: `feat/vision-line-detect`, `fix/bridge-checksum`
- 커밋 메시지: `feat(vision): 라인 검출 추가`

자세한 규칙과 담당 범위: [`CONTRIBUTING.md`](CONTRIBUTING.md)

---

## 8. 문서

| 문서 | 내용 |
|---|---|
| [`docs/hardware.md`](docs/hardware.md) | 배선, STM32 핀맵, Dynamixel/PSD 설정 |
| [`docs/ros2_interfaces.md`](docs/ros2_interfaces.md) | 토픽 / 서비스 / 파라미터 목록 |
| [`docs/development.md`](docs/development.md) | 빌드, 실행, 디버깅 방법 |
| [`protocol/PROTOCOL.md`](protocol/PROTOCOL.md) | Jetson ↔ STM32 패킷 규약 |
| [`jetson/README.md`](jetson/README.md) | Jetson 환경 설정 (ROS 2 Jazzy, 장치 이름, 노트북과 통신) |
