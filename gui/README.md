# gui/

노트북에서 실행하는 **Qt GUI**입니다. ROS 2 패키지(`turtle_gui`)로 만들고, 로봇과는 **ROS 2로만** 통신합니다.
빌드는 레포 루트에서 `bash scripts/build_gui.sh` (turtle_interfaces + turtle_gui만 빌드) 로 합니다.

## 생성 (처음 1회)

교육에서 쓴 Qt 템플릿(`ros2_create_qt_pkg`)으로 `gui/turtle_gui`를 만듭니다.

## 구조

```text
gui/turtle_gui/
├── ui/main_window.ui
├── include/turtle_gui/
│   ├── main_window.hpp     화면, 버튼, 표시만 (rclcpp 코드 없음)
│   └── qnode.hpp           ROS 2 통신 전담 (별도 스레드)
├── src/
│   ├── main.cpp
│   ├── main_window.cpp
│   └── qnode.cpp           구독/발행/서비스/파라미터 클라이언트
└── resources/
```

`main_window`와 `qnode`는 **Qt signal/slot으로만** 연결합니다.

```text
qnode  ── signal: imageReceived, psdUpdated, robotStateUpdated ──▶ main_window (표시)
main_window ── 버튼/슬라이더 ──▶ qnode 함수 호출 ──▶ ROS 2 (publish / service / set_parameters)
```

## 기능별 ROS 2 대응

| GUI 기능 | ROS 2 |
|---|---|
| 카메라 영상 | `/camera/image_raw/compressed`, `/vision/debug_image/compressed` 구독 |
| PSD 값 | `/sensor/psd` 구독 |
| 모터 상태 | `/motor/state` 구독 |
| 로봇 상태, 모드 | `/robot/state` 구독 |
| Start / Stop | `/robot/run` 서비스 (`std_srvs/SetBool`) |
| Emergency Stop | `/robot/estop` 서비스 (`std_srvs/SetBool`) |
| 수동 / 자율 모드 | `/robot/set_mode` 서비스 |
| 전후좌우 조종 | `/cmd_vel_manual` 발행 (누르는 동안 10 Hz) |
| 파라미터 변경 | `rclcpp::AsyncParametersClient` 로 각 노드 파라미터 get/set |
| 노드 상태 | 각 토픽 마지막 수신 시각 → 1초 넘으면 "끊김" 표시 |

GUI 안에 STM32 통신, OpenCV 처리, 제어 로직을 넣지 않습니다.
