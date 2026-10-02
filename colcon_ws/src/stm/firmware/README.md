# turrlebot

STM32F446RE + MX-64(2.0) x2 (RS-485, MAX13488E, Protocol 2.0) 바퀴 구동.

## 구조

CubeMX 재생성 시 `Core/`의 USER CODE 밖은 덮어써지므로 직접 작성한 코드는 모두 `App/`에 둔다.

| 파일 | 내용 |
|---|---|
| `App/dxl.c/h` | Dynamixel Protocol 2.0 (ping / broadcast ping / read / write / sync write, CRC, byte stuffing, TX echo 무시) |
| `App/mx64.c/h` | MX-64(2.0) Control Table, 속도 모드(Operating Mode 1), Goal Velocity |
| `App/app.c/h` | 젯슨 바퀴 속도 명령 / 스위치에 따라 두 바퀴 구동 |
| `App/proto.c/h` | 젯슨 <-> STM32 패킷 규칙 (바퀴 속도 명령, 상태 보고) |
| `App/sw.c/h` | 보드 슬라이드 스위치 S1~S4 (PB12~PB15) 읽기 |
| `App/psd.c/h` | PSD 거리 센서 3개 (ADC1 채널 0/1/2 = PA0 왼쪽 / PA1 앞 / PA2 오른쪽, DMA). Sharp GP2Y0A21 (10~80cm) 기준 |
| `App/host.c/h` | 젯슨과 시리얼 통신 (USART6, 인터럽트 수신) |
| `jetson/stm32_bridge.py` | ROS 2 노드: `/cmd_vel` -> 바퀴 속도 -> STM32 (팀 C++ `stm_bridge` 의 파이썬 버전) |
| `jetson/lane_follower.py` | ROS 2 노드: `/lane_info` -> `/cmd_vel` (차선 따라가기 PD 제어) |
| `jetson/teleop_test.py` | ROS 없이 키보드(w/s/a/d)로 속도 명령 테스트 |
| `jetson/stm32_link.py` | 위 두 스크립트가 쓰는 패킷 코드 |
| `jetson/psd_test.py` | ROS 없이 PSD 거리 3개를 막대로 보기 |
| `jetson/echo_test.py` | 에코 테스트 (예전 에코 펌웨어용, 지금 펌웨어에서는 FAIL 이 정상) |
| `jetson/find_port.py` | ttyTHS1 / ttyTHS2 중 STM32 와 연결된 포트를 자동으로 찾는 스크립트 |

`main.c`:

```c
/* USER CODE BEGIN Includes */
#include "app.h"
/* USER CODE END Includes */

/* USER CODE BEGIN 2 */
app_init();
/* USER CODE END 2 */

/* USER CODE BEGIN 3 */
app_loop();
/* USER CODE END 3 */
```

STM32CubeIDE: `App` 폴더를 Source Location에 추가하고 (Project Properties > C/C++ General > Paths and Symbols > Source Location), Include paths에도 `App`을 추가한다.

## CubeMX 설정

- USART3: Asynchronous, 1 Mbps, 8N1, **TX = PC10**, RX = PC11 (PB10은 LED4)
- USART3 드라이버는 LL (Project Manager > Advanced Settings). 송수신은 폴링이라 USART3 인터럽트는 필요 없음

## 모터

- 펌웨어 MX-64(2.0), Protocol 2.0. 통신 속도와 ID는 코드가 자동으로 찾는다.
  - 1M, 57600, 115200, 2M, 9600bps 순서로 broadcast ping을 보내 처음 응답한 속도를 쓴다.
  - 찾은 ID 중 작은 쪽이 왼쪽 바퀴다 (`app.c` 의 `LEFT_IS_SMALLER_ID` 로 바꿀 수 있음). 두 모터의 ID는 서로 달라야 한다.
- 속도 모드(Operating Mode = 1)는 `app_init()`에서 자동으로 설정한다 (EEPROM은 값이 다를 때만 쓴다).

## 동작 확인 (디버거 Expressions)

| 변수 | 의미 |
|---|---|
| `g_dxl_ready` | 1이면 설정 완료, 바퀴 구동 중 |
| `g_dxl_baud` | 모터가 응답한 통신 속도 (0이면 못 찾음) |
| `g_dxl_count`, `g_dxl_ids` | 찾은 모터 수와 ID ([0] 왼쪽, [1] 오른쪽) |
| `g_dxl_err` | 설정 중 마지막 에러 (-2 응답 없음, 양수는 모터 에러 코드, 128(0x80)은 하드웨어 에러 걸림) |
| `g_dxl_hw_err` | 모터에 걸려 있던 Hardware Error Status (0x01 전압, 0x04 과열, 0x08 엔코더, 0x10 전기 충격, 0x20 과부하). 토크가 켜지면(전압 에러 등) 경고로만 기록하고 그대로 구동, 토크가 안 켜질 때만 Reboot 후 한 번 더 시도 |
| `g_dxl_reboots` | 하드웨어 에러를 지우려고 재부팅한 횟수 (계속 늘어나면 원인이 남아 있음) |
| `g_dxl_rx_bytes` | 받은 바이트 수. **모터를 못 찾았을 때만** 기록되고, 찾으면 0으로 남는다. 못 찾았는데 0이면 에코도 안 돌아옴 = MCU~RS-485 칩 배선 문제, 0보다 크면 모터 쪽(A/B 배선, 12V, 통신 속도) 문제 |
| `g_dxl_volt`, `g_dxl_volt_min`, `g_dxl_volt_max` | 모터가 측정한 전압과 허용 범위 (0.1V 단위, 120 = 12.0V). 전압과 에러는 달리는 중에도 1초마다 다시 읽는다 |
| `g_dxl_torque` | 모터 토크 (1 = 켜짐, 젯슨 상태 보고에도 포함). 모터가 스스로 토크를 끄면 다시 설정(필요하면 재부팅)한다 |

- 12V가 늦게 들어와도 500ms마다 재시도한다. 모터 2대를 다 찾아야 준비 완료 (한 대만 찾으면 계속 다시 찾음, 젯슨 상태의 `모터수` 로 확인)
- 속도는 `app.c`의 `SPEED_1` / `SPEED_2` (1 = 0.229 rpm), 방향은 `LEFT_DIR` / `RIGHT_DIR`

## 스위치 (켜짐 = 핀 LOW, `sw.c` 의 `SW_ON_LEVEL`) 와 ROS 명령

위에서부터 먼저 맞는 것 하나만 적용된다.

| 조건 | 동작 | `g_state` |
|---|---|---|
| S2 (PB13) 켜짐 | 비상 정지 | 2 |
| S1 (PB12) 꺼짐 | 정지 | 0 |
| 젯슨 바퀴 명령이 0.3초 안에 옴 | 명령대로 주행 | 1 |
| S3 (PB14) 또는 S4 (PB15) 켜짐 | 고정 속도 테스트 (S4 = `SPEED_1`, S3 = `SPEED_2`, S4 우선) | 5 |
| 그 외 | 정지 (명령 기다림) | 3 |

모터 준비가 안 됐으면 `g_state` = 4. **S1 만 켜고 ROS 명령이 없으면 정지한다** (예전과 다름). 스위치로만 굴리려면 S1 + S3 또는 S1 + S4.

## 젯슨 -> STM32 바퀴 속도 명령

팀 저장소(`lkh0320/Robit_intelligence_turtlebot_team`)의 `stm` 패키지 `protocol.hpp` 와 같은 규칙이다.

```
[AA] [55] [01] [04] [왼쪽 mm/s int16] [오른쪽 mm/s int16] [CHK]     젯슨 -> STM32
[AA] [55] [81] [0B] [준비, 상태, 스위치, 에러0, 에러1, 전압 uint16, 모터 수, ID왼, ID오, 토크 비트] [CHK]   STM32 -> 젯슨 (0.1초마다)
[AA] [55] [10] [06] [왼쪽 mm, 앞 mm, 오른쪽 mm  uint16] [CHK]   STM32 -> 젯슨 (0.05초마다, 팀 stm_bridge 가 /psd 로 발행)
```

- `cmd_vel`(v, w) -> 바퀴 속도 변환은 **젯슨**에서 한다: 왼쪽 = v - w·L/2, 오른쪽 = v + w·L/2 (L = `wheel_separation`, 팀 `stm_bridge.yaml`)
- STM32 는 바퀴 선속도(m/s) / r -> rad/s -> rpm -> ÷ 0.229 로 MX-64 값을 만든다
- 바퀴 반지름 `WHEEL_RADIUS_M` = 0.05 m 는 **임시값** (`app.c`). 실제로 재서 바꿀 것
- 안전 제한: 바퀴 속도 최대 0.3 m/s, 모터 값 최대 250 (넘으면 두 바퀴를 같은 비율로 줄임)
- 0.3초 동안 명령이 없으면 정지. 젯슨은 멈춰 있을 때도 20Hz 로 계속 보낸다
- 팀 C++ `stm_bridge` 는 0x81 상태 패킷을 몰라서 5초마다 `Unknown frame id=0x81` 경고를 찍는다 (동작에는 문제 없음)

## 차선 따라가기 (젯슨, ROS 2)

```
카메라 -> bird_eye_view -> lane_detection --/lane_info--> lane_follower --/cmd_vel--> stm_bridge --시리얼--> STM32
                     (팀 저장소)                        (jetson/lane_follower.py)     (팀 C++ 또는 stm32_bridge.py)
```

`lane_follower.py` 는 `LaneInfo` 의 offset(차선 중심이 오른쪽이면 +), angle(차선이 오른쪽으로 휘면 +) 로 PD 제어:
`w = -(kp_offset·offset + kd_offset·d(offset)/dt + kp_angle·angle)`, 많이 꺾을수록 느리게. 차선을 0.5초 못 보면 정지.

```bash
# 공통 (터미널마다)
source /opt/ros/jazzy/setup.bash
source ~/Robit_intelligence_turtlebot_team/colcon_ws/install/setup.bash

# 1. 카메라 + 비전
ros2 launch vision_bringup camera_vision.launch.py
# 2. STM32 브리지 (둘 중 하나)
ros2 launch stm stm_bridge.launch.py
python3 ~/turrlebot/jetson/stm32_bridge.py --ros-args -p port:=/dev/ttyUSB0
# 3. 차선 따라가기
python3 ~/turrlebot/jetson/lane_follower.py --ros-args -p base_speed:=0.05
```

| 파라미터 | 기본 | 뜻 |
|---|---|---|
| `base_speed` | 0.08 | 직선 속도 (m/s) |
| `min_speed` | 0.03 | 크게 꺾을 때 최소 속도 |
| `kp_offset` | 0.8 | 옆으로 벗어난 만큼 돌기 |
| `kd_offset` | 0.05 | 벗어나는 속도만큼 미리 돌기 (좌우로 흔들리면 키움) |
| `kp_angle` | 1.0 | 차선이 휜 만큼 돌기 |
| `max_angular` | 1.2 | 회전 최대 (rad/s) |
| `lost_timeout` | 0.5 | 차선을 이만큼 못 보면 정지 (s) |

ROS 없이 키보드 테스트: `python3 teleop_test.py /dev/ttyUSB0` (`stm32_link.py` 와 같은 폴더)

디버거:

| 변수 | 의미 |
|---|---|
| `g_state` | 위 표의 상태 |
| `g_cmd_left_mm`, `g_cmd_right_mm` | 마지막으로 받은 왼쪽/오른쪽 바퀴 속도 (mm/s) |
| `g_goal` | 모터에 보내는 값 [0] 왼쪽, [1] 오른쪽 (방향 부호 곱하기 전) |
| `g_proto_ok`, `g_proto_bad` | 제대로 받은 패킷 수, 깨진 패킷 수 |
| `g_sw_on`, `g_speed` | 켜진 스위치 (bit0=S1 ... bit3=S4), 스위치 테스트 속도 |
| `g_psd_mm`, `g_psd_mv`, `g_psd_raw` | PSD 거리 [mm], 센서 전압 [mV], ADC 값 ([0] 왼쪽, [1] 앞, [2] 오른쪽) |

## 젯슨 통신 (USART6)

| STM32 (J10) | 젯슨 Orin Nano 40핀 |
|---|---|
| 1 GND | 6 GND |
| 3 PC6 USART6_TX | 10 RXD |
| 4 PC7 USART6_RX | 8 TXD |
| 2 +5V | 연결하지 않음 |

- CubeMX: USART6 Asynchronous 115200 8N1, 드라이버 LL, NVIC USART6 global interrupt 켜기
- 코드 생성 후 `Core/Src/stm32f4xx_it.c` 에 두 줄 추가 (USER CODE 구역이라 재생성해도 유지됨)
  - `/* USER CODE BEGIN Includes */` 아래: `#include "host.h"`
  - `/* USER CODE BEGIN USART6_IRQn 0 */` 아래: `host_irq();`
- 젯슨 40핀 UART(ttyTHS1/2)는 젯슨 혼자 루프백도 안 돼서 **USB-TTL 변환기(3.3V)** 로 연결했다: STM TX(J10 3) → 변환기 RX, STM RX(J10 4) ← 변환기 TX, GND 공통, 젯슨에서는 `/dev/ttyUSB0`
- 포트가 헷갈리면 젯슨에서 `python3 jetson/find_port.py` 실행 후 STM32 리셋 버튼을 누르면 연결된 포트를 알려준다
- STM32 가 켜질 때 `STM32 ready` 를 한 번 보내고, 그 뒤로 0.1초마다 상태 패킷을 보낸다
- 디버거: `g_host_rx_bytes` (받은 바이트 수), `g_host_tx_bytes` (보낸 바이트 수), `g_host_overflow` (버퍼 넘침)
