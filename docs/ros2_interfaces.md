# ROS 2 인터페이스 목록

토픽/서비스/파라미터를 추가하거나 바꾸면 **이 문서도 같이** 수정합니다. (초안)

## Topics

| 토픽 | 타입 | 발행 → 구독 | 비고 |
|---|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | camera_node → vision_node | Jetson 내부용 |
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | camera_node → GUI | image_transport 자동 생성 |
| `/vision/debug_image/compressed` | `sensor_msgs/CompressedImage` | vision_node → GUI | 검출 결과 그림 |
| `/vision/result` | `turtle_interfaces/VisionResult` | vision_node → control_node | |
| `/sensor/psd` | `turtle_interfaces/PsdArray` | stm32_bridge_node → control_node, GUI | 50 Hz |
| `/cmd_vel_manual` | `geometry_msgs/Twist` | GUI → control_node | MANUAL 모드에서만 사용 |
| `/motor/cmd` | `turtle_interfaces/MotorCommand` | control_node → stm32_bridge_node | |
| `/motor/state` | `turtle_interfaces/MotorState` | stm32_bridge_node → GUI | 20 Hz |
| `/robot/state` | `turtle_interfaces/RobotState` | control_node → GUI | 5 Hz |

## Services

| 서비스 | 타입 | 서버 | 설명 |
|---|---|---|---|
| `/robot/set_mode` | `turtle_interfaces/srv/SetMode` | control_node | MANUAL / AUTO |
| `/robot/run` | `std_srvs/SetBool` | control_node | true = Start, false = Stop |
| `/robot/estop` | `std_srvs/SetBool` | control_node | true = 비상정지, false = 해제 |

## Messages (turtle_interfaces)

```text
# msg/PsdArray.msg
std_msgs/Header header
uint16[3] raw            # 좌, 중앙, 우
uint16[3] distance_mm

# msg/MotorCommand.msg
float32 left_mps
float32 right_mps
bool enable              # false면 즉시 정지

# msg/MotorState.msg
std_msgs/Header header
float32 left_mps
float32 right_mps
uint8 left_error
uint8 right_error

# msg/RobotState.msg
uint8 MODE_MANUAL=0
uint8 MODE_AUTO=1
std_msgs/Header header
uint8 mode
bool running
bool estop
bool stm32_connected
uint16 mcu_error_flags   # protocol/turtle_protocol.h 의 ERR_*
string message

# msg/VisionResult.msg
std_msgs/Header header
bool detected
float32 offset           # 화면 중심 기준 -1.0(왼쪽) ~ 1.0(오른쪽)
string label

# srv/SetMode.srv
uint8 MODE_MANUAL=0
uint8 MODE_AUTO=1
uint8 mode
---
bool success
string message
```

## Parameters

기본값은 `config/*.yaml`. GUI에서 실행 중 변경 가능한 것은 ✅.

| 노드 | 파라미터 | GUI 변경 |
|---|---|---|
| camera_node | `device`, `width`, `height`, `fps` | ❌ (시작 시) |
| camera_node | `jpeg_quality` | ✅ |
| vision_node | `mode`, `blur_ksize`, `hsv_low`, `hsv_high`, `min_area`, `publish_debug_image` | ✅ |
| control_node | `max_linear_mps`, `max_angular_rps`, `obstacle_stop_mm`, `line_kp/ki/kd` | ✅ |
| stm32_bridge_node | `port`, `baudrate` | ❌ (시작 시) |
| stm32_bridge_node | `wheel_kp/ki/kd` | ✅ (STM32로 전달) |
