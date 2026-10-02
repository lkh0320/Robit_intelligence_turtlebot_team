# 다이나믹셀 바퀴 상태 (stm -> master/GUI)
# STM32 상태 패킷(0x81, 0.1초마다)을 stm_bridge 가 그대로 옮긴다
std_msgs/Header header

# STM32 동작 상태 (state)
uint8 STATE_STOP_SW   = 0   # S1 꺼짐 -> 정지
uint8 STATE_ROS       = 1   # 젯슨 바퀴 명령대로 주행 중
uint8 STATE_ESTOP     = 2   # S2 켜짐 -> 비상 정지
uint8 STATE_NO_CMD    = 3   # 젯슨 명령이 0.3초 넘게 없음 -> 정지
uint8 STATE_NOT_READY = 4   # 모터를 못 찾았거나 설정 실패
uint8 STATE_MANUAL    = 5   # 스위치 고정 속도 테스트 (S3/S4)

# 바퀴 선속도 [m/s]. 엔코더 피드백이 없어서 STM32 가 ROS 명령대로 주행 중(STATE_ROS)일 때만
# 마지막으로 보낸 바퀴 명령 값을 넣고, 그 외(정지 상태)에는 0
float32 left_velocity
float32 right_velocity
uint8 left_error         # 다이나믹셀 Hardware Error Status (0 = 정상)
uint8 right_error

bool ready               # 모터 설정 끝 (찾기 + 속도 모드 + 토크 ON)
uint8 state              # 위 STATE_* 중 하나
uint8 switches           # 켜진 보드 스위치 (bit0 = S1 ... bit3 = S4)
float32 voltage          # 모터 입력 전압 [V] (왼쪽 모터가 측정)
uint8 motor_count        # 찾은 모터 수 (정상 2)
uint8 left_id            # 왼쪽 모터 ID (못 찾으면 0)
uint8 right_id
bool left_torque         # 토크 켜짐 (false = 모터가 과부하 등으로 스스로 끔)
bool right_torque
