# 로봇 전체 상태 요약 (GUI 상단/상태 탭에 표시되는 내용)
# 발행: control_node (약 5 Hz)
# 구독: turtle_gui

# mode 값으로 쓰는 상수 (SetMode.srv의 상수와 같은 값)
uint8 MODE_MANUAL=0   # 수동: GUI의 W/A/S/D 조종(/cmd_vel_manual)으로 움직임
uint8 MODE_AUTO=1     # 자율: 영상/센서 기반으로 control_node가 스스로 주행

std_msgs/Header header
uint8 mode              # MODE_MANUAL 또는 MODE_AUTO
bool running            # GUI [Start]로 실행 중이면 true, [Stop]이면 false
bool estop              # 비상정지 상태면 true (해제 전까지 모터 명령 무시)
bool stm32_connected    # bridge가 STM32와 정상 통신 중이면 true
uint16 mcu_error_flags  # STM32 에러 비트 (protocol/turtle_protocol.h 의 ERR_*), 0 = 정상
string message          # 사람이 읽을 상태 메시지 (예: "장애물 감지로 정지")
