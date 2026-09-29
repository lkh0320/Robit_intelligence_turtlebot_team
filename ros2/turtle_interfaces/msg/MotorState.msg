# 좌/우 바퀴의 실제 상태 (Dynamixel에서 읽은 값)
# 발행: stm32_bridge_node (STM32 MOTOR_STATE 패킷을 변환, 약 20 Hz)
# 구독: turtle_gui (상태 탭 모터 속도 표시)

std_msgs/Header header
float32 left_mps      # 왼쪽 바퀴 현재 속도 [m/s]
float32 right_mps     # 오른쪽 바퀴 현재 속도 [m/s]
uint8 left_error      # Dynamixel Hardware Error Status 값 (0 = 정상)
uint8 right_error     # 비트 의미는 Dynamixel e-Manual의 Hardware Error Status 참고
