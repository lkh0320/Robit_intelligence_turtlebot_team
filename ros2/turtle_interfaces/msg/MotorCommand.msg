# 좌/우 바퀴 목표 속도 명령
# 발행: control_node (수동 조종 또는 자율 주행 결과)
# 구독: stm32_bridge_node → STM32 MOTOR_CMD 패킷으로 변환해서 전송
# 주의: STM32는 200 ms 동안 명령이 없으면 스스로 정지하므로 주기적으로(예: 20 Hz) 보내야 함

float32 left_mps     # 왼쪽 바퀴 목표 속도 [m/s], 앞으로 가면 +
float32 right_mps    # 오른쪽 바퀴 목표 속도 [m/s], 앞으로 가면 +
bool enable          # false면 속도 값과 상관없이 즉시 정지 (비상정지 등)
