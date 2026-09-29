# PSD 거리센서 3개의 측정값
# 발행: stm32_bridge_node (STM32 SENSOR 패킷을 변환, 약 50 Hz)
# 구독: control_node (장애물 정지), turtle_gui (상태 탭 표시)
# 배열 순서는 모두 [0]=왼쪽, [1]=중앙, [2]=오른쪽

std_msgs/Header header    # stamp: bridge가 패킷을 받은 시각
uint16[3] raw             # ADC 원본값 (0~4095). 센서 보정할 때 참고용
uint16[3] distance_mm     # raw를 거리로 환산한 값 [mm]. 제어에는 이 값을 사용
