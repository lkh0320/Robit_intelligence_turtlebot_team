# 다이나믹셀 바퀴 상태 (stm -> master/GUI)
std_msgs/Header header

float32 left_velocity    # [m/s] 왼쪽 바퀴 실제 선속도
float32 right_velocity   # [m/s] 오른쪽 바퀴 실제 선속도
uint8 left_error         # 다이나믹셀 Hardware Error Status (0 = 정상)
uint8 right_error
