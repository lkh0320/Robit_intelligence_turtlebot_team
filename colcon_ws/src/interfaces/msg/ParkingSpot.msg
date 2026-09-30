# 주차 공간 인식 결과
uint8 NONE  = 0
uint8 LEFT  = 1
uint8 RIGHT = 2

std_msgs/Header header

bool detected            # 주차 구역 진입 인식 여부
uint8 empty_side         # 비어있는 주차칸 방향 (위 상수)
float32 offset           # 목표 주차칸 중심 횡오차, 정규화 [-1.0, 1.0]
float32 confidence       # 0.0 ~ 1.0
