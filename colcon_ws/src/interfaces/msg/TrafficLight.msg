# 신호등 인식 결과 (출발 조건)
uint8 UNKNOWN = 0
uint8 RED     = 1
uint8 YELLOW  = 2
uint8 GREEN   = 3

std_msgs/Header header

uint8 state              # 위 상수 중 하나
float32 confidence       # 0.0 ~ 1.0
