# 차단바 인식 결과
uint8 UNKNOWN = 0
uint8 CLOSED  = 1        # 내려와 있음 -> 정지
uint8 OPEN    = 2        # 올라감 -> 통과

std_msgs/Header header

bool detected
uint8 state              # 위 상수 중 하나
float32 confidence       # 0.0 ~ 1.0
