# 표지판 인식 결과 (갈림길 방향 결정)
uint8 NONE  = 0
uint8 LEFT  = 1
uint8 RIGHT = 2

std_msgs/Header header

uint8 type               # 위 상수 중 하나
float32 area_ratio       # bbox 면적 / 이미지 면적, 접근 거리 판단용
float32 confidence       # 0.0 ~ 1.0
