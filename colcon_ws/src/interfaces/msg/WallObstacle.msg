# 벽 장애물(갈색 박스) 인식 결과 (좌우 번갈아 놓인 벽을 지그재그로 피해 가는 구간)
uint8 NONE  = 0
uint8 LEFT  = 1
uint8 RIGHT = 2

std_msgs/Header header

bool detected
uint8 side               # 가장 가까운 벽이 막고 있는 쪽 (위 상수)
float32 x_ratio          # 가장 가까운 벽 중심 가로 위치, 정규화 [-1.0, 1.0] (+: 오른쪽)
float32 bottom_ratio     # 가장 가까운 벽 아래 끝 세로 위치 [0.0(상단) ~ 1.0(하단)], 클수록 가까움
float32 free_offset      # 벽이 없는 쪽 빈 공간 중심, 정규화 [-1.0, 1.0] -> 회피 목표 방향
float32 area_ratio       # 벽 bbox 면적 / 이미지 면적
float32 confidence       # 0.0 ~ 1.0
