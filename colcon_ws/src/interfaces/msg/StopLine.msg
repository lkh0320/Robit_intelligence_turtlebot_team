# 정지선 검출 결과
std_msgs/Header header

bool detected
float32 y_ratio          # 이미지 내 정지선 세로 위치 [0.0(상단) ~ 1.0(하단)], 클수록 가까움
float32 confidence       # 0.0 ~ 1.0
