# 라인 검출 결과 (vision -> master 라인추적 PID)
std_msgs/Header header

bool detected            # 라인 검출 여부 (false면 offset/angle 무시)
float32 offset           # 차선 중심 대비 로봇 횡오차, 정규화 [-1.0, 1.0] (+: 라인이 오른쪽)
float32 angle            # 라인 진행방향 오차 [rad] (+: 라인이 오른쪽으로 기울어짐)
float32 confidence       # 0.0 ~ 1.0
