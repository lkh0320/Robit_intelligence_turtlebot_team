# master 미션 상태머신 -> cmd_vel mux 제어 모드
uint8 STOP        = 0    # 정지 (기본값)
uint8 LANE        = 1    # 라인추적 PID
uint8 AVOID       = 2    # PSD 장애물 회피
uint8 PARKING     = 3    # 주차 시퀀스
uint8 MANUAL      = 4    # 수동/디버그

std_msgs/Header header

uint8 mode               # 위 상수 중 하나
string mission_state     # 현재 미션 단계 이름 (디버그/로그용)
