# 표지판 인식 결과 (YOLO 클래스: yolo/classes.txt 의 sign_*)
uint8 NONE         = 0
uint8 LEFT         = 1   # sign_left: 갈림길 좌회전
uint8 RIGHT        = 2   # sign_right: 갈림길 우회전
uint8 CONSTRUCTION = 3   # sign_construction: 장애물(공사) 구간 진입
uint8 PARKING      = 4   # sign_parking: 주차 구간

std_msgs/Header header

uint8 type               # 위 상수 중 하나
float32 area_ratio       # bbox 면적 / 이미지 면적, 접근 거리 판단용
float32 confidence       # 0.0 ~ 1.0
