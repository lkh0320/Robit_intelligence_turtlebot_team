# 영상처리 결과 한 프레임 분
# 발행: vision_node (카메라 프레임마다)
# 구독: control_node (자율 주행 조향), turtle_gui (상태 탭 표시)

std_msgs/Header header    # stamp: 처리한 원본 카메라 이미지의 시각을 복사해서 넣기 (권장)
bool detected             # 라인/목표물을 찾았으면 true. false면 offset은 의미 없음
float32 offset            # 화면 중심 기준 위치: -1.0(맨 왼쪽) ~ 0.0(중앙) ~ 1.0(맨 오른쪽)
string label              # 무엇을 찾았는지 (예: "line", "red"). vision.yaml의 mode에 따라 달라짐
