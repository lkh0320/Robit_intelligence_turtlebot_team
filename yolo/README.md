# YOLO 객체 인식 (표지판 / 신호등 / 차단봉)

팀 공용 클래스 규칙. **번호와 순서는 셋이 반드시 똑같이 쓴다.** 기준 파일은 `classes.txt`와 `data.yaml`이다.
라벨링 툴에는 `classes.txt`를 불러와서 쓴다. 클래스 이름을 직접 타이핑하면 번호가 달라질 수 있다.

| 번호 | 클래스 | 무엇을 | 로봇 동작 (젯슨) |
|---|---|---|---|
| 0 | `sign_construction` | 장애물(공사) 구간 들어가기 전 표지판 | 벽 회피 모드 준비 |
| 1 | `sign_left` | 갈림길 좌회전 표지판 | 갈림길에서 왼쪽 |
| 2 | `sign_right` | 갈림길 우회전 표지판 | 갈림길에서 오른쪽 |
| 3 | `sign_parking` | 주차 구간 표지판 | 주차 시퀀스 준비 |
| 4 | `light_red` | 신호등 (빨간불 켜짐) | 정지 |
| 5 | `light_yellow` | 신호등 (노란불 켜짐) | 정지 |
| 6 | `light_green` | 신호등 (초록불 켜짐) | 출발 |
| 7 | `bar_closed` | 차단봉 내려옴 | 정지 |
| 8 | `bar_open` | 차단봉 올라감 | 통과 |

- 번호는 0부터 시작한다.
- 순서를 바꾸거나 중간에 끼워 넣지 않는다. 새 클래스는 맨 뒤(9번부터)에만 추가한다.
- 추가할 때는 `classes.txt`, `data.yaml`, 이 표를 같이 고치고 팀원에게 알린다.

## 라벨링 규칙

- **표지판**: 표지판 판(원/사각형)만 박스로 감싼다. 기둥은 빼고, 일부가 가려졌으면 보이는 부분만 감싼다.
- **신호등**: 신호등 몸체 전체를 감싸고, **켜진 불 색**으로 클래스를 정한다.
  - 불이 다 꺼졌거나 색을 구분할 수 없는 사진은 신호등 박스를 넣지 않는다.
- **차단봉**:
  - 내려와 있으면 봉 전체를 감싸서 `bar_closed`로 한다.
  - 올라가 있으면 세워진 봉을 감싸서 `bar_open`으로 한다.
  - 올라가거나 내려가는 중간 각도는 로봇이 멈춰야 하므로 `bar_closed`로 한다.
- 화면 가장자리에 반 이상 잘린 물체는 박스를 넣지 않는다.
- 아무 물체도 없는 트랙 사진(배경)도 전체의 10% 정도 섞는다. 라벨 파일은 비워 두거나 만들지 않는다.
- 사진은 **로봇 카메라(640x480)로** 찍는다. 다양하게 찍어 두면 실제 주행에서 덜 놓친다.
  - 거리: 멀리, 중간, 가까이
  - 각도: 정면, 비스듬히
  - 조명: 밝을 때, 어두울 때

## 폴더 구조

`dataset/`은 용량이 커서 git에 올리지 않는다. 공유 드라이브로 주고받는다.

```
yolo/
  classes.txt        클래스 이름 (라벨링 툴용, 한 줄에 하나, 0번부터)
  data.yaml          학습 설정 (같은 순서)
  check_labels.py    라벨 검사
  dataset/
    images/train/  images/val/     사진 (.jpg)
    labels/train/  labels/val/     같은 이름의 .txt (한 줄 = "번호 cx cy w h", 0~1 비율)
```

## 라벨 검사 (학습 전과 데이터 합칠 때 꼭)

```bash
python3 yolo/check_labels.py              # yolo/dataset 검사
python3 yolo/check_labels.py ~/내데이터셋   # 다른 폴더 검사
```

검사하는 것:
- 범위 밖 클래스 번호와 픽셀 좌표로 저장된 라벨을 찾는다.
- 이미지와 라벨 짝이 맞는지 본다.
- 클래스별 박스 수를 보여준다.

## 로봇(젯슨)에 올리기

젯슨에는 ultralytics/PyTorch/TensorRT 가 없고, JetPack 의 OpenCV 4.8 DNN(CPU)으로 ONNX 를 돌린다.
ONNX 는 기기와 상관없으므로 **노트북에서 변환**해서 복사한다 (TensorRT `.engine` 과 다름).

```bash
# 노트북
yolo export model=runs/detect/signs_v1/weights/best.pt format=onnx imgsz=640 opset=12 simplify=True
scp runs/detect/signs_v1/weights/best.onnx <젯슨계정>@<젯슨IP>:~/models/signs_v1.onnx   # 젯슨에서 먼저 mkdir -p ~/models

# 젯슨
ros2 launch vision_bringup camera_vision.launch.py
ros2 topic echo /sign
```

- 모델 경로, 입력 크기, 신뢰도 기준은 `colcon_ws/src/vision/sign_detection/config/sign_detection.yaml` 에서 바꾼다.
- 추론 시간은 GUI 객체 인식 화면 왼쪽 위에 나온다. 너무 느리면 `imgsz=416`(또는 320)으로 export 하고 `input_size` 를 같이 바꾼다.
- export 할 때 `nms=True` 를 쓰거나 YOLO26 같은 NMS 없는 모델로 바꾸면 출력 모양이 달라져 노드가 거부한다 (로그에 모양이 찍힌다).
