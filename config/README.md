# config/

모든 노드의 **파라미터 기본값**을 모아 두는 곳입니다. 형식은 ROS 2 parameter yaml 입니다.

| 파일 | 대상 노드 |
|---|---|
| `robot.yaml` | 모든 노드 공용 (바퀴 반지름, 바퀴 간격 등 로봇 치수) |
| `camera.yaml` | `camera_node` |
| `vision.yaml` | `vision_node` |
| `control.yaml` | `control_node` |
| `stm32_bridge.yaml` | `stm32_bridge_node` (STM32로 전달되는 값 포함) |
| `calibration/` | 카메라 캘리브레이션 결과 |

## 값은 어디에 두나?

| 종류 | 위치 | 예 |
|---|---|---|
| 실행 중 GUI로 바꾸는 값 | 여기 yaml → ROS 2 Parameter | 속도, PID, HSV threshold, 장애물 거리 |
| 시작할 때만 정하는 값 | 여기 yaml (노드에서 read-only로 선언) | 시리얼 포트, 보드레이트, 카메라 해상도 |
| 하드웨어에 고정된 값 | STM32 코드 + `docs/hardware.md` | 핀 번호, 타이머, Dynamixel ID |

## GUI에서 바꾼 값은?

1. GUI는 노드의 ROS 2 Parameter를 바꿉니다 (`ros2 param set`과 같은 방식). **실행 중에만 유지**됩니다.
2. 튜닝이 끝나면 값을 저장합니다.
   ```bash
   ros2 param dump /vision_node      # 출력된 값을 vision.yaml 에 반영
   ```
3. yaml 수정 → 커밋 → Jetson에서 `git pull` 하면 다음 실행부터 기본값이 됩니다.

STM32가 쓰는 값(바퀴 PID 등)도 `stm32_bridge_node`의 파라미터로 두고, 바뀌면 bridge가 STM32로 전달합니다.
그래서 GUI는 STM32의 존재를 몰라도 됩니다.

## 규칙

- 개인 테스트 값은 `xxx.local.yaml`로 저장하세요. git에서 자동 제외됩니다.
- **키 이름**을 바꾸거나 지우면 노드 코드가 깨지니 팀에 먼저 공유하세요. 값 변경은 자유입니다.
- `turtle_bringup`이 이 폴더를 설치하므로 `--symlink-install`로 빌드했다면 yaml 수정 후 재빌드 없이 재실행만 하면 됩니다.
