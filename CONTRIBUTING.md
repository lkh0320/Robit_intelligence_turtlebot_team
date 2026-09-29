# 협업 규칙

3명이 쓰기 좋은 **단순한 규칙**입니다. Git Flow(develop, release 브랜치)는 쓰지 않습니다.

---

## 1. 브랜치

| 브랜치 | 용도 |
|---|---|
| `main` | 항상 **빌드되는 상태**. 직접 push 금지, PR로만 merge |
| `<type>/<영역>-<요약>` | 작업 브랜치. 작업 하나당 하나, merge 후 삭제 |

`develop` 브랜치는 만들지 않습니다. 3명 규모에서는 관리할 브랜치만 늘어나고 얻는 게 적습니다.
대회/시연 직전 안정 버전이 필요하면 `main`에 태그를 답니다 (`git tag v0.1-demo`).

브랜치 이름 예시:

```text
feat/vision-line-detect
feat/gui-param-panel
fix/bridge-checksum
docs/protocol-v2
```

## 2. 작업 흐름

```bash
git switch main
git pull                                   # 최신 main 받기
git switch -c feat/vision-line-detect      # 작업 브랜치 생성

# ... 작업, 커밋 ...

git pull origin main                       # PR 전에 최신 main 합치기 (충돌은 내 브랜치에서 해결)
git push -u origin feat/vision-line-detect
```

GitHub에서 **Pull Request (base: main)** 생성 → 리뷰 → merge → 브랜치 삭제.

## 3. 커밋 메시지

```text
<type>(<영역>): <무엇을 했는지 한 줄>

feat(vision): HSV 기반 라인 검출 추가
fix(bridge): 체크섬 계산 시 LEN 누락 수정
docs(protocol): SENSOR 패킷 필드 설명 추가
```

| type | 의미 |
|---|---|
| `feat` | 기능 추가 |
| `fix` | 버그 수정 |
| `refactor` | 동작 변화 없는 구조 개선 |
| `docs` | 문서 |
| `test` | 테스트 |
| `chore` | 빌드 설정, .gitignore 등 |

영역: `vision` `control` `bridge` `interfaces` `bringup` `gui` `stm32` `protocol` `config` `docs`

## 4. Pull Request

- **작게** 올립니다. 하루 작업 정도 크기가 적당합니다.
- 본문에 적을 것: **무엇을 / 왜 / 어떻게 테스트했는지** (GUI 변경이면 스크린샷)
- 올리기 전 **빌드 확인** 필수 (`bash scripts/build_robot.sh` 또는 `build_gui.sh`, STM32는 CubeIDE Build)
- 리뷰어 **1명 Approve** 후 작성자가 merge (**Squash and merge** 권장)
- 관련 이슈가 있으면 본문에 `Closes #12`

## 5. ⚠️ 약속 파일 (변경 시 3명 모두 확인)

아래 파일은 여러 영역이 함께 의존합니다. 한 명이 바꾸면 다른 사람 코드가 깨집니다.

| 파일 | 규칙 |
|---|---|
| `protocol/` | 이슈나 단톡으로 먼저 공유 → **STM32 코드와 bridge 코드를 같은 PR에서** 수정 → `PROTOCOL_VERSION` 올리기 |
| `ros2/turtle_interfaces/` | 먼저 공유 → msg 수정 시 사용하는 노드/GUI 같이 수정 |
| `config/` 의 키 이름 | 값 변경은 자유, **키 이름 변경/삭제**는 공유 |
| `stm32/turtle_fw/*.ioc` | 동시에 두 명이 수정 금지 (merge가 거의 불가능). 수정 전 "ioc 잡습니다" 공유 |

이 파일을 바꾸는 PR은 **나머지 2명 모두** 리뷰합니다.

## 6. 담당 범위

역할 기준으로 나눕니다. 사람이 바뀌면 이 표의 이름만 바꾸세요.

| 역할 | 담당자 | 1차 담당 폴더 |
|---|---|---|
| A. Vision | (이름) | `ros2/turtle_vision/` |
| B. Firmware / 통신 | (이름) | `stm32/`, `protocol/`, `ros2/turtle_stm32_bridge/` |
| C. ROS 2 / GUI / 통합 | (이름) | `ros2/turtle_interfaces/`, `ros2/turtle_bringup/`, `ros2/turtle_control/`, `gui/` |

- 다른 사람 담당 폴더를 고쳐야 하면 그 사람을 **리뷰어로 지정**합니다.
- "하는 김에" 다른 영역 파일을 고치지 않습니다. 필요하면 PR을 따로 만듭니다.
- 통신 양끝(STM32 ↔ bridge)을 한 사람이 맡아 규약이 어긋나지 않게 합니다.

## 7. Issue

- 버그, 할 일, 논의거리는 Issue로 남깁니다.
- 라벨: `vision` `control` `bridge` `gui` `stm32` `protocol` / `bug` `feature` `question`
- 담당자(Assignee)를 꼭 지정합니다.

## 8. 커밋하면 안 되는 것

- 빌드 결과물 (`build/ install/ log/`, STM32 `Debug/ Release/`, `.elf .bin .hex`)
- 개인 IDE 설정 (`.vscode/`, `*.pro.user`, CubeIDE `*.launch`)
- 개인/환경 전용 값 → `*.local.yaml` 로 저장하면 자동 제외
- 비밀번호, 키, 토큰 (`.env`)
- 대용량 rosbag (`bags/`) → Google Drive 등에 공유

대부분 `.gitignore`가 막아 주지만 `git status`로 한 번 확인하고 커밋하세요.

## 9. main 보호 설정 (레포 관리자 1회)

GitHub → Settings → Branches → Add branch protection rule

- Branch name pattern: `main`
- ✅ Require a pull request before merging
- ✅ Require approvals: 1
