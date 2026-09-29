# stm32/

STM32CubeIDE 펌웨어 프로젝트 `turtle_fw`를 둡니다. Dynamixel 2개, PSD 3개, 저수준 속도 제어, Jetson 통신을 담당합니다.

## 구조

```text
stm32/turtle_fw/
├── turtle_fw.ioc          CubeMX 설정 (핀, 클럭, 주변장치)        ✅ 커밋
├── Core/                  CubeMX 자동 생성 (main.c, 주변장치 init) ✅ 커밋
├── Drivers/               HAL, CMSIS                              ✅ 커밋 (팀원이 바로 빌드 가능)
├── App/                   ★ 우리가 직접 쓰는 코드                  ✅ 커밋
│   ├── app.c / app.h      app_init(), app_loop()
│   ├── dynamixel/         Dynamixel Protocol 2.0 송수신
│   ├── psd/               ADC 읽기, 필터, mm 환산
│   ├── comm/              Jetson 패킷 송수신 (protocol/turtle_protocol.h 사용)
│   └── control/           바퀴 속도 제어, 명령 타임아웃
├── STM32xxxx_FLASH.ld     링커 스크립트                            ✅ 커밋
├── .project .cproject .mxproject                                  ✅ 커밋
├── Debug/  Release/       빌드 결과                                ❌ 제외
└── *.launch               개인 디버그 설정                          ❌ 제외
```

## 자동 생성 코드와 직접 쓴 코드 분리

CubeMX로 코드를 다시 생성하면 `Core/` 파일의 `USER CODE BEGIN/END` **밖은 덮어써집니다.**
그래서 직접 쓴 코드는 전부 `App/`에 두고, `main.c`에는 **호출 두 줄만** 넣습니다.

```c
/* USER CODE BEGIN Includes */
#include "app.h"
/* USER CODE END Includes */

/* USER CODE BEGIN 2 */
app_init();
/* USER CODE END 2 */

/* USER CODE BEGIN 3 */
app_loop();
/* USER CODE END 3 */
```

HAL 콜백(`HAL_UART_RxCpltCallback` 등)도 `App/` 쪽 파일에 정의합니다.

## 프로젝트 생성 후 1회 설정

1. CubeMX `Project Manager > Code Generator` → **"Generate peripheral initialization as a pair of '.c/.h' files"** 체크 (main.c 충돌 감소)
2. `App` 폴더 생성 → 우클릭 `Properties > C/C++ Build` 에서 빌드 제외가 아닌지 확인
3. `Project Properties > C/C++ General > Paths and Symbols`
   - Includes에 추가: `App`, `${ProjDirPath}/../../protocol`
   - Source Location에 `App` 추가
4. `.cproject` 변경분까지 커밋

## 주의

- `.ioc`는 **동시에 두 명이 수정하지 않습니다.** 수정 전 팀에 공유하세요.
- 핀맵, Dynamixel ID/보드레이트는 [`../docs/hardware.md`](../docs/hardware.md)에 같이 기록합니다.
- 통신 명령이 `CMD_TIMEOUT_MS` 동안 끊기면 반드시 모터를 정지시킵니다 ([`../protocol/PROTOCOL.md`](../protocol/PROTOCOL.md)).
