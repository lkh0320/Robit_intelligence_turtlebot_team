# Jetson ↔ STM32 통신 규약 (초안 v1)

> 코드상 원본은 [`turtle_protocol.h`](turtle_protocol.h) 입니다. 이 문서와 헤더가 다르면 **헤더가 맞는 것**이고, 문서를 고쳐 주세요.

## 누가 이 파일을 쓰나

```text
stm32/turtle_fw/App/comm/      ──include──┐
                                          ├──▶ protocol/turtle_protocol.h
ros2/turtle_stm32_bridge/      ──include──┘
```

두 쪽이 **같은 헤더**를 쓰므로 한쪽만 고쳐서 어긋나는 일이 없습니다.

## 물리 계층

| 항목 | 값 |
|---|---|
| 연결 | UART (USB-시리얼 또는 ST-Link VCP) |
| 보드레이트 | 115200 (config/stm32_bridge.yaml) |
| 형식 | 8N1 |
| 바이트 순서 | Little-endian (Jetson, STM32 모두) |

## 프레임

```text
| 0xAA | 0x55 | ID | LEN | PAYLOAD (LEN 바이트) | CHECKSUM |
```

- `CHECKSUM` = (`ID` + `LEN` + payload 모든 바이트) & 0xFF
- 체크섬이 틀린 패킷은 버리고 `ERR_BAD_CHECKSUM`을 세웁니다.

## 패킷 목록

### Jetson → STM32

| ID | 이름 | LEN | Payload | 주기 |
|---|---|---|---|---|
| 0x01 | MOTOR_CMD | 5 | `int16 left_mmps, int16 right_mmps, uint8 enable` | 20~50 Hz |
| 0x02 | SET_PARAM | 5 | `uint8 param_id, float value` | 값 바뀔 때 |
| 0x03 | HEARTBEAT | 0 | 없음 | 10 Hz (MOTOR_CMD 없을 때) |

### STM32 → Jetson

| ID | 이름 | LEN | Payload | 주기 |
|---|---|---|---|---|
| 0x81 | SENSOR | 12 | `uint16 psd_raw[3], uint16 psd_mm[3]` (좌, 중앙, 우) | 50 Hz |
| 0x82 | MOTOR_STATE | 6 | `int16 left_mmps, int16 right_mmps, uint8 left_error, uint8 right_error` | 20 Hz |
| 0x83 | STATUS | 4 | `uint8 protocol_version, uint8 state, uint16 error_flags` | 5 Hz |

## 안전 규칙

- STM32는 **`CMD_TIMEOUT_MS`(200 ms)** 동안 MOTOR_CMD나 HEARTBEAT를 못 받으면 모터를 정지하고 `ERR_CMD_TIMEOUT`을 세웁니다. (Jetson이 죽거나 케이블이 빠졌을 때 폭주 방지)
- `enable = 0`인 MOTOR_CMD는 속도값과 관계없이 즉시 정지입니다. GUI의 비상정지는 결국 이 패킷으로 전달됩니다.
- bridge는 시작 시 STATUS의 `protocol_version`이 헤더와 같은지 확인하고 다르면 에러 로그를 띄웁니다.

## 변경 절차

1. 이슈/단톡으로 변경 내용 공유
2. `turtle_protocol.h` 수정 + `PROTOCOL_VERSION` 1 증가 + 이 문서 표 수정
3. STM32 코드와 bridge 코드를 **같은 PR**에서 수정
4. 나머지 2명 리뷰 후 merge
