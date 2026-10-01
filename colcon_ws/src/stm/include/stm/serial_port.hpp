#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace stm
{

// termios 기반 non-blocking raw 시리얼 포트
//   raw      : 줄바꿈 변환, 에코 같은 터미널 가공 없이 바이트를 그대로 주고받는다
//   non-blocking : read() 가 데이터가 올 때까지 기다리지 않고 바로 돌아온다
//                  (ROS 타이머 콜백 안에서 호출하므로 멈추면 안 된다)
// 사용 예: open("/dev/ttyUSB0", 115200) -> read()/write() 반복 -> close() (소멸자에서도 자동 close)
class SerialPort
{
public:
  ~SerialPort();

  // 실패 시 false, error()에 사유
  bool open(const std::string & port, int baud);
  void close();
  bool is_open() const {return fd_ >= 0;}

  // 읽은 바이트 수 (데이터 없으면 0), 에러 시 -1
  ssize_t read(uint8_t * buf, size_t size);
  bool write(const std::vector<uint8_t> & data);

  const std::string & error() const {return error_;}

private:
  int fd_ = -1;          // 열린 장치의 파일 디스크립터, -1 이면 닫힌 상태
  std::string error_;    // 마지막 실패 사유 (로그 출력용)
};

}  // namespace stm
