#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace stm
{

// termios 기반 non-blocking raw 시리얼 포트
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
  int fd_ = -1;
  std::string error_;
};

}  // namespace stm
