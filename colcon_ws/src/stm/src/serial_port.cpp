#include "stm/serial_port.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace stm
{

namespace
{
// 숫자 baud rate -> termios 상수(B115200 등). 지원하지 않는 값이면 0
speed_t to_speed(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: return 0;
  }
}
}  // namespace

SerialPort::~SerialPort() {close();}

bool SerialPort::open(const std::string & port, int baud)
{
  close();   // 이미 열려 있으면 닫고 새로 연다
  const speed_t speed = to_speed(baud);
  if (speed == 0) {
    error_ = "unsupported baud rate " + std::to_string(baud);
    return false;
  }

  // O_RDWR: 읽기+쓰기, O_NOCTTY: 이 장치를 제어 터미널로 잡지 않음, O_NONBLOCK: 기다리지 않는 read
  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    error_ = port + ": " + std::strerror(errno);
    return false;
  }

  // 현재 포트 설정을 읽어 와서 필요한 부분만 바꾼다
  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    error_ = std::string("tcgetattr: ") + std::strerror(errno);
    close();
    return false;
  }
  cfmakeraw(&tty);                      // 가공 없는 raw 모드, 8비트 데이터 / 패리티 없음
  tty.c_cflag |= CLOCAL | CREAD;        // 모뎀 제어선 무시, 수신 켜기
  tty.c_cflag &= ~(CSTOPB | CRTSCTS);   // 정지 비트 1개, 하드웨어 흐름제어(RTS/CTS) 끄기 -> 8N1
  tty.c_cc[VMIN] = 0;                   // read() 는 받은 만큼만 (0바이트여도) 바로 반환
  tty.c_cc[VTIME] = 0;
  cfsetispeed(&tty, speed);             // 수신/송신 속도
  cfsetospeed(&tty, speed);
  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    error_ = std::string("tcsetattr: ") + std::strerror(errno);
    close();
    return false;
  }
  tcflush(fd_, TCIOFLUSH);   // 열기 전에 버퍼에 쌓여 있던 오래된 바이트는 버린다
  return true;
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

ssize_t SerialPort::read(uint8_t * buf, size_t size)
{
  const ssize_t n = ::read(fd_, buf, size);
  if (n < 0) {
    // EAGAIN/EWOULDBLOCK: non-blocking 이라 지금 읽을 데이터가 없다는 뜻 (에러 아님)
    // EINTR: 시그널 때문에 중간에 끊김 (다음에 다시 읽으면 됨)
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
      return 0;
    }
    error_ = std::string("read: ") + std::strerror(errno);
    return -1;
  }
  return n;
}

bool SerialPort::write(const std::vector<uint8_t> & data)
{
  // ::write 는 한 번에 일부만 보낼 수도 있으므로 전부 보낼 때까지 반복한다
  size_t sent = 0;
  while (sent < data.size()) {
    const ssize_t n = ::write(fd_, data.data() + sent, data.size() - sent);
    if (n < 0) {
      if (errno == EINTR) {
        continue;   // 시그널로 끊긴 것뿐이니 다시 시도
      }
      error_ = std::string("write: ") + std::strerror(errno);
      return false;
    }
    sent += static_cast<size_t>(n);
  }
  return true;
}

}  // namespace stm
