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
  close();
  const speed_t speed = to_speed(baud);
  if (speed == 0) {
    error_ = "unsupported baud rate " + std::to_string(baud);
    return false;
  }

  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    error_ = port + ": " + std::strerror(errno);
    return false;
  }

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    error_ = std::string("tcgetattr: ") + std::strerror(errno);
    close();
    return false;
  }
  cfmakeraw(&tty);
  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cflag &= ~(CSTOPB | CRTSCTS);
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);
  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    error_ = std::string("tcsetattr: ") + std::strerror(errno);
    close();
    return false;
  }
  tcflush(fd_, TCIOFLUSH);
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
  size_t sent = 0;
  while (sent < data.size()) {
    const ssize_t n = ::write(fd_, data.data() + sent, data.size() - sent);
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      error_ = std::string("write: ") + std::strerror(errno);
      return false;
    }
    sent += static_cast<size_t>(n);
  }
  return true;
}

}  // namespace stm
