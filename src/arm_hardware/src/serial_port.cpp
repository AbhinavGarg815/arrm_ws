#include "arm_hardware/serial_port.hpp"

// asm/termbits.h gives struct termios2 and BOTHER. It must not be combined with
// glibc's <termios.h>: the two define the same structs differently, and the
// result is a silent mess. Everything below therefore uses ioctl only.
#include <asm/termbits.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace arm_hardware
{

SerialPort::~SerialPort() { close(); }

bool SerialPort::open(const std::string & device, int baud, std::string & error)
{
  close();

  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    error = "cannot open " + device + ": " + std::strerror(errno);
    return false;
  }

  struct termios2 tio {};
  if (::ioctl(fd_, TCGETS2, &tio) != 0) {
    error = "TCGETS2 failed on " + device + ": " + std::strerror(errno);
    close();
    return false;
  }

  // Raw 8N1: no parity, one stop bit, no flow control, no line discipline
  // processing of any kind. A binary status frame contains bytes that a cooked
  // terminal would otherwise interpret.
  tio.c_cflag &= ~static_cast<unsigned>(CBAUD | CSIZE | PARENB | CSTOPB | CRTSCTS);
  tio.c_cflag |= BOTHER | CS8 | CLOCAL | CREAD;
  tio.c_iflag = 0;
  tio.c_oflag = 0;
  tio.c_lflag = 0;
  tio.c_ispeed = static_cast<unsigned>(baud);
  tio.c_ospeed = static_cast<unsigned>(baud);
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;

  if (::ioctl(fd_, TCSETS2, &tio) != 0) {
    error = "TCSETS2 failed on " + device + " at " + std::to_string(baud) +
            " baud: " + std::strerror(errno);
    close();
    return false;
  }
  return true;
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::read_available(std::vector<uint8_t> & buffer)
{
  if (fd_ < 0) {
    return false;
  }
  uint8_t chunk[512];
  for (;;) {
    const ssize_t n = ::read(fd_, chunk, sizeof(chunk));
    if (n > 0) {
      buffer.insert(buffer.end(), chunk, chunk + n);
      continue;                       // there may be more waiting
    }
    if (n == 0) {
      return true;                    // nothing left
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return true;                    // nothing left, non-blocking
    }
    if (errno == EINTR) {
      continue;
    }
    return false;                     // the link is gone
  }
}

bool SerialPort::write_all(const std::string & text)
{
  if (fd_ < 0) {
    return false;
  }
  size_t sent = 0;
  while (sent < text.size()) {
    const ssize_t n = ::write(fd_, text.data() + sent, text.size() - sent);
    if (n > 0) {
      sent += static_cast<size_t>(n);
      continue;
    }
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
      continue;                       // kernel buffer momentarily full
    }
    return false;
  }
  return true;
}

void SerialPort::flush_input()
{
  if (fd_ >= 0) {
    ::ioctl(fd_, TCFLSH, TCIFLUSH);
  }
}

}  // namespace arm_hardware
