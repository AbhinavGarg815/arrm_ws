// A minimal non-blocking serial port.
//
// Uses termios2 / BOTHER rather than the classic constants: the Mega runs at
// 250000 baud and there is no B250000. 250000 divides the ATmega2560's 16 MHz
// clock exactly (0.0% error), where 115200 is 2.1% off.

#ifndef ARM_HARDWARE__SERIAL_PORT_HPP_
#define ARM_HARDWARE__SERIAL_PORT_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace arm_hardware
{

class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort();

  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  /// Open raw, non-blocking, 8N1, no flow control. `error` says why on failure.
  bool open(const std::string & device, int baud, std::string & error);
  void close();
  bool is_open() const { return fd_ >= 0; }

  /// Append whatever has arrived to `buffer`. False only on a real error;
  /// "nothing to read" is success with nothing appended.
  bool read_available(std::vector<uint8_t> & buffer);

  /// Write the whole string. False if it could not all be written.
  bool write_all(const std::string & text);

  /// Discard anything already received.
  void flush_input();

private:
  int fd_{-1};
};

}  // namespace arm_hardware

#endif  // ARM_HARDWARE__SERIAL_PORT_HPP_
