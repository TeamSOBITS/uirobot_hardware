#pragma once

#include <fmt/core.h>
#include <libserial/SerialPort.h>

#include <chrono>
#include <string>

namespace uirobot_driver {

class SerialPort {
 public:
  explicit SerialPort(const std::string& dev);
  ~SerialPort();
  bool configure(std::size_t baud_rate = 1000000);
  bool open();
  bool close();
  std::vector<uint8_t> read_and_write(const std::vector<uint8_t>& command);

  std::string dev_;
  std::chrono::milliseconds timeout_ = std::chrono::milliseconds(10);
  LibSerial::SerialPort port_;
};
}  // namespace uirobot_driver
