#include <spdlog/spdlog.h>
#include "uirobot_hardware/serial_port.hpp"

namespace uirobot_driver {

LibSerial::BaudRate to_baudrate(const std::size_t baud) noexcept {
  using LibSerial::BaudRate;
  switch (baud) {
    case 50:
      return BaudRate::BAUD_50;
    case 75:
      return BaudRate::BAUD_75;
    case 110:
      return BaudRate::BAUD_110;
    case 134:
      return BaudRate::BAUD_134;
    case 150:
      return BaudRate::BAUD_150;
    case 200:
      return BaudRate::BAUD_200;
    case 300:
      return BaudRate::BAUD_300;
    case 600:
      return BaudRate::BAUD_600;
    case 1'200:
      return BaudRate::BAUD_1200;
    case 1'800:
      return BaudRate::BAUD_1800;
    case 2'400:
      return BaudRate::BAUD_2400;
    case 4'800:
      return BaudRate::BAUD_4800;
    case 9'600:
      return BaudRate::BAUD_9600;
    case 19'200:
      return BaudRate::BAUD_19200;
    case 38'400:
      return BaudRate::BAUD_38400;
    case 57'600:
      return BaudRate::BAUD_57600;
    case 115'200:
      return BaudRate::BAUD_115200;
    case 230'400:
      return BaudRate::BAUD_230400;
#ifdef __linux__
    case 460'800:
      return BaudRate::BAUD_460800;
    case 500'000:
      return BaudRate::BAUD_500000;
    case 576'000:
      return BaudRate::BAUD_576000;
    case 921'600:
      return BaudRate::BAUD_921600;
    case 1'000'000:
      return BaudRate::BAUD_1000000;
    case 1'152'000:
      return BaudRate::BAUD_1152000;
    case 1'500'000:
      return BaudRate::BAUD_1500000;
#if __MAX_BAUD > B2000000
    case 2'000'000:
      return BaudRate::BAUD_2000000;
    case 2'500'000:
      return BaudRate::BAUD_2500000;
    case 3'000'000:
      return BaudRate::BAUD_3000000;
    case 3'500'000:
      return BaudRate::BAUD_3500000;
    case 4'000'000:
      return BaudRate::BAUD_4000000;
#endif /* __MAX_BAUD */
#endif /* __linux__ */
  }

  return BaudRate::BAUD_57600; // default
}

SerialPort::SerialPort(const std::string& dev) : dev_(dev) { spdlog::info("Connecting to port: {}", dev); }

SerialPort::~SerialPort() { (void)close(); }

bool SerialPort::configure(std::size_t baud_rate) {
  if (!open()) {
    return false;
  }

  try {
    port_.SetBaudRate(to_baudrate(baud_rate));
  } catch (const std::runtime_error& e) {
    return false;
  }
  return true;
}

bool SerialPort::open() {
  try {
    if (!port_.IsOpen()) {
      port_.Open(dev_);
    }
  } catch (const LibSerial::OpenFailed& e) {
    return false;
  }

  return true;
}

bool SerialPort::close() {
  try {
    port_.Close();
  } catch (const LibSerial::AlreadyOpen& e) {
    return false;
  } catch (const std::runtime_error& e) {
    return false;
  }
  return true;
}

std::vector<uint8_t> SerialPort::read_and_write(const std::vector<uint8_t>& command) {
  try {
    port_.Write(command);
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    LibSerial::DataBuffer buffer;
    port_.Read(buffer, 16, 1000);
    std::vector<uint8_t> response(buffer.begin(), buffer.end());
    return response;
  }
  catch (...) {
    return {};
  }
}

}  // namespace uirobot_driver
