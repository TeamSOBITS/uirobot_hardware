#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace uirobot_driver
{

class CanPort
{
public:
  explicit CanPort(const std::string & interface_name);
  ~CanPort();

  bool configure(std::size_t timeout_ms = 100);
  bool open();
  bool close();
  std::vector<uint8_t> read_and_write(const std::vector<uint8_t> & command);

private:
  uint32_t make_can_id(uint8_t node_id, uint8_t control_word) const;
  std::vector<uint8_t> parse_frame(uint32_t can_id, const uint8_t * data, uint8_t dlc) const;

  std::string interface_name_;
  std::chrono::milliseconds timeout_{100};
  int socket_fd_{-1};
};

}  // namespace uirobot_driver
