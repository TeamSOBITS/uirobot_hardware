#include "uirobot_hardware/can_port.hpp"

#include <arpa/inet.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>

namespace uirobot_driver
{

CanPort::CanPort(const std::string & interface_name)
: interface_name_(interface_name)
{
}

CanPort::~CanPort()
{
  (void)close();
}

bool CanPort::configure(std::size_t timeout_ms)
{
  timeout_ = std::chrono::milliseconds(timeout_ms);
  if (!open()) {
    return false;
  }

  struct timeval tv;
  tv.tv_sec = static_cast<long>(timeout_.count() / 1000);
  tv.tv_usec = static_cast<long>((timeout_.count() % 1000) * 1000);

  if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
    return false;
  }

  int enable_can_fd = 0;
  (void)setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_can_fd, sizeof(enable_can_fd));
  return true;
}

bool CanPort::open()
{
  if (socket_fd_ >= 0) {
    return true;
  }

  socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    return false;
  }

  struct ifreq ifr;
  std::memset(&ifr, 0, sizeof(ifr));
  std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", interface_name_.c_str());

  if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
    (void)close();
    return false;
  }

  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(socket_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    (void)close();
    return false;
  }

  return true;
}

bool CanPort::close()
{
  if (socket_fd_ < 0) {
    return true;
  }

  if (::close(socket_fd_) < 0) {
    return false;
  }

  socket_fd_ = -1;
  return true;
}

std::vector<uint8_t> CanPort::read_and_write(const std::vector<uint8_t> & command)
{
  if (command.size() < 3 || socket_fd_ < 0) {
    return {};
  }

  const auto node_id = command[0];
  const auto control_word = command[1];
  const auto data_length = static_cast<uint8_t>(std::min<size_t>(command[2], CAN_MAX_DLEN));

  struct can_frame tx_frame;
  std::memset(&tx_frame, 0, sizeof(tx_frame));
  tx_frame.can_id = make_can_id(node_id, control_word);
  tx_frame.can_dlc = data_length;
  for (size_t i = 0; i < data_length && (3 + i) < command.size(); ++i) {
    tx_frame.data[i] = command[3 + i];
  }

  const auto bytes_written = write(socket_fd_, &tx_frame, sizeof(tx_frame));
  if (bytes_written != static_cast<ssize_t>(sizeof(tx_frame))) {
    return {};
  }

  const auto deadline = std::chrono::steady_clock::now() + timeout_;

  while (std::chrono::steady_clock::now() < deadline) {
    struct can_frame rx_frame;
    const auto bytes_read = read(socket_fd_, &rx_frame, sizeof(rx_frame));
    if (bytes_read < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        continue;
      }
      return {};
    }

    if (bytes_read != static_cast<ssize_t>(sizeof(rx_frame)) || !(rx_frame.can_id & CAN_EFF_FLAG)) {
      continue;
    }

    const auto parsed = parse_frame(rx_frame.can_id, rx_frame.data, rx_frame.can_dlc);
    if (!parsed.empty() && parsed[0] == node_id) {
      return parsed;
    }
  }

  return {};
}

uint32_t CanPort::make_can_id(uint8_t node_id, uint8_t control_word) const
{
  const uint32_t sid = ((static_cast<uint32_t>(node_id) << 1U) & 0x003FU) | 0x0100U;
  const uint32_t eid = (((static_cast<uint32_t>(node_id) << 1U) & 0x00C0U) << 8U) |
    static_cast<uint32_t>(control_word);
  return CAN_EFF_FLAG | (sid << 18U) | eid;
}

std::vector<uint8_t> CanPort::parse_frame(uint32_t can_id, const uint8_t * data, uint8_t dlc) const
{
  const auto eff_id = can_id & CAN_EFF_MASK;
  const auto sid = static_cast<uint16_t>((eff_id >> 18U) & 0x07FFU);
  const auto eid = static_cast<uint32_t>(eff_id & 0x3FFFFU);

  const auto node_id = static_cast<uint8_t>(((eid >> 11U) & 0x60U) | ((sid >> 6U) & 0x1FU));
  const auto control_word = static_cast<uint8_t>(eid & 0xFFU);

  std::vector<uint8_t> parsed(11, 0);
  parsed[0] = node_id;
  parsed[1] = control_word;
  parsed[2] = dlc;
  for (size_t i = 0; i < dlc && i < CAN_MAX_DLEN; ++i) {
    parsed[3 + i] = data[i];
  }
  return parsed;
}

}  // namespace uirobot_driver
