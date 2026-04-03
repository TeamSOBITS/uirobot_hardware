#ifndef UIROBOT_HARDWARE__UIROBOT_HARDWARE_HPP_
#define UIROBOT_HARDWARE__UIROBOT_HARDWARE_HPP_

#include <rclcpp/macros.hpp>

#include <map>
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "uirobot_hardware/visiblity_control.h"
#include "uirobot_hardware/serial_port.hpp"

namespace uirobot_hardware
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using return_type = hardware_interface::return_type;

struct JointValue
{
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
};

struct Joint
{
  JointValue state{};
  JointValue command{};
  JointValue prev_command{};
  double gear_ratio{1.0};
  double kp{1.0};
  double stop_threshold{0.005};
  double max_vel{0.5};
  int cpr{1}; // Counts Per Revolution

  // Mimic joint parameters
  int mimic_index{-1};  // -1 if not a mimic joint, otherwise index of the source joint
  double mimic_multiplier{1.0};
  double mimic_offset{0.0};
};


class UirobotHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(UirobotHardware)

  UIROBOT_HARDWARE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & info) override;

  UIROBOT_HARDWARE_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  UIROBOT_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  UIROBOT_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  UIROBOT_HARDWARE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  UIROBOT_HARDWARE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  UIROBOT_HARDWARE_PUBLIC
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  UIROBOT_HARDWARE_PUBLIC
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  return_type enable_torque(const bool enabled);

  return_type reset_command();

  CallbackReturn set_joint_positions(const rclcpp::Duration & period);
  CallbackReturn set_joint_params();
  CallbackReturn get_joint_params();

  std::vector<uint8_t> create_commands(std::string mode, int id, double pos=0.0, double vel=0.0);
  int32_t analyze_cmd(std::vector<uint8_t> cmd, std::string mode);

  std::vector<Joint> joints_;
  std::vector<uint8_t> joint_ids_;
  bool torque_enabled_{false};
  bool use_dummy_{false};

  std::unique_ptr<uirobot_driver::SerialPort> ser_;
};
}  // namespace uirobot_hardware

#endif  // UIROBOT_HARDWARE__UIROBOT_HARDWARE_HPP_
