#include "uirobot_hardware/uirobot_hardware.hpp"
#include "uirobot_hardware/serial_port.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"


namespace uirobot_hardware
{
constexpr const char * kUirobotHardware = "UirobotHardware";

constexpr const char * const kExtraJointParameters[] = {
  "Max_Velocity", // TODO : set the value for motor...
};

CallbackReturn UirobotHardware::on_init(const hardware_interface::HardwareComponentInterfaceParams & info)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kUirobotHardware), "on_init");
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;

  joints_.resize(info_.joints.size(), Joint());
  joint_ids_.resize(info_.joints.size(), 0);

  for (uint i = 0; i < info_.joints.size(); i++) {
    joint_ids_[i] = std::stoi(info_.joints[i].parameters.at("id"));
    joints_[i].state.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].state.velocity = std::numeric_limits<double>::quiet_NaN();
    joints_[i].state.effort = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.velocity = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.effort = std::numeric_limits<double>::quiet_NaN();
    joints_[i].prev_command.position = joints_[i].command.position;
    joints_[i].prev_command.velocity = joints_[i].command.velocity;
    joints_[i].prev_command.effort = joints_[i].command.effort;

    if (info_.joints[i].parameters.find("gear_ratio") != info_.joints[i].parameters.end()) 
      joints_[i].gear_ratio = std::stod(info_.joints[i].parameters.at("gear_ratio"));

    if (info_.joints[i].parameters.find("position_kp") != info_.joints[i].parameters.end()) 
      joints_[i].kp = std::stod(info_.joints[i].parameters.at("position_kp"));

    if (info_.joints[i].parameters.find("stop_threshold") != info_.joints[i].parameters.end()) 
      joints_[i].stop_threshold = std::stod(info_.joints[i].parameters.at("stop_threshold"));

    if (info_.joints[i].parameters.find("max_velocity") != info_.joints[i].parameters.end()) 
      joints_[i].max_vel = std::stod(info_.joints[i].parameters.at("max_velocity"));

    RCLCPP_INFO(rclcpp::get_logger(kUirobotHardware), "joint_id %d: %d", i, joint_ids_[i]);
  }

  // Mimic Initialization
  for (const auto & mimic_data : info_.mimic_joints) {
    uint mimic_idx = mimic_data.joint_index;
    uint src_idx = mimic_data.mimicked_joint_index;

    if (mimic_idx < joints_.size() && src_idx < joints_.size()) {
      joints_[mimic_idx].mimic_index = src_idx;
      joints_[mimic_idx].mimic_multiplier = mimic_data.multiplier;
      joints_[mimic_idx].mimic_offset = mimic_data.offset;

      RCLCPP_INFO(rclcpp::get_logger(kUirobotHardware), 
        "Mimic configured: Joint '%s' (index %d) follows '%s' (index %d) [mult: %f, offset: %f]", 
        info_.joints[mimic_idx].name.c_str(), mimic_idx,
        info_.joints[src_idx].name.c_str(), src_idx,
        joints_[mimic_idx].mimic_multiplier, joints_[mimic_idx].mimic_offset);
    } else {
      RCLCPP_ERROR(rclcpp::get_logger(kUirobotHardware), 
        "Invalid mimic configuration: mimic_index %d or source_index %d out of range", 
        mimic_idx, src_idx);
    }
  }

  // TODO: does this motors have to use the mode...?
  if (
    info_.hardware_parameters.find("use_dummy") != info_.hardware_parameters.end() &&
    (info_.hardware_parameters.at("use_dummy") == "true" ||
    info_.hardware_parameters.at("use_dummy") == "True"))
  {
    use_dummy_ = true;
    RCLCPP_INFO(rclcpp::get_logger(kUirobotHardware), "dummy mode");
    return CallbackReturn::SUCCESS;
  }

  auto port_name = info_.hardware_parameters.at("port_name");
  auto baud_rate = std::stoul(info_.hardware_parameters.at("baud_rate"));

  RCLCPP_INFO(rclcpp::get_logger(kUirobotHardware), "port_name: %s", port_name.c_str());
  RCLCPP_INFO(rclcpp::get_logger(kUirobotHardware), "baud_rate: %ld", baud_rate);

  ser_ = std::make_unique<uirobot_driver::SerialPort>(port_name);
  ser_->configure(static_cast<std::size_t>(baud_rate));

  return CallbackReturn::SUCCESS;
}

CallbackReturn UirobotHardware::on_configure(const rclcpp_lifecycle::State & /* previous_state */) 
{
  RCLCPP_DEBUG(rclcpp::get_logger(kUirobotHardware), "configure");

  for (uint i = 0; i < joints_.size(); i++) {
    if (use_dummy_ && std::isnan(joints_[i].state.position)) {
      joints_[i].state.position = 0.0;
      joints_[i].state.velocity = 0.0;
      joints_[i].state.effort = 0.0;
    }
  }

  if (read(rclcpp::Time{}, rclcpp::Duration(0, 0)) == return_type::ERROR) {
    RCLCPP_ERROR(rclcpp::get_logger(kUirobotHardware), "Read failed in on_configure");
    return CallbackReturn::ERROR;
  }

  enable_torque(false);
  set_joint_params();
  get_joint_params();
  // Ideally torque should be enabled in on_activate(), but this appears to cause issues 
  // due to conflict with RT loop read/write calls, so it is here instead
  enable_torque(true);

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> UirobotHardware::export_state_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger(kUirobotHardware), "export_state_interfaces");
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].state.position));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].state.velocity));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joints_[i].state.effort));
  }

  return state_interfaces;
}

CallbackReturn UirobotHardware::on_activate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kUirobotHardware), "activate");
  reset_command();

  return CallbackReturn::SUCCESS;
}

CallbackReturn UirobotHardware::on_deactivate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kUirobotHardware), "deactivate");
  return CallbackReturn::SUCCESS;
}

return_type UirobotHardware::read(
  const rclcpp::Time & /* time */,
  const rclcpp::Duration & /* period */)
{
  if (use_dummy_) {
    return return_type::OK;
  }

  for(uint i = 0; i < joint_ids_.size(); i++){
    std::vector<uint8_t> cmd = create_commands("get_pos", joint_ids_[i]);
    std::vector<uint8_t> res = ser_->read_and_write(cmd);
    joints_[i].state.position = ((float)analyze_cmd(res, "get_pos")) / joints_[i].cpr * (2*M_PI) / joints_[i].gear_ratio;
    // joints_[i].state.velocity = 0.0; // TODO
    // joints_[i].state.effort = 0.0; // TODO
  }

  // Update Mimic States
  for (auto & joint : joints_) {
    if (joint.mimic_index != -1) {
      const auto & src = joints_[joint.mimic_index];
      double m = joint.mimic_multiplier;

      joint.state.position = (m * src.state.position) + joint.mimic_offset;
      joint.state.velocity = m * src.state.velocity;
      
      // Physically consistent Effort: T_mimic = T_src / multiplier
      if (std::abs(m) > 1e-6) {
        joint.state.effort = src.state.effort / m;
      } else {
        joint.state.effort = 0.0; // Avoid division by zero, but this is a non-physical case
      }
    }
  }

  return return_type::OK;
}

return_type UirobotHardware::write(
  const rclcpp::Time & /* time */,
  const rclcpp::Duration & /* period */)
{
  // Update commands for mimic joints if they are linked to physical IDs
  for (auto & joint : joints_) {
    if (joint.mimic_index != -1) {
      const auto & src = joints_[joint.mimic_index];
      double m = joint.mimic_multiplier;

      joint.command.position = (m * src.command.position) + joint.mimic_offset;
      joint.command.velocity = m * src.command.velocity;
      
      if (std::abs(m) > 1e-6) {
        joint.command.effort = src.command.effort / m;
      } else {
        joint.command.effort = 0.0; // Avoid division by zero, but this is a non-physical case
      }
    }
  }

  // If in dummy mode, just copy commands to states and return
  if (use_dummy_) {
    for (auto & joint : joints_) {
      if (!std::isnan(joint.command.position)) joint.state.position = joint.command.position;
      if (!std::isnan(joint.command.velocity)) joint.state.velocity = joint.command.velocity;
      if (!std::isnan(joint.command.effort))   joint.state.effort   = joint.command.effort;
      
      joint.prev_command = joint.command;
    }
    return return_type::OK;
  }

  if (std::any_of(
      joints_.cbegin(), joints_.cend(), [](auto j) {
        return !std::isnan(j.command.position) && j.command.position != j.prev_command.position;
      }))
  {
    set_joint_positions();
  }
  
  return return_type::OK;
}

return_type UirobotHardware::enable_torque(const bool enabled)
{

  if (enabled && !torque_enabled_) {
    for (uint i = 0; i < info_.joints.size(); ++i) {
      std::vector<uint8_t> cmd = create_commands("on", joint_ids_[i]);
      std::vector<uint8_t> res = ser_->read_and_write(cmd);
      (void)res; // TODO Check the control is Okay?
    }
    RCLCPP_INFO(rclcpp::get_logger(kUirobotHardware), "Torque enabled");
  } else if (!enabled && torque_enabled_) {
    for (uint i = 0; i < info_.joints.size(); ++i) {
      std::vector<uint8_t> cmd = create_commands("off", joint_ids_[i]);
      std::vector<uint8_t> res = ser_->read_and_write(cmd);
      (void)res; // TODO Check the control is Okay?
    }
    RCLCPP_INFO(rclcpp::get_logger(kUirobotHardware), "Torque disabled");
  }

  torque_enabled_ = enabled;
  return return_type::OK;
}

return_type UirobotHardware::reset_command()
{
  for (uint i = 0; i < joints_.size(); i++) {
    joints_[i].command.position = joints_[i].state.position;
    joints_[i].command.velocity = 0.0;
    joints_[i].command.effort = 0.0;
    joints_[i].prev_command.position = joints_[i].command.position;
    joints_[i].prev_command.velocity = joints_[i].command.velocity;
    joints_[i].prev_command.effort = joints_[i].command.effort;
  }

  return return_type::OK;
}

CallbackReturn UirobotHardware::set_joint_positions()
{
  for (size_t i = 0; i < joints_.size(); i++) {

    double target = joints_[i].command.position;
    double current = joints_[i].state.position;

    if (std::isnan(target) || std::isnan(current)) continue;

    double error = target - current;

    double vel =  joints_[i].kp * error;

    // stop near target
    if (std::abs(error) < joints_[i].stop_threshold)
      vel = 0.0;

    // velocity limit: TODO set for motor...
    vel = std::clamp(vel, -std::fabs(joints_[i].max_vel * joints_[i].gear_ratio), std::fabs(joints_[i].max_vel * joints_[i].gear_ratio));

    // rad/s >> pulse/s
    double pps = vel * joints_[i].cpr * joints_[i].gear_ratio / (2*M_PI);

    std::vector<uint8_t> cmd = create_commands("set_vel", joint_ids_[i], 0, static_cast<int32_t>(pps));

    auto res = ser_->read_and_write(cmd);
    (void)res; // TODO Check the control is Okay?

    joints_[i].prev_command.position = joints_[i].command.position;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn UirobotHardware::set_joint_params()
{
  for (uint i = 0; i < info_.joints.size(); ++i) {
    for (auto paramName : kExtraJointParameters) {
      if (info_.joints[i].parameters.find(paramName) != info_.joints[i].parameters.end()) {
        auto value = std::stoi(info_.joints[i].parameters.at(paramName));
        // TODO: Set param list....
        RCLCPP_INFO(
          rclcpp::get_logger(
            kUirobotHardware), "%s set to %d for joint %d", paramName, value, i);
      }
    }
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn UirobotHardware::get_joint_params()
{
  for (uint i = 0; i < joints_.size(); ++i) {
    std::vector<uint8_t> cmd = create_commands("cpr", joint_ids_[i]);
    std::vector<uint8_t> res = ser_->read_and_write(cmd);
    joints_[i].cpr = analyze_cmd(res, "cpr");
  }
  return CallbackReturn::SUCCESS;
}

std::vector<uint8_t> UirobotHardware::create_commands(std::string mode, int id, double pos, double vel)
{
  (void)pos; // TODO pos...

  std::vector<uint8_t> cmd = {header_, static_cast<std::uint8_t>(id)};
  cmd.insert(cmd.end(), 2, static_cast<std::uint8_t>(NAN));
  cmd.insert(cmd.end(), 9, 0);
  cmd.insert(cmd.end(), 2, 0); // TODO: CRC mode...
  cmd.push_back(footer_);

  if (mode == "on") {
    cmd[2] = 0x95;
    cmd[3] = 0x01;
    cmd[4] = 0x01;
  } else if (mode == "off") {
    cmd[2] = 0x95;
    cmd[3] = 0x01;
  } else if (mode == "get_pos") {
    cmd[2] = 0x91;
    cmd[3] = 0x01;
    cmd[4] = 0x01;
  } else if (mode == "cpr") {
    cmd[2] = 0xBD;
    cmd[3] = 0x01;
    cmd[4] = 0x04;
  } else if (mode == "set_vel") {
    cmd[2] = 0x9E;
    cmd[3] = 0x04;
    cmd[4] = (static_cast<int32_t>(vel) & 0xFF);
    cmd[5] = ((static_cast<int32_t>(vel) >> 8) & 0xFF);
    cmd[6] = ((static_cast<int32_t>(vel) >> 16) & 0xFF);
    cmd[7] = ((static_cast<int32_t>(vel) >> 24) & 0xFF);
  } else if (mode == "move") {
    cmd[2] = 0x96;
  } else {
    return {};
  }
  return cmd;
}

int32_t UirobotHardware::analyze_cmd(std::vector<uint8_t> cmd, std::string mode) {
  int32_t val;

  if (mode == "get_pos") {
    val =
      (int32_t)cmd[8] |
      ((int32_t)cmd[9] << 8) |
      ((int32_t)cmd[10] << 16) |
      ((int32_t)cmd[11] << 24);
  } else if (mode == "cpr") {
    val =
      (int32_t)cmd[5] |
      ((int32_t)cmd[6] << 8) |
      ((int32_t)cmd[7] << 16) |
      ((int32_t)cmd[8] << 24);
  } else {
    return 0;
  }

  return val;
}

}  // namespace uirobot_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(uirobot_hardware::UirobotHardware, hardware_interface::SystemInterface)
