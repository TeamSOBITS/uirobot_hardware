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
constexpr const uint8_t header_ = 0xAD; // 0xAA - TODO: CRC mode...
constexpr const uint8_t footer_ = 0xCC;

constexpr const char * const kExtraJointParameters[] = {
  "Max_Velocity", // TODO : set the value for motor...
};

CallbackReturn UirobotHardware::on_init(const hardware_interface::HardwareComponentInterfaceParams & info)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kUirobotHardware), "on_init");
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

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

    if (info_.joints[i].parameters.find("gear_ratio") != info_.joints[i].parameters.end()) {
      joints_[i].gear_ratio = std::stod(info_.joints[i].parameters.at("gear_ratio"));
    }

    if (info_.joints[i].parameters.find("position_kp") != info_.joints[i].parameters.end()) {
      joints_[i].kp = std::stod(info_.joints[i].parameters.at("position_kp"));
    }

    if (info_.joints[i].parameters.find("stop_threshold") != info_.joints[i].parameters.end()) {
      joints_[i].stop_threshold = std::stod(info_.joints[i].parameters.at("stop_threshold"));
    }

    if (info_.joints[i].parameters.find("max_velocity") != info_.joints[i].parameters.end()) {
      joints_[i].max_vel = std::stod(info_.joints[i].parameters.at("max_velocity"));
    }

    for (const auto & command_interface : info_.joints[i].command_interfaces) {
      if (command_interface.name == hardware_interface::HW_IF_POSITION) {
        if (!command_interface.min.empty()) {
          joints_[i].min_pos = std::stod(command_interface.min);
        }
        if (!command_interface.max.empty()) {
          joints_[i].max_pos = std::stod(command_interface.max);
        }
        break;
      }
    }

    RCLCPP_INFO(rclcpp::get_logger(kUirobotHardware), "joint_id %d: %d", i, joint_ids_[i]);
    if (std::isfinite(joints_[i].min_pos) || std::isfinite(joints_[i].max_pos)) {
      const std::string min_pos_str =
        std::isfinite(joints_[i].min_pos) ? std::to_string(joints_[i].min_pos) : "none";
      const std::string max_pos_str =
        std::isfinite(joints_[i].max_pos) ? std::to_string(joints_[i].max_pos) : "none";
      RCLCPP_INFO(
        rclcpp::get_logger(kUirobotHardware),
        "Joint '%s' position limits: min=%s max=%s",
        info_.joints[i].name.c_str(),
        min_pos_str.c_str(),
        max_pos_str.c_str());
    }
  }

  for (const auto & mimic_data : info_.mimic_joints) {
    uint mimic_idx = mimic_data.joint_index;
    uint src_idx = mimic_data.mimicked_joint_index;

    if (mimic_idx < joints_.size() && src_idx < joints_.size()) {
      joints_[mimic_idx].mimic_index = src_idx;
      joints_[mimic_idx].mimic_multiplier = mimic_data.multiplier;
      joints_[mimic_idx].mimic_offset = mimic_data.offset;

      RCLCPP_INFO(
        rclcpp::get_logger(kUirobotHardware),
        "Mimic configured: Joint '%s' (index %d) follows '%s' (index %d) [mult: %f, offset: %f]",
        info_.joints[mimic_idx].name.c_str(), mimic_idx,
        info_.joints[src_idx].name.c_str(), src_idx,
        joints_[mimic_idx].mimic_multiplier, joints_[mimic_idx].mimic_offset);
    } else {
      RCLCPP_ERROR(
        rclcpp::get_logger(kUirobotHardware),
        "Invalid mimic configuration: mimic_index %d or source_index %d out of range",
        mimic_idx, src_idx);
    }
  }

  if (
    info_.hardware_parameters.find("use_dummy") != info_.hardware_parameters.end() &&
    (info_.hardware_parameters.at("use_dummy") == "true" ||
    info_.hardware_parameters.at("use_dummy") == "True"))
  {
    use_dummy_ = true;
    RCLCPP_INFO(rclcpp::get_logger(kUirobotHardware), "dummy mode");
    return CallbackReturn::SUCCESS;
  }

  if (info_.hardware_parameters.find("toggle_torque_on_configure") != info_.hardware_parameters.end()) {
    const auto & value = info_.hardware_parameters.at("toggle_torque_on_configure");
    toggle_torque_on_configure_ = !(value == "false" || value == "False" || value == "0");
  }
  if (info_.hardware_parameters.find("enable_torque_before_motion") != info_.hardware_parameters.end()) {
    const auto & value = info_.hardware_parameters.at("enable_torque_before_motion");
    enable_torque_before_motion_ = (value == "true" || value == "True" || value == "1");
  }
  RCLCPP_INFO(
  rclcpp::get_logger(kUirobotHardware),
  "toggle_torque_on_configure: %s",
  toggle_torque_on_configure_ ? "true" : "false");
  RCLCPP_INFO(
  rclcpp::get_logger(kUirobotHardware),
  "enable_torque_before_motion: %s",
  enable_torque_before_motion_ ? "true" : "false");


  auto port_name = info_.hardware_parameters.at("port_name");
  auto baud_rate = std::stoul(info_.hardware_parameters.at("baud_rate"));

  RCLCPP_INFO(rclcpp::get_logger(kUirobotHardware), "port_name: %s", port_name.c_str());
  RCLCPP_INFO(rclcpp::get_logger(kUirobotHardware), "baud_rate: %ld", baud_rate);

  ser_ = std::make_unique<uirobot_driver::SerialPort>(port_name);
  ser_->configure(static_cast<std::size_t>(baud_rate));

  return CallbackReturn::SUCCESS;
}

CallbackReturn UirobotHardware::on_configure(const rclcpp_lifecycle::State &)
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

  if (toggle_torque_on_configure_) {
    enable_torque(false);
  }
  set_joint_params();
  get_joint_params();
  if (toggle_torque_on_configure_) {
    enable_torque(true);
  }

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

std::vector<hardware_interface::CommandInterface> UirobotHardware::export_command_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger(kUirobotHardware), "export_command_interfaces");

  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_POSITION,
        &joints_[i].command.position));

    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_VELOCITY,
        &joints_[i].command.velocity));

    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_EFFORT,
        &joints_[i].command.effort));
  }

  return command_interfaces;
}

CallbackReturn UirobotHardware::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kUirobotHardware), "activate");
  reset_command();

  return CallbackReturn::SUCCESS;
}

CallbackReturn UirobotHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kUirobotHardware), "deactivate");
  return CallbackReturn::SUCCESS;
}

return_type UirobotHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (use_dummy_) {
    return return_type::OK;
  }

  for (uint i = 0; i < joint_ids_.size(); i++) {
    std::vector<uint8_t> cmd = create_commands("get_pos", joint_ids_[i]);
    std::vector<uint8_t> res = ser_->read_and_write(cmd);
    if (res.size() < 12) {
      RCLCPP_ERROR(
        rclcpp::get_logger(kUirobotHardware),
        "Failed to read position for joint '%s' (reply size=%zu)",
        info_.joints[i].name.c_str(), res.size());
      return return_type::ERROR;
    }
    const double position =
      (static_cast<float>(analyze_cmd(res, "get_pos")) / joints_[i].cpr * (2 * M_PI)) /
      joints_[i].gear_ratio;

    // The UIM2513 gateway occasionally returns a transient zero-position sample.
    // Keep the previous valid position instead of injecting a false origin jump.
    if (
      std::isfinite(joints_[i].state.position) &&
      std::abs(position) < 1e-9 &&
      std::abs(joints_[i].state.position) > 0.02)
    {
      continue;
    }

    joints_[i].state.position = position;
  }

  for (auto & joint : joints_) {
    if (joint.mimic_index != -1) {
      const auto & src = joints_[joint.mimic_index];
      double m = joint.mimic_multiplier;

      joint.state.position = (m * src.state.position) + joint.mimic_offset;
      joint.state.velocity = m * src.state.velocity;

      if (std::abs(m) > 1e-6) {
        joint.state.effort = src.state.effort / m;
      } else {
        joint.state.effort = 0.0;
      }
    }
  }

  return return_type::OK;
}

return_type UirobotHardware::write(const rclcpp::Time &, const rclcpp::Duration & period)
{
  for (auto & joint : joints_) {
    if (joint.mimic_index != -1) {
      const auto & src = joints_[joint.mimic_index];
      double m = joint.mimic_multiplier;

      joint.command.position = (m * src.command.position) + joint.mimic_offset;
      joint.command.velocity = m * src.command.velocity;

      if (std::abs(m) > 1e-6) {
        joint.command.effort = src.command.effort / m;
      } else {
        joint.command.effort = 0.0;
      }
    }
  }

  if (use_dummy_) {
    for (auto & joint : joints_) {
      if (!std::isnan(joint.command.position)) joint.state.position = joint.command.position;
      if (!std::isnan(joint.command.velocity)) joint.state.velocity = joint.command.velocity;
      if (!std::isnan(joint.command.effort)) joint.state.effort = joint.command.effort;

      joint.prev_command = joint.command;
    }
    return return_type::OK;
  }

  if (std::any_of(
      joints_.cbegin(), joints_.cend(), [](auto j) {
        return !std::isnan(j.command.position) && j.command.position != j.prev_command.position;
      }))
  {
    set_joint_positions(period);
  }

  return return_type::OK;
}

return_type UirobotHardware::enable_torque(const bool enabled)
{
  if (enabled && !torque_enabled_) {
    for (uint i = 0; i < info_.joints.size(); ++i) {
      std::vector<uint8_t> cmd = create_commands("on", joint_ids_[i]);
      std::vector<uint8_t> res = ser_->read_and_write(cmd);
      if (res.empty()) {
        return return_type::ERROR;
      }
    }
    RCLCPP_INFO(rclcpp::get_logger(kUirobotHardware), "Torque enabled");
  } else if (!enabled && torque_enabled_) {
    for (uint i = 0; i < info_.joints.size(); ++i) {
      std::vector<uint8_t> cmd = create_commands("off", joint_ids_[i]);
      std::vector<uint8_t> res = ser_->read_and_write(cmd);
      if (res.empty()) {
        return return_type::ERROR;
      }
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

CallbackReturn UirobotHardware::set_joint_positions(const rclcpp::Duration & period)
{
  const double dt = std::max(period.seconds(), 1e-3);

  for (size_t i = 0; i < joints_.size(); i++) {
    double target = joints_[i].command.position;
    double current = joints_[i].state.position;
    double prev_target = joints_[i].prev_command.position;

    if (std::isnan(target) || std::isnan(current) || std::isnan(prev_target)) continue;

    const double trajectory_vel = (target - prev_target) / dt;
    const double correction_vel = (target - current) * joints_[i].kp;
    double vel = trajectory_vel + correction_vel;
    vel = std::clamp(
      vel,
      -std::fabs(joints_[i].max_vel),
      std::fabs(joints_[i].max_vel));

    double pps = vel * joints_[i].cpr * joints_[i].gear_ratio / (2 * M_PI);
    double pls = target * joints_[i].cpr * joints_[i].gear_ratio / (2 * M_PI);

    // RCLCPP_INFO(
    //   rclcpp::get_logger(kUirobotHardware),
    //   "Joint '%s' command: target=%.6f current=%.6f prev_target=%.6f (pls=%.2f) "
    //   "traj_vel=%.6f corr_vel=%.6f vel=%.6f m/s (pps=%.2f)",
    //   info_.joints[i].name.c_str(), target, current, prev_target, pls,
    //   trajectory_vel, correction_vel, vel, pps);

    // UIM342 PTP expects target position first, then target speed, then begin motion.
    std::vector<uint8_t> cmd = create_commands("set_pos", joint_ids_[i], static_cast<int32_t>(pls), 0);
    auto res = ser_->read_and_write(cmd);
    if (res.empty()) {
      return CallbackReturn::ERROR;
    }

    cmd = create_commands("set_vel", joint_ids_[i], 0, static_cast<int32_t>(pps));
    res = ser_->read_and_write(cmd);
    if (res.empty()) {
      return CallbackReturn::ERROR;
    }

    cmd = create_commands("move", joint_ids_[i]);
    res = ser_->read_and_write(cmd);
    if (res.empty()) {
      return CallbackReturn::ERROR;
    }

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
        RCLCPP_INFO(
          rclcpp::get_logger(kUirobotHardware), "%s set to %d for joint %d", paramName, value, i);
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
    if (res.size() < 9) {
      return CallbackReturn::ERROR;
    }
    joints_[i].cpr = analyze_cmd(res, "cpr");
    RCLCPP_INFO(
      rclcpp::get_logger(kUirobotHardware),
      "Joint '%s' CPR from controller: %d",
      info_.joints[i].name.c_str(), joints_[i].cpr);
  }
  return CallbackReturn::SUCCESS;
}

std::vector<uint8_t> UirobotHardware::create_commands(std::string mode, int id, double pos, double vel)
{
  std::vector<uint8_t> cmd = {header_, static_cast<std::uint8_t>(id)};
  cmd.insert(cmd.end(), 2, 0);
  cmd.insert(cmd.end(), 9, 0);
  cmd.insert(cmd.end(), 2, 0);
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
  } else if (mode == "set_pos") {
    // Absolute position command (PA). ROS publishes absolute joint targets.
    cmd[2] = 0xA0;
    cmd[3] = 0x04;
    cmd[4] = (static_cast<int32_t>(pos) & 0xFF);
    cmd[5] = ((static_cast<int32_t>(pos) >> 8) & 0xFF);
    cmd[6] = ((static_cast<int32_t>(pos) >> 16) & 0xFF);
    cmd[7] = ((static_cast<int32_t>(pos) >> 24) & 0xFF);
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

int32_t UirobotHardware::analyze_cmd(std::vector<uint8_t> cmd, std::string mode)
{
  if (mode == "get_pos" && cmd.size() < 12) {
    return 0;
  }
  if (mode == "cpr" && cmd.size() < 9) {
    return 0;
  }

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
