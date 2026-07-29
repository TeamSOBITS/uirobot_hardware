#include "uirobot_hardware/uirobot_hardware.hpp"
#include "uirobot_hardware/serial_port.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include <bits/stdc++.h>

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
  joint_modes_.resize(info_.joints.size(), "");

  for (uint i = 0; i < info_.joints.size(); i++) {
    joint_ids_[i] = std::stoi(info_.joints[i].parameters.at("id"));
    joint_modes_[i] = info_.joints[i].parameters.at("mode");
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

    if (info_.joints[i].parameters.find("min_velocity") != info_.joints[i].parameters.end()) {
      joints_[i].min_vel = std::stod(info_.joints[i].parameters.at("min_velocity"));
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
    RCLCPP_INFO(rclcpp::get_logger(kUirobotHardware), "joint_mode %d: %s", i, joint_modes_[i].c_str());
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

  // RCLCPP_INFO(this->get_logger(), "joint_modes_[i] DEBUG: %s", joint_modes_[i]);


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

  for (size_t i = 0; i < joint_ids_.size(); i++) {
    RCLCPP_INFO(rclcpp::get_logger(kUirobotHardware), "Releasing brake for Joint ID: %d ...", joint_ids_[i]);
    if (!release_brake_hardware(joint_ids_[i])) {
      RCLCPP_ERROR(rclcpp::get_logger(kUirobotHardware), "Failed to release brake for Joint ID: %d", joint_ids_[i]);
    } else {
      RCLCPP_INFO(rclcpp::get_logger(kUirobotHardware), "Brake released successfully for Joint ID: %d", joint_ids_[i]);
    }
  }

  return CallbackReturn::SUCCESS;
}

uint16_t UirobotHardware::calculate_modbus_crc(const std::vector<uint8_t> & data)
{
  uint16_t crc = 0xFFFF;
  for (uint8_t b : data) {
    crc ^= b;
    for (int i = 0; i < 8; ++i) {
      if (crc & 1) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc & 0xFFFF;
}

bool UirobotHardware::release_brake_hardware(uint8_t device_id)
{
  if (!ser_) {//When the serial number isn't even connected in the first place
    RCLCPP_WARN(rclcpp::get_logger(kUirobotHardware), "⚠️ [ID: %d] Serial port object is null.", device_id);
    return false;
  }

  // Construct the command based on release_brake.py.
  std::vector<uint8_t> payload = {
    device_id, 0x90, 0x03,
    0x05, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };

  uint16_t crc = calculate_modbus_crc(payload);
  uint8_t crc_lo = crc & 0xFF;
  uint8_t crc_hi = (crc >> 8) & 0xFF;

  // Construct the entire packet (Header: 0xAA, Footer: 0xCC)
  std::vector<uint8_t> packet;
  packet.reserve(1 + payload.size() + 2 + 1);
  packet.push_back(0xAA); // Header
  packet.insert(packet.end(), payload.begin(), payload.end());
  packet.push_back(crc_lo);
  packet.push_back(crc_hi);
  packet.push_back(0xCC); // Footer

  // Send command to motor and receive response
  auto reply = ser_->read_and_write(packet);

  // If transmission fails or empty response is received, output visual log
  if (reply.empty()) {
    RCLCPP_WARN(rclcpp::get_logger(kUirobotHardware), "⚠️ [ID: %d] Connection failed. No response received.", device_id);
    return false;
  }

  if (reply.size() >= 4 && reply[0] == 0xAA && reply[1] == device_id) {
    RCLCPP_INFO(rclcpp::get_logger(kUirobotHardware), "✅ [ID: %d] Brake released successfully.", device_id);
    return true;
  }

  RCLCPP_WARN(rclcpp::get_logger(kUirobotHardware), "⚠️ [ID: %d] Invalid response format.", device_id);
  return false;
}

CallbackReturn UirobotHardware::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kUirobotHardware), "configure");

  for (uint i = 0; i < joints_.size(); i++) {
    if (std::isnan(joints_[i].state.position)) {
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
  std::vector<JointValue> prev_states;
  prev_states.reserve(joints_.size());
  for (const auto & joint : joints_) {
    prev_states.push_back(joint.state);
  }

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

    std::vector<uint8_t> cmd_vel = create_commands("get_vel", joint_ids_[i]);
    std::vector<uint8_t> res_vel = ser_->read_and_write(cmd_vel);
    if (res_vel.size() < 8) {
      RCLCPP_ERROR(
        rclcpp::get_logger(kUirobotHardware),
        "Failed to read velocity for joint '%s' (reply size=%zu)",
        info_.joints[i].name.c_str(), res_vel.size());
      return return_type::ERROR;
    }
    
    int32_t raw_velocity = analyze_cmd(res_vel, "get_vel");
    // Conversion from pps to m/s
    const double velocity =
      (static_cast<double>(raw_velocity) / joints_[i].cpr * (2 * M_PI)) / joints_[i].gear_ratio;
    
    // Debug logs
    // RCLCPP_INFO(this->get_logger(), "raw_velocity DEBUG: %d", raw_velocity);
    // RCLCPP_INFO(this->get_logger(), "velocity DEBUG: %f", velocity);
    // RCLCPP_INFO(this->get_logger(), "joints_[i].cpr DEBUG: %d", joints_[i].cpr);
    // RCLCPP_INFO(this->get_logger(), "joints_[i].kp DEBUG: %f", joints_[i].kp);

    joints_[i].state.position = position;
    joints_[i].state.velocity = velocity;
    joints_[i].state.effort   = 0.0;
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

  for (uint i = 0; i < joints_.size(); ++i) {
    if (std::isnan(joints_[i].state.position)) {
      joints_[i].state.position = std::isnan(prev_states[i].position) ? 0.0 : prev_states[i].position;
    }
    if (std::isnan(joints_[i].state.velocity)) {
      joints_[i].state.velocity = std::isnan(prev_states[i].velocity) ? 0.0 : prev_states[i].velocity;
    }
    if (std::isnan(joints_[i].state.effort)) {
      joints_[i].state.effort = std::isnan(prev_states[i].effort) ? 0.0 : prev_states[i].effort;
    }
  }

  return return_type::OK;
}

//Code for mode switching (retaining both PTP and JOG)
hardware_interface::return_type UirobotHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Handling of Mimic joints
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

  bool target_changed = false;
  bool target_not_reached = false; 

  // Loop processing for each joint
  for (size_t i = 0; i < joints_.size(); i++) {

    if (joint_modes_[i] == "JOG") {
      if (std::isnan(joints_[i].command.position) || std::isnan(joints_[i].command.velocity)) continue;

      // Check if the command values themselves have changed
      if (std::isnan(joints_[i].prev_command.position) || 
          std::abs(joints_[i].command.position - joints_[i].prev_command.position) > 1e-6 ||
          std::abs(joints_[i].command.velocity - joints_[i].prev_command.velocity) > 1e-6) {
        target_changed = true;
      }

      // Check if the current position has not yet reached the target value (within stop_threshold)
      double position_error = joints_[i].command.position - joints_[i].state.position;
      if (std::abs(position_error) >= joints_[i].stop_threshold) {
        target_not_reached = true;
      }
    } 
    else if (joint_modes_[i] == "PTP") {
      if (std::isnan(joints_[i].command.position)) continue;

      if (std::isnan(joints_[i].prev_command.position) || 
          std::abs(joints_[i].command.position - joints_[i].prev_command.position) > 1e-3) { 
        target_changed = true;
        break;
      }
    } else {
      RCLCPP_ERROR(rclcpp::get_logger(kUirobotHardware), "⚠️⚠️⚠️⚠️⚠️⚠️⚠️ Unknown joint mode '%s' for joint '%s'⚠️⚠️⚠️⚠️⚠️⚠️⚠️", joint_modes_[i].c_str(), info_.joints[i].name.c_str());
      return return_type::ERROR;
    }

  } 

  // If the commanded values have been updated or not yet reached, enter this block
  if (target_changed || target_not_reached) {
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

//Code for mode switching (retaining both PTP and JOG)
CallbackReturn UirobotHardware::set_joint_positions(const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < joints_.size(); i++) {
    if (joint_modes_[i] == "JOG"){
      // RCLCPP_INFO(this->get_logger(), "---------------------------------------------------------------------");
      double target_pos = joints_[i].command.position; 
      double target_vel = joints_[i].command.velocity; 
      double current_pos = joints_[i].state.position;//JOG

      if (std::isnan(target_pos)) continue;

      if (std::isnan(target_vel)) {
        target_vel = 0.0;
      }

      // There was an issue where it would move unexpectedly upon startup. To address this, I added logic to check if the current position matches the target; if so, the process simply continues without taking any action.
      if (target_pos == joints_[i].prev_command.position) {
        // Since the command remains unchanged—indicating a standby state—skip sending unnecessary 'stop' or 'set_vel' commands.
        continue;
      }

      // Calculate the deviation between the target position and the current position.
      double position_error = target_pos - current_pos;
      double abs_error = std::abs(position_error);

      // RCLCPP_INFO(this->get_logger(), "current_pos: %f target_pos: %f.", current_pos, target_pos);
      // RCLCPP_INFO(this->get_logger(), "abs_error: %f < stop_threshold: %f.", abs_error, joints_[i].stop_threshold);

      // Stop when the absolute error falls below the threshold (stop_threshold).
      if (abs_error < joints_[i].stop_threshold) {
        RCLCPP_INFO(
          this->get_logger(), 
          "Joint %zu reached target. Error: %f < Threshold: %f. Sending STOP.", 
          i, abs_error, joints_[i].stop_threshold);

        std::vector<uint8_t> stop_cmd = create_commands("stop", joint_ids_[i]);
        auto res = ser_->read_and_write(stop_cmd);
        if (res.empty()) {
          RCLCPP_ERROR(this->get_logger(), "Failed to send stop motion command for joint %zu", i);
          return CallbackReturn::ERROR;
        }
        
        // Since the target has been reached and movement has stopped, synchronize prev_command with the current target_pos
        // so that the operation is skipped in the next cycle via the "target_pos == prev_command.position" check above.
        joints_[i].prev_command.position = target_pos;
        joints_[i].prev_command.velocity = target_vel;
        continue; 
      }

      // If the target has not been reached, standard-speed streaming (PV) processing is performed.
      double cmd_vel = target_vel + (position_error * joints_[i].kp);
      cmd_vel = std::clamp(cmd_vel, -std::fabs(joints_[i].max_vel), std::fabs(joints_[i].max_vel));

      // Conversion from physical units to pulse rate units
      double pps = cmd_vel * joints_[i].cpr * joints_[i].gear_ratio / (2 * M_PI);
      int32_t target_pps = static_cast<int32_t>(std::round(pps));
      // RCLCPP_INFO(this->get_logger(), "pps: %f ", pps);

      // Send speed command
      std::vector<uint8_t> cmd = create_commands("set_vel", joint_ids_[i], 0, target_pps);
      auto res = ser_->read_and_write(cmd);
      if (res.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send set_vel command");
        return CallbackReturn::ERROR;
      }

      cmd = create_commands("move", joint_ids_[i]);
      res = ser_->read_and_write(cmd);
      if (res.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send move for velocity stream");
        return CallbackReturn::ERROR;
      }
    
    } else if (joint_modes_[i] == "PTP"){
      double target_pos = joints_[i].command.position;
      double target_vel = joints_[i].command.velocity;

      if (std::isnan(target_pos)) continue;

      // Safety when no speed command (command.velocity) is received from MoveIt (NaN)
      if (std::isnan(target_vel) || target_vel <= 0.0) {
        target_vel = joints_[i].max_vel; // Set the maximum velocity from parameters as default
      }

      // Clamp to hardware limits
      if (std::isfinite(joints_[i].min_pos)) {
        target_pos = std::max(target_pos, joints_[i].min_pos);
      }
      if (std::isfinite(joints_[i].max_pos)) {
        target_pos = std::min(target_pos, joints_[i].max_pos);
      }
      target_vel = std::clamp(target_vel, 0.001, std::fabs(joints_[i].max_vel));

      // Conversion from physical units to pulse rate units
      double pls = target_pos * joints_[i].cpr * joints_[i].gear_ratio / (2 * M_PI);
      double pps = target_vel * joints_[i].cpr * joints_[i].gear_ratio / (2 * M_PI);

      int32_t target_pls = static_cast<int32_t>(std::round(pls));
      int32_t target_pps = static_cast<int32_t>(std::round(std::abs(pps)));

      // RCLCPP_INFO(this->get_logger(), "pps DEBUG: %f", pps);
      // RCLCPP_INFO(this->get_logger(), "pls DEBUG: %f", pls);

      // If the speed is below the minimum, set the limit according to the controller
      if (target_pps < 1) target_pps = 1;

      // RCLCPP_INFO(this->get_logger(), "PTP Command Joint %zu: Target PLS=%d, Speed PPS=%d", i, target_pls, target_pps);

      // Step 1: Specify target speed (PPS) (set_vel: 0x9E)
      std::vector<uint8_t> cmd = create_commands("set_vel", joint_ids_[i], 0, target_pps);
      auto res = ser_->read_and_write(cmd);
      if (res.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send set_vel");
        return CallbackReturn::ERROR;
      }

      // Step 2: Specify the target position (PLS) (set_pos: 0xA0)
      cmd = create_commands("set_pos", joint_ids_[i], target_pls, 0);
      res = ser_->read_and_write(cmd);
      if (res.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send set_pos");
        return CallbackReturn::ERROR;
      }

      // Step 3: Execute (move: 0x96)
      cmd = create_commands("move", joint_ids_[i]);
      res = ser_->read_and_write(cmd);
      if (res.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send move");
        return CallbackReturn::ERROR;
      }

      // Update the previous command values to prevent continuous transmission
      joints_[i].prev_command.position = joints_[i].command.position;
      joints_[i].prev_command.velocity = joints_[i].command.velocity;
    } else {
      RCLCPP_ERROR(rclcpp::get_logger(kUirobotHardware), "⚠️⚠️⚠️⚠️⚠️⚠️⚠️ Unknown joint mode '%s' for joint '%s' ⚠️⚠️⚠️⚠️⚠️⚠️⚠️", joint_modes_[i].c_str(), info_.joints[i].name.c_str());
      return CallbackReturn::ERROR;
    }

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
  } else if (mode == "get_vel") {
    // 0x9D (JV Command): Get current speed (ACK requested)
    cmd[2] = 0x9D;
    cmd[3] = 0x00; // The transmission data length is 0.
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
  } else if (mode == "stop") {
    cmd[2] = 0x97;  
    cmd[3] = 0x00;
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
  if (mode == "get_vel" && cmd.size() < 8) { 
    return 0;
  }
  if (mode == "cpr" && cmd.size() < 9) {
    return 0;
  }

  int32_t val = 0;

  if (mode == "get_pos") {
    val =
      (int32_t)cmd[8] |
      ((int32_t)cmd[9] << 8) |
      ((int32_t)cmd[10] << 16) |
      ((int32_t)cmd[11] << 24);
  } else if (mode == "get_vel") {
    val =
      (int32_t)cmd[4] |
      ((int32_t)cmd[5] << 8) |
      ((int32_t)cmd[6] << 16) |
      ((int32_t)cmd[7] << 24);
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
