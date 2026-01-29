// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "openarm_hardware/oy_hardware.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

namespace openarm_hardware {

OpenArm_oyHW::OpenArm_oyHW() = default;

bool OpenArm_oyHW::parse_config(const hardware_interface::HardwareInfo& info) {
  // Parse CAN interface (default: can0)
  auto it = info.hardware_parameters.find("can_interface");
  can_interface_ = (it != info.hardware_parameters.end()) ? it->second : "can0";

  // Parse arm prefix (default: empty for single arm, "left_" or "right_" for
  // bimanual)
  it = info.hardware_parameters.find("arm_prefix");
  arm_prefix_ = (it != info.hardware_parameters.end()) ? it->second : "";

  // Parse gripper enable (default: true)
  it = info.hardware_parameters.find("hand");
  if (it == info.hardware_parameters.end()) {
    hand_ = true;  // Default to true
  } else {
    // Handle both "true"/"True" and "false"/"False"
    std::string value = it->second;
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    hand_ = (value == "true");
  }

  // Parse CAN-FD enable (default: true)
  it = info.hardware_parameters.find("can_fd");
  if (it == info.hardware_parameters.end()) {
    can_fd_ = true;  // Default to true
  } else {
    // Handle both "true"/"True" and "false"/"False"
    std::string value = it->second;
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    can_fd_ = (value == "true");
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_oyHW"),
              "Configuration: CAN=%s, arm_prefix=%s, arm_dof=%zu, hand=%s, can_fd=%s",
              can_interface_.c_str(), arm_prefix_.c_str(), ARM_DOF,
              hand_ ? "enabled" : "disabled", can_fd_ ? "enabled" : "disabled");
  return true;
}

void OpenArm_oyHW::generate_joint_names() {
  joint_names_.clear();
  // TODO: read from urdf properly and sort in the future.
  // Currently, the joint names are hardcoded for order consistency to align
  // with hardware. Generate arm joint names: openarm_{arm_prefix}joint{N}
  for (size_t i = 1; i <= ARM_DOF; ++i) {
    std::string joint_name =
        "openarm_" + arm_prefix_ + "joint" + std::to_string(i);
    joint_names_.push_back(joint_name);
  }

  // Generate gripper joint name if enabled
  if (hand_) {
    std::string gripper_joint_name = "openarm_" + arm_prefix_ + "finger_joint1";
    joint_names_.push_back(gripper_joint_name);
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_oyHW"), "Added gripper joint: %s",
                gripper_joint_name.c_str());
  } else {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_oyHW"),
                "Gripper joint NOT added because hand_=false");
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_oyHW"),
              "Generated %zu joint names for arm prefix '%s'",
              joint_names_.size(), arm_prefix_.c_str());
}

hardware_interface::CallbackReturn OpenArm_oyHW::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  // Parse configuration
  if (!parse_config(info)) {
    return CallbackReturn::ERROR;
  }

  // Generate joint names based on arm prefix
  generate_joint_names();

  // Validate joint count (arm joints + optional gripper)
  size_t expected_joints = ARM_DOF + (hand_ ? 1 : 0);
  if (joint_names_.size() != expected_joints) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_oyHW"),
                 "Generated %zu joint names, expected %zu", joint_names_.size(),
                 expected_joints);
    return CallbackReturn::ERROR;
  }

  // Initialize OpenArm with configurable CAN-FD setting
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_oyHW"),
              "Initializing OpenArm (OY Motor) on %s with CAN-FD %s...",
              can_interface_.c_str(), can_fd_ ? "enabled" : "disabled");
  openarm_ =
      std::make_unique<openarm::can::socket::OpenArm>(can_interface_, can_fd_);

  // Initialize arm motors with defaults (OY motors)
  // Use only the required number of motors based on ARM_DOF
  std::vector<openarm::oy_motor::MotorType> motor_types(
      DEFAULT_MOTOR_TYPES.begin(), 
      DEFAULT_MOTOR_TYPES.begin() + std::min(ARM_DOF, DEFAULT_MOTOR_TYPES.size()));
  std::vector<uint32_t> send_ids(
      DEFAULT_SEND_CAN_IDS.begin(),
      DEFAULT_SEND_CAN_IDS.begin() + std::min(ARM_DOF, DEFAULT_SEND_CAN_IDS.size()));
  std::vector<uint32_t> recv_ids(
      DEFAULT_RECV_CAN_IDS.begin(),
      DEFAULT_RECV_CAN_IDS.begin() + std::min(ARM_DOF, DEFAULT_RECV_CAN_IDS.size()));
  
  openarm_->init_arm_motors(motor_types, send_ids, recv_ids);

  // Initialize gripper if enabled
  if (hand_) {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_oyHW"), "Initializing gripper...");
    openarm_->init_gripper_motor(DEFAULT_GRIPPER_MOTOR_TYPE,
                                 DEFAULT_GRIPPER_SEND_CAN_ID,
                                 DEFAULT_GRIPPER_RECV_CAN_ID);
  }

  // Initialize state and command vectors based on generated joint count
  const size_t total_joints = joint_names_.size();
  pos_commands_.resize(total_joints, 0.0);
  vel_commands_.resize(total_joints, 0.0);
  tau_commands_.resize(total_joints, 0.0);
  pos_states_.resize(total_joints, 0.0);
  vel_states_.resize(total_joints, 0.0);
  tau_states_.resize(total_joints, 0.0);

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_oyHW"),
              "OpenArm OY Motor HW initialized successfully with %zu DOF",
              ARM_DOF + (hand_ ? 1 : 0));

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArm_oyHW::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Refresh motor states during configuration
  openarm_->refresh_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all();

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
OpenArm_oyHW::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
OpenArm_oyHW::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  // TODO: consider exposing only needed interfaces to avoid undefined behavior.
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION,
        &pos_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY,
        &vel_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn OpenArm_oyHW::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_oyHW"), "Activating OpenArm OY Motor...");
  
  // Enable all motors
  openarm_->enable_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all();

  // Return to zero position
  return_to_zero();

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_oyHW"), "OpenArm OY Motor activated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArm_oyHW::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_oyHW"),
              "Deactivating OpenArm OY Motor...");

  // Disable all motors
  openarm_->disable_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all();

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_oyHW"), "OpenArm OY Motor deactivated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type OpenArm_oyHW::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // Refresh and receive all motor states
  openarm_->refresh_all();
  openarm_->recv_all();

  // Read arm joint states
  const auto& arm_motors = openarm_->get_arm().get_motors();
  for (size_t i = 0; i < ARM_DOF && i < arm_motors.size(); ++i) {
    pos_states_[i] = arm_motors[i].get_position();
    vel_states_[i] = arm_motors[i].get_velocity();
    tau_states_[i] = arm_motors[i].get_torque();
  }

  // Read gripper state if enabled
  if (hand_ && joint_names_.size() > ARM_DOF) {
    const auto& gripper_motors = openarm_->get_gripper().get_motors();
    if (!gripper_motors.empty()) {
      // TODO the mappings are approximates
      // Convert motor position (radians) to joint value (0-0.044m)
      double motor_pos = gripper_motors[0].get_position();
      pos_states_[ARM_DOF] = motor_radians_to_joint(motor_pos);

      //Velocity and torque mapping
      vel_states_[ARM_DOF] = gripper_motors[0].get_velocity();
      tau_states_[ARM_DOF] = gripper_motors[0].get_torque();
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OpenArm_oyHW::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // Control arm motors with MIT control
  // OY motor MITParam order: {q, dq, kp, kd, tau}
  std::vector<openarm::oy_motor::MITParam> arm_params;
  for (size_t i = 0; i < ARM_DOF; ++i) {
    double kp = (i < DEFAULT_KP.size()) ? DEFAULT_KP[i] : DEFAULT_KP.back();
    double kd = (i < DEFAULT_KD.size()) ? DEFAULT_KD[i] : DEFAULT_KD.back();
    arm_params.push_back({pos_commands_[i], vel_commands_[i], 
                          kp, kd, tau_commands_[i]});
  }
  openarm_->get_arm().oy_mit_control_all(arm_params);

  // Control gripper if enabled
  if (hand_ && joint_names_.size() > ARM_DOF) {
    // TODO the true mappings are unimplemented.
    double motor_command = joint_to_motor_radians(pos_commands_[ARM_DOF]);
    // OY motor MITParam order: {q, dq, kp, kd, tau}
    openarm_->get_gripper().oy_mit_control_all(
        {{motor_command, 0.0, GRIPPER_DEFAULT_KP, GRIPPER_DEFAULT_KD, 0.0}});
  }
  openarm_->recv_all(1000);
  return hardware_interface::return_type::OK;
}

void OpenArm_oyHW::return_to_zero() {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_oyHW"),
              "Returning to zero position...");

  // Return arm to zero with MIT control
  // OY motor MITParam order: {q, dq, kp, kd, tau}
  std::vector<openarm::oy_motor::MITParam> arm_params;
  for (size_t i = 0; i < ARM_DOF; ++i) {
    double kp = (i < DEFAULT_KP.size()) ? DEFAULT_KP[i] : DEFAULT_KP.back();
    double kd = (i < DEFAULT_KD.size()) ? DEFAULT_KD[i] : DEFAULT_KD.back();
    arm_params.push_back({0.0, 0.0, kp, kd, 0.0});
  }
  openarm_->get_arm().oy_mit_control_all(arm_params);

  // Return gripper to zero if enabled
  if (hand_) {
    // OY motor MITParam order: {q, dq, kp, kd, tau}
    openarm_->get_gripper().oy_mit_control_all(
        {{GRIPPER_JOINT_0_POSITION, 0.0, GRIPPER_DEFAULT_KP, GRIPPER_DEFAULT_KD, 0.0}});
  }
  std::this_thread::sleep_for(std::chrono::microseconds(1000));
  openarm_->recv_all();
}

// Gripper mapping helper functions
double OpenArm_oyHW::joint_to_motor_radians(double joint_value) {
  // Joint 0=closed -> motor 0 rad, Joint 0.044=open -> motor -1.0472 rad
  return (joint_value / GRIPPER_JOINT_0_POSITION) *
         GRIPPER_MOTOR_1_RADIANS;  // Scale from 0-0.044 to 0 to -1.0472
}

double OpenArm_oyHW::motor_radians_to_joint(double motor_radians) {
  // Motor 0 rad=closed -> joint 0, Motor -1.0472 rad=open -> joint 0.044
  return GRIPPER_JOINT_0_POSITION *
         (motor_radians /
          GRIPPER_MOTOR_1_RADIANS);  // Scale from 0 to -1.0472 to 0-0.044
}

}  // namespace openarm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(openarm_hardware::OpenArm_oyHW,
                       hardware_interface::SystemInterface)
