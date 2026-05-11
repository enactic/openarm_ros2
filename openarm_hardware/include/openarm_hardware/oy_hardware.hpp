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

#pragma once

#include <chrono>
#include <memory>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/oy_motor/oy_motor_constants.hpp>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "openarm_hardware/visibility_control.h"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace openarm_hardware {

/**
 * @brief OpenArm Hardware Interface for OY Motors
 *
 * This hardware interface uses the OpenArm CAN API with OY motors,
 * following the pattern from full_arm.cpp example.
 * Configurable for different arm configurations via hardware parameters.
 */
class OpenArm_oyHW : public hardware_interface::SystemInterface {
 public:
  OpenArm_oyHW();

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  // default configuration
  static constexpr size_t ARM_DOF = 7;
  static constexpr bool ENABLE_GRIPPER = true;

  // Default OY motor configuration for V10
  // Motor types: GIM8115_9p for large joints, GIM4310_40/GIM4315_8 for small joints
  const std::vector<openarm::oy_motor::MotorType> DEFAULT_MOTOR_TYPES = {
      openarm::oy_motor::MotorType::GIM8115_9p,  // Joint 1
      openarm::oy_motor::MotorType::GIM8115_9p,  // Joint 2
      openarm::oy_motor::MotorType::GIM4310_40,  // Joint 3
      openarm::oy_motor::MotorType::GIM4310_40,  // Joint 4
      openarm::oy_motor::MotorType::GIM4315_8,   // Joint 5
      openarm::oy_motor::MotorType::GIM4315_8,   // Joint 6
      openarm::oy_motor::MotorType::GIM4315_8    // Joint 7
  };

  const std::vector<uint32_t> DEFAULT_SEND_CAN_IDS = {0x101, 0x102, 0x103, 0x104,
                                                      0x105, 0x106, 0x107};
  const std::vector<uint32_t> DEFAULT_RECV_CAN_IDS = {0x01, 0x02, 0x03, 0x04,
                                                      0x05, 0x06, 0x07};

  const openarm::oy_motor::MotorType DEFAULT_GRIPPER_MOTOR_TYPE =
      openarm::oy_motor::MotorType::GIM4315_8;
  const uint32_t DEFAULT_GRIPPER_SEND_CAN_ID = 0x108;
  const uint32_t DEFAULT_GRIPPER_RECV_CAN_ID = 0x08;

  // Default gains for OY motors (may need tuning)
  // Order: Joint 1-7 + Gripper
  // 帶載機械臂優化：降低 KP 避免過衝，適當提高 KD 增加阻尼抑制震盪
  const std::vector<double> DEFAULT_KP = {40.0, 40.0, 15.0, 15.0,
                                        2.0, 2.0,  2.0,  2.0};
  const std::vector<double> DEFAULT_KD = {2.0,  2.0,  1.5,  1.2,
                                        0.2,  0.2,  0.2,  0.15};

  // Default gains
//   const std::vector<double> DEFAULT_KP = {20.0, 20.0, 20.0, 20.0,
//                                           5.0,  5.0,  5.0,  0.5};
//   const std::vector<double> DEFAULT_KD = {2.75, 2.5, 0.7, 0.4,
//                                           0.7,  0.6, 0.5, 0.1};

  const double GRIPPER_JOINT_0_POSITION = 0.044;
  const double GRIPPER_JOINT_1_POSITION = 0.0;
  const double GRIPPER_MOTOR_0_RADIANS = 0.0;
  const double GRIPPER_MOTOR_1_RADIANS = -1.0472;
  const double GRIPPER_DEFAULT_KP = 5.0;
  const double GRIPPER_DEFAULT_KD = 0.1;

  // Configuration
  std::string can_interface_;
  std::string arm_prefix_;
  bool hand_;
  bool can_fd_;

  // OpenArm instance
  std::unique_ptr<openarm::can::socket::OpenArm> openarm_;

  // Generated joint names for this arm instance
  std::vector<std::string> joint_names_;

  // ROS2 control state and command vectors
  std::vector<double> pos_commands_;
  std::vector<double> vel_commands_;
  std::vector<double> tau_commands_;
  std::vector<double> pos_states_;
  std::vector<double> vel_states_;
  std::vector<double> tau_states_;

  // Helper methods
  void return_to_zero();
  bool parse_config(const hardware_interface::HardwareInfo& info);
  void generate_joint_names();

  // Gripper mapping functions
  double joint_to_motor_radians(double joint_value);
  double motor_radians_to_joint(double motor_radians);
};

}  // namespace openarm_hardware
