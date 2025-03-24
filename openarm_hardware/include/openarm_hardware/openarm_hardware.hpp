// Copyright (c) 2025, Reazon Holdings, Inc.
// Copyright (c) 2025, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

#ifndef OPENARM_HARDWARE__OPENARM_HARDWARE_HPP_
#define OPENARM_HARDWARE__OPENARM_HARDWARE_HPP_

#include <string>
#include <vector>
#include <memory>

#include "openarm_hardware/visibility_control.h"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "canbus.hpp"
#include "motor.hpp"
#include "motor_control.hpp"

namespace openarm_hardware
{

const std::vector<DM_Motor_Type> MOTORS_TYPES = {DM_Motor_Type::DM4340, DM_Motor_Type::DM4340, DM_Motor_Type::DM4340, DM_Motor_Type::DM4340, DM_Motor_Type::DM4310, DM_Motor_Type::DM4310, DM_Motor_Type::DM4310};
const std::vector<uint16_t> CAN_DEVICE_IDS = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
const std::vector<uint16_t> CAN_MASTER_IDS = {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17};
const std::vector<bool> MOTOR_WITH_TORQUE = {true,true,true,true,true,true,true};
const Control_Type CONTROL_MODE = Control_Type::MIT;
const double DEFAULT_KP = 1.0;
const double DEFAULT_KD = 0.0;

class OpenArmHW : public hardware_interface::SystemInterface
{
public:
  OpenArmHW();
  
  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::unique_ptr<CANBus> canbus_;
  MotorControl motor_control_;
  std::vector<double> pos_commands_;
  std::vector<double> pos_states_;
  std::vector<double> vel_commands_;
  std::vector<double> vel_states_;
  std::vector<double> tau_ff_commands_;
  std::vector<double> tau_states_;
  std::vector<std::unique_ptr<Motor>> motors_;
};

}  // namespace openarm_hardware

#endif  // OPENARM_HARDWARE__OPENARM_HARDWARE_HPP_
