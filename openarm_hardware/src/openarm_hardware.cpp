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

#include <limits>
#include <vector>

#include "openarm_hardware/openarm_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace openarm_hardware
{

static const std::string& can_device_name = "can0";

OpenArmHW::OpenArmHW(): 
  canbus_(std::make_unique<CANBus>(can_device_name)),
  motor_control_(MotorControl(*canbus_)) {
    for(size_t i = 0; i < MOTORS_TYPES.size(); ++i){
      motors_[i] = std::make_unique<Motor>(MOTORS_TYPES[i], CAN_DEVICE_IDS[i], CAN_MASTER_IDS[i]);
    }
    for(const auto& motor: motors_){
      motor_control_.addMotor(*motor);
    }
}

hardware_interface::CallbackReturn OpenArmHW::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  pos_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  pos_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  vel_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  vel_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  tau_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  tau_ff_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArmHW::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // zero position or calibrate to pose

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> OpenArmHW::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &pos_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &tau_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> OpenArmHW::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &pos_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &tau_ff_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn OpenArmHW::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for(const auto& motor: motors_){
    motor_control_.enable(*motor);
  }
  read(rclcpp::Time(0), rclcpp::Duration(0, 0));

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArmHW::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for(const auto& motor: motors_){
    motor_control_.disable(*motor);
  }

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type OpenArmHW::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for(size_t i = 0; i < motors_.size(); ++i){
    pos_states_[i] = motors_[i]->getPosition();
    vel_states_[i] = motors_[i]->getVelocity();
    tau_states_[i] = motors_[i]->getTorque();
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OpenArmHW::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for(size_t i = 0; i < motors_.size(); ++i){
    motor_control_.controlMIT(*motors_[i], DEFAULT_KP, DEFAULT_KD, pos_commands_[i], vel_commands_[i], tau_ff_commands_[i]);
  }
  return hardware_interface::return_type::OK;
}

}  // namespace openarm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  openarm_hardware::OpenArmHW, hardware_interface::SystemInterface)
