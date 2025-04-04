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
#include "rclcpp/logging.hpp"

namespace openarm_hardware
{

static const std::string& can_device_name = "can0";

OpenArmHW::OpenArmHW() = default;

hardware_interface::CallbackReturn OpenArmHW::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }
  
  //read hardware parameters
  if (info.hardware_parameters.find("can_device") == info.hardware_parameters.end()){
    RCLCPP_ERROR(rclcpp::get_logger("OpenArmHW"), "No can_device parameter found");
    return CallbackReturn::ERROR;
  }
  
  auto it = info.hardware_parameters.find("prefix");
  if (it == info.hardware_parameters.end()){
    prefix_ = "";
  }
  else{
    prefix_ = it->second;
  }
  canbus_ = std::make_unique<CANBus>(info.hardware_parameters.at("can_device"));
  motor_control_ = std::make_unique<MotorControl>(*canbus_);

  const bool using_gripper = true;
  if(using_gripper){
    motor_types.emplace_back(DM_Motor_Type::DM4310);
    can_device_ids.emplace_back(0x08);
    can_master_ids.emplace_back(0x18);
    ++arm_dof;
  }

  motors_.resize(arm_dof);
  for(size_t i = 0; i < arm_dof; ++i){
    motors_[i] = std::make_unique<Motor>(motor_types[i], can_device_ids[i], can_master_ids[i]);
  }
  for(const auto& motor: motors_){
    motor_control_->addMotor(*motor);
  }

  pos_states_.resize(arm_dof, 0.0);
  pos_commands_.resize(arm_dof, 0.0);
  vel_states_.resize(arm_dof, 0.0);
  vel_commands_.resize(arm_dof, 0.0);
  tau_states_.resize(arm_dof, 0.0);
  tau_ff_commands_.resize(arm_dof, 0.0);

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArmHW::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{

  read(rclcpp::Time(0), rclcpp::Duration(0, 0));
  // zero position or calibrate to pose
  // for (std::size_t i = 0; i < arm_dof; ++i)
  // {
  //   motor_control_->set_zero_position(*motors_[i]);
  // }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> OpenArmHW::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < arm_dof; ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &pos_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &tau_states_[i]));
    RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"), "Exporting state interface for joint %s", info_.joints[i].name.c_str());
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> OpenArmHW::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < arm_dof; ++i)
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
  // read(rclcpp::Time(0), rclcpp::Duration(0, 0));
  // for (std::size_t m = 0; m < arm_dof; ++m){
  //   if (std::abs(pos_states_[0] - pos_commands_[0]) > START_POS_TOLERANCE_RAD)
  //   {
  //     RCLCPP_ERROR(rclcpp::get_logger("OpenArmHW"), "Motor %zu not in start position", m);
  //     return CallbackReturn::ERROR;
  //   }
  // }
  for(const auto& motor: motors_){
    motor_control_->controlMIT(*motor, 0.0, 0.0, 0.0, 0.0, 0.0);
    motor_control_->enable(*motor);
  }
  read(rclcpp::Time(0), rclcpp::Duration(0, 0));

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArmHW::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for(const auto& motor: motors_){
    motor_control_->controlMIT(*motor, 0.0, 0.0, 0.0, 0.0, 0.0);
    motor_control_->disable(*motor);
  }

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type OpenArmHW::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for(size_t i = 0; i < arm_dof; ++i){
    pos_states_[i] = motors_[i]->getPosition();
    vel_states_[i] = motors_[i]->getVelocity();
    tau_states_[i] = motors_[i]->getTorque();
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OpenArmHW::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for(size_t i = 0; i < arm_dof; ++i){
    motor_control_->controlMIT(*motors_[i], DEFAULT_KP, DEFAULT_KD, pos_commands_[i], vel_commands_[i], tau_ff_commands_[i]);
  }
  return hardware_interface::return_type::OK;
}

}  // namespace openarm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  openarm_hardware::OpenArmHW, hardware_interface::SystemInterface)
