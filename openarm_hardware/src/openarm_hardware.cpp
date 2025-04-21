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
  it = info.hardware_parameters.find("disable_torque");
  if (it == info.hardware_parameters.end()){
    disable_torque_ = false;
  }
  else{
    disable_torque_ = it->second == "true";
  }


  canbus_ = std::make_unique<CANBus>(info.hardware_parameters.at("can_device"));
  motor_control_ = std::make_unique<MotorControl>(*canbus_);

  if(USING_GRIPPER){
    motor_types.emplace_back(DM_Motor_Type::DM4310);
    can_device_ids.emplace_back(0x08);
    can_master_ids.emplace_back(0x18);
    ++curr_dof;
  }

  motors_.resize(curr_dof);
  for(size_t i = 0; i < curr_dof; ++i){
    motors_[i] = std::make_unique<Motor>(motor_types[i], can_device_ids[i], can_master_ids[i]);
    motor_control_->addMotor(*motors_[i]);
  }

  pos_states_.resize(curr_dof, 0.0);
  pos_commands_.resize(curr_dof, 0.0);
  vel_states_.resize(curr_dof, 0.0);
  vel_commands_.resize(curr_dof, 0.0);
  tau_states_.resize(curr_dof, 0.0);
  tau_ff_commands_.resize(curr_dof, 0.0);
  refresh_motors();
  read(rclcpp::Time(0), rclcpp::Duration(0, 0));

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArmHW::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{

  read(rclcpp::Time(0), rclcpp::Duration(0, 0));
  // zero position or calibrate to pose
  // for (std::size_t i = 0; i < curr_dof; ++i)
  // {
  //   motor_control_->set_zero_position(*motors_[i]);
  // }


  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> OpenArmHW::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < curr_dof; ++i)
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
  for (size_t i = 0; i < curr_dof; ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &pos_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &tau_ff_commands_[i]));
  }

  return command_interfaces;
}

void OpenArmHW::refresh_motors()
{
  for(const auto& motor: motors_){
    motor_control_->controlMIT(*motor, 0.0, 0.0, 0.0, 0.0, 0.0);
  }
}

hardware_interface::CallbackReturn OpenArmHW::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  read(rclcpp::Time(0), rclcpp::Duration(0, 0));
  bool zeroed = false;
  for(const auto& motor: motors_){
    motor_control_->enable(*motor);
  }

  while(!zeroed){
    bool all_zero = true;
    for (std::size_t m = 0; m < curr_dof; ++m){
      const double diff = pos_commands_[m] - pos_states_[m];
      if (std::abs(diff) > START_POS_TOLERANCE_RAD){
        all_zero = false;
      }
  
      const double max_step = std::min(POS_JUMP_TOLERANCE_RAD, std::abs(diff));
      double command = pos_states_[m];
      if (pos_states_[m] < pos_commands_[m]){
        command += max_step;
      }
      else{
        command -= max_step;
      }
      motor_control_->controlMIT(*motors_[m], KP[m], KD[m], command, 0.0, 0.0);
    
    }
    if (all_zero){
      zeroed = true;
    }
    else{
      sleep(0.01);
      read(rclcpp::Time(0), rclcpp::Duration(0, 0));
    }
  }

  read(rclcpp::Time(0), rclcpp::Duration(0, 0));

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArmHW::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  refresh_motors();
  for(const auto& motor: motors_){
    motor_control_->disable(*motor);
  }

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type OpenArmHW::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for(size_t i = 0; i < ARM_DOF; ++i){
    pos_states_[i] = motors_[i]->getPosition();
    vel_states_[i] = motors_[i]->getVelocity();
    tau_states_[i] = motors_[i]->getTorque();
  }
  if(USING_GRIPPER){
    pos_states_[GRIPPER_INDEX] = -motors_[GRIPPER_INDEX]->getPosition() * GRIPPER_REFERENCE_GEAR_RADIUS_M * GRIPPER_GEAR_DIRECTION_MULTIPLIER;
    vel_states_[GRIPPER_INDEX] = motors_[GRIPPER_INDEX]->getVelocity() * GRIPPER_REFERENCE_GEAR_RADIUS_M * GRIPPER_GEAR_DIRECTION_MULTIPLIER;
    tau_states_[GRIPPER_INDEX] = motors_[GRIPPER_INDEX]->getTorque() * GRIPPER_REFERENCE_GEAR_RADIUS_M * GRIPPER_GEAR_DIRECTION_MULTIPLIER;
  }


  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OpenArmHW::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (disable_torque_){
    // refresh motor state on write
    for(size_t i = 0; i < curr_dof; ++i){
      motor_control_->controlMIT(*motors_[i], 0.0, 0.0, 0.0, 0.0, 0.0);
      return hardware_interface::return_type::OK;
    }
  }


  for(size_t i = 0; i < ARM_DOF; ++i){
    if (std::abs(pos_commands_[i] - pos_states_[i]) > POS_JUMP_TOLERANCE_RAD)
    {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArmHW"), "Position jump detected for joint %s: %f -> %f", info_.joints[i].name.c_str(), pos_states_[i], pos_commands_[i]);
      return hardware_interface::return_type::ERROR;
    }
    motor_control_->controlMIT(*motors_[i], KP.at(i), KD.at(i), pos_commands_[i], vel_commands_[i], tau_ff_commands_[i]);
  }
  if(USING_GRIPPER){
    motor_control_->controlMIT(*motors_[GRIPPER_INDEX], KP.at(GRIPPER_INDEX), KD.at(GRIPPER_INDEX), 
    -pos_commands_[GRIPPER_INDEX] / GRIPPER_REFERENCE_GEAR_RADIUS_M * GRIPPER_GEAR_DIRECTION_MULTIPLIER, 
    vel_commands_[GRIPPER_INDEX] / GRIPPER_REFERENCE_GEAR_RADIUS_M * GRIPPER_GEAR_DIRECTION_MULTIPLIER, 
    tau_ff_commands_[GRIPPER_INDEX] / GRIPPER_REFERENCE_GEAR_RADIUS_M * GRIPPER_GEAR_DIRECTION_MULTIPLIER);
  }
  return hardware_interface::return_type::OK;
}

}  // namespace openarm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  openarm_hardware::OpenArmHW, hardware_interface::SystemInterface)
