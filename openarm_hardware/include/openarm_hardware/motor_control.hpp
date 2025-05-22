// Copyright 2025 Reazon Holdings, Inc.
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

#include <unistd.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
#include <functional>
#include <iostream>
#include <map>
#include <thread>
#include <vector>

#include "canbus.hpp"
#include "motor.hpp"

class MotorControl {
 public:
  explicit MotorControl(CANBus& canbus);
  bool addMotor(Motor& motor);
  void enable(Motor& motor);
  void disable(Motor& motor);
  void set_zero_position(Motor& motor);
  void controlMIT(Motor& motor, double kp, double kd, double q, double dq,
                  double tau);
  void controlMIT2(Motor& motor, double kp, double kd, double q, double dq,
                   double tau);
  void sendData(uint16_t motor_id, const std::array<uint8_t, 8>& data);
  void recv();

  void control_delay(Motor& motor, double kp, double kd, double q, double dq,
                     double tau, double delay);
  void controlPosVel(Motor& motor, double q, double dq);
  void controlPosVel2(Motor& motor, double q, double dq);
  void controlVel(Motor& motor, double dq);
  void controlVel2(Motor& motor, double dq);
  void controlPosForce(Motor& motor, double q, double vel, double tau);
  void controlPosForce2(Motor& motor, double q, double vel, double tau);

  bool switchControlMode(Motor& motor, Control_Type controlMode);
  void save_motor_param(Motor& motor);
  void change_limit_param();
  // void change_limit_param(DM_Motor_Type motor_type, double PMAX, double VMAX,
  // double TMAX);

  void recv_set_param_data();

 private:
  CANBus& canbus_;

  std::map<uint16_t, Motor*> motors_map_;
  static constexpr double Limit_Param[12][3] = {
      {12.5, 30, 10},   // DM4310
      {12.5, 50, 10},   // DM4310_48V
      {12.5, 8, 28},    // DM4340
      {12.5, 10, 28},   // DM4340_48V
      {12.5, 45, 20},   // DM6006
      {12.5, 45, 40},   // DM8006
      {12.5, 45, 54},   // DM8009
      {12.5, 25, 200},  // DM10010L
      {12.5, 20, 200},  // DM10010
      {12.5, 280, 1},   // DMH3510
      {12.5, 45, 10},   // DMH6215
      {12.5, 45, 10},   // DMG6220
  };

  void processPacket(const can_frame& frame);
  void processPacketFD(const canfd_frame& frame);
  void controlCmd(Motor& motor, uint8_t cmd);
  void readRIDParam(Motor& motor, DM_variable RID);
  void writeMotorParam(Motor& motor, DM_variable RID, double value);
};
