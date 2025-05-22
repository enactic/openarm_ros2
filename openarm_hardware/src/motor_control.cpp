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

#include "openarm_hardware/motor_control.hpp"

#include "openarm_hardware/motor.hpp"

MotorControl::MotorControl(CANBus& canbus) : canbus_(canbus) {}

bool MotorControl::addMotor(Motor& motor) {
  motors_map_[motor.SlaveID] = &motor;
  if (motor.MasterID != 0) {
    motors_map_[motor.MasterID] = &motor;
  }
  return true;
}

void MotorControl::enable(Motor& motor) {
  controlCmd(motor, 0xFC);
  sleep(0.3);
}

void MotorControl::disable(Motor& motor) {
  controlCmd(motor, 0xFD);
  sleep(0.3);
}

void MotorControl::set_zero_position(Motor& motor) {
  controlCmd(motor, 0xFE);
  sleep(0.3);
  recv();
}

void MotorControl::controlMIT(Motor& motor, double kp, double kd, double q,
                              double dq, double tau) {
  if (motors_map_.find(motor.SlaveID) == motors_map_.end()) {
    std::cerr << "controlMIT ERROR: Motor ID not found" << std::endl;
    return;
  }

  uint16_t kp_uint = double_to_uint(kp, 0, 500, 12);
  uint16_t kd_uint = double_to_uint(kd, 0, 5, 12);

  int motor_index = static_cast<int>(motor.MotorType);
  double Q_MAX = Limit_Param[motor_index][0];
  double DQ_MAX = Limit_Param[motor_index][1];
  double TAU_MAX = Limit_Param[motor_index][2];

  uint16_t q_uint = double_to_uint(q, -Q_MAX, Q_MAX, 16);
  uint16_t dq_uint = double_to_uint(dq, -DQ_MAX, DQ_MAX, 12);
  uint16_t tau_uint = double_to_uint(tau, -TAU_MAX, TAU_MAX, 12);

  std::array<uint8_t, 8> data = {
      static_cast<uint8_t>((q_uint >> 8) & 0xFF),
      static_cast<uint8_t>(q_uint & 0xFF),
      static_cast<uint8_t>(dq_uint >> 4),
      static_cast<uint8_t>(((dq_uint & 0xF) << 4) | ((kp_uint >> 8) & 0xF)),
      static_cast<uint8_t>(kp_uint & 0xFF),
      static_cast<uint8_t>(kd_uint >> 4),
      static_cast<uint8_t>(((kd_uint & 0xF) << 4) | ((tau_uint >> 8) & 0xF)),
      static_cast<uint8_t>(tau_uint & 0xFF)};

  sendData(motor.SlaveID, data);
  recv();
}

void MotorControl::controlMIT2(Motor& motor, double kp, double kd, double q,
                               double dq, double tau) {
  if (motors_map_.find(motor.SlaveID) == motors_map_.end()) {
    std::cerr << "controlMIT ERROR: Motor ID not found" << std::endl;
    return;
  }

  uint16_t kp_uint = double_to_uint(kp, 0, 500, 12);
  uint16_t kd_uint = double_to_uint(kd, 0, 5, 12);

  int motor_index = static_cast<int>(motor.MotorType);
  double Q_MAX = Limit_Param[motor_index][0];
  double DQ_MAX = Limit_Param[motor_index][1];
  double TAU_MAX = Limit_Param[motor_index][2];

  uint16_t q_uint = double_to_uint(q, -Q_MAX, Q_MAX, 16);
  uint16_t dq_uint = double_to_uint(dq, -DQ_MAX, DQ_MAX, 12);
  uint16_t tau_uint = double_to_uint(tau, -TAU_MAX, TAU_MAX, 12);

  std::array<uint8_t, 8> data = {
      static_cast<uint8_t>((q_uint >> 8) & 0xFF),
      static_cast<uint8_t>(q_uint & 0xFF),
      static_cast<uint8_t>(dq_uint >> 4),
      static_cast<uint8_t>(((dq_uint & 0xF) << 4) | ((kp_uint >> 8) & 0xF)),
      static_cast<uint8_t>(kp_uint & 0xFF),
      static_cast<uint8_t>(kd_uint >> 4),
      static_cast<uint8_t>(((kd_uint & 0xF) << 4) | ((tau_uint >> 8) & 0xF)),
      static_cast<uint8_t>(tau_uint & 0xFF)};

  sendData(motor.SlaveID, data);
}

void MotorControl::sendData(uint16_t motor_id,
                            const std::array<uint8_t, 8>& data) {
  canbus_.send(motor_id, data);
}

void MotorControl::recv() {
  uint16_t id;
  uint8_t len;
  std::array<uint8_t, 64> data = canbus_.recv(id, len);

  if (canbus_.whichCAN() == CAN_MODE_CLASSIC) {
    can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = id;
    frame.can_dlc = len;
    std::memcpy(frame.data, data.data(), len);

    processPacket(frame);
  } else if (canbus_.whichCAN() == CAN_MODE_FD) {
    canfd_frame fd_frame;
    std::memset(&fd_frame, 0, sizeof(fd_frame));
    fd_frame.can_id = id;
    fd_frame.len = len;
    std::memcpy(fd_frame.data, data.data(), len);

    processPacketFD(fd_frame);
  }
}

void MotorControl::control_delay(Motor& motor, double kp, double kd, double q,
                                 double dq, double tau, double delay) {
  controlMIT(motor, kp, kd, q, dq, tau);
  std::this_thread::sleep_for(
      std::chrono::milliseconds(static_cast<int>(delay)));
}

void MotorControl::controlPosVel(Motor& motor, double pos, double vel) {
  if (motors_map_.find(motor.SlaveID) == motors_map_.end()) {
    std::cerr << "controlPosVel ERROR: Motor ID not found" << std::endl;
    return;
  }

  uint16_t motor_id = 0x100 + motor.SlaveID;
  std::array<uint8_t, 8> data_buf = {0};

  auto vel_buf = float_to_uint8s(static_cast<float>(vel));
  auto pos_buf = float_to_uint8s(static_cast<float>(pos));

  for (int i = 0; i < 4; ++i) {
    data_buf[i] = pos_buf[i];
    data_buf[i + 4] = vel_buf[i];
  }

  sendData(motor_id, data_buf);

  recv();
}

void MotorControl::controlPosVel2(Motor& motor, double pos, double vel) {
  if (motors_map_.find(motor.SlaveID) == motors_map_.end()) {
    std::cerr << "controlPosVel2 ERROR: Motor ID not found" << std::endl;
    return;
  }

  uint16_t motor_id = 0x100 + motor.SlaveID;
  std::array<uint8_t, 8> data_buf = {0};

  auto pos_buf = float_to_uint8s(static_cast<float>(pos));
  auto vel_buf = float_to_uint8s(static_cast<float>(vel));

  for (int i = 0; i < 4; ++i) {
    data_buf[i] = pos_buf[i];
    data_buf[i + 4] = vel_buf[i];
  }

  sendData(motor_id, data_buf);
}

void MotorControl::controlVel(Motor& motor, double vel) {
  if (motors_map_.find(motor.SlaveID) == motors_map_.end()) {
    std::cerr << "controlVel ERROR: Motor ID not found" << std::endl;
    return;
  }

  uint16_t motor_id = 0x200 + motor.SlaveID;
  std::array<uint8_t, 8> data_buf = {0};

  auto vel_buf = float_to_uint8s(static_cast<float>(vel));
  for (int i = 0; i < 4; ++i) {
    data_buf[i] = vel_buf[i];
  }

  sendData(motor_id, data_buf);
  recv();
}

void MotorControl::controlVel2(Motor& motor, double vel) {
  if (motors_map_.find(motor.SlaveID) == motors_map_.end()) {
    std::cerr << "controlVel2 ERROR: Motor ID not found" << std::endl;
    return;
  }

  uint16_t motor_id = 0x200 + motor.SlaveID;
  std::array<uint8_t, 8> data_buf = {0};

  auto vel_buf = float_to_uint8s(static_cast<float>(vel));
  for (int i = 0; i < 4; ++i) {
    data_buf[i] = vel_buf[i];
  }

  sendData(motor_id, data_buf);
}

void MotorControl::controlPosForce(Motor& motor, double pos, double vel,
                                   double tau) {
  if (motors_map_.find(motor.SlaveID) == motors_map_.end()) {
    std::cerr << "controlPosForce ERROR: Motor ID not found" << std::endl;
    return;
  }

  uint16_t motor_id = 0x300 + motor.SlaveID;
  std::array<uint8_t, 8> data_buf = {0};

  auto pos_buf = float_to_uint8s(static_cast<float>(pos));
  auto vel_buf = float_to_uint8s(static_cast<float>(vel));
  auto tau_buf = float_to_uint8s(static_cast<float>(tau));

  for (int i = 0; i < 4; ++i) {
    data_buf[i] = pos_buf[i];
  }

  data_buf[4] = vel_buf[0];
  data_buf[5] = vel_buf[1];

  data_buf[6] = tau_buf[0];
  data_buf[7] = tau_buf[1];

  sendData(motor_id, data_buf);
  recv();
}

void MotorControl::controlPosForce2(Motor& motor, double pos, double vel,
                                    double tau) {
  if (motors_map_.find(motor.SlaveID) == motors_map_.end()) {
    std::cerr << "controlPosForce ERROR: Motor ID not found" << std::endl;
    return;
  }

  uint16_t motor_id = 0x300 + motor.SlaveID;
  std::array<uint8_t, 8> data_buf = {0};

  auto pos_buf = float_to_uint8s(static_cast<float>(pos));
  auto vel_buf = float_to_uint8s(static_cast<float>(vel));
  auto tau_buf = float_to_uint8s(static_cast<float>(tau));

  for (int i = 0; i < 4; ++i) {
    data_buf[i] = pos_buf[i];
  }

  data_buf[4] = vel_buf[0];
  data_buf[5] = vel_buf[1];

  data_buf[6] = tau_buf[0];
  data_buf[7] = tau_buf[1];

  sendData(motor_id, data_buf);
}

bool MotorControl::switchControlMode(Motor& motor, Control_Type control_mode) {
  const int max_retries = 20;
  const double retry_interval = 0.1;
  DM_variable RID = DM_variable::CTRL_MODE;

  writeMotorParam(motor, RID, static_cast<int>(control_mode));

  for (int i = 0; i < max_retries; ++i) {
    usleep(static_cast<useconds_t>(retry_interval * 1e6));
    recv_set_param_data();
    if (motor.getParam(static_cast<int>(RID)) ==
        static_cast<int>(control_mode)) {
      return true;
    }
  }
  return false;
}
void MotorControl::save_motor_param(Motor& motor) {
  std::array<uint8_t, 8> data = {
      static_cast<uint8_t>(motor.SlaveID & 0xFF),
      static_cast<uint8_t>((motor.SlaveID >> 8) & 0xFF),
      0xAA,
      0x00,
      0x00,
      0x00,
      0x00,
      0x00};
  disable(motor);
  canbus_.send(0x7FF, data);
  usleep(1000);
}
// void MotorControl::change_limit_param(DM_Motor_Type motor_type, double PMAX,
// double VMAX, double TMAX) { int index = static_cast<int>(motor_type);
// Limit_Param[index][0] = PMAX;
// Limit_Param[index][1] = VMAX;
// Limit_Param[index][2] = TMAX;
// }

void MotorControl::recv_set_param_data() {
  uint16_t id;
  uint8_t len;
  std::array<uint8_t, 64> data = canbus_.recv(id, len);

  uint8_t cmd = 0x11;

  if (len >= 8) {
    std::cout << "CANID: 0x" << std::hex << id << ", CMD: 0x"
              << static_cast<int>(cmd) << std::dec << std::endl;
    for (int i = 0; i < 8; ++i) {
      std::cout << "0x" << std::hex << static_cast<int>(data[i]) << " ";
    }
    std::cout << std::dec << std::endl;
  }
}

void MotorControl::processPacket(const can_frame& frame) {
  uint16_t motorID = frame.data[0];
  uint8_t cmd = 0x11;  // someday fix

  if (cmd == 0x11) {
    if (motorID != 0x00) {
      auto it = motors_map_.find(motorID);
      if (it != motors_map_.end() && it->second) {
        Motor* motor = it->second;

        uint16_t q_uint = (frame.data[1] << 8) | frame.data[2];
        uint16_t dq_uint = (frame.data[3] << 4) | (frame.data[4] >> 4);
        uint16_t tau_uint = ((frame.data[4] & 0xf) << 8) | frame.data[5];
        int t_mos = frame.data[6];
        int t_rotor = frame.data[7];

        double Q_MAX = Limit_Param[static_cast<int>(motor->MotorType)][0];
        double DQ_MAX = Limit_Param[static_cast<int>(motor->MotorType)][1];
        double TAU_MAX = Limit_Param[static_cast<int>(motor->MotorType)][2];

        double recv_q = uint_to_double(q_uint, -Q_MAX, Q_MAX, 16);
        double recv_dq = uint_to_double(dq_uint, -DQ_MAX, DQ_MAX, 12);
        double recv_tau = uint_to_double(tau_uint, -TAU_MAX, TAU_MAX, 12);

        motor->recv_data(recv_q, recv_dq, recv_tau, t_mos, t_rotor);
      }
    } else {
      uint16_t MasterID = frame.data[0] & 0x0F;
      auto it = motors_map_.find(MasterID);
      if (it != motors_map_.end() && it->second) {
        Motor* motor = it->second;

        uint16_t q_uint = (frame.data[1] << 8) | frame.data[2];
        uint16_t dq_uint = (frame.data[3] << 4) | (frame.data[4] >> 4);
        uint16_t tau_uint = ((frame.data[4] & 0xf) << 8) | frame.data[5];
        int t_mos = frame.data[6];
        int t_rotor = frame.data[7];

        double Q_MAX = Limit_Param[static_cast<int>(motor->MotorType)][0];
        double DQ_MAX = Limit_Param[static_cast<int>(motor->MotorType)][1];
        double TAU_MAX = Limit_Param[static_cast<int>(motor->MotorType)][2];

        double recv_q = uint_to_double(q_uint, -Q_MAX, Q_MAX, 16);
        double recv_dq = uint_to_double(dq_uint, -DQ_MAX, DQ_MAX, 12);
        double recv_tau = uint_to_double(tau_uint, -TAU_MAX, TAU_MAX, 12);

        motor->recv_data(recv_q, recv_dq, recv_tau, t_mos, t_rotor);
      }
    }
  }
}

void MotorControl::processPacketFD(const canfd_frame& frame) {
  uint16_t motorID = frame.data[0];
  uint8_t cmd = 0x11;  // someday fix

  if (cmd == 0x11) {
    if (motorID != 0x00) {
      auto it = motors_map_.find(motorID);
      if (it != motors_map_.end() && it->second) {
        Motor* motor = it->second;

        uint16_t q_uint = (frame.data[1] << 8) | frame.data[2];
        uint16_t dq_uint = (frame.data[3] << 4) | (frame.data[4] >> 4);
        uint16_t tau_uint = ((frame.data[4] & 0xf) << 8) | frame.data[5];
        int t_mos = frame.data[6];
        int t_rotor = frame.data[7];

        double Q_MAX = Limit_Param[static_cast<int>(motor->MotorType)][0];
        double DQ_MAX = Limit_Param[static_cast<int>(motor->MotorType)][1];
        double TAU_MAX = Limit_Param[static_cast<int>(motor->MotorType)][2];

        double recv_q = uint_to_double(q_uint, -Q_MAX, Q_MAX, 16);
        double recv_dq = uint_to_double(dq_uint, -DQ_MAX, DQ_MAX, 12);
        double recv_tau = uint_to_double(tau_uint, -TAU_MAX, TAU_MAX, 12);

        motor->recv_data(recv_q, recv_dq, recv_tau, t_mos, t_rotor);
      }
    } else {
      uint16_t MasterID = frame.data[0] & 0x0F;
      auto it = motors_map_.find(MasterID);
      if (it != motors_map_.end() && it->second) {
        Motor* motor = it->second;

        uint16_t q_uint = (frame.data[1] << 8) | frame.data[2];
        uint16_t dq_uint = (frame.data[3] << 4) | (frame.data[4] >> 4);
        uint16_t tau_uint = ((frame.data[4] & 0xf) << 8) | frame.data[5];
        int t_mos = frame.data[6];
        int t_rotor = frame.data[7];

        double Q_MAX = Limit_Param[static_cast<int>(motor->MotorType)][0];
        double DQ_MAX = Limit_Param[static_cast<int>(motor->MotorType)][1];
        double TAU_MAX = Limit_Param[static_cast<int>(motor->MotorType)][2];

        double recv_q = uint_to_double(q_uint, -Q_MAX, Q_MAX, 16);
        double recv_dq = uint_to_double(dq_uint, -DQ_MAX, DQ_MAX, 12);
        double recv_tau = uint_to_double(tau_uint, -TAU_MAX, TAU_MAX, 12);

        motor->recv_data(recv_q, recv_dq, recv_tau, t_mos, t_rotor);
      }
    }
  }
}

void MotorControl::controlCmd(Motor& motor, uint8_t cmd) {
  std::array<uint8_t, 8> data_buf = {0xFF, 0xFF, 0xFF, 0xFF,
                                     0xFF, 0xFF, 0xFF, cmd};
  sendData(motor.SlaveID, data_buf);
}

void MotorControl::readRIDParam(Motor& motor, DM_variable RID) {
  std::array<uint8_t, 8> data = {
      static_cast<uint8_t>(motor.SlaveID & 0xFF),
      static_cast<uint8_t>((motor.SlaveID >> 8) & 0xFF),
      0x33,
      static_cast<uint8_t>(RID),
      0x00,
      0x00,
      0x00,
      0x00};
  canbus_.send(0x7FF, data);
}

void MotorControl::writeMotorParam(Motor& motor, DM_variable RID,
                                   double value) {
  std::array<uint8_t, 8> data = {
      static_cast<uint8_t>(motor.SlaveID & 0xFF),
      static_cast<uint8_t>((motor.SlaveID >> 8) & 0xFF), 0x55,
      static_cast<uint8_t>(RID)};

  if (is_in_ranges(static_cast<int>(RID))) {
    auto intData = data_to_uint8s(static_cast<uint32_t>(value));
    std::copy(intData.begin(), intData.end(), data.begin() + 4);
  } else {
    auto doubleData = double_to_uint8s(value);
    std::copy(doubleData.begin(), doubleData.end(), data.begin() + 4);
  }

  canbus_.send(0x7FF, data);
}
