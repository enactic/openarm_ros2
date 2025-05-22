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

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <array>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>

enum CANMode { CAN_MODE_CLASSIC = 0, CAN_MODE_FD = 1 };
class CANBus {
 public:
  explicit CANBus(const std::string& interface, int mode);
  ~CANBus();
  int whichCAN();
  bool send(uint16_t motor_id, const std::array<uint8_t, 8>& data);
  std::array<uint8_t, 64> recv(uint16_t& out_id, uint8_t& out_len);

 private:
  bool sendClassic(uint16_t motor_id, const std::array<uint8_t, 8>& data);
  bool sendFD(uint16_t motor_id, const std::array<uint8_t, 8>& data);

  struct can_frame recvClassic();
  struct canfd_frame recvFD();

  int sock_;
  int mode_;
};
