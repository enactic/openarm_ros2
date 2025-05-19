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
