#include "openarm_hardware/canbus.hpp"

CANBus::CANBus(const std::string& interface, int mode) : mode_(mode) {
  struct ifreq ifr;
  struct sockaddr_can addr;

  sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (sock_ < 0) {
    perror("Error while opening CAN socket");
    exit(EXIT_FAILURE);
  }

  std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ);
  if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
    perror("Error getting interface index");
    exit(EXIT_FAILURE);
  }

  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (mode_ == CAN_MODE_FD) {
    int enable_canfd = 1;
    if (setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd,
                   sizeof(enable_canfd)) < 0) {
      perror("CAN FD setsockopt failed");
      exit(EXIT_FAILURE);
    }
  }

  if (bind(sock_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
    perror("Error in CAN socket bind");
    exit(EXIT_FAILURE);
  }
}

CANBus::~CANBus() { close(sock_); }

int CANBus::whichCAN() { return mode_; }

bool CANBus::send(uint16_t motor_id, const std::array<uint8_t, 8>& data) {
  if (mode_ == CAN_MODE_FD)
    return sendFD(motor_id, data);
  else
    return sendClassic(motor_id, data);
}

std::array<uint8_t, 64> CANBus::recv(uint16_t& out_id, uint8_t& out_len) {
  std::array<uint8_t, 64> buffer = {};
  if (mode_ == CAN_MODE_FD) {
    auto frame = recvFD();
    out_id = frame.can_id;
    out_len = frame.len;
    std::copy(frame.data, frame.data + frame.len, buffer.begin());
  } else {
    auto frame = recvClassic();
    out_id = frame.can_id;
    out_len = frame.can_dlc;
    std::copy(frame.data, frame.data + frame.can_dlc, buffer.begin());
  }
  return buffer;
}

bool CANBus::sendClassic(uint16_t motor_id,
                         const std::array<uint8_t, 8>& data) {
  struct can_frame frame;
  std::memset(&frame, 0, sizeof(frame));

  frame.can_id = motor_id;
  frame.can_dlc = data.size();
  std::copy(data.begin(), data.end(), frame.data);

  if (write(sock_, &frame, sizeof(frame)) != sizeof(frame)) {
    perror("Error sending CAN frame");
    return false;
  }
  return true;
}

bool CANBus::sendFD(uint16_t motor_id, const std::array<uint8_t, 8>& data) {
  struct canfd_frame frame;
  std::memset(&frame, 0, sizeof(frame));

  frame.can_id = motor_id;
  frame.len = data.size();
  frame.flags = CANFD_BRS;
  std::copy(data.begin(), data.end(), frame.data);

  if (write(sock_, &frame, sizeof(frame)) != sizeof(frame)) {
    perror("Error sending CAN FD frame");
    return false;
  }
  return true;
}

struct can_frame CANBus::recvClassic() {
  struct can_frame frame;
  std::memset(&frame, 0, sizeof(frame));

  int nbytes = read(sock_, &frame, sizeof(struct can_frame));
  if (nbytes < 0) {
    perror("CAN read error");
  }
  return frame;
}

struct canfd_frame CANBus::recvFD() {
  struct canfd_frame frame;
  std::memset(&frame, 0, sizeof(frame));

  int nbytes = read(sock_, &frame, sizeof(struct canfd_frame));
  if (nbytes < 0) {
    perror("CAN FD read error");
  }
  return frame;
}
