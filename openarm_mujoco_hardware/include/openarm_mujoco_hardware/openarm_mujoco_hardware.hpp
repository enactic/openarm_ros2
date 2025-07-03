#ifndef MUJOCO_HARDWARE_INTERFACE_HPP_
#define MUJOCO_HARDWARE_INTERFACE_HPP_

#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <deque>
#include <iostream>
#include <mutex>
#include <nlohmann/json.hpp>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace openarm_mujoco_hardware {

class WebSocketSession;

class MujocoHardware : public hardware_interface::SystemInterface {
 public:
  MujocoHardware() = default;

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;
  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& /*previous_state*/) override;
  hardware_interface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State& /*previous_state*/) override;
  hardware_interface::CallbackReturn on_shutdown(
      const rclcpp_lifecycle::State& /*previous_state*/) override;
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& /*previous_state*/) override;
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& /*previous_state*/) override;
  hardware_interface::CallbackReturn on_error(
      const rclcpp_lifecycle::State& /*previous_state*/) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;
  hardware_interface::return_type read(
      const rclcpp::Time& /*time*/,
      const rclcpp::Duration& /*period*/) override;
  hardware_interface::return_type write(
      const rclcpp::Time& /*time*/,
      const rclcpp::Duration& /*period*/) override;

  friend class WebSocketSession;

 private:
  static constexpr size_t TOTAL_DOF =
      8;  // Total degrees of freedom, including gripper
  inline static constexpr std::array<double, TOTAL_DOF> KP_ = {
      180.0, 180.0, 140.0, 155.0, 115.0, 115.0, 115.0, 25.0};
  inline static constexpr std::array<double, TOTAL_DOF> KD_ = {
      0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01};
  static constexpr double MAX_MOTOR_TORQUE = 100.0;

  std::vector<double> qpos_;
  std::vector<double> qvel_;
  std::vector<double> qtau_;

  std::vector<double> cmd_qpos_;
  std::vector<double> cmd_qvel_;
  std::vector<double> cmd_qtau_ff_;

  std::mutex state_mutex_;

  void clear_cmd_torque();

  // websocket connection to mujoco
  boost::asio::ip::tcp::endpoint endpoint_;
  boost::asio::ip::address address_;
  static constexpr double kDefaultWebsocketPort = 1337;
  double websocket_port_;
  boost::asio::io_context ioc_{};
  boost::asio::ip::tcp::acceptor acceptor_{ioc_};
  std::thread ioc_thread_;

  std::shared_ptr<WebSocketSession> ws_session_;
  void start_accept();

  static const std::string kMuJoCoWebSocketURL_ =
      "https://thomasonzhou.github.io/mujoco_anywhere/";
};

class WebSocketSession : public std::enable_shared_from_this<WebSocketSession> {
 public:
  static std::shared_ptr<WebSocketSession> create(
      boost::asio::ip::tcp::socket socket, MujocoHardware* hw);
  void run();
  WebSocketSession(boost::asio::ip::tcp::socket socket, MujocoHardware* hw);

  void send_json(const nlohmann::json& j);

 private:
  void flush();
  void do_handshake();
  void on_accept(boost::beast::error_code ec);
  void do_read();
  void on_read(boost::beast::error_code ec, std::size_t);

  boost::beast::websocket::stream<boost::asio::ip::tcp::socket> ws_;
  boost::beast::flat_buffer buffer_;
  MujocoHardware* hw_;
  std::deque<std::shared_ptr<std::string>> send_queue_;
  bool write_in_progress_;
};
};  // namespace openarm_mujoco_hardware

#endif  // MUJOCO_HARDWARE_INTERFACE_HPP_

// namespace net   = boost::asio;            // from <boost/asio.hpp>
// namespace beast = boost::beast;           // from <boost/beast.hpp>
// namespace ws    = beast::websocket;       // from <boost/beast/websocket.hpp>
// namespace json  = boost::json;            // from <boost/json.hpp>
// using tcp       = net::ip::tcp;           // from <boost/asio/ip/tcp.hpp>
