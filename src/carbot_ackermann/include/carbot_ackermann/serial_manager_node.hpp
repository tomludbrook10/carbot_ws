// include/ackermann_robot/serial_manager_node.hpp
#ifndef ACKERMANN_ROBOT__SERIAL_MANAGER_NODE_HPP_
#define ACKERMANN_ROBOT__SERIAL_MANAGER_NODE_HPP_

#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include "carbot_ackermann/msg/odometry_data.hpp"
#include "carbot_ackermann/msg/control_command.hpp"

namespace ackermann_robot
{

class SerialManagerNode : public rclcpp::Node
{
public:
  explicit SerialManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~SerialManagerNode();

private:
  // timer sync
  static constexpr int SYNC_GPIO = 7;
  static constexpr int PULSE_WIDTH_US = 10; // 10 mirco seconds
  bool gpio_initialized_ = false;
  std::atomic<uint64_t> reference_time_;
  rclcpp::TimerBase::SharedPtr timer_;

  void setupGPIO();
  void send_pulse();

  void reset_timer();

  // Serial communication methods
  bool initializeSerial();
  void resetESP32();
  void syncESP32Clock();
  void serialReadThread();
  void processIncomingData(const std::string& data, std::string& token);
  void sendCommandToESP32(float speed, float steering_angle);
  void processLogMessage(const std::string& log_msg);
  
  // Callback for control commands
  void controlCommandCallback(const carbot_ackermann::msg::ControlCommand::SharedPtr msg);

  // Publishers - publishes raw wheel data from ESP32
  rclcpp::Publisher<carbot_ackermann::msg::OdometryData>::SharedPtr wheel_data_pub_;
  
  // Subscribers - receives control commands
  rclcpp::Subscription<carbot_ackermann::msg::ControlCommand>::SharedPtr control_cmd_sub_;

  // Serial port handling
  std::unique_ptr<boost::asio::io_service> io_service_;
  std::unique_ptr<boost::asio::serial_port> serial_port_;
  std::thread serial_thread_;
  std::atomic<bool> serial_thread_running_;
  std::mutex serial_mutex_;
  
  // Serial configuration
  std::string serial_port_name_;
  int baud_rate_;

  // convertion from linear_velocity to rps. 
  float wheel_circumference_;
  
  // Communication state
  bool connection_established_;

  const int MAX_FAILED_SYNC = 5;
  int failed_sync_count_ = 0;
};

}  // namespace ackermann_robot

#endif  // ACKERMANN_ROBOT__SERIAL_MANAGER_NODE_HPP_