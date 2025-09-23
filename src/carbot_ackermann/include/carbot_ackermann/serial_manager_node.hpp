// include/ackermann_robot/serial_manager_node.hpp
#ifndef ACKERMANN_ROBOT__SERIAL_MANAGER_NODE_HPP_
#define ACKERMANN_ROBOT__SERIAL_MANAGER_NODE_HPP_

#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>

namespace ackermann_robot
{

class SerialManagerNode : public rclcpp::Node
{
public:
  explicit SerialManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~SerialManagerNode();

private:
  // Serial communication methods
  bool initializeSerial();
  void resetESP32();
  void serialReadThread();
  void processIncomingData(const std::string& data);
  void sendCommandToESP32(double acceleration, double steering_angle);
  
  // Callback for control commands
  void controlCommandCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);
  
  // Watchdog timer
  void watchdogCallback();

  // Publishers - publishes raw wheel data from ESP32
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_data_pub_;
  
  // Subscribers - receives control commands
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr control_cmd_sub_;
  
  // Timer for watchdog
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  
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
  double wheel_circumference_;
  
  // Communication state
  rclcpp::Time last_serial_msg_time_;
  double serial_timeout_sec_;
  bool connection_established_;
};

}  // namespace ackermann_robot

#endif  // ACKERMANN_ROBOT__SERIAL_MANAGER_NODE_HPP_