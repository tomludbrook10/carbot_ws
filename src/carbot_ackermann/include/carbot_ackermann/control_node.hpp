// include/ackermann_robot/control_node.hpp
#ifndef ACKERMANN_ROBOT__CONTROL_NODE_HPP_
#define ACKERMANN_ROBOT__CONTROL_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

namespace ackermann_robot
{

class ControlNode : public rclcpp::Node
{
public:
  explicit ControlNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Callback for Twist commands (standard ROS navigation)
  void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  
  // Callback for Ackermann commands
  void ackermannCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);
  
  // Convert Twist to Ackermann command
  void convertTwistToAckermann(const geometry_msgs::msg::Twist& twist);
  
  // Send command with rate limiting and smoothing
  void sendCommand(double acceleration, double steering_angle);

  // Publishers
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_pub_;
  
  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_sub_;
  
  // Control parameters
  double max_steering_angle_;
  double max_speed_;
  double wheelbase_;
  
  // Time tracking
  rclcpp::Time last_cmd_time_;
  
  // Safety timeout
  double cmd_timeout_sec_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
  void timeoutCallback();
};

}  // namespace ackermann_robot

#endif  // ACKERMANN_ROBOT__CONTROL_NODE_HPP_