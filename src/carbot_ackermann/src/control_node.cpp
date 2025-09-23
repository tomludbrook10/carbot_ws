// src/control_node.cpp
#include "carbot_ackermann/control_node.hpp"
#include <cmath>
#include <algorithm>

namespace ackermann_robot
{

ControlNode::ControlNode(const rclcpp::NodeOptions & options)
: Node("control_node", options) {
  // Declare parameters
  this->declare_parameter("max_steering_angle", 0.4363326);   // radians
  this->declare_parameter("max_speed", 1.7);
  this->declare_parameter("wheelbase", 0.178);            // meters
  this->declare_parameter("cmd_timeout_sec", 0.5);      // seconds
  
  // Get parameters
  max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();
  max_speed_ = this->get_parameter("max_speed").as_double();
  wheelbase_ = this->get_parameter("wheelbase").as_double();
  cmd_timeout_sec_ = this->get_parameter("cmd_timeout_sec").as_double();
  
  // Create publisher
  ackermann_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
    "ackermann_cmd", 10);
  
  // Create subscribers
  twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    std::bind(&ControlNode::twistCallback, this, std::placeholders::_1));
  
  ackermann_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
    "ackermann_cmd_in", 10,
    std::bind(&ControlNode::ackermannCallback, this, std::placeholders::_1));
  
  // Create timeout timer
  timeout_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(cmd_timeout_sec_ * 1000)),
    std::bind(&ControlNode::timeoutCallback, this));
  
  last_cmd_time_ = this->now();
  
  RCLCPP_INFO(this->get_logger(), 
    "Control Node initialized - max_speed: %.2f, max_steering: %.2f rad",
    max_speed_, max_steering_angle_);
}

void ControlNode::twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // Convert Twist to Ackermann
  convertTwistToAckermann(*msg);
  last_cmd_time_ = this->now();
}

void ControlNode::ackermannCallback(
  const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
{
  // Forward Ackermann command with safety limits
  sendCommand(msg->drive.acceleration, msg->drive.steering_angle);
  last_cmd_time_ = this->now();
}

void ControlNode::convertTwistToAckermann(const geometry_msgs::msg::Twist& twist)
{
  // Extract linear and angular velocities
  double linear_vel = twist.linear.x;
  double angular_vel = twist.angular.z;
  
  // Calculate steering angle from angular velocity
  // For Ackermann: steering_angle = atan(L * omega / v)
  double steering_angle = 0.0;
  
  if (std::abs(linear_vel) > 0.01) {
    steering_angle = std::atan(wheelbase_ * angular_vel / linear_vel);
  } else if (std::abs(angular_vel) > 0.01) {
    // Turning in place - use maximum steering
    steering_angle = (angular_vel > 0) ? max_steering_angle_ : -max_steering_angle_;
  }

  sendCommand(linear_vel, steering_angle);
}

void ControlNode::sendCommand(double speed, double steering_angle)
{
  // Apply acceleration limits
  speed = std::max(-max_speed_, 
                          std::min(max_speed_, speed));
  
  // Apply steering angle limits
  steering_angle = std::max(-max_steering_angle_, 
                           std::min(max_steering_angle_, steering_angle));
  
  // Create and publish Ackermann message
  auto msg = ackermann_msgs::msg::AckermannDriveStamped();
  msg.header.stamp = this->now();
  msg.header.frame_id = "base_link";
  msg.drive.speed = speed;
  msg.drive.steering_angle = steering_angle;
  ackermann_pub_->publish(msg);
}

void ControlNode::timeoutCallback()
{
  double time_since_cmd = (this->now() - last_cmd_time_).seconds();
  
  if (time_since_cmd > cmd_timeout_sec_) {
    // Emergency stop
    //RCLCPP_WARN(this->get_logger(), "Command timeout - emergency stop!");
    //sendCommand(0.0, 0.0);
  }
}

}  // namespace ackermann_robot

// Main function
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ackermann_robot::ControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}