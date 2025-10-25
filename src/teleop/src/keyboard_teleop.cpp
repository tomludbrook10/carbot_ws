#include "teleop/keyboard_teleop.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <termios.h>
#include <iostream>
#include <chrono>
#include <string>
#include <memory>
#include <thread>

#include "teleoperation_server.h"
#include "defs.h"

KeyboardTeleop::KeyboardTeleop()
: Node("keyboard_teleop"), 
    linear_vel_(0.0),
    steering_angle_(0.0) {
    this->declare_parameter("max_steering_angle", 0.4363326);
    this->declare_parameter("max_speed", 2.0);
    this->declare_parameter("acc_increment", 0.1);
    this->declare_parameter("steering_increment", 0.05);
    this->declare_parameter("client_ip_address", "192.168.99.254");
    this->declare_parameter("server_ip_address", "192.168.99.48");

      // Get parameters
    max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();
    max_speed_ = this->get_parameter("max_speed").as_double();
    acc_increment_ = this->get_parameter("acc_increment").as_double();
    steering_increment_ = this->get_parameter("steering_increment").as_double();
    std::string client_ip = this->get_parameter("client_ip_address").as_string();
    std::string server_ip = this->get_parameter("server_ip_address").as_string();

    teleop_server_ = std::make_unique<TeleoperationServer>(server_ip, client_ip);

      pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("ackermann_cmd_in", 10);

      teleop_thread_ = std::thread(&KeyboardTeleop::publishCmd, this);
      rclcpp::on_shutdown([this]() {
           teleop_server_->Stop();
           if (teleop_thread_.joinable()) {
              teleop_thread_.join();
          }
      });
}

void KeyboardTeleop::publishCmd() {
    CommandRequest command;

    while (teleop_server_->IsRunning()) {
      if (teleop_server_->GetNextCommand(command)) {
          linear_vel_ = command.speed;
          steering_angle_ = command.steering_angle;

          ackermann_msgs::msg::AckermannDriveStamped  cmd;
          cmd.drive.speed= linear_vel_;
          cmd.drive.steering_angle = steering_angle_;
          pub_->publish(cmd);
      } else {
          break;
      }
    }

   // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
    //  "Vel: %.2f m/s, Steer: %.2f rad", linear_vel_, steering_angle_);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KeyboardTeleop>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}