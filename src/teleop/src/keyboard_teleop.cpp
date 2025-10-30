#include "teleop/keyboard_teleop.hpp"
#include "carbot_ackermann/msg/control_command.hpp"
#include <termios.h>
#include <iostream>
#include <chrono>
#include <string>
#include <memory>
#include <thread>
#include <cmath>

#include "teleoperation_server.h"
#include "defs.h"

KeyboardTeleop::KeyboardTeleop()
: Node("keyboard_teleop"), 
    linear_vel_(0.0),
    steering_angle_(0.0) {
    this->declare_parameter("max_steering_angle", 0.4363326);
    this->declare_parameter("max_speed", 2.0);
    this->declare_parameter("client_ip_address", "192.168.99.254");
    this->declare_parameter("server_ip_address", "192.168.99.48");

      // Get parameters
    max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();
    max_speed_ = this->get_parameter("max_speed").as_double();
    std::string client_ip = this->get_parameter("client_ip_address").as_string();
    std::string server_ip = this->get_parameter("server_ip_address").as_string();

    teleop_server_ = std::make_unique<TeleoperationServer>(server_ip, client_ip);

    pub_ = this->create_publisher<carbot_ackermann::msg::ControlCommand>("control_cmd", 10);

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
          carbot_ackermann::msg::ControlCommand msg;
          msg.linear_velocity = command.speed;
          msg.steering_angle = command.steering_angle;
          msg.header.stamp = this->now();

          pub_->publish(msg);
      } else {
          break;
      }
    }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KeyboardTeleop>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}