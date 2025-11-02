#include "teleop/keyboard_teleop.hpp"
#include "carbot_ackermann/msg/control_command.hpp"
#include <termios.h>
#include <iostream>
#include <chrono>
#include <string>
#include <memory>
#include <thread>
#include <cmath>
#include <fstream>
#include <filesystem>

#include "teleoperation_server.h"
#include "defs.h"

KeyboardTeleop::KeyboardTeleop()
: Node("keyboard_teleop"), 
    linear_vel_(0.0),
    steering_angle_(0.0) {
    this->declare_parameter("max_steering_angle", 0.4363326);
    this->declare_parameter("max_speed", 2.0);
    this->declare_parameter("client_ip_address", "192.168.99.129");
    this->declare_parameter("server_ip_address", "192.168.99.201");

      // Get parameters
    max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();
    max_speed_ = this->get_parameter("max_speed").as_double();
    std::string client_ip = this->get_parameter("client_ip_address").as_string();
    std::string server_ip = this->get_parameter("server_ip_address").as_string();

    current_rollout_file_path_ = home_data_dir + "/" + std::to_string(this->now().seconds());
    std::filesystem::create_directories(current_rollout_file_path_);


    std::cout << "Rollout data will be saved to: " << current_rollout_file_path_ << std::endl;

    teleop_server_ = std::make_unique<TeleoperationServer>(server_ip, client_ip, current_rollout_file_path_);


    std::string csv_file_path = current_rollout_file_path_ + "/rollout_data.csv";

    rollout_file_ = std::make_unique<std::ofstream>(csv_file_path);
    if (!rollout_file_->is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open rollout file");
        throw std::runtime_error("Failed to open rollout file");
    }
    *rollout_file_ << "timestamp,x,y,yaw,speed\n";


    pub_ = this->create_publisher<carbot_ackermann::msg::ControlCommand>("control_cmd", 10);
    pose_sub_ = this->create_subscription<carbot_ackermann::msg::Pose>("carbot_pose", 10, std::bind(&KeyboardTeleop::poseCallback, this, std::placeholders::_1));

    teleop_thread_ = std::thread(&KeyboardTeleop::publishCmd, this);
    rclcpp::on_shutdown([this]() {
          teleop_server_->Stop();
          if (teleop_thread_.joinable()) {
            teleop_thread_.join();
        }
    });
}

KeyboardTeleop::~KeyboardTeleop() {
  rollout_file_->close();
}

void KeyboardTeleop::poseCallback(const carbot_ackermann::msg::Pose::SharedPtr msg) {
    if (rollout_file_->is_open()) {
        *rollout_file_ << msg->monotonic_timestamp_ns << ","
                       << msg->x << ","
                       << msg->y << ","
                      << msg->yaw << ","
                      << msg->linear_velocity << "\n";
    }

    Kinematics kin(msg->linear_velocity, 0, msg->yaw, msg->x, msg->y);
    teleop_server_->UpdateKinematics(kin);

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