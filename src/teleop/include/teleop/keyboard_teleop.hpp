#pragma once
#include <rclcpp/rclcpp.hpp>
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "teleoperation_server.h"
#include "nav_msgs/msg/odometry.hpp"
#include "carbot_ackermann/msg/control_command.hpp"
#include "carbot_ackermann/msg/pose.hpp"

#include <memory>
#include <thread>
#include <fstream>

class KeyboardTeleop : public rclcpp::Node 
{
public:
    KeyboardTeleop();
    ~KeyboardTeleop();
    void publishCmd();
    void poseCallback(const carbot_ackermann::msg::Pose::SharedPtr msg);
private:

    /// const parameters. 
    double max_steering_angle_; 
    double max_speed_; 
    double acc_increment_; 
    double steering_increment_;  

    double linear_vel_;
    double steering_angle_;
    rclcpp::Publisher<carbot_ackermann::msg::ControlCommand>::SharedPtr pub_;
    rclcpp::Subscription<carbot_ackermann::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<TeleoperationServer> teleop_server_;

    const std::string home_data_dir = "/home/tom/carbot_ws/rollouts";
    std::string current_rollout_file_path_;
    std::unique_ptr<std::ofstream> rollout_file_;

    std::thread teleop_thread_;
};