#pragma once
#include <rclcpp/rclcpp.hpp>
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "teleoperation_server.h"
#include "nav_msgs/msg/odometry.hpp"
#include "carbot_ackermann/msg/control_command.hpp"

#include <memory>
#include <thread>

class KeyboardTeleop : public rclcpp::Node 
{
public:
    KeyboardTeleop();
    void publishCmd();
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
private:

    /// const parameters. 
    double max_steering_angle_; 
    double max_speed_; 
    double acc_increment_; 
    double steering_increment_;  

    double linear_vel_;
    double steering_angle_;
    rclcpp::Publisher<carbot_ackermann::msg::ControlCommand>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<TeleoperationServer> teleop_server_;
    

    // odometry subscriber
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    std::thread teleop_thread_;
};