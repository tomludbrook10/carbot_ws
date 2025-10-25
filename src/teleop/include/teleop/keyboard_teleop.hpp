#pragma once
#include <rclcpp/rclcpp.hpp>
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "teleoperation_server.h"

#include <memory>
#include <thread>

class KeyboardTeleop : public rclcpp::Node 
{
public:
    KeyboardTeleop();
    void publishCmd();
private:

    /// const parameters. 
    double max_steering_angle_; 
    double max_speed_; 
    double acc_increment_; 
    double steering_increment_;  

    double linear_vel_;
    double steering_angle_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<TeleoperationServer> teleop_server_;

    std::thread teleop_thread_;
};