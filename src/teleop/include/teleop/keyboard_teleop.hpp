#pragma once
#include <rclcpp/rclcpp.hpp>
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class KeyboardTeleop : public rclcpp::Node 
{
public:
    KeyboardTeleop();
    int getKey();
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
};