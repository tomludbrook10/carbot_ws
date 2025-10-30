#pragma once

#include <string>

// ros2 messages.
#include "rclcpp/rclcpp.hpp"
#include "carbot_ackermann/msg/imu_data.hpp"
#include "carbot_ackermann/MPU6050.hpp"

class ImuPublisher : public rclcpp::Node {
public:
    ImuPublisher();

private:
    rclcpp::Publisher<carbot_ackermann::msg::ImuData>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<MPU6050> mpu_;
    
    // params
    std::string topic_name_;
    int sample_freq_;

    void publishImu();
};