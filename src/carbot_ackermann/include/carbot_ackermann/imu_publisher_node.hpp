#include <string>

// ros2 messages.
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "carbot_ackermann/MPU6050.hpp"


class ImuPublisher : public rclcpp::Node {
public:
    ImuPublisher();

private:
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<MPU6050> mpu_;
    
    std::array<double, 9> orientation_covariance_;
    std::array<double, 9> angular_velocity_covariance_;
    std::array<double, 9> linear_acceleration_covariance_;

    // params
    std::string topic_name_;
    int sample_freq_;

    void initializeCovarianceMatrices();
    void publishImu();
    geometry_msgs::msg::Quaternion eulerToQuaternion(double roll, double pitch, double yaw);
};