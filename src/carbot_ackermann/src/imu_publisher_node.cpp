#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <cmath>
#include <string>
#include "carbot_ackermann/imu_publisher_node.hpp"
#include "carbot_ackermann/msg/imu_data.hpp"
#include "carbot_ackermann/MPU6050.hpp"

ImuPublisher::ImuPublisher() : Node("imu_publisher")  {
    this->declare_parameter("topic_name", "imu");
    this->declare_parameter("sample_freq_mill", 10);
  
    // Get parameters
    topic_name_ = this->get_parameter("topic_name").as_string();
    sample_freq_ = this->get_parameter("sample_freq_mill").as_int();

    // Create publisher
    publisher_ = this->create_publisher<carbot_ackermann::msg::ImuData>(topic_name_.c_str(), 10);
    
    // Initialize MPU6050
    // Parameters: gyro sensitivity (250 deg/s), acc sensitivity (2g), tau (0.98)
    mpu_ = std::make_unique<MPU6050>(500);
    
    // Set up sensor
    if (!mpu_->setUp()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize MPU6050");
        rclcpp::shutdown();
        return;
    }

    // Calibrate gyroscope with 2000 samples
    mpu_->calibrateGyro(2000);
    
    // Create timer for publishing at 100 Hz
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(sample_freq_),
        std::bind(&ImuPublisher::publishImu, this));

    RCLCPP_INFO(this->get_logger(), "IMU Publisher initialized and running at %d Hz", 1000 / sample_freq_);
}
    
void ImuPublisher::publishImu() {
    mpu_->processYaw();

    auto msg = carbot_ackermann::msg::ImuData();
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";
    msg.yaw = static_cast<float>(mpu_->getYaw() * M_PI / 180.0); // convert to radians
    publisher_->publish(msg);
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    
    // Create and spin node
    auto node = std::make_shared<ImuPublisher>();
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}