#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <memory>
#include <cmath>
#include <string>

#include "carbot_ackermann/imu_publisher_node.hpp"
#include "carbot_ackermann/MPU6050.hpp"

ImuPublisher::ImuPublisher() : Node("imu_publisher") 
{
    this->declare_parameter("topic_name", "imu");
    this->declare_parameter("sample_freq_mill", 50);
  
    // Get parameters
    topic_name_ = this->get_parameter("topic_name").as_string();
    sample_freq_ = this->get_parameter("sample_freq_mill").as_int();

    // Create publisher
    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(topic_name_.c_str(), 10);
    
    // Initialize MPU6050
    // Parameters: gyro sensitivity (250 deg/s), acc sensitivity (2g), tau (0.98)
    mpu_ = std::make_unique<MPU6050>(250, 2, 0.98);
    
    // Set up sensor
    if (!mpu_->setUp()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize MPU6050");
        rclcpp::shutdown();
        return;
    }
    
    // Calibrate gyroscope with 500 samples
    mpu_->calibrateGyro(sample_freq_, 100);
    
    // Initialize covariance matrices
    initializeCovarianceMatrices();
    
    // Create timer for publishing at 10 Hz
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(sample_freq_),
        std::bind(&ImuPublisher::publishImu, this));
    
    RCLCPP_INFO(this->get_logger(), "IMU Publisher initialized and running at 10 Hz");
}


void ImuPublisher::initializeCovarianceMatrices() {
    // Orientation covariance (roll, pitch, yaw) in rad^2
    // Higher uncertainty for yaw due to gyro drift integration
    std::array<double, 9> orientation_cov = {
        0.001, 0.0,   0.0,
        0.0,   0.001, 0.0,
        0.0,   0.0,   0.01
    };
    
    // Angular velocity covariance in (rad/s)^2
    // MPU6050 gyro noise: ~0.005 deg/s/√Hz
    // At 10Hz: ~0.003 rad/s variance
    double gyro_noise_variance = 0.003 * 0.003;
    std::array<double, 9> angular_velocity_cov = {
        gyro_noise_variance, 0.0, 0.0,
        0.0, gyro_noise_variance, 0.0,
        0.0, 0.0, gyro_noise_variance
    };
    
    // Linear acceleration covariance in (m/s^2)^2
    // MPU6050 accel noise: ~400 μg/√Hz
    // At 10Hz: ~0.012 m/s^2 variance
    double accel_noise_variance = 0.012 * 0.012;
    std::array<double, 9> linear_acceleration_cov = {
        accel_noise_variance, 0.0, 0.0,
        0.0, accel_noise_variance, 0.0,
        0.0, 0.0, accel_noise_variance
    };
    
    // Copy to class members
    std::copy(orientation_cov.begin(), orientation_cov.end(), 
                orientation_covariance_.begin());
    std::copy(angular_velocity_cov.begin(), angular_velocity_cov.end(), 
                angular_velocity_covariance_.begin());
    std::copy(linear_acceleration_cov.begin(), linear_acceleration_cov.end(), 
                linear_acceleration_covariance_.begin());
}
    
geometry_msgs::msg::Quaternion ImuPublisher::eulerToQuaternion(double roll, double pitch, double yaw) {
    // Convert Euler angles (radians) to quaternion
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    q.normalize();
    
    geometry_msgs::msg::Quaternion quat_msg;
    quat_msg.x = q.x();
    quat_msg.y = q.y();
    quat_msg.z = q.z();
    quat_msg.w = q.w();
    
    return quat_msg;
}
    
void ImuPublisher::publishImu() {
    // Update sensor data with complementary filter
    mpu_->compFilter();
    
    // Create IMU message
    auto msg = sensor_msgs::msg::Imu();
    
    // Set timestamp and frame
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";
    
    // Get orientation (convert degrees to radians)
    double roll_rad = mpu_->getRoll() * M_PI / 180.0;
    double pitch_rad = mpu_->getPitch() * M_PI / 180.0;
    double yaw_rad = mpu_->getYaw() * M_PI / 180.0;
    msg.orientation = eulerToQuaternion(roll_rad, pitch_rad, yaw_rad);
    
    // Get angular velocity (convert deg/s to rad/s)
    float gx, gy, gz;
    mpu_->getGyro(gx, gy, gz);
    msg.angular_velocity.x = gx * M_PI / 180.0;
    msg.angular_velocity.y = gy * M_PI / 180.0;
    msg.angular_velocity.z = gz * M_PI / 180.0;
    
    // Get linear acceleration (convert g to m/s^2)
    float ax, ay, az;
    mpu_->getAccel(ax, ay, az);
    msg.linear_acceleration.x = ax * 9.81;
    msg.linear_acceleration.y = ay * 9.81;
    msg.linear_acceleration.z = az * 9.81;
    
    // Set covariance matrices
    std::copy(orientation_covariance_.begin(), orientation_covariance_.end(),
                msg.orientation_covariance.begin());
    std::copy(angular_velocity_covariance_.begin(), angular_velocity_covariance_.end(),
                msg.angular_velocity_covariance.begin());
    std::copy(linear_acceleration_covariance_.begin(), linear_acceleration_covariance_.end(),
                msg.linear_acceleration_covariance.begin());
    
    // Publish message
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