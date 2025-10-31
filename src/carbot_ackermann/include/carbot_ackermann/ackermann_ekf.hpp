#pragma once

#include <rclcpp/rclcpp.hpp>
#include "carbot_ackermann/msg/imu_data.hpp"
#include "carbot_ackermann/msg/odometry_data.hpp"
#include "carbot_ackermann/msg/control_command.hpp"
#include "carbot_ackermann/msg/pose.hpp"

#include <vector>
#include <mutex>
#include <condition_variable>
#include <Eigen/Dense>
#include <thread>
#include <atomic>

#define Q_COV_PARAMETER "Q_covariance"
#define R_COV_PARAMETER "R_covariance"
#define STATE_SIZE 4 // [x, y, theta, v]
#define MEASUREMENT_SIZE 2 // [yaw, linear_velocity]


struct Imu {
    double timestamp_in_secs;
    double yaw;
};

struct Odometry {
    double timestamp_in_secs;
    double linear_velocity;
};

struct Control {
    double timestamp_in_secs;
    double linear_velocity;
    double steering_angle;

    Control() 
        : timestamp_in_secs(0.0), linear_velocity(0.0), steering_angle(0.0) {}

    Control(double ts, double lv, double sa)
        : timestamp_in_secs(ts), linear_velocity(lv), steering_angle(sa) {}
};

class AckermannEKF : public rclcpp::Node {
public:
    AckermannEKF();
    ~AckermannEKF();

private:
    rclcpp::Subscription<carbot_ackermann::msg::ImuData>::SharedPtr imu_subscription_;
    rclcpp::Subscription<carbot_ackermann::msg::OdometryData>::SharedPtr odom_subscription_;
    rclcpp::Subscription<carbot_ackermann::msg::ControlCommand>::SharedPtr control_subscription_;
    rclcpp::Publisher<carbot_ackermann::msg::Pose>::SharedPtr pose_publisher_;
    rclcpp::TimerBase::SharedPtr ekf_timer_;
    std::thread ekf_thread_;
    bool running_ = true;

    // callbacks
    void imuCallback(const carbot_ackermann::msg::ImuData::SharedPtr msg);
    void odomCallback(const carbot_ackermann::msg::OdometryData::SharedPtr msg);
    void controlCallback(const carbot_ackermann::msg::ControlCommand::SharedPtr msg);

    // control_buffer
    int control_buffer_size_;
    std::vector<Control> control_buffer_;
    Control last_control_accumulated_ {0, 0.0, 0.0};
    std::mutex control_mutex_; // guard access to control_buffer_ and last_control_accumulated_
    bool updateLastestControl();
    double prev_control_timestamp_ = 0.0;

    // imu.
    std::atomic<double> current_imu_val_;
    std::atomic<double> current_odom_val_;

    // carbot parameters
    double WHEELBASE;

    // ekf 
    // maintain two copies of state vector to avoid dynamic memory allocation. 
    // the correction state is the inital state and the final state. 
    Eigen::Vector<double, STATE_SIZE> correction_state_ = Eigen::Vector<double, STATE_SIZE>::Zero(); // [x, y, theta, v]
    Eigen::Vector<double, STATE_SIZE> prediction_state_ = Eigen::Vector<double, STATE_SIZE>::Zero(); // [x, y, theta, v]

    Eigen::Vector<double, MEASUREMENT_SIZE> measurement_vector_ = Eigen::Vector<double, MEASUREMENT_SIZE>::Zero(); // [yaw, linear_velocity]

    // maintain two copies of P_t, to avoid dynamic allocation of P_t.
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> prediction_covariance_;
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> correction_covariance_;

    // Jacobian matrices
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> f_jacobian_ = Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>::Identity(); // state transition jacobian
    Eigen::Matrix<double, MEASUREMENT_SIZE, STATE_SIZE> h_jacobian_ = Eigen::Matrix<double, MEASUREMENT_SIZE, STATE_SIZE>::Zero();

    // covariance matrices
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> q_covariance_; // process noise covariance
    Eigen::Matrix<double, MEASUREMENT_SIZE, MEASUREMENT_SIZE> r_covariance_; // measurement noise covariance

    Eigen::Matrix<double, STATE_SIZE, MEASUREMENT_SIZE> k_gain_; // Kalman gain

    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> identity_4d_ = Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>::Identity();

    void loadQMatrix();
    void loadRMatrix();
    void initialiseCovariance(const double initial_variance = 1.0e-3);
    void initialiseHJacobian();
    void predictState(const double dt);
    void correctState(const double yaw, const double linear_velocity);
    void computeFJacobian(const double dt);
    double computeKappa();
    void checkCovarianceStability();

    // called every ~50ms.
    void run_ekf_localisation();
    void ekf_localisation();
    int64_t last_ekf_timestamp_ = 0;

};