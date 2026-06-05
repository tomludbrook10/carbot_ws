#include <carbot_ackermann/ackermann_ekf.hpp>

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <mutex>
#include <limits>

AckermannEKF::AckermannEKF() : Node("ackermann_ekf_node") {
    this->declare_parameter("control_buffer_size", 10);
    this->declare_parameter("wheel_base", 0.178); 
    this->declare_parameter("publish_rate_hz", 20);
    this->declare_parameter(Q_COV_PARAMETER, std::vector<double>{});
    this->declare_parameter(R_COV_PARAMETER, std::vector<double>{});

    control_buffer_size_ = this->get_parameter("control_buffer_size").as_int();
    WHEELBASE = this->get_parameter("wheel_base").as_double();
    int publish_rate_hz = this->get_parameter("publish_rate_hz").as_int();

    std::cout << WHEELBASE << std::endl;
    std::cout << control_buffer_size_ << std::endl;

    loadQMatrix();
    loadRMatrix();
    initialiseCovariance();
    initialiseHJacobian();

    // Subscribers
    imu_subscription_ = this->create_subscription<carbot_ackermann::msg::ImuData>(
        "imu", 10, std::bind(&AckermannEKF::imuCallback, this, std::placeholders::_1));

    odom_subscription_ = this->create_subscription<carbot_ackermann::msg::OdometryData>(
        "wheel_odometry", 10, std::bind(&AckermannEKF::odomCallback, this, std::placeholders::_1));

    control_subscription_ = this->create_subscription<carbot_ackermann::msg::ControlCommand>(
        "control_cmd", 10, std::bind(&AckermannEKF::controlCallback, this, std::placeholders::_1));

    // Publisher
    pose_publisher_ = this->create_publisher<carbot_ackermann::msg::Pose>(
        "carbot_pose", 10);

    control_buffer_.reserve(control_buffer_size_); // preallocate space for control commands

    prev_control_timestamp_ = getCurrentTime();

    RCLCPP_INFO(this->get_logger(), "Ackermann EKF node initialized");
    RCLCPP_INFO(this->get_logger(), "Publishing at %d Hz", publish_rate_hz);    
}

uint64_t AckermannEKF::getCurrentTime() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

void AckermannEKF::controlCallback(const carbot_ackermann::msg::ControlCommand::SharedPtr msg) {
    {
        std::lock_guard<std::mutex> lock(control_mutex_);
        control_buffer_.emplace_back(
            static_cast<double>(msg->header.stamp.sec) +
            static_cast<double>(msg->header.stamp.nanosec) * 1e-9,
            msg->linear_velocity,
            msg->steering_angle
        );
    }
}

void AckermannEKF::imuCallback(const carbot_ackermann::msg::ImuData::SharedPtr msg) {
        current_imu_val_.store(static_cast<double>(msg->yaw), std::memory_order_relaxed);
}

void AckermannEKF::odomCallback(const carbot_ackermann::msg::OdometryData::SharedPtr msg) {
    ekf_localisation(msg->monotonic_timestamp_ns, static_cast<double>(msg->linear_velocity));
}

bool AckermannEKF::updateLastestControl() {
    {
        std::lock_guard<std::mutex> lock(control_mutex_);
        int size = control_buffer_.size();
        if (size > 0) {
            float accumulated_linear_velocity = 0.0f;
            float accumulated_steering_angle = 0.0f;
            for (const auto& control : control_buffer_) {
                accumulated_linear_velocity += control.linear_velocity;
                accumulated_steering_angle += control.steering_angle;
            }
            last_control_accumulated_.timestamp_in_secs = control_buffer_[size - 1].timestamp_in_secs;
            last_control_accumulated_.linear_velocity = accumulated_linear_velocity / size;
            last_control_accumulated_.steering_angle = accumulated_steering_angle / size;
            control_buffer_.clear();
        } else {
            last_control_accumulated_.timestamp_in_secs = this->now().seconds();
            last_control_accumulated_.linear_velocity = 0.0;
            last_control_accumulated_.steering_angle = 0.0;
        }
    }
    return true;
}

void AckermannEKF::loadQMatrix() {
    std::vector<double> q_matrix_in;

    if (!this->get_parameter(Q_COV_PARAMETER, q_matrix_in)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get Q covariance matrix parameter");
        throw std::runtime_error("Failed to get Q covariance matrix parameter");
    }

    if (q_matrix_in.size() != STATE_SIZE * STATE_SIZE) {
        RCLCPP_ERROR(this->get_logger(), "Q covariance matrix must be %dx%d, size is %d", STATE_SIZE, STATE_SIZE, (int)q_matrix_in.size());
        throw std::runtime_error("Q covariance matrix must be %dx%d");
    }
    
    for (int row = 0; row < STATE_SIZE; ++row) {
        for (int col = 0; col < STATE_SIZE; ++col) {
            q_covariance_(row, col) = q_matrix_in[row * STATE_SIZE + col];
        }
    }
}

void AckermannEKF::loadRMatrix() {
    std::vector<double> r_matrix_in;

    if (!this->get_parameter(R_COV_PARAMETER, r_matrix_in)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get R covariance matrix parameter");
        throw std::runtime_error("Failed to get R covariance matrix parameter");
    }

    if (r_matrix_in.size() != MEASUREMENT_SIZE * MEASUREMENT_SIZE) {
        RCLCPP_ERROR(this->get_logger(), "R covariance matrix must be %dx%d, size is %d", MEASUREMENT_SIZE, MEASUREMENT_SIZE, (int)r_matrix_in.size());
        throw std::runtime_error("R covariance matrix must be %dx%d");
    }

    for (int row = 0; row < MEASUREMENT_SIZE; ++row) {
        for (int col = 0; col < MEASUREMENT_SIZE; ++col) {
            r_covariance_(row, col) = r_matrix_in[row * MEASUREMENT_SIZE + col];
        }
    }
}

void AckermannEKF::initialiseCovariance(const double initial_variance) {
    correction_covariance_ = Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>::Identity() * initial_variance;
}

void AckermannEKF::initialiseHJacobian() {
    h_jacobian_(0, 2) = 1.0; // theta
    h_jacobian_(1, 3) = 1.0; // v
}

double AckermannEKF::computeKappa() {
    return std::tan(last_control_accumulated_.steering_angle) / WHEELBASE;

}

void AckermannEKF::predictState(const double dt) {
    double theta = correction_state_(2);
    double v = correction_state_(3);

    prediction_state_(0) += v * std::cos(theta) * dt; // x
    prediction_state_(1) += v * std::sin(theta) * dt; // y
    prediction_state_(2) += v * computeKappa() * dt; // theta
    prediction_state_(3) = last_control_accumulated_.linear_velocity; // v
}

void AckermannEKF::correctState(const double yaw, const double linear_velocity) {
    measurement_vector_(0) = yaw;
    measurement_vector_(1) = linear_velocity;
    correction_state_.noalias() = prediction_state_ + k_gain_ * (measurement_vector_ - h_jacobian_ * prediction_state_);
}

void AckermannEKF::checkCovarianceStability() {
    for (int i = 0; i < STATE_SIZE; ++i) {
        if (std::isnan(correction_covariance_(i, i)) || std::isinf(correction_covariance_(i, i) ) || correction_covariance_(i, i) > 0.1) {
            RCLCPP_ERROR(this->get_logger(), "Covariance matrix unstable at index %d: %f", i, correction_covariance_(i, i));
        }
    }
}

void AckermannEKF::computeFJacobian(const double dt) {
    // we use the state from correction step at time t-1. 
    // note that on the diagonal it's 1.0, for any state. 
    double theta = correction_state_(2);
    double v = correction_state_(3);
    f_jacobian_(0, 2) = -v * std::sin(theta) * dt;
    f_jacobian_(0, 3) = std::cos(theta) * dt;
    f_jacobian_(1, 2) = v * std::cos(theta) * dt;
    f_jacobian_(1, 3) = std::sin(theta) * dt;
    f_jacobian_(2, 3) = computeKappa() * dt;
}


void AckermannEKF::ekf_localisation(const uint64_t current_time_ns, const double linear_velocity) {
    // obtain u_t. 
    updateLastestControl();
    double yaw = current_imu_val_.load(std::memory_order_relaxed);
    double dt = static_cast<double>(current_time_ns - prev_control_timestamp_) * 1e-9; // in seconds
    prev_control_timestamp_ = current_time_ns;

    // predict step.
    computeFJacobian(dt); // must come first before predictState, since we use correction state at t-1.
    predictState(dt);
    prediction_covariance_.noalias() = f_jacobian_ * correction_covariance_ * f_jacobian_.transpose() + q_covariance_;

    // correct step.
    k_gain_.noalias() = prediction_covariance_ * h_jacobian_.transpose() * (h_jacobian_ * prediction_covariance_ * h_jacobian_.transpose() + r_covariance_).inverse();
    correctState(yaw, linear_velocity);

    // Update covariance
    correction_covariance_.noalias() = (identity_4d_ - k_gain_ * h_jacobian_) * prediction_covariance_;

    // publish pose
    auto pose_msg = carbot_ackermann::msg::Pose();
    pose_msg.header.stamp = this->now(); // I'm not sure about this one yet. If I should use time from last control, imu, odom or now. 
    pose_msg.x = static_cast<float>(correction_state_(0));
    pose_msg.y = static_cast<float>(correction_state_(1));
    pose_msg.yaw = static_cast<float>(correction_state_(2));
    pose_msg.linear_velocity = static_cast<float>(correction_state_(3));

    pose_msg.monotonic_timestamp_ns = current_time_ns;
    pose_publisher_->publish(pose_msg);

    if (dt >= 0.025) {
        RCLCPP_WARN(this->get_logger(), "EKF localisation loop is running slowly. dt = %.4f seconds", dt);
    }
    
    //checkCovarianceStability();
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    // we need atleast two threads, because the localisation thread blocks while waiting for imu and odom data.
    auto node = std::make_shared<AckermannEKF>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
