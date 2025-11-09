#include "carbot_inference/pure_pursuit_controller.hpp"
#include <cmath>
#include <rclcpp/rclcpp.hpp>

PurePursuitController::PurePursuitController() : Node("pure_pursuit_controller") {
    waypoints_subscriber_ = this->create_subscription<carbot_inference::msg::Waypoints>(
        "waypoints", 10, std::bind(&PurePursuitController::waypoints_callback, this, std::placeholders::_1));

    control_publisher_ = this->create_publisher<carbot_ackermann::msg::ControlCommand>("control_cmd", 10);

    pose_sub_ = this->create_subscription<carbot_ackermann::msg::Pose>("carbot_pose", 10,
        std::bind(&PurePursuitController::pose_callback, this, std::placeholders::_1));
}

Pose PurePursuitController::odom_to_local_frame(const Pose& odom_pose, const Pose& ref_pose) const {
    float dx = odom_pose.x - ref_pose.x;
    float dy = odom_pose.y - ref_pose.y;

    float local_x =  dx * cos(-ref_pose.yaw) - dy * sin(-ref_pose.yaw);
    float local_y =  dx * sin(-ref_pose.yaw) + dy * cos(-ref_pose.yaw);
    float local_yaw = odom_pose.yaw - ref_pose.yaw;

    return {local_x, local_y, local_yaw};
}

void PurePursuitController::pose_callback(const carbot_ackermann::msg::Pose::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(waypoints_mutex_);
    if (current_waypoints_.empty()) {
        return;
    }

    // Get the current pose information
    prev_odom_pose_ = {msg->x, msg->y, msg->yaw};
    Pose current_local_pose = odom_to_local_frame(prev_odom_pose_, current_ref_pose_);

    // Find the target waypoint
    while (current_waypoint_index_ < current_waypoints_.size()) {
        float waypoint_x = current_waypoints_[current_waypoint_index_];
        float waypoint_y = current_waypoints_[current_waypoint_index_ + 1];

        // Check if the waypoint is within the lookahead distance
        if (std::hypot(waypoint_x - current_local_pose.x, waypoint_y - current_local_pose.y) < LOOKAHEAD_DISTANCE) {
            current_waypoint_index_ += 2;
        } else {
            break;
        }
    }

    float target_x = 0.0;
    float target_y = 0.0;

    if (current_waypoint_index_ < current_waypoints_.size()) {
        target_x = current_waypoints_[current_waypoint_index_];
        target_y = current_waypoints_[current_waypoint_index_ + 1];
    } else {
        target_x = current_waypoints_[current_waypoints_.size() - 2];
        target_y = current_waypoints_[current_waypoints_.size() - 1];
    }

    float distance = std::hypot(target_x - current_local_pose.x, target_y - current_local_pose.y);
    float steering_angle = atan( (2 * WHEELBASE * (target_y - current_local_pose.y)) / (distance * distance) );


    RCLCPP_INFO(this->get_logger(), "Steering Angle: %.4f, Target Waypoint: (%.6f, %.6f), Car Position: (%.6f, %.6f, %.6f)", 
    steering_angle, target_x, target_y, current_local_pose.x, current_local_pose.y, current_local_pose.yaw);

    // Publish the control command
    auto control_msg = carbot_ackermann::msg::ControlCommand();
    control_msg.steering_angle = steering_angle;
    control_msg.linear_velocity = SPEED;
    control_publisher_->publish(control_msg);
}

void PurePursuitController::waypoints_callback(const carbot_inference::msg::Waypoints::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(waypoints_mutex_);
    current_waypoints_ = msg->waypoints;
    current_waypoint_index_ = 0;
    current_ref_pose_ = prev_odom_pose_;

    if (current_waypoints_.size() != NUM_WAYPOINTS * 2) {
        RCLCPP_WARN(this->get_logger(), "Received waypoints size (%zu) does not match expected size (%ld).",
                    current_waypoints_.size(), NUM_WAYPOINTS * 2);
        return;
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PurePursuitController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}