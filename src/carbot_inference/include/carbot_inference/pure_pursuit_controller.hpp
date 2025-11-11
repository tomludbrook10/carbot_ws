#pragma once

#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <vector>

#include "carbot_inference/msg/waypoints.hpp"
#include "carbot_ackermann/msg/control_command.hpp"
#include "carbot_ackermann/msg/pose.hpp"


struct Pose {
    float x;
    float y;
    float yaw;
};

struct DebugInfo {
    size_t frame_index;
    Pose ref_pose; 
    std::vector<float> waypoints;

    DebugInfo() : frame_index(0), ref_pose{0.0, 0.0, 0.0}, waypoints{} {}
    DebugInfo(size_t index, const Pose& pose, std::vector<float>& wpts)
        : frame_index(index), ref_pose(pose), waypoints(wpts) {}
};


class PurePursuitController : public rclcpp::Node {
public:
    PurePursuitController();

private:
    const float LOOKAHEAD_DISTANCE = 0.15;
    const float SPEED = 0.3;
    size_t NUM_WAYPOINTS =  8;
    const float WHEELBASE = 0.178;

    rclcpp::Subscription<carbot_inference::msg::Waypoints>::SharedPtr waypoints_subscriber_;
    rclcpp::Publisher<carbot_ackermann::msg::ControlCommand>::SharedPtr control_publisher_;
    rclcpp::Subscription<carbot_ackermann::msg::Pose>::SharedPtr pose_sub_;

    void waypoints_callback(const carbot_inference::msg::Waypoints::SharedPtr msg);
    void pose_callback(const carbot_ackermann::msg::Pose::SharedPtr msg);
    Pose odom_to_local_frame(const Pose& odom_pose, const Pose& ref_pose) const;

    std::mutex waypoints_mutex_;
    std::vector<float> current_waypoints_;
    size_t current_waypoint_index_ = 0;

    Pose prev_odom_pose_ {0.0, 0.0, 0.0};
    Pose current_ref_pose_ {0.0, 0.0, 0.0};

    bool debug_mode_ = false;
    size_t num_frames_ = 0;
    std::string debug_save_path_ = "";
    std::vector<DebugInfo> debug_data_;

    void save_debug_data();

};