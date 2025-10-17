// include/ackermann_robot/odometry_node.hpp
#ifndef ACKERMANN_ROBOT__ODOMETRY_NODE_HPP_
#define ACKERMANN_ROBOT__ODOMETRY_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "carbot_ackermann/msg/odometry_data.hpp"

namespace ackermann_robot
{

class OdometryNode : public rclcpp::Node
{
public:
  explicit OdometryNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Callback for wheel data from serial manager
  void wheelDataCallback(const carbot_ackermann::msg::OdometryData::SharedPtr msg);
  
  // Compute odometry using Ackermann model
  void computeOdometry(const rclcpp::Time& current_time, const double rps, double steering_angle);
  
  // Publish odometry and TF
  void publishOdometry(const rclcpp::Time& current_time);

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  
  // Subscribers
  rclcpp::Subscription<carbot_ackermann::msg::OdometryData>::SharedPtr wheel_data_sub_;
  
  // TF broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  // Vehicle parameters (loaded from config)
  double wheelbase_;
  double wheel_circumference_;
  double max_steering_angle_;
  
  // Odometry state
  double x_;
  double y_;
  double theta_;
  double vx_;
  double vth_;
  
  // Time tracking
  int64_t prev_time_;
  bool first_message_;
  
  // Frame IDs
  std::string odom_frame_id_;
  std::string base_frame_id_;
  bool publish_tf_;
};

}  // namespace ackermann_robot

#endif  // ACKERMANN_ROBOT__ODOMETRY_NODE_HPP_