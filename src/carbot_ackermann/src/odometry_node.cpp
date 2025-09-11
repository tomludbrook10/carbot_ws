#include "carbot_ackermann/odometry_node.hpp"
#include <cmath>

namespace ackermann_robot
{

OdometryNode::OdometryNode(const rclcpp::NodeOptions & options)
: Node("odometry_node", options),
  x_(0.0),
  y_(0.0),
  theta_(0.0),
  vx_(0.0),
  vth_(0.0),
  first_message_(true)
{
  // Declare parameters - these will be loaded from config file
  this->declare_parameter("wheelbase", 0.170);
  this->declare_parameter("wheel_circumference", 0.3078);  // 2*pi*r for r=0.1m
  this->declare_parameter("steering_angle_max", 0.436332);
  this->declare_parameter("odom_frame_id", "odom");
  this->declare_parameter("base_frame_id", "base_link");
  this->declare_parameter("publish_tf", false);
  
  // Get parameters
  wheelbase_ = this->get_parameter("wheelbase").as_double();
  wheel_circumference_ = this->get_parameter("wheel_circumference").as_double();
  steering_angle_max_ = this->get_parameter("steering_angle_max").as_double();
  odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();
  base_frame_id_ = this->get_parameter("base_frame_id").as_string();
  publish_tf_ = this->get_parameter("publish_tf").as_bool();
  
  // Create publisher
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  
  // Create subscriber
  wheel_data_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
    "wheel_feedback", 10,
    std::bind(&OdometryNode::wheelDataCallback, this, std::placeholders::_1));
  
  // Initialize TF broadcaster
  if (publish_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  }
  
  prev_time_ = this->now();
  
  RCLCPP_INFO(this->get_logger(), 
    "Odometry Node initialized - wheelbase: %.3fm, wheel_circumference: %.3fm",
    wheelbase_, wheel_circumference_);
}

void OdometryNode::wheelDataCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 2) {
    RCLCPP_WARN(this->get_logger(), "Invalid wheel data message");
    return;
  }
  
  double rps = msg->data[0];  // Revolutions per second
  
  // the angle need to be flipped as it's not using right hand rule. 
  // also angle get converted to radians.
  double steering_angle = -(msg->data[1] * (M_PI / 180.0)); // Radians
  
  computeOdometry(rps, steering_angle);
  publishOdometry();
}

void OdometryNode::computeOdometry(double rps, double steering_angle)
{
  auto current_time = this->now();
  
  if (first_message_) {
    prev_time_ = current_time;
    first_message_ = false;
    return;
  }
  
  // Calculate time delta
  double dt = (current_time - prev_time_).seconds();
  if (dt <= 0.0) {
    return;
  }
  
  // Convert RPS to linear velocity
  double velocity = rps * wheel_circumference_;  // m/s
  
  // Clamp steering angle
  steering_angle = std::max(-steering_angle_max_, 
                           std::min(steering_angle_max_, steering_angle));
  
  // Update velocities
  vx_ = velocity;
  
  // Ackermann steering model
  if (std::abs(steering_angle) > 0.001) {
    // Angular velocity: omega = v * tan(delta) / L
    vth_ = vx_ * std::tan(steering_angle) / wheelbase_;
    
    // Update position using arc motion
    double delta_theta = vth_ * dt;
    double delta_s = vx_ * dt;
    
    if (std::abs(delta_theta) > 0.001) {
      // Circular arc motion
      double radius = delta_s / delta_theta;
      x_ += radius * (std::sin(theta_ + delta_theta) - std::sin(theta_));
      y_ += radius * (-std::cos(theta_ + delta_theta) + std::cos(theta_));
      theta_ += delta_theta;
    } else {
      // Nearly straight motion
      x_ += delta_s * std::cos(theta_);
      y_ += delta_s * std::sin(theta_);
    }
  } else {
    // Straight line motion
    vth_ = 0.0;
    double delta_s = vx_ * dt;
    x_ += delta_s * std::cos(theta_);
    y_ += delta_s * std::sin(theta_);
  }
  
  // Normalize theta to [-pi, pi]
  while (theta_ > M_PI) theta_ -= 2.0 * M_PI;
  while (theta_ < -M_PI) theta_ += 2.0 * M_PI;
  
  prev_time_ = current_time;
}

void OdometryNode::publishOdometry()
{
  auto current_time = this->now();
  
  // Create quaternion from yaw
  tf2::Quaternion q;
  q.setRPY(0, 0, theta_);
  
  // Publish TF if enabled
  if (publish_tf_) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = current_time;
    transform.header.frame_id = odom_frame_id_;
    transform.child_frame_id = base_frame_id_;
    
    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.translation.z = 0.0;
    
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    
    tf_broadcaster_->sendTransform(transform);
  }
  
  // Create and publish odometry message
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = odom_frame_id_;
  odom.child_frame_id = base_frame_id_;
  
  // Set position
  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();
  
  // Set velocity
  odom.twist.twist.linear.x = vx_;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = vth_;
  
  // TODO come back and tunes these when we use robot_localization.
  // Set covariance (simplified - should be tuned)
  odom.pose.covariance[0] = 0.01;   // x
  odom.pose.covariance[7] = 0.01;   // y
  odom.pose.covariance[14] = 1e-6;  // z
  odom.pose.covariance[21] = 1e-6;  // roll
  odom.pose.covariance[28] = 1e-6;  // pitch
  odom.pose.covariance[35] = 0.03;  // yaw
  
  odom.twist.covariance[0] = 0.01;   // vx
  odom.twist.covariance[7] = 0.01;   // vy
  odom.twist.covariance[14] = 1e-6;  // vz
  odom.twist.covariance[21] = 1e-6;  // wx
  odom.twist.covariance[28] = 1e-6;  // wy
  odom.twist.covariance[35] = 0.03;  // wz
  odom_pub_->publish(odom);
}
}  // namespace ackermann_robot

// Main function
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ackermann_robot::OdometryNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}