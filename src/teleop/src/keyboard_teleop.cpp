#include "teleop/keyboard_teleop.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <termios.h>
#include <iostream>
#include <chrono>

KeyboardTeleop::KeyboardTeleop()
: Node("keyboard_teleop"), 
    linear_vel_(0.0),
    steering_angle_(0.0) 
{
    this->declare_parameter("max_steering_angle", 0.4363326);
    this->declare_parameter("max_speed", 2.0);
    this->declare_parameter("acc_increment", 0.1);
    this->declare_parameter("steering_increment", 0.05);

      // Get parameters
    max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();
    max_speed_ = this->get_parameter("max_speed").as_double();
    acc_increment_ = this->get_parameter("acc_increment").as_double();
    steering_increment_ = this->get_parameter("steering_increment").as_double();

    pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("ackermann_cmd_in", 10);
    RCLCPP_INFO(this->get_logger(), "Keyboard teleop started. Use W/A/S/D keys.");

    // Timer for periodic publishing
    timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&KeyboardTeleop::publishCmd, this));
}

int KeyboardTeleop::getKey() {
    struct termios oldt, newt;
    int ch;
    int oldf;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    struct timeval tv = {0L, 0L};
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    int ret = select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);
    if (ret > 0)
      ch = getchar();
    else
      ch = -1;
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

void KeyboardTeleop::publishCmd() {
    int key = getKey();
    if (key != -1)
    {
      switch (key)
      {
        case 'w':
        case 'W':
          linear_vel_ += acc_increment_;
          if (linear_vel_ > max_speed_) linear_vel_ = max_speed_;
          break;
        case 's':
        case 'S':
          linear_vel_ -= acc_increment_;
          if (linear_vel_ < -max_speed_) linear_vel_ = -max_speed_;
          break;
        case 'a':
        case 'A':
          steering_angle_ += steering_increment_;
          if (steering_angle_ > max_steering_angle_) steering_angle_ = max_steering_angle_;
          break;
        case 'd':
        case 'D':
          steering_angle_ -= steering_increment_;
          if (steering_angle_ < -max_steering_angle_) steering_angle_ = -max_steering_angle_;
          break;
        case 'q':
        case 'Q':
          RCLCPP_INFO(this->get_logger(), "Exiting teleop...");
          rclcpp::shutdown();
          return;
      }
    }
    else
    {
      // Gradually return steering to zero when no input
      if (steering_angle_ > 0.0)
        steering_angle_ = std::max(0.0, steering_angle_ - steering_increment_ * 2);
      else if (steering_angle_ < 0.0)
        steering_angle_ = std::min(0.0, steering_angle_ + steering_increment_ * 2);
    }

    ackermann_msgs::msg::AckermannDriveStamped  cmd;
    cmd.drive.speed= linear_vel_;
    cmd.drive.steering_angle = steering_angle_;
    pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
      "Vel: %.2f m/s, Steer: %.2f rad", linear_vel_, steering_angle_);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KeyboardTeleop>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}