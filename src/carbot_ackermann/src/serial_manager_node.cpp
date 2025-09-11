// src/serial_manager_node.cpp
#include "carbot_ackermann/serial_manager_node.hpp"
#include <sstream>
#include <iomanip>

namespace ackermann_robot
{

SerialManagerNode::SerialManagerNode(const rclcpp::NodeOptions & options)
: Node("serial_manager_node", options),
  serial_thread_running_(false),
  connection_established_(false)
{
  // Declare parameters
  this->declare_parameter("serial_port", "/dev/ttyACM0");
  this->declare_parameter("baud_rate", 115200);
  this->declare_parameter("serial_timeout_sec", 2.0);
  
  // Get parameters
  serial_port_name_ = this->get_parameter("serial_port").as_string();
  baud_rate_ = this->get_parameter("baud_rate").as_int();
  serial_timeout_sec_ = this->get_parameter("serial_timeout_sec").as_double();

    // Create publisher for wheel data
  wheel_data_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
    "wheel_feedback", 10);
  
  // Create subscriber for control commands
  control_cmd_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
    "ackermann_cmd", 10,
    std::bind(&SerialManagerNode::controlCommandCallback, this, std::placeholders::_1));

  
  // Create watchdog timer
  watchdog_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&SerialManagerNode::watchdogCallback, this));
  
  last_serial_msg_time_ = this->now();

  // Initialize serial - CRITICAL: fail if cannot connect
  if (!initializeSerial()) {
    RCLCPP_FATAL(this->get_logger(), "Failed to initialize serial communication with ESP32");
    throw std::runtime_error("Serial initialization failed - cannot start robot system");
  }
  
  RCLCPP_INFO(this->get_logger(), 
    "Serial Manager initialized successfully on %s at %d baud",
    serial_port_name_.c_str(), baud_rate_);
}

SerialManagerNode::~SerialManagerNode()
{
  serial_thread_running_ = false;
  if (serial_thread_.joinable()) {
    serial_thread_.join();
  }
  
  if (serial_port_ && serial_port_->is_open()) {
    serial_port_->close();
  }
}

bool SerialManagerNode::initializeSerial()
{
  try {
    io_service_ = std::make_unique<boost::asio::io_service>();
    serial_port_ = std::make_unique<boost::asio::serial_port>(*io_service_);
    
    // Try to open serial port
    serial_port_->open(serial_port_name_);
    
    // Configure serial port
    serial_port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
    serial_port_->set_option(boost::asio::serial_port_base::character_size(8));
    serial_port_->set_option(boost::asio::serial_port_base::stop_bits(
      boost::asio::serial_port_base::stop_bits::one));
    serial_port_->set_option(boost::asio::serial_port_base::parity(
      boost::asio::serial_port_base::parity::none));
    serial_port_->set_option(boost::asio::serial_port_base::flow_control(
      boost::asio::serial_port_base::flow_control::none));
    
    // Reset ESP32
    resetESP32();
    
    // Start serial reading thread
    serial_thread_running_ = true;
    serial_thread_ = std::thread(&SerialManagerNode::serialReadThread, this);
    
    // Wait briefly to confirm connection
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    connection_established_ = true;
    return true;
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Serial port error: %s", e.what());
    return false;
  }
}

void SerialManagerNode::resetESP32()
{
    RCLCPP_INFO(this->get_logger(), "Resetting ESP32...");
    /*
    // Use DTR/RTS for auto-reset (standard on ESP32 dev boards)
    serial_port_->set_option(boost::asio::serial_port_base::dtr(false));
    serial_port_->set_option(boost::asio::serial_port_base::rts(true));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    serial_port_->set_option(boost::asio::serial_port_base::dtr(false));
    serial_port_->set_option(boost::asio::serial_port_base::rts(false));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    serial_port_->set_option(boost::asio::serial_port_base::dtr(true));
    serial_port_->set_option(boost::asio::serial_port_base::rts(false));
    */

    int fd = serial_port_->native_handle();
    if (fd == -1) {
        std::cerr << "Invalid serial port file descriptor." << std::endl;
        return;
    }

    // Set DTR and RTS low
    int data = TIOCM_DTR | TIOCM_RTS;
    ioctl(fd, TIOCMBIC, &data); // TIOCMBIC clears the bits
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Cycle DTR and RTS to trigger a reset
    data = TIOCM_RTS;
    ioctl(fd, TIOCMBIC, &data); // Pull RTS low
    data = TIOCM_DTR;
    ioctl(fd, TIOCMBIS, &data); // Pull DTR high
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    data = TIOCM_RTS;
    ioctl(fd, TIOCMBIS, &data); // Pull RTS high
    data = TIOCM_DTR;
    ioctl(fd, TIOCMBIC, &data); // Pull DTR low
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Return DTR and RTS to high
    data = TIOCM_DTR | TIOCM_RTS;
    ioctl(fd, TIOCMBIS, &data);


    // Wait for ESP32 to boot
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // Clear any startup messages
    boost::asio::streambuf buffer;
    boost::system::error_code ec;

    // This is very brittle. 
    while (true && rclcpp::ok()) {
      boost::asio::read_until(*serial_port_, buffer, '\n', ec);
      if (!ec) {
        std::istream is(&buffer);
        std::string line;
        std::getline(is, line);
        if (line.find("Setup Complete") != std::string::npos) break;
      }
    }
    
    RCLCPP_INFO(this->get_logger(), "ESP32 reset complete");
}

void SerialManagerNode::serialReadThread()
{
  boost::asio::streambuf buffer;
  
  while (serial_thread_running_ && rclcpp::ok()) {
    try {
      boost::system::error_code ec;
      size_t bytes_transferred = boost::asio::read_until(
        *serial_port_, buffer, '\n', ec);
      
      if (!ec) {
        std::istream is(&buffer);
        std::string line;
        std::getline(is, line);
        
        processIncomingData(line);
        last_serial_msg_time_ = this->now();
      }
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Serial thread exception: %s", e.what());
    }
  }
}

void SerialManagerNode::processIncomingData(const std::string& data)
{
  // Expected format from ESP32: "RPS,STEERING_ANGLE"
  // Example: "2.5,0.15"
  
  try {
    std::stringstream ss(data);
    std::string token;
    // using a vector here is very inefficent. 
    // and using all of these strings.
    std::vector<float> values;
    
    while (std::getline(ss, token, ',')) {
      values.push_back(std::stof(token));
    }
    
    if (values.size() == 2) {
      // Publish wheel feedback data
      std_msgs::msg::Float32MultiArray msg;
      msg.data.push_back(values[0]);  // RPS (revolutions per second)
      msg.data.push_back(values[1]);  // Steering angle (radians)
      wheel_data_pub_->publish(msg);
    }
  }
  catch (const std::exception& e) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Failed to parse ESP32 data: %s", e.what());
  }
}

void SerialManagerNode::controlCommandCallback(
  const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
{
  // Send control command to ESP32
  sendCommandToESP32(msg->drive.acceleration, msg->drive.steering_angle);
}

void SerialManagerNode::sendCommandToESP32(double acceleration, double steering_angle)
{
  if (!connection_established_) {
    RCLCPP_WARN(this->get_logger(), "Cannot send command - no ESP32 connection");
    return;
  }
  
  // Format command: "A:acceleration,S:steering_angle\n"
  std::stringstream ss;
  ss << std::fixed << std::setprecision(3);
  ss << "A:" << acceleration << ",S:" << steering_angle << "\n";
  
  std::string command = ss.str();
  
  try {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    boost::asio::write(*serial_port_, boost::asio::buffer(command));
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to send command: %s", e.what());
  }
}

void SerialManagerNode::watchdogCallback()
{
  double time_since_last = (this->now() - last_serial_msg_time_).seconds();
  
  if (time_since_last > serial_timeout_sec_) {
    RCLCPP_ERROR(this->get_logger(), 
      "Serial timeout! No data for %.1f seconds", time_since_last);
    connection_established_ = false;
    
    // Try to reset
    if (time_since_last > serial_timeout_sec_ * 2) {
      RCLCPP_INFO(this->get_logger(), "Attempting reconnection...");
      resetESP32();
      last_serial_msg_time_ = this->now();
      connection_established_ = true;
    }
  }
}

}  // namespace ackermann_robot

// Main function
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  try {
    auto node = std::make_shared<ackermann_robot::SerialManagerNode>();
    rclcpp::spin(node);
  }
  catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("serial_manager"), 
      "Failed to start: %s", e.what());
    return 1;
  }
  
  rclcpp::shutdown();
  return 0;
}