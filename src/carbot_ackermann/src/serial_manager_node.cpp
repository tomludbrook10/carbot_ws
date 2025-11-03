// src/serial_manager_node.cpp
#include "carbot_ackermann/serial_manager_node.hpp"
#include <sstream>
#include "carbot_ackermann/msg/odometry_data.hpp"
#include "carbot_ackermann/msg/control_command.hpp"
#include <iomanip>
#include <jetgpio.h>
#include <thread>
#include <chrono>

namespace ackermann_robot
{

SerialManagerNode::SerialManagerNode(const rclcpp::NodeOptions & options)
: Node("serial_manager_node", options),
  serial_thread_running_(false),
  connection_established_(false) {
  // Declare parameters
  this->declare_parameter("serial_port", "/dev/ttyACM0");
  this->declare_parameter("baud_rate", 115200);
  this->declare_parameter("wheel_circumference", 0.3078);
  
  // Get parameters
  serial_port_name_ = this->get_parameter("serial_port").as_string();
  baud_rate_ = this->get_parameter("baud_rate").as_int();
  wheel_circumference_ = this->get_parameter("wheel_circumference").as_double();

    // Create publisher for wheel data
  wheel_data_pub_ = this->create_publisher<carbot_ackermann::msg::OdometryData>(
    "wheel_odometry", 10);
  
  // Create subscriber for control commands
  control_cmd_sub_ = this->create_subscription<carbot_ackermann::msg::ControlCommand>(
    "control_cmd", 10,
    std::bind(&SerialManagerNode::controlCommandCallback, this, std::placeholders::_1));

  // setting up the gpio.
  setupGPIO();

  // Initialize serial
  if (!initializeSerial()) {
    RCLCPP_FATAL(this->get_logger(), "Failed to initialize serial communication with ESP32");
    throw std::runtime_error("Serial initialization failed - cannot start robot system");
  }

  // sync the esp32 clock and ros2 time every 30 seconds.
  timer_ = this->create_wall_timer(
                std::chrono::seconds(30),
                std::bind(&SerialManagerNode::send_pulse, this));

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

  if (gpio_initialized_) {
    gpioWrite(SYNC_GPIO, 0);
    gpioTerminate();
    RCLCPP_INFO(this->get_logger(), "GPIO terminated");
  }
}

void SerialManagerNode::setupGPIO() {
  if (gpioInitialise() < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize jetgpio");
    throw std::runtime_error("jetgpio initialization failed");
  }

  gpio_initialized_ = true;
  
  // Setup GPIO pin
  gpioSetMode(SYNC_GPIO, JET_OUTPUT);
  gpioWrite(SYNC_GPIO, 0);  // Start low
  
  RCLCPP_INFO(this->get_logger(), "GPIO initialized on pin %d", SYNC_GPIO);
}

uint64_t SerialManagerNode::getCurrentTime() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

void SerialManagerNode::send_pulse() {
  // Get ROS time just before pulse
  auto pre_pulse_time = getCurrentTime();
  
  // Send hardware pulse with minimal latency
  gpioWrite(SYNC_GPIO, 1);
  std::this_thread::sleep_for(std::chrono::microseconds(PULSE_WIDTH_US)); // Microsecond delay
  gpioWrite(SYNC_GPIO, 0);
  
  // Get ROS time just after pulse
  auto post_pulse_time = getCurrentTime();

  auto interpolated_time = (pre_pulse_time + post_pulse_time) / 2;
  // Calculate best estimate of pulse time
  reference_time_.store(interpolated_time, std::memory_order_relaxed);

  RCLCPP_INFO(this->get_logger(), 
              "Sync pulse sent at linux monotonic time : %ld", interpolated_time);
  failed_sync_count_ = 0;
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

    // Sync clock
    syncESP32Clock();

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
    
    RCLCPP_INFO(this->get_logger(), "ESP32 reset complete, now syncing clocks");
}

void SerialManagerNode::syncESP32Clock() {
  send_pulse();
}

void SerialManagerNode::serialReadThread()
{
  boost::asio::streambuf buffer;
  std::string line; line.reserve(100);
  std::string token; token.reserve(10);

  while (serial_thread_running_ && rclcpp::ok()) {
    try {
      boost::system::error_code ec;
      boost::asio::read_until(*serial_port_, buffer, '\n', ec);
      
      if (!ec) {
        std::istream is(&buffer);
        std::getline(is, line);

        processIncomingData(line, token);
      }
      token.clear();
      line.clear();
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Serial thread exception: %s", e.what());
    }
  }
}

void SerialManagerNode::processLogMessage(const std::string& log_msg)
{
  RCLCPP_WARN(this->get_logger(), "ESP32 Log: %s", log_msg.c_str());
}

void SerialManagerNode::processIncomingData(const std::string& data, std::string& token)
{
  // Expected format from ESP32: "(),TimeStamp,RPS,STEERING_ANGLE"
  // () can be == D for data or L for log message.
  
  try {
    std::stringstream ss(data);
    // using a vector here is very inefficent. 
    // and using all of these strings.

    char data_type;
    int64_t offset_time; 
    float rps;

    int count = 0;
    while (std::getline(ss, token, ',')) {
      if (count == 0) {
        data_type = token[0];
        if (data_type == 'L') {
          // log message, the rest is the log.
          break;
        }
        count++;
      } else if(count == 1) {
        offset_time = std::stoull(token);
        count++;
      } else if (count == 2) {
        rps = std::stof(token);
        count++;
      }
    }

    if (data_type != 'D' && data_type != 'L') {
      RCLCPP_WARN(this->get_logger(), "Unknown data type received: %c, message: %s", data_type, data.c_str());
      return;
    }

    if (data_type == 'L') {
      processLogMessage(data.substr(2)); // Skip "L,"
      return;
    }

    if (count != 3 || std::isnan(rps)) {
      RCLCPP_WARN(this->get_logger(), "Invalid data format received: %s", data.c_str());
      return;
    }
    
    // Publish wheel feedback data
    carbot_ackermann::msg::OdometryData msg;
    auto current_reference_time = reference_time_.load(std::memory_order_relaxed);
    int64_t time_stamp_in_namo = current_reference_time + offset_time * 1000;

    // the data should always be recorded in the past. 
    // in addtion this doesn't allow any packets for the esp32 have the wrong timestamp caused.
    // by a delay when re-sync the clocks and consequently having a time_stamp in the future.
    int64_t now_nano = getCurrentTime();
    if (time_stamp_in_namo < now_nano) {
        msg.monotonic_timestamp_ns = time_stamp_in_namo; // convert the offset time to nano-seconds.
        msg.linear_velocity = rps * wheel_circumference_; // convert rps to linear velocity in m/s
        wheel_data_pub_->publish(msg);
    } else {
      RCLCPP_INFO(this->get_logger(), "Time stamp was in the future, ignoring. received: %ld, now: %ld", time_stamp_in_namo, now_nano);
      failed_sync_count_++;
      if (failed_sync_count_ >= MAX_FAILED_SYNC) {
        RCLCPP_WARN(this->get_logger(), "Multiple failed syncs detected, re-syncing clocks.");
        syncESP32Clock();
      }
    }
  }
  catch (const std::exception& e) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Failed to parse ESP32 data: %s", e.what());
  }
}

void SerialManagerNode::controlCommandCallback(
  const carbot_ackermann::msg::ControlCommand::SharedPtr msg)
{
  // Send control command to ESP32
  sendCommandToESP32(msg->linear_velocity, msg->steering_angle);
}

void SerialManagerNode::sendCommandToESP32(float speed, float steering_angle)
{
  /// takes speed in m/s and steering angle in radius and makes a converstion to robots rsp and steerrign angle in degrees.

  if (!connection_established_) {
    RCLCPP_WARN(this->get_logger(), "Cannot send command - no ESP32 connection");
    return;
  }
  
  float rps = speed / wheel_circumference_;
  int steering_angle_output = static_cast<int>(-steering_angle * (180.0 / M_PI));
  
  // Format command: "A:acceleration,S:steering_angle\n"
  std::stringstream ss;
  ss << std::fixed << std::setprecision(2);
  ss << "<" << rps << "," << steering_angle_output << ">" << "\n";
  
  std::string command = ss.str();
  
  // note that the only place that uses the serial port is here so we don't need the lock.
  try {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    boost::asio::write(*serial_port_, boost::asio::buffer(command));
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to send command: %s", e.what());
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