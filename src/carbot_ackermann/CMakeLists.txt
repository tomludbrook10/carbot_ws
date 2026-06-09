
cmake_minimum_required(VERSION 3.8)
project(carbot_ackermann)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)


# Generate custom message.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ImuData.msg"
  "msg/OdometryData.msg"
  "msg/ControlCommand.msg"
  "msg/Pose.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

######################################
# Create and Find Libraries
######################################
find_library(JETGPIO_LIBRARY 
  NAMES jetgpio
  HINTS /home/tom/JETGPIO
)

if(NOT JETGPIO_LIBRARY)
  message(FATAL_ERROR "jetgpio library not found")
endif()

message(STATUS "Found jetgpio: ${JETGPIO_LIBRARY}")

# Create MPU6050 library
add_library(mpu6050_lib src/MPU6050.cpp)
target_include_directories(mpu6050_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mpu6050_lib ${JETGPIO_LIBRARY})


# Include directories
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  /usr/local/include)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

######################################
# IMU Publisher node.
######################################

add_executable(imu_publisher src/imu_publisher_node.cpp)
target_link_libraries(imu_publisher 
    mpu6050_lib 
  "${cpp_typesupport_target}"
)
ament_target_dependencies(imu_publisher
  rclcpp
  std_msgs
)

######################################
# Serial Manager Node
######################################
add_executable(serial_manager_node 
  src/serial_manager_node.cpp
)

target_compile_features(serial_manager_node PUBLIC c_std_99 cxx_std_17)

target_link_libraries(serial_manager_node
  ${Boost_LIBRARIES}
  ${JETGPIO_LIBRARY}
  "${cpp_typesupport_target}"
)

ament_target_dependencies(serial_manager_node
  rclcpp
  std_msgs
)

######################################
# Ackermann EKF Node
######################################

add_executable(ackermann_ekf_node
  src/ackermann_ekf.cpp
)

target_link_libraries(ackermann_ekf_node
  Eigen3::Eigen
  "${cpp_typesupport_target}"
)
ament_target_dependencies(ackermann_ekf_node
  rclcpp
  std_msgs
)

# Install executables
install(TARGETS
  serial_manager_node
  imu_publisher
  ackermann_ekf_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
  mpu6050_lib
  LIBRARY  DESTINATION lib
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()