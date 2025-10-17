
cmake_minimum_required(VERSION 3.8)
project(carbot_ackermann)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

# Generate custom message.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/OdometryData.msg"
  DEPENDENCIES std_msgs
)

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

######################################
# IMU Publisher node.
######################################
add_executable(imu_publisher src/imu_publisher_node.cpp)
target_link_libraries(imu_publisher mpu6050_lib)
ament_target_dependencies(imu_publisher
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)


######################################
# Serial Manager Node
######################################
add_executable(serial_manager_node 
  src/serial_manager_node.cpp
)

target_compile_features(serial_manager_node PUBLIC c_std_99 cxx_std_17)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(serial_manager_node
  ${Boost_LIBRARIES}
  ${JETGPIO_LIBRARY}
  "${cpp_typesupport_target}"
)

ament_target_dependencies(serial_manager_node
  rclcpp
  std_msgs
  ackermann_msgs
)


######################################
# Odometry Node
######################################

add_executable(odometry_node 
  src/odometry_node.cpp
)

target_link_libraries(odometry_node 
  "${cpp_typesupport_target}"
)

target_compile_features(odometry_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(odometry_node
  rclcpp
  std_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

######################################
# Control Node
######################################

add_executable(control_node 
  src/control_node.cpp
)

target_compile_features(control_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(control_node
  rclcpp
  geometry_msgs
  ackermann_msgs
)


# Install executables
install(TARGETS
  serial_manager_node
  odometry_node
  control_node
  imu_publisher
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