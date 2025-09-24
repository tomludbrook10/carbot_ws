
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
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/OdometryData.msg"
  DEPENDENCIES std_msgs
)

# Find Boost for serial communication
find_package(Boost REQUIRED COMPONENTS system thread)

# DON'T use find_package(pigpio) - use find_library instead:
find_library(PIGPIO_LIBRARY 
  NAMES pigpio
  HINTS /usr/local/lib
)

if(NOT PIGPIO_LIBRARY)
  message(FATAL_ERROR "pigpio library not found")
endif()

message(STATUS "Found pigpio: ${PIGPIO_LIBRARY}")

# Find threads (required by pigpio)
find_package(Threads REQUIRED)

# Include directories
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  /usr/local/include
  )

# Serial Manager Node
add_executable(serial_manager_node 
  src/serial_manager_node.cpp
)

target_compile_features(serial_manager_node PUBLIC c_std_99 cxx_std_17)


rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(serial_manager_node
  ${Boost_LIBRARIES}
  ${PIGPIO_LIBRARY}
  "${cpp_typesupport_target}"
)


ament_target_dependencies(serial_manager_node
  rclcpp
  std_msgs
  ackermann_msgs
)

# Odometry Node
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

# Control Node
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
  DESTINATION lib/${PROJECT_NAME}
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