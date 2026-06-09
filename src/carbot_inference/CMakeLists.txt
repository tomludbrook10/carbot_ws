cmake_minimum_required(VERSION 3.8)
project(carbot_inference)

list(APPEND CMAKE_PREFIX_PATH "$ENV{HOME}/.local")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(CarbotInference REQUIRED carbot_inference)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(carbot_ackermann REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Waypoints.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

include_directories(include)

### Waypoint Sender Executable
add_executable(waypoint_sender src/waypoint_sender.cpp)
ament_target_dependencies(waypoint_sender
  rclcpp
  carbot_ackermann
  std_msgs
)

target_link_libraries(waypoint_sender
  CarbotInference::carbot_inference
  "${cpp_typesupport_target}"
)

## setting the RPATH for runtime linking to find shared libraries in local paths
target_link_options(waypoint_sender PRIVATE -Wl,--enable-new-dtags)
set_target_properties(waypoint_sender PROPERTIES
  BUILD_RPATH "$ENV{HOME}/.local/lib"
  INSTALL_RPATH "$ENV{HOME}/.local/lib"
)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

### PP controller. 
add_executable(pure_pursuit_controller src/pure_pursuit_controller.cpp)
ament_target_dependencies(pure_pursuit_controller
  rclcpp
  carbot_ackermann
  std_msgs
)

target_link_libraries(pure_pursuit_controller
  "${cpp_typesupport_target}"
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  waypoint_sender
  pure_pursuit_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)



ament_package()
