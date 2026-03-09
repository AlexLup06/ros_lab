# Create the lib/surros_lib directory to satisfy ROS 2 launch requirements
install(DIRECTORY DESTINATION lib/surros_lib)
cmake_minimum_required(VERSION 3.8)
project(surros_lib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(trajectory_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
#find_package(surros_hardware REQUIRED)
#find_package(surros_lib REQUIRED)
find_package(surros_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)

find_package(std_srvs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(controller_manager_msgs REQUIRED)
#find_package(surros_hardware REQUIRED)

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


# Build the surros_lib library from source files
add_library(surros_lib SHARED
  src/kinematics.cpp
  src/surros_interface.cpp
)

# Link against ROS 2 libraries
ament_target_dependencies(surros_lib
  rclcpp
  sensor_msgs
  trajectory_msgs
  rclcpp_action
  controller_manager_msgs
  geometry_msgs
  #surros_hardware
  surros_msgs
  tf2
  tf2_ros
  tf2_eigen
  visualization_msgs
  std_msgs
  control_msgs
  std_srvs
  interactive_markers
)

# Export include directories and libraries for downstream packages
ament_export_include_directories(include)
ament_export_libraries(surros_lib)

# Optionally, keep the robot_node executable if needed for testing/demo
add_executable(robot_node src/main.cpp)
ament_target_dependencies(robot_node rclcpp)
target_link_libraries(robot_node surros_lib)



install(TARGETS
  surros_lib
  robot_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

# Also install robot_node to libexec/surros_lib for ROS 2 launch compatibility
install(TARGETS robot_node
  RUNTIME DESTINATION libexec/surros_lib
)
install(TARGETS robot_node
  RUNTIME DESTINATION lib/surros_lib
)

# Install header files for downstream usage
install(DIRECTORY include/ DESTINATION include)

include_directories(
  include
)

  # Automatically set LD_LIBRARY_PATH for surros_lib when sourcing setup.bash
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/surros_lib_library_path.sh"
)

  # Export the library subfolder for downstream packages

ament_package()
