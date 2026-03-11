cmake_minimum_required(VERSION 3.8)
project(workpackage_3)

# Ensure local surros_lib package config is found
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/../2025-pa-tekempel-gravemeier/surros_lib/../../install/surros_lib/share/surros_lib/cmake")
#list(APPEND CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/../2025-pa-tekempel-gravemeier/surros_hardware/../../install/surros_hardware/share/surros_hardware/cmake")
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/../2025-pa-tekempel-gravemeier/surros_msgs/../../install/surros_msgs/share/surros_msgs/cmake")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# # Add surros_lib as a subdirectory dependency
# add_subdirectory(
#     ../2025-pa-tekempel-gravemeier/surros_lib
#     ${CMAKE_BINARY_DIR}/surros_lib_build
# )

# add_subdirectory(
#     ../2025-pa-tekempel-gravemeier/surros_hardware
#     ${CMAKE_BINARY_DIR}/surros_hardware_build
# )

list(APPEND CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/../../install")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(surros_lib REQUIRED)
#find_package(surros_hardware REQUIRED)
find_package(surros_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)

find_package(std_srvs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(controller_manager_msgs REQUIRED)

find_package(Sophus REQUIRED)
find_package(fmt REQUIRED)

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


add_executable(robot_control
  src/robot_control.cpp
)

# Add surros_lib include directory
include_directories("${CMAKE_SOURCE_DIR}/../2025-pa-tekempel-gravemeier/surros_lib/include")

# Add surros_hardware include directory
#include_directories("${CMAKE_SOURCE_DIR}/../2025-pa-tekempel-gravemeier/surros_hardware/include")

# Add surros_msgs include directory
include_directories("${CMAKE_SOURCE_DIR}/../2025-pa-tekempel-gravemeier/surros_msgs/include")

# Add Eigen3 include directory
include_directories("/usr/include/eigen3")
include_directories("/opt/ros/humble/include")

# # Link against surros_lib
# target_link_libraries(workpackage_3_node
#     PRIVATE surros_lib
#     PRIVATE surros_hardware
# )
target_link_libraries(robot_control
  ${surros_lib_LIBRARIES}
  fmt::fmt
  #${surros_hardware_LIBRARIES}
)

# Link against ROS 2 libraries
ament_target_dependencies(robot_control
  rclcpp
  surros_lib
  #surros_hardware
  surros_msgs
  sensor_msgs
  nav_msgs
  Eigen3
  tf2_eigen
  tf2_ros
  visualization_msgs
  std_msgs
  control_msgs
  std_srvs
  interactive_markers
  controller_manager_msgs
  Sophus
  #fmt
)


# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)


# Create the lib/workpackage_3 directory to satisfy ROS 2 launch requirements
install(DIRECTORY DESTINATION lib/workpackage_3)



# Install the executable to lib/workpackage_3 for ROS 2 launch compatibility
install(TARGETS robot_control
  RUNTIME DESTINATION lib/workpackage_3
)

ament_package()
