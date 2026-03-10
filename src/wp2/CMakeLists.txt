
cmake_minimum_required(VERSION 3.8)
project(workpackage_2)

# Ensure local surros_lib package config is found
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/../2025-pa-tekempel-gravemeier/surros_lib/../../install/surros_lib/share/surros_lib/cmake")
#list(APPEND CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/../2025-pa-tekempel-gravemeier/surros_hardware/../../install/surros_hardware/share/surros_hardware/cmake")

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
find_package(Eigen3 REQUIRED)

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


add_executable(exercise1
  src/exercise1.cpp
)

add_executable(exercise2
  src/exercise2.cpp
)

add_executable(exercise3
  src/exercise3.cpp
)

# Add surros_lib include directory
include_directories("${CMAKE_SOURCE_DIR}/../2025-pa-tekempel-gravemeier/surros_lib/include")

# Add surros_hardware include directory
#include_directories("${CMAKE_SOURCE_DIR}/../2025-pa-tekempel-gravemeier/surros_hardware/include")

# Add Eigen3 include directory
include_directories("/usr/include/eigen3")

# # Link against surros_lib
# target_link_libraries(workpackage_2_node
#     PRIVATE surros_lib
#     PRIVATE surros_hardware
# )
target_link_libraries(exercise1
  ${surros_lib_LIBRARIES}
  #${surros_hardware_LIBRARIES}
)

target_link_libraries(exercise2
  ${surros_lib_LIBRARIES}
  #${surros_hardware_LIBRARIES}
)

target_link_libraries(exercise3
  ${surros_lib_LIBRARIES}
  #${surros_hardware_LIBRARIES}
)

# Link against ROS 2 libraries
ament_target_dependencies(exercise1
  rclcpp
  surros_lib
  Eigen3
)

ament_target_dependencies(exercise2
  rclcpp
  surros_lib
  Eigen3
)

ament_target_dependencies(exercise3
  rclcpp
  surros_lib
  Eigen3
)


# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)


# Create the lib/workpackage_2 directory to satisfy ROS 2 launch requirements
install(DIRECTORY DESTINATION lib/workpackage_2)



# Install the executable to lib/workpackage_2 for ROS 2 launch compatibility
install(TARGETS exercise1 exercise2 exercise3
  RUNTIME DESTINATION lib/workpackage_2
)

ament_package()
