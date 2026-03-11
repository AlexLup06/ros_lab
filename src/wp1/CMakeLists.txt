cmake_minimum_required(VERSION 3.8)
project(workpackage_1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# --- frames_tf executable ---
add_executable(frames_tf src/frames_tf.cpp)
ament_target_dependencies(frames_tf
  rclcpp
  tf2_ros
  geometry_msgs
  tf2_eigen
)

target_include_directories(frames_tf PUBLIC
  ${EIGEN3_INCLUDE_DIR}
)

# --- frames_eigen executable ---
add_executable(frames_eigen src/frames_eigen.cpp)
ament_target_dependencies(frames_eigen
  rclcpp
  visualization_msgs
  geometry_msgs
  tf2_eigen
)

target_include_directories(frames_eigen PUBLIC
  ${EIGEN3_INCLUDE_DIR}
)

# Install executables
install(TARGETS frames_tf frames_eigen
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
