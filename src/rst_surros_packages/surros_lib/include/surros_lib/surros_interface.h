/**
 * \mainpage Surros Library Documentation
 *
 * # Surros Library
 *
 * Welcome to the documentation for **surros_lib** – a C++ library for controlling and simulating the surros robotic arm in ROS2.
 *
 * ---
 *
 * ## Features
 * - High-level API for joint, trajectory, and end-effector control
 * - ROS 2 integration with action clients, publishers, and interactive markers
 * - Kinematics and workspace visualization
 * - Collision checking and joint limit enforcement
 * - Modular design for research and education
 *
 * ---
 *
 * ## Getting Started
 * 1. **Build the package:**  
 *    Use `colcon build` in your ROS 2 workspace.
 * 2. **Include the library:**  
 *    Add `#include <surros_lib/surros_interface.h>` to your code.
 * 3. **Create a node and SurrosControl instance:**  
 *    ```cpp
 *    auto node = rclcpp::Node::make_shared("my_surros_node");
 *    robot::SurrosControl surros(node);
 *    surros.initialize();
 *    ```
 * 4. **Explore the API:**  
 *    - Set joint positions and velocities
 *    - Send trajectories
 *    - Visualize workspace and markers
 *
 * ---
 *
 * ## Documentation Structure
 * - **Classes:**  
 *   - `robot::SurrosControl` – Main interface for robot control
 *   - `KinematicModel` – Kinematics and workspace computation
 * - **Namespaces:**  
 *   - `robot` – All Surros functionality
 *
 * ---
 *
 * ## Resources
 * - [GitHub Repository](https://github.com/your-org/surros_lib)
 * - [ROS 2 Documentation](https://docs.ros.org/en/ros2/)
 * - [Doxygen-Awesome Theme](https://jothepro.github.io/doxygen-awesome-css/)
 *
 *
 * ---
 */

/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#ifndef SURROS_INTERFACE_H_
#define SURROS_INTERFACE_H_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <surros_lib/kinematics.h>
#include <surros_lib/misc.h>
#include <surros_lib/types.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <cmath>
#include <memory>
#include <mutex>
#include <numeric>
#include <unordered_map>
#include <map>
#include <string>
#include <vector>
#include <Eigen/Geometry>

#include <std_srvs/srv/empty.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <rclcpp/duration.hpp>
#include <surros_msgs/srv/relax.hpp>

namespace robot {

/**
 * @class SurrosControl
 * @brief Provides a high-level interface for controlling the Koch V1.1 robot via ROS 2.
 *
 * This class wraps ROS 2 action clients, publishers, and service clients to provide methods for
 * joint position, velocity, and trajectory control, as well as gripper and kinematic operations.
 * It does not directly communicate with hardware, but relies on the pxpincher_hardware package.
 * The API is designed for educational and research use, and is not intended to replace MoveIt.
 */
class SurrosControl
{
 public:
    /**
     * @brief Construct the SurrosControl interface.
     * @param node Shared pointer to the ROS 2 node.
     */
    SurrosControl(std::shared_ptr<rclcpp::Node> node);

    /**
     * @brief Destructor.
     */
    virtual ~SurrosControl();

    /**
     * @brief Initialize the interface, set up action clients, publishers, subscribers, and kinematics.
     *
     * This method must be called before using other methods. It waits for required action servers,
     * sets up joint names, limits, and initializes the kinematic model.
     */
    void initialize();

    /**
     * @brief Get the slowest maximum joint speed.
     * @return Minimum of all max joint speeds.
     */
    double getSlowestMaxSpeed() const;

    /**
     * @brief Set all joints to the default position q=[0,0,0,0,0]^T.
     * @param duration Duration for the transition.
     * @param blocking If true, waits until the goal is reached or timeout.
     */
    void setJointsDefault(const rclcpp::Duration& duration = rclcpp::Duration(5,0), bool blocking = true);

    /**
     * @brief Set all joints to the default position q=[0,0,0,0,0]^T.
     * @param speed Scalar speed for the transition.
     * @param blocking If true, waits until the goal is reached or timeout.
     */
    void setJointsDefault(double speed, bool blocking = true);

    /**
     * @brief Set all joints to the default position q=[0,0,0,0,0]^T.
     * @param speed Vector of individual joint speeds.
     * @param blocking If true, waits until the goal is reached or timeout.
     */
    void setJointsDefault(const Eigen::Ref<const JointVector>& speed, bool blocking);

    /**
     * @brief Set joints to the given values with a duration.
     * @param values Target joint values.
     * @param duration Duration for the transition.
     * @param relative If true, values are relative to current state.
     * @param blocking If true, waits until the goal is reached or timeout.
     */
    void setJoints(const Eigen::Ref<const JointVector>& values, const rclcpp::Duration& duration, bool relative = false, bool blocking = true);

    /**
     * @brief Set joints to the given values with a duration.
     * @param values Target joint values as std::vector.
     * @param duration Duration for the transition.
     * @param relative If true, values are relative to current state.
     * @param blocking If true, waits until the goal is reached or timeout.
     */
    void setJoints(const std::vector<double>& values, const rclcpp::Duration& duration, bool relative = false, bool blocking = true);

    /**
     * @brief Set joints to the given values with a scalar speed.
     * @param values Target joint values.
     * @param speed Scalar speed for the transition.
     * @param relative If true, values are relative to current state.
     * @param blocking If true, waits until the goal is reached or timeout.
     */
    void setJoints(const Eigen::Ref<const JointVector>& values, double speed, bool relative = false, bool blocking = true);

    /**
     * @brief Set joints to the given values with a scalar speed.
     * @param values Target joint values as std::vector.
     * @param speed Scalar speed for the transition.
     * @param relative If true, values are relative to current state.
     * @param blocking If true, waits until the goal is reached or timeout.
     */
    void setJoints(const std::vector<double>& values, double speed, bool relative = false, bool blocking = true);

    /**
     * @brief Set joints to the given values with individual speeds.
     * @param values Target joint values.
     * @param speed Vector of individual joint speeds.
     * @param relative If true, values are relative to current state.
     * @param blocking If true, waits until the goal is reached or timeout.
     */
    void setJoints(const Eigen::Ref<const JointVector>& values, const Eigen::Ref<const JointVector>& speed, bool relative = false, bool blocking = true);

    /**
     * @brief Set joints to the given values with individual speeds.
     * @param values Target joint values as std::vector.
     * @param speed Vector of individual joint speeds as std::vector.
     * @param relative If true, values are relative to current state.
     * @param blocking If true, waits until the goal is reached or timeout.
     */
    void setJoints(const std::vector<double>& values, const std::vector<double>& speed, bool relative = false, bool blocking = true);

    /**
     * @brief Set joints using a trajectory point.
     * @param joint_state JointTrajectoryPoint specifying positions and optionally velocities/duration.
     * @param relative If true, values are relative to current state.
     * @param blocking If true, waits until the goal is reached or timeout.
     */
    void setJoints(const trajectory_msgs::msg::JointTrajectoryPoint& joint_state, bool relative = false, bool blocking = true);

    /**
     * @brief Get current joint angles.
     * @param[out] values_out Joint angles written to Eigen vector.
     */
    void getJointAngles(Eigen::Ref<JointVector> values_out);

    /**
     * @brief Get current joint angles.
     * @param[out] values_out Joint angles written to std::vector.
     */
    void getJointAngles(std::vector<double>& values_out);

    /**
     * @brief Get current joint angles (copy version).
     * @return Joint angles as Eigen vector.
     */
    JointVector getJointAngles();

    /**
     * @brief Get joint names in the order used by JointVector.
     * @return Read-only reference to joint name vector.
     */
    const std::vector<std::string>& getJointNames() const { return _joint_names_arm; }

    /**
     * @brief Check if a joint vector exceeds joint limits.
     * @param joint_values Joint values to check.
     * @return True if any joint exceeds limits, false otherwise.
     */
    bool isExceedingJointLimits(const Eigen::Ref<const JointVector>& joint_values);

    /**
     * @brief Create a point-to-point joint trajectory with individual velocities.
     * @param start_conf Start joint configuration.
     * @param goal_conf Goal joint configuration.
     * @param speed Individual joint velocities.
     * @param[out] trajectory Output joint trajectory.
     */
    void createP2PTrajectoryWithIndividualVel(const Eigen::Ref<const JointVector>& start_conf, const Eigen::Ref<const JointVector>& goal_conf,
                                              const Eigen::Ref<const JointVector>& speed, trajectory_msgs::msg::JointTrajectory& trajectory);

    /**
     * @brief Create a point-to-point joint trajectory with individual velocities.
     * @param start_conf Start joint configuration as std::vector.
     * @param goal_conf Goal joint configuration as std::vector.
     * @param speed Individual joint velocities as std::vector.
     * @param[out] trajectory Output joint trajectory.
     */
    void createP2PTrajectoryWithIndividualVel(const std::vector<double>& start_conf, const std::vector<double>& goal_conf,
                                              const std::vector<double>& speed, trajectory_msgs::msg::JointTrajectory& trajectory);

    /**
     * @brief Send a joint trajectory to the robot.
     * @param trajectory JointTrajectory message containing positions and velocities.
     * @param blocking If true, waits until the goal is reached or timeout.
     */
    void setJointTrajectory(trajectory_msgs::msg::JointTrajectory& trajectory, bool blocking = true);

    /**
     * @brief Send a joint trajectory to the robot (FollowJointTrajectoryGoal format).
     * @param goal Trajectory goal message.
     * @param blocking If true, waits until the goal is reached or timeout.
     */
    void setJointTrajectory(control_msgs::action::FollowJointTrajectory_Goal& goal, bool blocking = true);

    /**
     * @brief Verify that a trajectory is feasible (joint limits, velocities, collision).
     * @param trajectory Trajectory to check and possibly modify.
     * @return True if feasible, false otherwise.
     */
    //bool verifyTrajectory(trajectory_msgs::msg::JointTrajectory& trajectory);

    /**
     * @brief Check if a joint configuration leads to self-collision.
     *
     * Only checks collision with the base and fourth joint. Collision check is disabled during velocity control.
     * @param joint_values Joint values to check.
     * @return True if self-collision detected, false otherwise.
     */
    bool checkSelfCollision(const Eigen::Ref<const JointVector>& joint_values);

    /**
     * @brief Print the position, velocity, and time profile of a trajectory.
     * @param trajectory Trajectory to print.
     */
    void printTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory);

    /**
     * @brief Get current joint velocities.
     * @param[out] velocities_out Joint velocities written to Eigen vector.
     */
    void getJointVelocities(Eigen::Ref<JointVector> velocities_out);

    /**
     * @brief Command robot by specifying joint velocities (non-blocking).
     *
     * The robot will hold the specified velocities until joint limits are exceeded or a new command is sent.
     * Some collision checks are disabled in this mode.
     * @param velocities Desired joint velocities.
     */
    void setJointVel(const Eigen::Ref<const JointVector>& velocities);

    /**
     * @brief Command robot by specifying joint velocities (non-blocking).
     * @param velocities Desired joint velocities as std::vector.
     */
    void setJointVel(const std::vector<double>& velocities);

    /**
     * @brief Get the transformation matrix to the gripper (TCP) in the robot base frame.
     * @param[out] base_T_gripper Output transformation as Eigen::Affine3d.
     */
    void getEndeffectorState(Eigen::Affine3d& base_T_gripper);

    /**
     * @brief Get the transformation matrix to the gripper (TCP) in the robot base frame.
     * @param[out] base_T_gripper Output transformation as geometry_msgs::TransformStamped.
     */
    void getEndeffectorState(geometry_msgs::msg::TransformStamped& base_T_gripper);

    /**
     * @brief Set the end effector to a given pose in the robot base frame.
     *
     * Uses inverse kinematics to compute joint angles for the desired pose, considering joint limits.
     * If relative is true, the pose is applied relative to the current TCP pose.
     * @param desired_pose Desired pose as Eigen::Affine3d.
     * @param duration Duration for the transition.
     * @param relative If true, pose is relative to current TCP.
     * @param blocking If true, waits until the goal is reached or timeout.
     */
    void setEndeffectorPose(const Eigen::Affine3d& desired_pose, const rclcpp::Duration& duration = rclcpp::Duration(5,0), bool relative = false,
                            bool blocking = true);

    /**
     * @brief Set the end effector to a given pose in the robot base frame.
     * @param desired_pose Desired pose as Eigen::Affine3d.
     * @param speed Speed for the transition.
     * @param relative If true, pose is relative to current TCP.
     * @param blocking If true, waits until the goal is reached or timeout.
     */
    void setEndeffectorPose(const Eigen::Affine3d& desired_pose, double speed, bool relative = false, bool blocking = true);

    /**
     * @brief Set the end effector to a given 5D pose (position + pitch/yaw) in the robot base frame.
     * @param desired_xyz Desired [x,y,z] coordinates.
     * @param psi Desired pitch angle.
     * @param phi Desired yaw angle.
     * @param duration Duration for the transition.
     * @param relative If true, pose is relative to current TCP.
     * @param blocking If true, waits until the goal is reached or timeout.
     * @return True if a solution was found, false otherwise.
     */
    bool setEndeffectorPose(const Eigen::Ref<const Eigen::Vector3d>& desired_xyz, double psi, double phi,
                            const rclcpp::Duration& duration = rclcpp::Duration(5,0), bool relative = false, bool blocking = true);

    /**
     * @brief Set the end effector to a given 5D pose (position + pitch/yaw) in the robot base frame.
     * @param desired_xyz Desired [x,y,z] coordinates.
     * @param psi Desired pitch angle.
     * @param phi Desired yaw angle.
     * @param speed Speed for the transition.
     * @param relative If true, pose is relative to current TCP.
     * @param blocking If true, waits until the goal is reached or timeout.
     * @return True if a solution was found, false otherwise.
     */
    bool setEndeffectorPose(const Eigen::Ref<const Eigen::Vector3d>& desired_xyz, double psi, double phi, double speed, bool relative = false,
                            bool blocking = true);

    /**
     * @brief Set the end effector to a given 5D pose (position + pitch/yaw) in the robot base frame.
     * @param desired_xyz Desired [x,y,z] coordinates as std::vector.
     * @param psi Desired pitch angle.
     * @param phi Desired yaw angle.
     * @param speed Speed for the transition.
     * @param relative If true, pose is relative to current TCP.
     * @param blocking If true, waits until the goal is reached or timeout.
     * @return True if a solution was found, false otherwise.
     */
    bool setEndeffectorPose(const std::vector<double>& desired_xyz, double psi, double phi, double speed, bool relative = false, bool blocking = true);

    /**
     * @brief Incrementally change the position of the end effector in world coordinates.
     * @param dx Increment of x coordinate.
     * @param dy Increment of y coordinate.
     * @param dz Increment of z coordinate.
     * @param speed Speed for the transition.
     * @param blocking If true, waits until the goal is reached or timeout.
     * @return True if a solution was found, false otherwise.
     */
    bool setEndeffectorPoseInc(double dx, double dy, double dz, double speed, bool blocking = true)
    {
        return setEndeffectorPose(Eigen::Vector3d(dx, dy, dz), 0, speed, true, blocking);
    }

    /**
     * @brief Compute the geometric Jacobian of the end effector w.r.t. the base frame.
     *
     * The Jacobian relates joint velocities to end effector velocity: v = J*qdot.
     * The resulting matrix is 6x5 for a 5-DOF robot.
     * @param[out] jacobian Output Jacobian matrix.
     */
    void getJacobian(RobotJacobian& jacobian);

    /**
     * @brief Compute reduced robot Jacobian (x, y, z, pitch, yaw) w.r.t. the base frame.
     *
     * The reduced Jacobian is 5x5, corresponding to the position and two orientation components.
     * @param[out] jacobian Output reduced Jacobian matrix.
     */
    void getJacobianReduced(RobotJacobianReduced& jacobian);

    /**
     * @brief Stop any running transition (trajectory or velocity).
     */
    void stopMoving();

    /**
     * @brief Relax all servos (deactivate torque).
     * @param relaxed If true, relaxes servos; otherwise, enables torque.
     * @return True if successful, false otherwise.
     */
    bool relaxServos(bool relaxed);

    /**
     * @brief Open or close the gripper by percentage.
     * @param percent_open Desired separation in percent [0..100] (0 = closed).
     * @param blocking If true, waits until the action is completed or timeout.
     */
    void setGripperJoint(int percent_open, bool blocking = true);

    /**
     * @brief Set the gripper joint angle directly.
     * @param joint_value Actual joint angle of the servo.
     * @param blocking If true, waits until the action is completed or timeout.
     */
    void setGripperRawJointAngle(double joint_value, bool blocking = true);

    /**
     * @brief Return the current joint angle of the gripper.
     * @return Current joint angle.
     */
    double getGripperRawJointAngle();

    /**
     * @brief Return the current joint angle of the gripper in percentage.
     * @return Current joint angle [%].
     */
    int getGripperJointPercentage();

    //! Access lower joint limits (read-only)
    const JointVector& getLowerJointLimits() const { return _joint_lower_bounds; }
    //! Access upper joint limits (read-only)
    const JointVector& getUpperJointLimits() const { return _joint_upper_bounds; }
    //! Access maximum (absolute) joint velocities (read-only)
    const JointVector& getMaxJointSpeeds() const { return _joint_max_speeds; }

    /**
     * @brief Get the logger for this node.
     * @return rclcpp::Logger object.
     */
    rclcpp::Logger get_logger() const { return node_->get_logger(); }

protected:

    enum class ArmControlMode { TRAJECTORY_FOLLOWING, SPEED_FORWARDING };

    /**
     * @brief Switch arm control mode (trajectory following or speed forwarding).
     * @param mode Desired control mode.
     * @return True if successful, false otherwise.
     */
    bool switchArmControlMode(ArmControlMode mode);

    /**
     * @brief Check and adapt trajectory to satisfy joint restrictions.
     * @param trajectory Trajectory to check and modify.
     * @return True if trajectory is feasible, false otherwise.
     */
    bool verifyTrajectory(trajectory_msgs::msg::JointTrajectory& trajectory);

 private:

    /**
     * @brief Callback for joint state messages.
     * @param msg JointState message.
     */
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    // ROS 2 action clients
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr _arm_action;
    rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr _gripper_action;

    KinematicModel _kinematics;

    // ROS 2 subscriber for joint states
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _joints_sub;
    bool _joint_values_received = false;

    // TF2 buffer and listener
    std::shared_ptr<tf2_ros::Buffer> _tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> _tf_listener;
    // ROS 2 publisher for markers
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _marker_pub;

    // ROS 2 service clients
    rclcpp::Client<surros_msgs::srv::Relax>::SharedPtr _joint_relax_service;
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr _arm_control_mode_service;
    ArmControlMode _arm_control_mode = ArmControlMode::TRAJECTORY_FOLLOWING;

    // ROS 2 publisher for arm speed forwarding
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr _arm_speed_forwarding_pub;


    Eigen::Affine3d _base_T_j1 = Eigen::Affine3d::Identity();  //!< Coordinate transformation from the base to the first joint
    Eigen::Affine3d _j1_T_base = Eigen::Affine3d::Identity();  //!< Coordinate transformation from the first joint to the base

    std::mutex _joints_mutex;
    JointVector _joint_angles     = JointVector::Zero();
    JointVector _joint_velocities = JointVector::Zero();
    double _gripper_value         = 0;

    JointVector _joint_lower_bounds;
    JointVector _joint_upper_bounds;
    JointVector _joint_max_speeds;

    double _gripper_lower_bound = 0;
    double _gripper_upper_bound = 0;
    //   double _gripper_neutral = 0;
    //   double _gripper_max_speed = 0;
    std::string _gripper_joint_name;

    std::string _arm_base_link_frame = "arm_base_link";  // TODO ros param

    std::map<std::string, int> _map_joint_to_index;
    std::map<int, std::string> _map_joint_to_joint_frame;

    std::vector<std::string> _joint_names_arm;  //!< Store names for all joints of the arm

    bool _collision_check_enabled = true;  //! Workaround, this variable is set to false in case of velocity control

    bool _initialized = false;

    std::unique_ptr<interactive_markers::InteractiveMarkerServer> _marker_server;
    std::unordered_map<std::string, int>
        _marker_to_joint;  // store mapping from marker name to joint number // TODO: maybe merge with _map_joint_to_index
    rclcpp::TimerBase::SharedPtr _marker_pose_updater;

    std::shared_ptr<rclcpp::Node> node_;

 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void computeJacobianFromRviz(const Eigen::Ref<const JointVector>& joint_values, RobotJacobianReduced& jacobian4d) const;
};

}  // namespace robot

#endif /* SURROS_INTERFACE_H_ */
