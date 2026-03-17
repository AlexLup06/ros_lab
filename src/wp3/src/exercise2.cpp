#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <Eigen/Dense>
#include <chrono>
#include <thread>
#include <vector>

#include <surros_lib/misc.h>
#include <surros_lib/surros_interface.h>

using Vector5d = Eigen::Matrix<double, 5, 1>;
using Matrix5d = Eigen::Matrix<double, 5, 5>;

static Vector5d solveDampedLeastSquares(const Matrix5d &jacobian,
                                        const Vector5d &task_command,
                                        double damping)
{
    // Damped least squares solution: qdot = J^T * (J * J^T + lambda^2 I)^(-1) * task_command
    const double lambda_sq = damping * damping;
    const Matrix5d damped_system =
        jacobian * jacobian.transpose() + lambda_sq * Matrix5d::Identity();
    const Vector5d workspace_solution = damped_system.ldlt().solve(task_command);
    return jacobian.transpose() * workspace_solution;
}

static Vector5d clampJointVel(const Vector5d &vel, const robot::JointVector &max_speeds)
{
    // Clip joint velocities to safe limits
    Vector5d out = vel;
    for (int i = 0; i < out.size(); ++i)
    {
        const double limit = std::abs(max_speeds(i));
        if (limit > 0.0)
        {
            if (out(i) > limit)
                out(i) = limit;
            if (out(i) < -limit)
                out(i) = -limit;
        }
    }
    return out;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("wp3_exercise2");

    const std::string trajectory_frame_id =
        node->declare_parameter<std::string>("trajectory_frame_id", "base_link");

    // Robot interface
    robot::SurrosControl robot(node);
    robot.initialize();

    // Publish control error for rqt_plot
    auto error_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("/wp3/ex2/control_error", 10);
    const auto path_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    auto desired_marker_pub = node->create_publisher<visualization_msgs::msg::Marker>("/wp3/ex2/desired_trajectory_marker", path_qos);
    auto actual_marker_pub = node->create_publisher<visualization_msgs::msg::Marker>("/wp3/ex2/actual_trajectory_marker", path_qos);

    nav_msgs::msg::Path desired_path_msg;
    desired_path_msg.header.frame_id = trajectory_frame_id;
    nav_msgs::msg::Path actual_path_msg;
    actual_path_msg.header.frame_id = trajectory_frame_id;

    visualization_msgs::msg::Marker desired_marker_msg;
    desired_marker_msg.header.frame_id = trajectory_frame_id;
    desired_marker_msg.ns = "wp3_ex2";
    desired_marker_msg.id = 0;
    desired_marker_msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
    desired_marker_msg.action = visualization_msgs::msg::Marker::ADD;
    desired_marker_msg.scale.x = 0.002;
    desired_marker_msg.scale.y = 0.002;
    desired_marker_msg.color.r = 0.10f;
    desired_marker_msg.color.g = 0.75f;
    desired_marker_msg.color.b = 0.20f;
    desired_marker_msg.color.a = 1.0f;
    desired_marker_msg.pose.orientation.w = 1.0;

    visualization_msgs::msg::Marker actual_marker_msg;
    actual_marker_msg.header.frame_id = trajectory_frame_id;
    actual_marker_msg.ns = "wp3_ex2";
    actual_marker_msg.id = 1;
    actual_marker_msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
    actual_marker_msg.action = visualization_msgs::msg::Marker::ADD;
    actual_marker_msg.scale.x = 0.002;
    actual_marker_msg.scale.y = 0.002;
    actual_marker_msg.color.r = 0.95f;
    actual_marker_msg.color.g = 0.20f;
    actual_marker_msg.color.b = 0.15f;
    actual_marker_msg.color.a = 1.0f;
    actual_marker_msg.pose.orientation.w = 1.0;

    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // Read current TCP pose
    Eigen::Affine3d start_pose = Eigen::Affine3d::Identity();
    robot.getEndeffectorState(start_pose);

    // Initial position and orientation
    Eigen::Vector3d p0 = start_pose.translation();
    const Eigen::Matrix3d R0 = start_pose.linear();
    Eigen::Quaterniond q0(R0);

    // Define the house positions
    const std::vector<double> origin_xyz = {0.0, 0.13, 0.03};
    const double house_scale = 0.04;
    const double z_level = origin_xyz[2];

    // Target orientation in base_link parameterized by psi/phi.
    const double target_psi = node->declare_parameter<double>("target_psi", 1.57);
    const double target_phi = node->declare_parameter<double>("target_phi", 0.0);

    const Eigen::Vector3d base_ref(origin_xyz[0], origin_xyz[1], z_level);
    const Eigen::Affine3d T_base_ref = robot::createPoseFromPosAndPitch(base_ref, target_psi, target_phi);

    // Define the 2D house shape (square with a roof) in local coordinates
    const std::vector<Eigen::Vector2d> house_2d = {
        {0.0, 0.0},
        {1.0, 0.0},
        {1.0, 1.0},
        {0.0, 1.0},
        {0.0, 0.0},
        {1.0, 1.0},
        {0.5, 1.5},
        {0.0, 1.0},
        {1.0, 0.0},
    };

    // Transform the 2D house points into 3D world coordinates
    std::vector<Eigen::Vector3d> waypoints;
    waypoints.reserve(house_2d.size());
    for (const auto& p : house_2d)
    {
        // Scale and transform to world coordinates
        const Eigen::Vector3d p_local(house_scale * p.x(), house_scale * p.y(), 0.0);
        const Eigen::Vector3d p_world = T_base_ref * p_local;
        waypoints.emplace_back(p_world.x(), p_world.y(), p_world.z());
    }

    // Trajectory and control parameters
    const double total_time = node->declare_parameter<double>("total_time", 8.0);
    const double settle_time = node->declare_parameter<double>("settle_time", 0.0);
    const double dt = node->declare_parameter<double>("dt", 0.04);
    const double kp_pos = node->declare_parameter<double>("kp_pos", 2.0);
    const double kp_ori = node->declare_parameter<double>("kp_ori", 0.05);
    const double damping = node->declare_parameter<double>("damping", 0.08);
    const bool use_feedforward = node->declare_parameter<bool>("use_feedforward", false);
    const double max_joint_speed_scale = node->declare_parameter<double>("max_joint_speed_scale", 0.25);

    // Number of control steps for the trajectory
    const int steps = static_cast<int>(total_time / dt);

    // Additional steps to allow settling at the target pose after trajectory completion
    const int settle_steps = static_cast<int>(settle_time / dt);

    // Simple proportional gains
    Matrix5d K = Matrix5d::Zero();
    K(0, 0) = kp_pos;
    K(1, 1) = kp_pos;
    K(2, 2) = kp_pos;
    K(3, 3) = kp_ori;
    K(4, 4) = kp_ori;

    // Precompute scaled max joint speeds for velocity clamping
    rclcpp::Rate rate(1.0 / dt);
    const robot::JointVector scaled_max_speeds = max_joint_speed_scale * robot.getMaxJointSpeeds();

    // Loop over each waypoint
    for (size_t idx = 0; idx < waypoints.size(); ++idx)
    {
        const Eigen::Vector3d p1 = waypoints[idx];
        const Eigen::Affine3d target_pose = robot::createPoseFromPosAndPitch(p1, target_psi, target_phi);
        const Eigen::Quaterniond q1(target_pose.linear());

        // Trajectory tracking loop
        for (int i = 0; i <= steps + settle_steps && rclcpp::ok(); ++i)
        {
            // Compute trajectory parameter s in [0,1]
            const int traj_idx = std::min(i, steps);
            const double s = static_cast<double>(traj_idx) / static_cast<double>(steps);

            // Desired end-effector pose on the trajectory
            const Eigen::Vector3d p_des = p0 + s * (p1 - p0);
            const Eigen::Quaterniond q_des = q0.slerp(s, q1);
            const Eigen::Matrix3d R_des = q_des.toRotationMatrix();

            // Desired end-effector velocity (feedforward term)
            Vector5d xd_dot = Vector5d::Zero();
            if (use_feedforward && i < steps)
            {
                // Compute feedforward term based on finite difference of desired trajectory
                const double s_next = static_cast<double>(traj_idx + 1) / static_cast<double>(steps);
                const Eigen::Vector3d p_des_next = p0 + s_next * (p1 - p0);
                const Eigen::Matrix3d R_des_next = q0.slerp(s_next, q1).toRotationMatrix();
                const Eigen::Vector3d ori_ff = robot::computeOrientationError(R_des, R_des_next) / dt;

                // Feedforward command: desired velocity to stay on the trajectory
                xd_dot << (p_des_next - p_des) / dt,
                    ori_ff(1),
                    ori_ff(2);
            }

            // Read current end-effector pose
            Eigen::Affine3d current_pose = Eigen::Affine3d::Identity();
            robot.getEndeffectorState(current_pose);

            // Compute error in position and orientation
            const Eigen::Vector3d p_cur = current_pose.translation();
            const Eigen::Matrix3d R_cur = current_pose.linear();
            const Eigen::Quaterniond q_cur(R_cur);
            const Eigen::Vector3d ori_error = robot::computeOrientationError(R_cur, R_des);

            // 5D error: position + (pitch,yaw)
            Vector5d e;
            e << (p_des - p_cur),
                ori_error(1),
                ori_error(2);

            // Reduced 5x5 Jacobian (x,y,z,pitch,yaw)
            Matrix5d J;
            robot.getJacobianReduced(J);

            // Damped least squares solution with velocity clamping
            Vector5d qdot = solveDampedLeastSquares(J, xd_dot + K * e, damping);
            qdot = clampJointVel(qdot, scaled_max_speeds);

            // Send joint velocity command
            robot::JointVector qdot_cmd = qdot;
            robot.setJointVel(qdot_cmd);

            // Publish control error for visualization
            std_msgs::msg::Float64MultiArray err_msg;
            err_msg.data.assign({e(0), e(1), e(2), e(3), e(4)});
            error_pub->publish(err_msg);

            const rclcpp::Time stamp = node->now();

            geometry_msgs::msg::Point desired_point;
            desired_point.x = p_des.x();
            desired_point.y = p_des.y();
            desired_point.z = p_des.z();
            desired_marker_msg.header.stamp = stamp;
            desired_marker_msg.points.push_back(desired_point);
            desired_marker_pub->publish(desired_marker_msg);

            geometry_msgs::msg::Point actual_point;
            actual_point.x = p_cur.x();
            actual_point.y = p_cur.y();
            actual_point.z = p_cur.z();
            actual_marker_msg.header.stamp = stamp;
            actual_marker_msg.points.push_back(actual_point);
            actual_marker_pub->publish(actual_marker_msg);

            // Spin and sleep to maintain loop rate
            rclcpp::spin_some(node);
            rate.sleep();
        }

        // Update start for next segment
        p0 = p1;
        q0 = q1;
    }

    // Stop the robot after trajectory completion
    robot::JointVector zero = robot::JointVector::Zero();
    robot.setJointVel(zero);

    rclcpp::shutdown();
    return 0;
}
