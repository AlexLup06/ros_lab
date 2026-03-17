#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

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
    auto node = rclcpp::Node::make_shared("wp3_exercise1");

    // Robot interface
    robot::SurrosControl robot(node);
    robot.initialize();

    // Publish control error for rqt_plot
    auto error_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("/wp3/ex1/control_error", 10);

    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // Read current TCP pose
    Eigen::Affine3d start_pose = Eigen::Affine3d::Identity();
    robot.getEndeffectorState(start_pose);

    // Initial position and orientation
    const Eigen::Vector3d p0 = start_pose.translation();
    const Eigen::Matrix3d R0 = start_pose.linear();
    Eigen::Quaterniond q0(R0);

    // Point-to-point target in base_link.
    const std::vector<double> target_xyz = node->declare_parameter<std::vector<double>>(
        "target_xyz", {0.0, 0.13, 0.0});

    const Eigen::Vector3d p1(target_xyz[0], target_xyz[1], target_xyz[2]);

    // Target orientation in base_link parameterized by psi/phi.
    const double target_psi = node->declare_parameter<double>("target_psi", 1.570);
    const double target_phi = node->declare_parameter<double>("target_phi", 0.0);
    const Eigen::Affine3d target_pose = robot::createPoseFromPosAndPitch(p1, target_psi, target_phi);
    const Eigen::Quaterniond q1(target_pose.linear());

    // Trajectory and control parameters
    const double total_time = node->declare_parameter<double>("total_time", 8.0);
    const double settle_time = node->declare_parameter<double>("settle_time", 2.0);
    const double dt = node->declare_parameter<double>("dt", 0.02);
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

        // Spin and sleep to maintain loop rate
        rclcpp::spin_some(node);
        rate.sleep();
    }

    // Stop the robot after trajectory completion
    robot::JointVector zero = robot::JointVector::Zero();
    robot.setJointVel(zero);

    rclcpp::shutdown();
    return 0;
}
