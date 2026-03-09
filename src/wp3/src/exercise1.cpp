#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <Eigen/Dense>
#include <chrono>
#include <thread>
#include <vector>

#include <sensor_msgs/msg/joint_state.hpp>
#include <surros_lib/kinematics.h>
#include <surros_lib/misc.h>
#include <surros_lib/surros_interface.h>

using Vector5d = Eigen::Matrix<double, 5, 1>;
using Matrix5d = Eigen::Matrix<double, 5, 5>;

static Vector5d clampJointVel(const Vector5d &vel, const robot::JointVector &max_speeds)
{
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

    robot::SurrosControl robot(node);
    robot.initialize();

    auto error_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("/wp3/ex1/control_error", 10);

    rclcpp::sleep_for(std::chrono::milliseconds(500));

    Eigen::Affine3d start_pose = Eigen::Affine3d::Identity();
    robot.getEndeffectorState(start_pose);

    const Eigen::Vector3d p0 = start_pose.translation();
    const Eigen::Matrix3d R0 = start_pose.linear();
    Eigen::Quaterniond q0(R0);

    const std::vector<double> target_xyz = node->declare_parameter<std::vector<double>>(
        "target_xyz", {p0.x() + 0.05, p0.y() - 0.05, p0.z() + 0.03});
    if (target_xyz.size() != 3)
    {
        RCLCPP_ERROR(node->get_logger(), "target_xyz must be a 3-element vector.");
        executor.cancel();
        spin_thread.join();
        rclcpp::shutdown();
        return 1;
    }
    const Eigen::Vector3d p1(target_xyz[0], target_xyz[1], target_xyz[2]);
    const robot::RpyVector rpy0 = robot::convertRotMatToRpy(R0);
    const std::vector<double> target_rpy = node->declare_parameter<std::vector<double>>(
        "target_rpy", {rpy0(0) + 0, rpy0(1) + 0, rpy0(2) + 0});
    if (target_rpy.size() != 3)
    {
        RCLCPP_ERROR(node->get_logger(), "target_rpy must be a 3-element vector [roll, pitch, yaw].");
        rclcpp::shutdown();
        return 1;
    }
    robot::RpyVector rpy1;
    rpy1 << target_rpy[0], target_rpy[1], target_rpy[2];
    Eigen::Quaterniond q1(robot::convertRpyToRotMat(rpy1));

    const double total_time = 8.0;
    const double dt = 0.02;
    const int steps = static_cast<int>(total_time / dt);


    Matrix5d K = Matrix5d::Zero();
    K(0, 0) = 2.0;
    K(1, 1) = 2.0;
    K(2, 2) = 2.0;
    K(3, 3) = 1.0;
    K(4, 4) = 1.0;

    rclcpp::Rate rate(1.0 / dt);

    for (int i = 0; i <= steps && rclcpp::ok(); ++i)
    {
        const double s = static_cast<double>(i) / static_cast<double>(steps);

        const Eigen::Vector3d p_des = p0 + s * (p1 - p0);
        const Eigen::Quaterniond q_des = q0.slerp(s, q1);
        const Eigen::Matrix3d R_des = q_des.toRotationMatrix();

        Eigen::Affine3d current_pose = Eigen::Affine3d::Identity();
        robot.getEndeffectorState(current_pose);

        const Eigen::Vector3d p_cur = current_pose.translation();
        const Eigen::Matrix3d R_cur = current_pose.linear();

        const robot::RpyVector rpy_des = robot::convertRotMatToRpy(R_des);
        const robot::RpyVector rpy_cur = robot::convertRotMatToRpy(R_cur);

        Vector5d e;
        e << (p_des - p_cur),
            (rpy_des(1) - rpy_cur(1)),
            (rpy_des(2) - rpy_cur(2));

        Matrix5d J;
        robot.getJacobianReduced(J);

        Vector5d qdot = J.completeOrthogonalDecomposition().solve(K * e);
        qdot = clampJointVel(qdot, robot.getMaxJointSpeeds());

        robot::JointVector qdot_cmd = qdot;
        robot.setJointVel(qdot_cmd);

        std_msgs::msg::Float64MultiArray err_msg;
        err_msg.data.assign({e(0), e(1), e(2), e(3), e(4)});
        error_pub->publish(err_msg);

        rate.sleep();
    }

    robot::JointVector zero = robot::JointVector::Zero();
    robot.setJointVel(zero);

    rclcpp::shutdown();
    return 0;
}
