#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <Eigen/Dense>
#include <chrono>
#include <vector>

#include <surros_lib/misc.h>
#include <surros_lib/surros_interface.h>

using Vector5d = Eigen::Matrix<double, 5, 1>;
using Matrix5d = Eigen::Matrix<double, 5, 5>;

static Vector5d solveDampedLeastSquares(const Matrix5d& jacobian, const Vector5d& task_command, double damping)
{
    const double lambda_sq = damping * damping;
    const Matrix5d damped_system =
        jacobian * jacobian.transpose() + lambda_sq * Matrix5d::Identity();
    const Vector5d workspace_solution = damped_system.ldlt().solve(task_command);
    return jacobian.transpose() * workspace_solution;
}

struct Sample
{
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
};

static Vector5d clampJointVel(const Vector5d& vel, const robot::JointVector& max_speeds)
{
    // Clip joint velocities to safe limits
    Vector5d out = vel;
    for (int i = 0; i < out.size(); ++i)
    {
        const double limit = std::abs(max_speeds(i));
        if (limit > 0.0)
        {
            if (out(i) > limit) out(i) = limit;
            if (out(i) < -limit) out(i) = -limit;
        }
    }
    return out;
}

static std::vector<Sample> buildTrajectorySamples(const std::vector<Eigen::Vector3d>& waypoints,
                                                  double speed,
                                                  double dt)
{
    // Linear interpolation between waypoints with constant speed
    std::vector<Sample> samples;
    if (waypoints.size() < 2) return samples;
    if (speed <= 0.0 || dt <= 0.0) return samples;

    for (size_t i = 0; i + 1 < waypoints.size(); ++i)
    {
        const Eigen::Vector3d p0 = waypoints[i];
        const Eigen::Vector3d p1 = waypoints[i + 1];
        const Eigen::Vector3d delta = p1 - p0;
        const double dist = delta.norm();
        if (dist == 0.0) continue;

        const int steps = std::max(1, static_cast<int>(dist / (speed * dt)) + 1);
        const Eigen::Vector3d vel = delta / (steps * dt);

        for (int k = 0; k < steps; ++k)
        {
            const double s = static_cast<double>(k + 1) / static_cast<double>(steps);
            Sample sample;
            sample.pos = p0 + s * delta;
            sample.vel = vel;
            samples.push_back(sample);
        }
    }

    return samples;
}

static nav_msgs::msg::Path buildPathMessage(const std::vector<Sample>& samples,
                                            const Eigen::Quaterniond& q,
                                            const rclcpp::Time& stamp)
{
    // Convert samples into a Path message for RViz
    nav_msgs::msg::Path path;
    path.header.frame_id = "base_link";
    path.header.stamp = stamp;

    path.poses.reserve(samples.size());
    for (const auto& s : samples)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = s.pos.x();
        pose.pose.position.y = s.pos.y();
        pose.pose.position.z = s.pos.z();
        pose.pose.orientation.w = q.w();
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        path.poses.push_back(pose);
    }

    return path;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("wp3_exercise2");

    // Robot interface
    robot::SurrosControl robot(node);
    robot.initialize();

    // Publishers for error and paths
    auto error_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("/wp3/ex2/control_error", 10);
    auto desired_path_pub = node->create_publisher<nav_msgs::msg::Path>("/wp3/ex2/desired_path", 1);
    auto actual_path_pub = node->create_publisher<nav_msgs::msg::Path>("/wp3/ex2/actual_path", 10);

    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // Read current TCP pose
    Eigen::Affine3d start_pose = Eigen::Affine3d::Identity();
    robot.getEndeffectorState(start_pose);

    const Eigen::Vector3d p_start = start_pose.translation();
    const Eigen::Matrix3d R_start = start_pose.linear();

    // Trajectory parameters
    const std::vector<double> origin_xyz = node->declare_parameter<std::vector<double>>(
        "origin_xyz", {0.0, 0.10, 0.0});
    const double house_scale = node->declare_parameter<double>("house_scale", 0.06);
    const double z_level = node->declare_parameter<double>("z_level", origin_xyz.size() == 3 ? origin_xyz[2] : p_start.z());

    const double speed = node->declare_parameter<double>("path_speed", 0.03);
    const double dt = node->declare_parameter<double>("dt", 0.02);
    const bool use_feedforward = node->declare_parameter<bool>("use_feedforward", true);

    const double kp_pos = node->declare_parameter<double>("kp_pos", 1.0);
    const double kp_ori = node->declare_parameter<double>("kp_ori", 0.3);
    const double damping = node->declare_parameter<double>("damping", 0.1);
    const double max_joint_speed_scale = node->declare_parameter<double>("max_joint_speed_scale", 0.25);

    if (origin_xyz.size() != 3)
    {
        RCLCPP_ERROR(node->get_logger(), "origin_xyz must be a 3-element vector.");
        rclcpp::shutdown();
        return 1;
    }

    // Reference frame (T_base_ref) defined in base_link coordinates
    Eigen::Matrix3d R_down;
    R_down << 1, 0, 0,
              0, -1, 0,
              0, 0, -1;
    Eigen::Affine3d T_base_ref = Eigen::Affine3d::Identity();
    T_base_ref.translation() = Eigen::Vector3d(origin_xyz[0], origin_xyz[1], z_level);
    T_base_ref.linear() = R_down;

    // Keep the orientation of the starting pose, with a small tilt for pen contact
    const std::vector<double> pen_tilt_rpy = node->declare_parameter<std::vector<double>>(
        "pen_tilt_rpy", {0.0, 0.15, 0.0});  // roll, pitch, yaw [rad]
    if (pen_tilt_rpy.size() != 3)
    {
        RCLCPP_ERROR(node->get_logger(), "pen_tilt_rpy must be a 3-element vector.");
        rclcpp::shutdown();
        return 1;
    }
    robot::RpyVector tilt;
    tilt << pen_tilt_rpy[0], pen_tilt_rpy[1], pen_tilt_rpy[2];
    const Eigen::Matrix3d R_tilt = robot::convertRpyToRotMat(tilt);

    const Eigen::Matrix3d R_des = R_start * R_tilt;
    const Eigen::Quaterniond q_des(R_des);

    // House of Nikolaus points in 2D (scaled) relative to origin
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
        {1.0, 1.0}
    };

    std::vector<Eigen::Vector3d> waypoints;
    waypoints.reserve(house_2d.size());
    for (const auto& p : house_2d)
    {
        const Eigen::Vector3d p_local(house_scale * p.x(), house_scale * p.y(), 0.0);
        const Eigen::Vector3d p_world = T_base_ref * p_local;
        waypoints.emplace_back(p_world.x(), p_world.y(), p_world.z());
    }

    const auto samples = buildTrajectorySamples(waypoints, speed, dt);
    if (samples.empty())
    {
        RCLCPP_ERROR(node->get_logger(), "Trajectory is empty. Check parameters.");
        rclcpp::shutdown();
        return 1;
    }

    // Publish desired path once
    const auto desired_path = buildPathMessage(samples, q_des, node->get_clock()->now());
    desired_path_pub->publish(desired_path);

    nav_msgs::msg::Path actual_path;
    actual_path.header.frame_id = "base_link";

    // Proportional gains
    Matrix5d K = Matrix5d::Zero();
    K(0, 0) = kp_pos;
    K(1, 1) = kp_pos;
    K(2, 2) = kp_pos;
    K(3, 3) = kp_ori;
    K(4, 4) = kp_ori;

    rclcpp::Rate rate(1.0 / dt);
    const robot::JointVector scaled_max_speeds = max_joint_speed_scale * robot.getMaxJointSpeeds();

    // Trajectory tracking loop
    for (size_t i = 0; i < samples.size() && rclcpp::ok(); ++i)
    {
        const auto& sample = samples[i];

        Eigen::Affine3d current_pose = Eigen::Affine3d::Identity();
        robot.getEndeffectorState(current_pose);

        const Eigen::Vector3d p_cur = current_pose.translation();
        const Eigen::Matrix3d R_cur = current_pose.linear();
        const Eigen::Vector3d orientation_error = robot::computeOrientationError(R_cur, R_des);

        // 5D error: position + (pitch,yaw)
        Vector5d e;
        e << (sample.pos - p_cur),
             orientation_error(1),
             orientation_error(2);

        // Optional feedforward velocity
        Vector5d xd_dot = Vector5d::Zero();
        if (use_feedforward)
        {
            xd_dot << sample.vel, 0.0, 0.0;
        }

        // Reduced Jacobian (x,y,z,pitch,yaw)
        robot::RobotJacobianReduced J;
        robot.getJacobianReduced(J);

        Vector5d qdot = solveDampedLeastSquares(J, xd_dot + K * e, damping);
        qdot = clampJointVel(qdot, scaled_max_speeds);

        robot::JointVector qdot_cmd = qdot;
        robot.setJointVel(qdot_cmd);

        std_msgs::msg::Float64MultiArray err_msg;
        err_msg.data.assign({e(0), e(1), e(2), e(3), e(4)});
        error_pub->publish(err_msg);

        // Append actual pose to path for RViz
        geometry_msgs::msg::PoseStamped actual_pose;
        actual_pose.header.frame_id = "base_link";
        actual_pose.header.stamp = node->get_clock()->now();
        actual_pose.pose.position.x = p_cur.x();
        actual_pose.pose.position.y = p_cur.y();
        actual_pose.pose.position.z = p_cur.z();
        Eigen::Quaterniond q_cur(R_cur);
        actual_pose.pose.orientation.w = q_cur.w();
        actual_pose.pose.orientation.x = q_cur.x();
        actual_pose.pose.orientation.y = q_cur.y();
        actual_pose.pose.orientation.z = q_cur.z();

        actual_path.header.stamp = actual_pose.header.stamp;
        actual_path.poses.push_back(actual_pose);
        actual_path_pub->publish(actual_path);

        rate.sleep();
    }

    robot::JointVector zero = robot::JointVector::Zero();
    robot.setJointVel(zero);

    rclcpp::shutdown();
    return 0;
}
