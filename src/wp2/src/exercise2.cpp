#include <rclcpp/rclcpp.hpp>
#include <surros_lib/misc.h>
#include <surros_lib/surros_interface.h>

#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

static std::string vec3ToString(const Eigen::Vector3d &v, int precision = 3)
{
    // Compact vector print for logs
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision);
    oss << "[" << v.x() << ", " << v.y() << ", " << v.z() << "]";
    return oss.str();
}

static std::string rpyToString(const robot::RpyVector &rpy, int precision = 3)
{
    // Compact RPY print for logs
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision);
    oss << "[roll " << rpy(0) << ", pitch " << rpy(1) << ", yaw " << rpy(2) << "]";
    return oss.str();
}

struct TargetPose
{
    Eigen::Vector3d desired_xyz;
    std::string label;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // ROS node + robot interface
    auto node = rclcpp::Node::make_shared("wp2_exercise2");
    robot::SurrosControl surros(node);
    surros.initialize();

    rclcpp::sleep_for(std::chrono::milliseconds(500));

    const std::vector<double> pose_5_xyz = node->declare_parameter<std::vector<double>>(
        "pose_5_xyz", {0.0, 0.15, 0.0});
    const std::vector<double> pose_1_xyz = node->declare_parameter<std::vector<double>>(
        "pose_1_xyz", {0.0, 0.10, 0.05});
    const std::vector<double> pose_2_xyz = node->declare_parameter<std::vector<double>>(
        "pose_2_xyz", {0.0, 0.10, 0.0});
    const std::vector<double> pose_3_xyz = node->declare_parameter<std::vector<double>>(
        "pose_3_xyz", {0.0, 0.13, 0.0});
    const std::vector<double> pose_4_xyz = node->declare_parameter<std::vector<double>>(
        "pose_4_xyz", {0.0, 0.15, 0.0});
    const double tool_pitch = node->declare_parameter<double>("tool_pitch", 1.5);
    const double tool_yaw = node->declare_parameter<double>("tool_yaw", 0.0);

    // List of target poses to visit
    std::vector<TargetPose> targets;
        targets.push_back({Eigen::Vector3d(pose_5_xyz[0], pose_5_xyz[1], pose_5_xyz[2]), "pose_5"});
    targets.push_back({Eigen::Vector3d(pose_1_xyz[0], pose_1_xyz[1], pose_1_xyz[2]), "pose_1"});
    targets.push_back({Eigen::Vector3d(pose_2_xyz[0], pose_2_xyz[1], pose_2_xyz[2]), "pose_2"});
    targets.push_back({Eigen::Vector3d(pose_3_xyz[0], pose_3_xyz[1], pose_3_xyz[2]), "pose_3"});
    targets.push_back({Eigen::Vector3d(pose_4_xyz[0], pose_4_xyz[1], pose_4_xyz[2]), "pose_4"});

    for (size_t i = 0; i < targets.size(); ++i)
    {
        const auto &target = targets[i];
        const Eigen::Affine3d desired_pose = robot::createPoseFromPosAndPitch(target.desired_xyz, tool_pitch, tool_yaw);

        RCLCPP_INFO(node->get_logger(), "Target %zu (%s): moving to desired pose.", i + 1, target.label.c_str());

        const bool success = surros.setEndeffectorPose(target.desired_xyz, tool_pitch, tool_yaw, rclcpp::Duration(3, 0), false, true);
        if (!success)
        {
            RCLCPP_ERROR(node->get_logger(), "IK rejected target %zu (%s).", i + 1, target.label.c_str());
        }

        rclcpp::sleep_for(std::chrono::milliseconds(300));

        Eigen::Affine3d actual_pose = Eigen::Affine3d::Identity();
        surros.getEndeffectorState(actual_pose);

        // Simple pose error check (position + orientation)
        const Eigen::Vector3d pos_error = actual_pose.translation() - desired_pose.translation();
        const Eigen::Vector3d ori_error = robot::computeOrientationError(desired_pose.linear(), actual_pose.linear());

        const robot::RpyVector desired_rpy = robot::convertRotMatToRpy(desired_pose.linear());
        const robot::RpyVector actual_rpy = robot::convertRotMatToRpy(actual_pose.linear());

        RCLCPP_INFO(node->get_logger(), "Desired position: %s", vec3ToString(desired_pose.translation()).c_str());
        RCLCPP_INFO(node->get_logger(), "Actual position:  %s", vec3ToString(actual_pose.translation()).c_str());
        RCLCPP_INFO(node->get_logger(), "Position error:  %s | L2 norm: %.4f",
                    vec3ToString(pos_error).c_str(), pos_error.norm());

        RCLCPP_INFO(node->get_logger(), "Desired RPY: %s", rpyToString(desired_rpy).c_str());
        RCLCPP_INFO(node->get_logger(), "Actual RPY:  %s", rpyToString(actual_rpy).c_str());
        RCLCPP_INFO(node->get_logger(), "Orientation error (approx): %s | L2 norm: %.4f",
                    vec3ToString(ori_error).c_str(), ori_error.norm());
    }

    rclcpp::shutdown();
    return 0;
}
