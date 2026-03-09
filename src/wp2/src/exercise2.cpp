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
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision);
    oss << "[" << v.x() << ", " << v.y() << ", " << v.z() << "]";
    return oss.str();
}

static std::string rpyToString(const robot::RpyVector &rpy, int precision = 3)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision);
    oss << "[roll " << rpy(0) << ", pitch " << rpy(1) << ", yaw " << rpy(2) << "]";
    return oss.str();
}

struct TargetPose
{
    Eigen::Affine3d desired_pose;
    std::string label;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("wp2_exercise2");
    robot::SurrosControl surros(node);
    surros.initialize();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]()
                            { executor.spin(); });

    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // Define desired end-effector poses (T_base_tcp) here.
    Eigen::Matrix3d R_down;
    R_down << 1, 0, 0,
        0, -1, 0,
        0, 0, -1;

    std::vector<TargetPose> targets;
    Eigen::Affine3d pose_1 = Eigen::Affine3d::Identity();
    pose_1.translation() = Eigen::Vector3d(0.0, 0.12, 0.05);
    pose_1.linear() = R_down;
    targets.push_back({pose_1,
                       "pose_1"});
    Eigen::Affine3d pose_2 = Eigen::Affine3d::Identity();
    pose_2.translation() = Eigen::Vector3d(0.0, 0.12, 0.0125);
    pose_2.linear() = R_down;
    targets.push_back({pose_2, "pose_2"});

    Eigen::Affine3d pose_3 = Eigen::Affine3d::Identity();
    pose_3.translation() = Eigen::Vector3d(0.0, 0.12, 0.05);
    pose_3.linear() = R_down;
    targets.push_back({pose_3, "pose_3"});

    Eigen::Affine3d pose_4 = Eigen::Affine3d::Identity();
    pose_4.translation() = Eigen::Vector3d(0.0, 0.05, 0.05);
    pose_4.linear() = R_down;
    targets.push_back({pose_4, "pose_4"});

    for (size_t i = 0; i < targets.size(); ++i)
    {
        const auto &target = targets[i];

        RCLCPP_INFO(node->get_logger(), "Target %zu (%s): moving to desired pose.", i + 1, target.label.c_str());

        surros.setEndeffectorPose(target.desired_pose, rclcpp::Duration(4, 0), false, true);

        rclcpp::sleep_for(std::chrono::milliseconds(300));

        Eigen::Affine3d actual_pose = Eigen::Affine3d::Identity();
        surros.getEndeffectorState(actual_pose);

        const Eigen::Vector3d pos_error = actual_pose.translation() - target.desired_pose.translation();
        const Eigen::Vector3d ori_error = robot::computeOrientationError(target.desired_pose.linear(), actual_pose.linear());

        const robot::RpyVector desired_rpy = robot::convertRotMatToRpy(target.desired_pose.linear());
        const robot::RpyVector actual_rpy = robot::convertRotMatToRpy(actual_pose.linear());

        RCLCPP_INFO(node->get_logger(), "Desired position: %s", vec3ToString(target.desired_pose.translation()).c_str());
        RCLCPP_INFO(node->get_logger(), "Actual position:  %s", vec3ToString(actual_pose.translation()).c_str());
        RCLCPP_INFO(node->get_logger(), "Position error:  %s | L2 norm: %.4f",
                    vec3ToString(pos_error).c_str(), pos_error.norm());

        RCLCPP_INFO(node->get_logger(), "Desired RPY: %s", rpyToString(desired_rpy).c_str());
        RCLCPP_INFO(node->get_logger(), "Actual RPY:  %s", rpyToString(actual_rpy).c_str());
        RCLCPP_INFO(node->get_logger(), "Orientation error (approx): %s | L2 norm: %.4f",
                    vec3ToString(ori_error).c_str(), ori_error.norm());
    }

    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}
