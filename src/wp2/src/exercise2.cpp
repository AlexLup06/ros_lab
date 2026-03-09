#include <rclcpp/rclcpp.hpp>
#include <surros_lib/misc.h>
#include <surros_lib/surros_interface.h>

#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

static std::string vec3ToString(const Eigen::Vector3d& v, int precision = 3)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision);
    oss << "[" << v.x() << ", " << v.y() << ", " << v.z() << "]";
    return oss.str();
}

static std::string rpyToString(const robot::RpyVector& rpy, int precision = 3)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision);
    oss << "[roll " << rpy(0) << ", pitch " << rpy(1) << ", yaw " << rpy(2) << "]";
    return oss.str();
}

struct TargetPoseDelta
{
    Eigen::Vector3d delta_xyz;
    double psi;
    double phi;
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
    std::thread spin_thread([&executor]() { executor.spin(); });

    rclcpp::sleep_for(std::chrono::milliseconds(500));

    std::vector<TargetPoseDelta> targets = {
        {Eigen::Vector3d(0.03, 0.00, 0.02),  0.0,  0.0,  "delta_1"},
        {Eigen::Vector3d(-0.02, 0.03, 0.00), 0.0,  0.2,  "delta_2"},
        {Eigen::Vector3d(0.00, -0.03, 0.015), -0.2, 0.0, "delta_3"},
        {Eigen::Vector3d(0.02, -0.02, -0.01), 0.15, -0.2, "delta_4"},
    };

    for (size_t i = 0; i < targets.size(); ++i)
    {
        const auto& target = targets[i];

        Eigen::Affine3d current_pose = Eigen::Affine3d::Identity();
        surros.getEndeffectorState(current_pose);

        Eigen::Affine3d desired_pose =
            robot::createPoseFromPosAndPitch(target.delta_xyz, target.psi, target.phi) * current_pose;

        RCLCPP_INFO(node->get_logger(),
                    "Target %zu (%s): dxyz=%s psi=%.3f phi=%.3f",
                    i + 1,
                    target.label.c_str(),
                    vec3ToString(target.delta_xyz).c_str(),
                    target.psi,
                    target.phi);

        bool success = surros.setEndeffectorPose(
            target.delta_xyz, target.psi, target.phi, rclcpp::Duration(4, 0), true, true);

        if (!success)
        {
            RCLCPP_WARN(node->get_logger(), "Target %zu: IK failed, skipping error check.", i + 1);
            continue;
        }

        rclcpp::sleep_for(std::chrono::milliseconds(300));

        Eigen::Affine3d actual_pose = Eigen::Affine3d::Identity();
        surros.getEndeffectorState(actual_pose);

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

    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}
