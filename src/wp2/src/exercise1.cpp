#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <surros_lib/kinematics.h>
#include <surros_lib/surros_interface.h>

#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

static std::string jointVectorToString(const robot::JointVector& q, int precision = 3)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision);
    oss << "[";
    for (int i = 0; i < q.size(); ++i)
    {
        oss << q(i);
        if (i + 1 < q.size())
        {
            oss << ", ";
        }
    }
    oss << "]";
    return oss.str();
}

static robot::JointVector vectorFromRatio(const robot::JointVector& lower, const robot::JointVector& upper, double ratio)
{
    return lower + ratio * (upper - lower);
}

static robot::JointVector mixedVector(const robot::JointVector& lower, const robot::JointVector& upper)
{
    robot::JointVector q;
    for (int i = 0; i < q.size(); ++i)
    {
        const double ratio = (i % 2 == 0) ? 0.35 : 0.65;
        q(i) = lower(i) + ratio * (upper(i) - lower(i));
    }
    return q;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("exercise1");
    robot::SurrosControl surros(node);
    surros.initialize();

    rclcpp::sleep_for(std::chrono::milliseconds(500));

    const auto lower = surros.getLowerJointLimits();
    const auto upper = surros.getUpperJointLimits();

    std::vector<robot::JointVector> targets;
    targets.push_back(vectorFromRatio(lower, upper, 0.20));
    targets.push_back(vectorFromRatio(lower, upper, 0.50));
    targets.push_back(vectorFromRatio(lower, upper, 0.80));
    targets.push_back(mixedVector(lower, upper));

    RCLCPP_INFO(node->get_logger(), "Moving to default joint configuration...");
    surros.setJointsDefault(rclcpp::Duration(3, 0), true);
    rclcpp::sleep_for(std::chrono::milliseconds(300));

    for (size_t i = 0; i < targets.size(); ++i)
    {
        const auto& target = targets[i];

        if (surros.isExceedingJointLimits(target))
        {
            RCLCPP_WARN(node->get_logger(), "Target %zu exceeds joint limits, skipping.", i + 1);
            continue;
        }

        RCLCPP_INFO(node->get_logger(), "Target %zu: %s", i + 1, jointVectorToString(target).c_str());

        surros.setJoints(target, rclcpp::Duration(4, 0), false, true);
        rclcpp::sleep_for(std::chrono::milliseconds(300));

        const auto actual = surros.getJointAngles();
        const auto error = actual - target;
        const auto abs_error = error.cwiseAbs();

        RCLCPP_INFO(node->get_logger(), "Actual %zu: %s", i + 1, jointVectorToString(actual).c_str());
        RCLCPP_INFO(node->get_logger(), "Abs error %zu: %s | L2 norm: %.4f",
                    i + 1, jointVectorToString(abs_error).c_str(), error.norm());
    }

    rclcpp::shutdown();
    return 0;
}
