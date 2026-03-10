#include <rclcpp/rclcpp.hpp>
#include <surros_lib/surros_interface.h>

#include <chrono>
#include <string>
#include <thread>
#include <vector>

struct Waypoint
{
    Eigen::Vector3d xyz;
    std::string label;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // ROS node + robot interface
    auto node = rclcpp::Node::make_shared("wp2_exercise3");
    robot::SurrosControl surros(node);
    surros.initialize();

    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // Parameters for pick/place positions and gripper settings
    const std::vector<double> pick_xyz = node->declare_parameter<std::vector<double>>(
        "pick_xyz", {0.0, 0.12, 0.0});
    const std::vector<double> place_xyz = node->declare_parameter<std::vector<double>>(
        "place_xyz", {0.0, 0.7, 0.0});

    const double block_size = node->declare_parameter<double>("block_size", 0.025);  // [m] edge length
    const double approach_z_offset = node->declare_parameter<double>("approach_z_offset", block_size * 2.0);
    const double grasp_z_offset = node->declare_parameter<double>("grasp_z_offset", block_size * 0.5);

    const double move_speed = node->declare_parameter<double>("move_speed", 0.5);

    const int gripper_open = node->declare_parameter<int>("gripper_open", 80);
    const int gripper_closed = node->declare_parameter<int>("gripper_closed", 20);

    if (pick_xyz.size() != 3 || place_xyz.size() != 3)
    {
        RCLCPP_ERROR(node->get_logger(), "pick_xyz and place_xyz must be 3-element vectors.");
        rclcpp::shutdown();
        return 1;
    }

    const Eigen::Vector3d pick(pick_xyz[0], pick_xyz[1], pick_xyz[2]);
    const Eigen::Vector3d place(place_xyz[0], place_xyz[1], place_xyz[2]);

    // Build approach and grasp points
    const Eigen::Vector3d pick_above = pick + Eigen::Vector3d(0.0, 0.0, approach_z_offset);
    const Eigen::Vector3d pick_grasp = pick + Eigen::Vector3d(0.0, 0.0, grasp_z_offset);
    const Eigen::Vector3d place_above = place + Eigen::Vector3d(0.0, 0.0, approach_z_offset);
    const Eigen::Vector3d place_release = place + Eigen::Vector3d(0.0, 0.0, grasp_z_offset);

    // Keep TCP z-axis pointing down
    Eigen::Matrix3d R_down;
    R_down << 1, 0, 0,
              0, -1, 0,
              0, 0, -1;

    // Pick-and-place sequence
    std::vector<Waypoint> sequence = {
        {pick_above, "above_pick"},
        {pick_grasp, "grasp"},
        {pick_above, "lift"},
        {place_above, "above_place"},
        {place_release, "release"},
        {place_above, "retreat"},
    };

    RCLCPP_INFO(node->get_logger(), "Pick-and-place sequence starting.");

    // Open gripper before approaching the object
    surros.setGripperJoint(gripper_open, true);

    for (size_t i = 0; i < sequence.size(); ++i)
    {
        const auto& step = sequence[i];

        // Command absolute pose in base_link
        Eigen::Affine3d desired_pose = Eigen::Affine3d::Identity();
        desired_pose.translation() = step.xyz;
        desired_pose.linear() = R_down;

        bool success = surros.setEndeffectorPose(desired_pose, move_speed, false, true);
        if (!success)
        {
            RCLCPP_WARN(node->get_logger(), "IK failed at step %zu (%s).", i + 1, step.label.c_str());
            continue;
        }

        // Gripper actions at specific steps
        if (step.label == "grasp")
        {
            surros.setGripperJoint(gripper_closed, true);
        }
        else if (step.label == "release")
        {
            surros.setGripperJoint(gripper_open, true);
        }
    }

    RCLCPP_INFO(node->get_logger(), "Pick-and-place sequence finished.");

    rclcpp::shutdown();
    return 0;
}
