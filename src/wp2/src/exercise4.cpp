#include <rclcpp/rclcpp.hpp>
#include <surros_lib/surros_interface.h>

#include <chrono>
#include <string>
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
    auto node = rclcpp::Node::make_shared("wp2_exercise4");
    robot::SurrosControl surros(node);
    surros.initialize();

    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // Parameters for pick/place positions and gripper settings
    const std::vector<double> pick_xyz = node->declare_parameter<std::vector<double>>(
        "pick_xyz", {0.0, 0.10, 0.0});
    const std::vector<double> place_xyz = node->declare_parameter<std::vector<double>>(
        "place_xyz", {0.04, 0.10, 0.0});

    const double block_size = node->declare_parameter<double>("block_size", 0.025); // [m] edge length
    const double approach_z_offset = node->declare_parameter<double>("approach_z_offset", block_size * 2.0);
    const double grasp_z_offset = node->declare_parameter<double>("grasp_z_offset", 0);

    const double move_speed = node->declare_parameter<double>("move_speed", 0.5);
    const double tool_pitch = node->declare_parameter<double>("tool_pitch", 1.5);
    const double tool_yaw = node->declare_parameter<double>("tool_yaw", 0.0);
    const int max_cycles = node->declare_parameter<int>("max_cycles", 10); // <=0 means infinite
    const double cycle_pause_s = node->declare_parameter<double>("cycle_pause_s", 0.5);

    const int gripper_open = node->declare_parameter<int>("gripper_open", 20);
    const int gripper_closed = node->declare_parameter<int>("gripper_closed", 80);

    if (pick_xyz.size() != 3 || place_xyz.size() != 3)
    {
        RCLCPP_ERROR(node->get_logger(), "pick_xyz and place_xyz must be 3-element vectors.");
        rclcpp::shutdown();
        return 1;
    }

    const Eigen::Vector3d pick(pick_xyz[0], pick_xyz[1], pick_xyz[2]);
    const Eigen::Vector3d place(place_xyz[0], place_xyz[1], place_xyz[2]);

    // Build a pick-and-place path from one point to another
    auto build_sequence = [&](const Eigen::Vector3d &from, const Eigen::Vector3d &to)
    {
        const Eigen::Vector3d from_above = from + Eigen::Vector3d(0.0, 0.0, approach_z_offset);
        const Eigen::Vector3d from_grasp = from + Eigen::Vector3d(0.0, 0.0, grasp_z_offset);
        const Eigen::Vector3d to_above = to + Eigen::Vector3d(0.0, 0.0, approach_z_offset);
        const Eigen::Vector3d to_release = to + Eigen::Vector3d(0.0, 0.0, grasp_z_offset);

        return std::vector<Waypoint>{
            {from_above, "above_pick"},
            {from_grasp, "grasp"},
            {from_above, "lift"},
            {to_above, "above_place"},
            {to_release, "release"},
            {to_above, "retreat"},
        };
    };

    RCLCPP_INFO(node->get_logger(), "Pick-and-place loop starting.");

    int completed_cycles = 0;
    int cycle = 0;
    // Loop cycles: A->B then B->A
    while (rclcpp::ok() && (max_cycles <= 0 || cycle < max_cycles))
    {
        const bool forward = (cycle % 2 == 0);
        const Eigen::Vector3d from = forward ? pick : place;
        const Eigen::Vector3d to = forward ? place : pick;

        const auto sequence = build_sequence(from, to);

        // Open gripper before approaching the object
        surros.setGripperJoint(gripper_open, true);

        for (size_t i = 0; i < sequence.size(); ++i)
        {
            const auto &step = sequence[i];

            surros.setEndeffectorPose(step.xyz, tool_pitch, tool_yaw, move_speed, false, true);

            if (step.label == "grasp")
            {
                surros.setGripperJoint(gripper_closed, true);
            }
            else if (step.label == "release")
            {
                surros.setGripperJoint(gripper_open, true);
            }
        }

        completed_cycles += 1;
        cycle += 1;

        RCLCPP_INFO(node->get_logger(), "Completed cycle %d.", completed_cycles);
        if (cycle_pause_s > 0.0)
        {
            rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(cycle_pause_s)));
        }
    }

    RCLCPP_INFO(node->get_logger(), "Pick-and-place loop finished. Completed cycles: %d", completed_cycles);

    rclcpp::shutdown();
    return 0;
}
