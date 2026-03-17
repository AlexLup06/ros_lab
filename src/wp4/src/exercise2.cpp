#include <rclcpp/rclcpp.hpp>
#include <surros_lib/surros_interface.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <algorithm>
#include <thread>
#include <cmath>
#include <cctype>

using namespace cv;
using namespace std;
struct Cube
{
    string color;
    Point2f center;
};

static vector<Cube> detectCubesByColor(const Mat &bgr)
{
    // Detect colored blocks in HSV
    vector<Cube> cubes;

    Mat hsv;
    cvtColor(bgr, hsv, COLOR_BGR2HSV);

    struct ColorRange
    {
        string name;
        Scalar low;
        Scalar high;
    };

    // HSV ranges for red/green/blue
    vector<ColorRange> ranges = {
        {"green", Scalar(45, 80, 80), Scalar(75, 255, 255)},
        {"blue", Scalar(100, 80, 80), Scalar(130, 255, 255)},
    };

    // Red wraps around HSV hue -> use two ranges
    vector<pair<Scalar, Scalar>> red_ranges = {
        {Scalar(0, 80, 80), Scalar(10, 255, 255)},
        {Scalar(170, 80, 80), Scalar(179, 255, 255)}};

    // Extract contours and centers for one color
    auto process_mask = [&](const Mat &mask, const string &name)
    {
        vector<vector<Point>> contours;
        findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        for (const auto &c : contours)
        {
            double area = contourArea(c);
            if (area < 500.0)
                continue;
            Moments m = moments(c);
            if (m.m00 == 0)
                continue;
            Point2f center(static_cast<float>(m.m10 / m.m00), static_cast<float>(m.m01 / m.m00));
            cubes.push_back({name, center});
        }
    };

    // Process non-red colors (green, blue)
    for (const auto &r : ranges)
    {
        Mat mask;
        inRange(hsv, r.low, r.high, mask);
        process_mask(mask, r.name);
    }

    // Process red
    Mat red_mask = Mat::zeros(hsv.size(), CV_8UC1);
    for (const auto &rr : red_ranges)
    {
        Mat mask;
        inRange(hsv, rr.first, rr.second, mask);
        red_mask |= mask;
    }
    process_mask(red_mask, "red");

    return cubes;
}

static bool moveToPose(robot::SurrosControl &surros,
                       const Eigen::Vector3d &xyz,
                       double yaw_rad,
                       double speed)
{
    // Try a small set of tool pitch angles and keep the first IK solution that works.
    const vector<double> tool_pitch_candidates = {1.05, 1.00, 1.10, 0.95, 1.15};
    for (double tool_pitch : tool_pitch_candidates)
    {
        if (surros.setEndeffectorPose(xyz, tool_pitch, yaw_rad, speed, false, true))
        {
            return true;
        }
    }
    return false;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // ROS node + robot interface
    auto node_ = std::make_shared<rclcpp::Node>("wp4_exercise2");
    auto surros_control = std::make_shared<robot::SurrosControl>(node_);

    // Initialize the SurrosControl instance
    surros_control->initialize();
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    string default_image_path = "test_image.jpeg";
    try
    {
        default_image_path =
            ament_index_cpp::get_package_share_directory("workpackage_4") + "/test_image.jpeg";
    }
    catch (const std::exception &e)
    {
        RCLCPP_WARN(node_->get_logger(),
                    "Could not resolve package share directory, falling back to '%s': %s",
                    default_image_path.c_str(),
                    e.what());
    }

    // Load the image path from a ROS parameter so launch does not pass '--ros-args' as a filename.
    const string image_path = node_->declare_parameter<string>("image_path", default_image_path);

    Mat img = imread(image_path);
    if (img.empty())
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load image: %s", image_path.c_str());
        return 1;
    }

    // 2) Detect colored blocks using HSV thresholding + contours
    vector<Cube> cubes = detectCubesByColor(img);

    RCLCPP_INFO(node_->get_logger(), "Detected cubes: %zu", cubes.size());
    for (const auto &c : cubes)
    {
        RCLCPP_INFO(node_->get_logger(),
                    "Color: %s | Center: (%.1f, %.1f)",
                    c.color.c_str(), c.center.x, c.center.y);
    }

    // Order cubes by x (left->right)
    vector<Cube> ordered = cubes;
    sort(ordered.begin(), ordered.end(),
         [](const Cube &a, const Cube &b)
         { return a.center.x < b.center.x; });

    map<string, string> color_to_position;
    if (ordered.size() >= 1)
        color_to_position[ordered[0].color] = "left";
    if (ordered.size() >= 2)
        color_to_position[ordered[1].color] = "middle";
    if (ordered.size() >= 3)
        color_to_position[ordered[2].color] = "right";

    RCLCPP_INFO(node_->get_logger(), "Color -> position mapping:");
    for (const auto &kv : color_to_position)
    {
        RCLCPP_INFO(node_->get_logger(), "  %s => %s", kv.first.c_str(), kv.second.c_str());
    }

    // Parameters for block positions (base_link) and stacking
    const vector<double> left_xyz = node_->declare_parameter<vector<double>>("left_xyz", {-0.078, 0.19, 0.0});
    const vector<double> middle_xyz = node_->declare_parameter<vector<double>>("middle_xyz", {0.017, 0.20, 0.0});
    const vector<double> right_xyz = node_->declare_parameter<vector<double>>("right_xyz", {0.098, 0.179, 0.0});

    const double left_yaw = node_->declare_parameter<double>("left_yaw", 0.0);
    const double middle_yaw = node_->declare_parameter<double>("middle_yaw", 0.0);
    const double right_yaw = node_->declare_parameter<double>("right_yaw", -0.0);

    const vector<double> stack_xyz = node_->declare_parameter<vector<double>>("stack_xyz", {0.18, 0.05, 0.0});
    const double stack_yaw = node_->declare_parameter<double>("stack_yaw", 0.0);

    const double block_size = node_->declare_parameter<double>("block_size", 0.03);
    const double approach_z_offset = node_->declare_parameter<double>("approach_z_offset", 0.06);
    const double grasp_z_offset = node_->declare_parameter<double>("grasp_z_offset", 0.01);
    const double move_speed = node_->declare_parameter<double>("move_speed", 0.5);
    const double action_wait_s = node_->declare_parameter<double>("action_wait_s", 1.0);
    const int gripper_open = node_->declare_parameter<int>("gripper_open", 20);
    const int gripper_closed = node_->declare_parameter<int>("gripper_closed", 80);

    const string sequence = node_->declare_parameter<string>("sequence", "rgb");

    if (left_xyz.size() != 3 || middle_xyz.size() != 3 || right_xyz.size() != 3 || stack_xyz.size() != 3)
    {
        RCLCPP_ERROR(node_->get_logger(), "left_xyz, middle_xyz, right_xyz, stack_xyz must be 3-element vectors.");
        rclcpp::shutdown();
        return 1;
    }

    struct BlockPose
    {
        Eigen::Vector3d pos;
        double yaw;
    };
    map<string, BlockPose> position_to_pose;
    position_to_pose["left"] = {Eigen::Vector3d(left_xyz[0], left_xyz[1], left_xyz[2]), left_yaw};
    position_to_pose["middle"] = {Eigen::Vector3d(middle_xyz[0], middle_xyz[1], middle_xyz[2]), middle_yaw};
    position_to_pose["right"] = {Eigen::Vector3d(right_xyz[0], right_xyz[1], right_xyz[2]), right_yaw};

    auto colorNameForChar = [](char c) -> string
    {
        if (c == 'r')
            return "red";
        if (c == 'g')
            return "green";
        if (c == 'b')
            return "blue";
        return "";
    };

    auto toLower = [](string s)
    {
        for (auto &ch : s)
            ch = static_cast<char>(tolower(ch));
        return s;
    };

    const string seq = toLower(sequence);

    // Stack position base
    const Eigen::Vector3d stack_base(stack_xyz[0], stack_xyz[1], stack_xyz[2]);

    RCLCPP_INFO(node_->get_logger(), "Starting stacking sequence: %s", seq.c_str());

    const auto waitAfterAction = [action_wait_s]()
    {
        const auto wait_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(action_wait_s));
        rclcpp::sleep_for(wait_ns);
    };

    const auto moveAndWait =
        [&](const Eigen::Vector3d &xyz, double yaw, const string &error_msg) -> bool
    {
        if (!moveToPose(*surros_control, xyz, yaw, move_speed))
        {
            RCLCPP_ERROR(node_->get_logger(), "%s", error_msg.c_str());
            return false;
        }
        waitAfterAction();
        return true;
    };

    const auto setGripperAndWait = [&](int command)
    {
        surros_control->setGripperJoint(command, true);
        waitAfterAction();
    };

    int idx = 0;
    for (char c : seq)
    {
        const string color = colorNameForChar(c);
        if (color.empty())
        {
            RCLCPP_WARN(node_->get_logger(), "Unknown color code '%c' in sequence.", c);
            continue;
        }

        if (color_to_position.find(color) == color_to_position.end())
        {
            RCLCPP_WARN(node_->get_logger(), "Color '%s' not detected in image.", color.c_str());
            continue;
        }

        const string pos_label = color_to_position[color];
        const auto pose_it = position_to_pose.find(pos_label);
        if (pose_it == position_to_pose.end())
        {
            RCLCPP_WARN(node_->get_logger(), "No pose defined for position '%s'.", pos_label.c_str());
            continue;
        }

        const BlockPose pick_pose = pose_it->second;
        const Eigen::Vector3d pick_above = pick_pose.pos + Eigen::Vector3d(0.0, 0.0, approach_z_offset);
        const Eigen::Vector3d pick_grasp = pick_pose.pos + Eigen::Vector3d(0.0, 0.0, grasp_z_offset);

        const double place_z = stack_base.z() + idx * block_size;
        const Eigen::Vector3d place_above = Eigen::Vector3d(stack_base.x(), stack_base.y(), place_z + approach_z_offset);
        const Eigen::Vector3d place_grasp = Eigen::Vector3d(stack_base.x(), stack_base.y(), place_z + grasp_z_offset);

        // Pick
        if (!moveAndWait(pick_above, pick_pose.yaw,
                         "Failed to reach pick_above pose for " + color + "."))
        {
            continue;
        }
        setGripperAndWait(gripper_open);
        if (!moveAndWait(pick_grasp, pick_pose.yaw,
                         "Failed to reach pick_grasp pose for " + color + "."))
        {
            continue;
        }
        setGripperAndWait(gripper_closed);
        if (!moveAndWait(pick_above, pick_pose.yaw,
                         "Failed to lift " + color + " after grasp."))
        {
            continue;
        }

        // Place
        if (!moveAndWait(place_above, stack_yaw,
                         "Failed to reach place_above pose for " + color + "."))
        {
            continue;
        }
        if (!moveAndWait(place_grasp, stack_yaw,
                         "Failed to reach place_grasp pose for " + color + "."))
        {
            continue;
        }
        setGripperAndWait(gripper_open);
        if (!moveAndWait(place_above, stack_yaw,
                         "Failed to retreat after placing " + color + "."))
        {
            continue;
        }
                if (!moveAndWait(place_above, stack_yaw,
                         "Failed to reach place_above pose for " + color + "."))
        {
            continue;
        }

        idx += 1;
    }

    rclcpp::shutdown();
    return 0;
}
