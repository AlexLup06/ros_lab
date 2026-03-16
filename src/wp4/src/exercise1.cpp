#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <algorithm>

using namespace cv;
using namespace std;
struct Cube {
    string color;
    Point2f center;
};

static vector<Cube> detectCubesByColor(const Mat& bgr)
{
    // Detect colored blocks in HSV space
    vector<Cube> cubes;

    Mat hsv;
    cvtColor(bgr, hsv, COLOR_BGR2HSV);

    struct ColorRange { string name; Scalar low; Scalar high; };

    // HSV ranges for red/green/blue
    vector<ColorRange> ranges = {
        {"green",  Scalar(45, 80, 80),  Scalar(75, 255, 255)},
        {"blue",   Scalar(100, 80, 80), Scalar(130, 255, 255)},
    };

    // Red wraps around HSV hue -> use two ranges
    vector<pair<Scalar, Scalar>> red_ranges = {
        {Scalar(0, 80, 80),   Scalar(10, 255, 255)},
        {Scalar(170, 80, 80), Scalar(179, 255, 255)}
    };

    // Extract contours and compute centers
    auto process_mask = [&](const Mat& mask, const string& name) {
        vector<vector<Point>> contours;
        findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        for (const auto& c : contours)
        {
            double area = contourArea(c);
            if (area < 500.0) continue;
            Moments m = moments(c);
            if (m.m00 == 0) continue;
            Point2f center(static_cast<float>(m.m10 / m.m00), static_cast<float>(m.m01 / m.m00));
            cubes.push_back({name, center});
        }
    };

    // Process non-red colors (green, blue)
    for (const auto& r : ranges)
    {
        Mat mask;
        inRange(hsv, r.low, r.high, mask);
        process_mask(mask, r.name);
    }

    // Process red
    Mat red_mask = Mat::zeros(hsv.size(), CV_8UC1);
    for (const auto& rr : red_ranges)
    {
        Mat mask;
        inRange(hsv, rr.first, rr.second, mask);
        red_mask |= mask;
    }
    process_mask(red_mask, "red");

    return cubes;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // Vision-only node; no robot interface is needed for this exercise.
    auto node_ = std::make_shared<rclcpp::Node>("wp4_exercise1");

    string default_image_path = "test_image.jpeg";
    try
    {
        default_image_path =
            ament_index_cpp::get_package_share_directory("workpackage_4") + "/test_image.jpeg";
    }
    catch (const std::exception& e)
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
    for (const auto& c : cubes)
    {
        RCLCPP_INFO(node_->get_logger(),
                    "Color: %s | Center: (%.1f, %.1f)",
                    c.color.c_str(), c.center.x, c.center.y);
    }

    // Order cubes by x (left->right)
    vector<Cube> ordered = cubes;
    sort(ordered.begin(), ordered.end(),
         [](const Cube& a, const Cube& b) { return a.center.x < b.center.x; });

    map<string, string> color_to_position;
    if (ordered.size() >= 1) color_to_position[ordered[0].color] = "left";
    if (ordered.size() >= 2) color_to_position[ordered[1].color] = "middle";
    if (ordered.size() >= 3) color_to_position[ordered[2].color] = "right";

    RCLCPP_INFO(node_->get_logger(), "Color -> position mapping:");
    for (const auto& kv : color_to_position)
    {
        RCLCPP_INFO(node_->get_logger(), "  %s => %s", kv.first.c_str(), kv.second.c_str());
    }

    return 0;
}
