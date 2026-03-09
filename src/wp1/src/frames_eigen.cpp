#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <string>
#include <chrono>

using visualization_msgs::msg::Marker;

void visualizeFrame(const Eigen::Matrix4d& H,
                    rclcpp::Publisher<Marker>::SharedPtr publisher,
                    const std::string& name)
{
    Eigen::Vector3d origin = H.block<3,1>(0,3);
    Eigen::Matrix3d rotation = H.block<3,3>(0,0);

    Eigen::Vector3d x_axis = origin + rotation.col(0);
    Eigen::Vector3d y_axis = origin + rotation.col(1);
    Eigen::Vector3d z_axis = origin + rotation.col(2);

    auto publishArrow = [&](int id,
                            const Eigen::Vector3d& start,
                            const Eigen::Vector3d& end,
                            float r, float g, float b)
    {
        Marker marker;
        marker.header.frame_id = "world";
        marker.ns = name;
        marker.id = id;
        marker.type = Marker::ARROW;
        marker.action = Marker::ADD;

        geometry_msgs::msg::Point p1;
        p1.x = start.x();
        p1.y = start.y();
        p1.z = start.z();

        geometry_msgs::msg::Point p2;
        p2.x = end.x();
        p2.y = end.y();
        p2.z = end.z();

        marker.points.push_back(p1);
        marker.points.push_back(p2);

        marker.scale.x = 0.03;
        marker.scale.y = 0.06;
        marker.scale.z = 0.10;

        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0;

        publisher->publish(marker);
    };

    publishArrow(0, origin, x_axis, 1.0f, 0.0f, 0.0f);
    publishArrow(1, origin, y_axis, 0.0f, 1.0f, 0.0f);
    publishArrow(2, origin, z_axis, 0.0f, 0.0f, 1.0f);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("frames_eigen");
    auto publisher = node->create_publisher<Marker>("visualization_marker", 10);

    Eigen::Matrix4d T1 = Eigen::Matrix4d::Identity();
    double a = M_PI / 4.0;
    T1(0,0) = std::cos(a);
    T1(0,1) = -std::sin(a);
    T1(1,0) = std::sin(a);
    T1(1,1) = std::cos(a);
    T1(0,3) = 2.0;

    Eigen::Matrix4d T2 = Eigen::Matrix4d::Identity();
    double b = M_PI / 2.0;
    T2(0,0) = std::cos(b);
    T2(0,2) = std::sin(b);
    T2(2,0) = -std::sin(b);
    T2(2,2) = std::cos(b);
    T2(0,3) = 4.0;
    T2(2,3) = 2.0;

    Eigen::Matrix4d T3 = T1 * T2;

    rclcpp::sleep_for(std::chrono::milliseconds(500));

    visualizeFrame(T1, publisher, "T1");
    visualizeFrame(T2, publisher, "T2");
    visualizeFrame(T3, publisher, "T3");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
