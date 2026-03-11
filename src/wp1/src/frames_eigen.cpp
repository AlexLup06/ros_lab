#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <Eigen/Dense>
#include <cmath>

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

void visualizeFrame(const Eigen::Matrix4d& H, 
    rclcpp::Publisher<Marker>::SharedPtr& publisher, 
    const std::string& name)
{
    // get the x,y,z axes of the frame and the origin (o)
    Eigen::Vector3d x(H(0,0), H(1,0), H(2,0));
    Eigen::Vector3d y(H(0,1), H(1,1), H(2,1));
    Eigen::Vector3d z(H(0,2), H(1,2), H(2,2));
    Eigen::Vector3d o(H(0,3), H(1,3), H(2,3));

    
    auto publishArrow = [&](int id,
        const Eigen::Vector3d& start, 
        const Eigen::Vector3d& end, 
        float r, float g, float b){
        // initialize marker
        Marker marker;
        // put the marker in the reference frame
        marker.header.frame_id = "world";
        marker.id = id;
        marker.ns = name;
        marker.action = Marker::ADD;

        // marker type arrow
        marker.type = Marker::ARROW;
        // marker color
        marker.color.r = r;
        marker.color.b = b;
        marker.color.g = g;
        marker.color.a = 1.0;

        // scale the marker
        marker.scale.x = 0.003; // thickness of arrow
        marker.scale.y = 0.006; // thickness of arrow head
        marker.scale.z = 0.01; // length of arrow head


        // create start and end points
        geometry_msgs::msg::Point p1;
        p1.x = start.x();
        p1.y = start.y();
        p1.z = start.z();

        geometry_msgs::msg::Point p2;
        p2.x = end.x();
        p2.y = end.y();
        p2.z = end.z();


        // add start and end point to the marker
        marker.points.push_back(p1);
        marker.points.push_back(p2);

        publisher->publish(marker);

    };

    publishArrow(0, o, o+x, 1.0f, 0.0f, 0.0f);
    publishArrow(1, o, o+y, 0.0f, 1.0f, 0.0f);
    publishArrow(2, o, o+z, 0.0f, 0.0f, 1.0f);

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("frames_eigen");

    //initializing the publisher
    auto publisher = node->create_publisher<Marker>("visualization_marker", 10);

    //First transformation matrix -> 45° rotation about the z-axis and translation by 2 along the x-axis.
    Eigen::Matrix4d T1 = Eigen::Matrix4d::Identity();
    double rot1 = M_PI / 4.0;
    T1(0,0) = std::cos(rot1);
    T1(0,1) = -std::sin(rot1);
    T1(1, 0)= std::sin(rot1);
    T1(1, 1)= std::cos(rot1);
    T1(0,3) = 2.0;


    //Second transformation matrix -> 90° rotation about the y-axis and translation by 4 along the x-axis and 2 along the z-axis.
    Eigen::Matrix4d T2 = Eigen::Matrix4d::Identity();
    double rot2 = M_PI / 2.0;
    T2(0,0) = std::cos(rot2);
    T2(0,2) = std::sin(rot2);
    T2(2, 0)= -std::sin(rot2);
    T2(2, 2)= std::cos(rot2);
    T2(0,3) = 4.0;
    T2(2,3) = 2.0;

    //Third transformation matrix -> combined
    Eigen::Matrix4d T3 = T1*T2;

    // send the messages to create markers for each transformation
    visualizeFrame(T1, publisher, "T1");
    visualizeFrame(T2, publisher, "T2");
    visualizeFrame(T3, publisher, "T3");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
