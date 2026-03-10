#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <Eigen/Dense>
#include <cmath>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // Node + static TF broadcaster
    auto node = rclcpp::Node::make_shared("frames_tf");
    auto broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // ----- T1: rotate around Z and translate in X -----
    Eigen::Isometry3d T1 = Eigen::Isometry3d::Identity();
    T1.linear() = Eigen::AngleAxisd(M_PI/4.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    T1.translation() = Eigen::Vector3d(2.0, 0.0, 0.0);

    geometry_msgs::msg::TransformStamped tf_t1 = tf2::eigenToTransform(T1);
    tf_t1.header.frame_id = "world";
    tf_t1.child_frame_id = "t1";
    tf_t1.header.stamp = node->get_clock()->now();

    // ----- T2: rotate around Y and translate in X/Z -----
    Eigen::Isometry3d T2 = Eigen::Isometry3d::Identity();
    T2.linear() = Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
    T2.translation() = Eigen::Vector3d(4.0, 0.0, 2.0);

    geometry_msgs::msg::TransformStamped tf_t2 = tf2::eigenToTransform(T2);
    tf_t2.header.frame_id = "world";
    tf_t2.child_frame_id = "t2";
    tf_t2.header.stamp = node->get_clock()->now();

    // ----- T3: same transform as T2 but expressed relative to T1 -----
    geometry_msgs::msg::TransformStamped tf_t3 = tf2::eigenToTransform(T2);
    tf_t3.header.frame_id = "t1";
    tf_t3.child_frame_id = "t3";
    tf_t3.header.stamp = node->get_clock()->now();

    // Publish static transforms
    broadcaster->sendTransform(tf_t1);
    broadcaster->sendTransform(tf_t2);
    broadcaster->sendTransform(tf_t3);

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
