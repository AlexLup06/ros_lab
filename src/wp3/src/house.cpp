#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Dense>
#include <tuple>
#include <vector>

#include <sensor_msgs/msg/joint_state.hpp>
#include <surros_lib/kinematics.h>
#include <surros_lib/surros_interface.h>

#include <sophus/se3.hpp>

using Vector5d = Eigen::Matrix<double, 5, 1>;
using Matrix5d = Eigen::Matrix<double, 5, 5>;
using Matrix3x2 = Eigen::Matrix<double, 3, 2>;
using namespace Sophus;

// ========================= Velocity and Pose Profiles ============================================================
Vector3d linear_position_profile(Vector3d start_point, Vector3d end_point, double t, double t_start, double t_end){
    if(t> t_end){
        return end_point;
    }
    if (t < t_start){
        return start_point;
    }
    
    return start_point+(t-t_start)/(t_end-t_start)*(end_point-start_point);
}

Vector3d linear_velocity_profile(Vector3d start_point, Vector3d end_point, double t, double t_start, double t_end){
    if(t> t_end || t < t_start){
        return Vector3d(0.0, 0.0, 0.0);
    }
    
    return 1/(t_end-t_start)*(end_point-start_point);
}

Matrix3x2 house_profile(double t, double t_end, double t_buffer){
    // points of the house
    Vector3d p1(-0.03, 0.1, 0.04);
    Vector3d p2(-0.03, 0.15, 0.04);
    Vector3d p3(0.03, 0.1, 0.04);
    Vector3d p4(0.03, 0.15, 0.04);
    Vector3d p5(0.0, 0.18, 0.04);

    Eigen::Matrix<double, 3, 9> point_order;
    point_order.col(0) = p1;
    point_order.col(1) = p2;
    point_order.col(2) = p3;
    point_order.col(3) = p4;
    point_order.col(4) = p5;
    point_order.col(5) = p2;
    point_order.col(6) = p4;
    point_order.col(7) = p1;
    point_order.col(8) = p3;

    double t_line = t_end/8.0-t_buffer;
    Matrix3x2 result;

    for(int i = 0; i < 8; i++){
        if(t<(1+i)*t_line+i*t_buffer){
            result.col(0) << linear_position_profile(point_order.col(i), point_order.col(i+1), t, i*t_line + i*t_buffer, (i+1)*t_line+ i*t_buffer);
            result.col(1) << linear_velocity_profile(point_order.col(i), point_order.col(i+1), t, i*t_line + i*t_buffer, (i+1)*t_line+ i*t_buffer);
            break;
        }
        else if(t<(1+i)*t_line+(i+1)*t_buffer){
            result.col(0) << point_order.col(i+1);
            result.col(1) << Vector3d::Zero();
            break;
        }
    }

    return result;
}

// ======================= Helper Functions ==========================
// Convert a rotation matrix to the angles Psi (return(0)) and Phi (return(1))
Vector2d getPsiAndPhi(const Eigen::Matrix3d& rot){
    // change the orientation matrix to the base axis: z->x; x->y; y->z mapping in the world frame
    // if all euler angles are 0, the end effector coordinate system matches the base coordinate system
    Matrix3d rotMat;
    rotMat.col(0) << rot.col(2);
    rotMat.col(1) << rot.col(0);
    rotMat.col(2) << rot.col(1);
    // get the ZYX euler angles from this rotation matrix ==> psi: rotation about y, phi: rotation about x
    Vector3d euler = rotMat.eulerAngles(2,1,0);
    double psi = euler(1);
    double phi = euler(2);
    return Vector2d(psi, phi);
}

// Compute the joint velocities according to the inverse kinematics control algorithm
Vector5d inverseKinematicsControl(const Vector5d& error, const Vector3d& desired_velocity_position, robot::SurrosControl& robot){
    // Gain Matrix
    Matrix5d gain = Matrix5d::Identity();
    gain(0,0) = 1.0;
    gain(1,1) = 1.0;
    gain(2,2) = 1.0;
    gain(3,3) = 0.5;
    gain(4,4) = 0.5;

    // get the current jacobian
    Matrix5d jacobian;
    robot.getJacobianReduced(jacobian);

    //Matrix5d pseudo_inverse = jacobian.transpose()*(jacobian*jacobian.transpose()).inverse();

    // extend the velocity vector to 5 dim
    Vector5d desired_velocity(desired_velocity_position(0), desired_velocity_position(1), desired_velocity_position(2), 0.0, 0.0);
    return jacobian.inverse()*(desired_velocity+gain*error);
}

// Get the Pose error between the desired pose and the current pose => First three etries: translation xyz; Last two entries: differential orientation dRy, dRx => corresponding to psi, phi
Vector5d getPoseError(const Eigen::Affine3d& desired_pose, robot::SurrosControl& robot){
    // get the current pose
    Eigen::Affine3d current_pose;
    robot.getEndeffectorState(current_pose);

    // calculate the position error
    Vector3d position_error = desired_pose.translation() - current_pose.translation();

    // calculate the orientation error
    Vector3d error_orientation = robot::computeOrientationError(current_pose.rotation(), desired_pose.rotation()); 
    //Returns: differential motion [dRx, dRy, dRz]^T as approximation to the average spatial velocity multiplied by time
    //std::cout << "Orientation Error: \n" << error_orientation << "\n";

    return Vector5d(position_error(0), position_error(1), position_error(2), error_orientation(0), error_orientation(1));
}

void publishError(const Vector5d& error, rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr& publisher){
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {error(0), error(1), error(2), error(3), error(4)};
    publisher->publish(msg);
}

void publishPath(const Vector3d& position, rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr& publisher, rclcpp::Node::SharedPtr& node, nav_msgs::msg::Path& msg){
    
    msg.header.frame_id = "base_link";
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = node->now();
    pose_stamped.header.frame_id = "base_link";
    pose_stamped.pose.position.x = position(0);
    pose_stamped.pose.position.y = position(1);
    pose_stamped.pose.position.z = position(2);
    pose_stamped.pose.orientation.w = 1.0;

    msg.poses.push_back(pose_stamped);
    publisher->publish(msg);
}

// =============== Main function =================
int main(int argc, char** argv)
{
    // ----------------- Initialization ------------------------
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("surros3");
    auto node_error = rclcpp::Node::make_shared("error_node");

    robot::SurrosControl robot(node);
    robot.initialize();

    // initialize the publisher for the error
    auto publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>("/control_error", 10);
    // initialize the publisher for the desired path
    auto publisher_x_d = node->create_publisher<nav_msgs::msg::Path>("/desired_path", 10);
    // initialize the publisher for the actual path
    auto publisher_x_e = node->create_publisher<nav_msgs::msg::Path>("/ee_path", 10);

    // define the path messages
    nav_msgs::msg::Path msg_x_d;
    nav_msgs::msg::Path msg_x_e;

    // ----------------- Setting Poses, Fequencies, ... -----------------------
    // max time for the control loop in milliseconds
    double t_end = 30*1000; // 20s
    // time step until joint velocities are updated in milliseconds
    int delta_t = 20;
    // time buffer for reching the end points
    double t_buffer = 2*1000; // 1s

    // setting the start pose
    robot.setEndeffectorPose(Vector3d(-0.03, 0.1, 0.0), 1.5, 0.0);
    rclcpp::sleep_for(std::chrono::seconds(2));

    
    // Get and Print the starting End Effector Pose
    Eigen::Affine3d eePose;
    robot.getEndeffectorState(eePose);
    std::cout << "Starting Translation: \n" << eePose.translation() << "\n";
    std::cout << "Starting Psi and Phi: \n" << getPsiAndPhi(eePose.rotation()) << "\n";

    // defining the starting pose
    Vector3d starting_position(-0.03, 0.1, 0.04);
    double starting_psi = 1.5;
    double starting_phi = 0.0;
    Eigen::Affine3d starting_pose = robot::createPoseFromPosAndPitch(starting_position, starting_psi, starting_phi);
    
    // defining the desired orientation
    double desired_psi = 1.5;
    double desired_phi = 0.0;

    // -------------- Control Loop To Drive the End-Effector to the starting pose for 2 seconds --------------------------------
    for(double t = 0.0; t < 2000; t+=delta_t){
        Vector5d error = getPoseError(starting_pose, robot);
        //std::cout << "Current Pose Error: \n" << error << "\n";

        publishError(error, publisher);

        double threshold_position = 0.005;
        double threshold_orientation = 0.05;
        if(std::abs(error(0)) < threshold_position && std::abs(error(1)) < threshold_position && std::abs(error(2)) < threshold_position && std::abs(error(3)) < threshold_orientation && std::abs(error(4)) < threshold_orientation){
            std::cout << "Pose Threshold Reached. \n";
            break;
        }
        
        Vector5d joint_velocities = inverseKinematicsControl(error, Vector3d::Zero(),robot);
        //std::cout << "Current Joint Velocities: \n" << joint_velocities << "\n";
        //limitJointVel(joint_velocities);

        robot.setJointVel(joint_velocities);
        rclcpp::sleep_for(std::chrono::milliseconds(delta_t));
    }

    // time to put the pen in the gripper
    robot.setJointVel(Vector5d::Zero());
    rclcpp::sleep_for(std::chrono::seconds(10));

    /*
    // Open and close the gripper to grab the pen
    robot.setGripperJoint(20);
    rclcpp::sleep_for(std::chrono::seconds(1));
    robot.setGripperJoint(99);
    rclcpp::sleep_for(std::chrono::seconds(1));
    */

    // -------------- Control Loop --------------------------------
    for(double t = 0.0; t < t_end; t+=delta_t){
        Matrix3x2 desired_pos_ori = house_profile(t, t_end, t_buffer);
        Vector3d desired_position = desired_pos_ori.col(0);
        Vector3d desired_velocity = desired_pos_ori.col(1);

        Eigen::Affine3d desired_pose = robot::createPoseFromPosAndPitch(desired_position, desired_psi, desired_phi);
        Vector5d error = getPoseError(desired_pose, robot);
        //std::cout << "Current Pose Error: \n" << error << "\n";

        robot.getEndeffectorState(eePose);

        // publish error, desired path and actual end-effector path
        publishError(error, publisher);
        publishPath(desired_position, publisher_x_d, node, msg_x_d);
        publishPath(eePose.translation(), publisher_x_e, node, msg_x_e);
        
        Vector5d joint_velocities = inverseKinematicsControl(error, desired_velocity, robot);
        //std::cout << "Current Joint Velocities: \n" << joint_velocities << "\n";
        //limitJointVel(joint_velocities);

        robot.setJointVel(joint_velocities);
        rclcpp::sleep_for(std::chrono::milliseconds(delta_t));
    }

    robot.setJointVel(Vector5d::Zero());

    // Get and Print the ending End Effector Pose
    robot.getEndeffectorState(eePose);
    std::cout << "Ending Translation: \n" << eePose.translation() << "\n";
    std::cout << "Ending Psi and Phi: \n" << getPsiAndPhi(eePose.rotation()) << "\n";

    //std::cout << "Resulting Pose Error: \n" << getPoseError(desired_pose, robot) << "\n";
    

    return 0;
}
