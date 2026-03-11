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
using namespace Sophus;

// desired pose profile at time t when motion is supposed to last for t_end seconds
Vector5d pose_profile(Vector5d start_pose, Vector5d end_pose, double t, double t_end){
    return start_pose+t/t_end*(end_pose-start_pose);
}

// =============== Main function =================
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("robot_control");

    robot::SurrosControl robot(node);
    robot.initialize();

    // initialize the publisher for the error
    auto publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>("control_error", 10); //topic not found, message not displaying

    // desired pose
    Vector5d end_pose(0.15, 0.0, 0.1, 0.0, 0.0);

    // duration of robot motion
    double t_end = 5; // in seconds
    
    // move to the default position
    robot.setJointsDefault();
    // get the start pose and convert it to a 5d vector
    Eigen::Affine3d start_pose_matrix;
    robot.getEndeffectorState(start_pose_matrix);
    Vector3d start_rpy = robot::convertRotMatToRpy(start_pose_matrix.rotation());
    Vector5d start_pose(start_pose_matrix(0,3), start_pose_matrix(1,3), start_pose_matrix(2,3), start_rpy(0), start_rpy(1));

    // constant velocity trajectory
    Vector5d desired_velocity= 1/t_end*(end_pose-start_pose);

    //Kinematics position error control -> calculates the joint velocities
    auto inverseKinematicsControl = [&](const Vector5d& x_e,
        const Vector5d& x_d,
        const Vector5d& desired_velocity,
        Vector5d& joint_velocity){

            // get the reduced Jacobian
            Matrix5d jacobian;
            robot.getJacobianReduced(jacobian);

            // calculate pose error
            Vector5d e = x_d-x_e;

            // publish the current control error
            std::vector<double> error_msg_array = {e(0), e(1), e(2), e(3), e(4)};
            std_msgs::msg::Float64MultiArray error_msg;
            error_msg.data = error_msg_array;
            publisher->publish(error_msg);


            // define Gain Matrix
            Matrix5d k = Matrix5d::Identity();
            k(0,0) = 2;
            k(1,1) = 2;
            k(2,2) = 4;

            // calculate joint velocity
            joint_velocity = jacobian.inverse()*(desired_velocity+k*e);
    };

    
    int delta_t = 20; // in milliseconds
    for (double t = 0.0; t < t_end; t+= delta_t/1000.0)
    {
        Vector5d desired_pose = pose_profile(start_pose, end_pose, t, t_end); //computing the current desired pose

        // get the current pose and convert it to a 5d vector
        Eigen::Affine3d current_pose_matrix;
        robot.getEndeffectorState(current_pose_matrix);
        Vector3d current_rpy = robot::convertRotMatToRpy(current_pose_matrix.rotation());
        Vector5d current_pose(current_pose_matrix(0,3), current_pose_matrix(1,3), current_pose_matrix(2,3), current_rpy(0), current_rpy(1));

        // get the current joint angles configuration
        Vector5d joint_angles;
        robot.getJointAngles(joint_angles);

        //check for collision and joint limits
        if(robot.checkSelfCollision(joint_angles)){
            break;       
        }

        if(robot.isExceedingJointLimits(joint_angles)){
            break;
        }

        // get the joint velocity through inverse kinematics
        Vector5d joint_velocity;
        inverseKinematicsControl(current_pose, desired_pose, desired_velocity, joint_velocity);


        //set the current joint velocity that was computed
        robot.setJointVel(joint_velocity);
        rclcpp::sleep_for(std::chrono::milliseconds(delta_t));
    }

    //end motion
    robot.setJointVel(Vector5d(0.0, 0.0, 0.0, 0.0, 0.0));
    
    // print the measured end pose
    Eigen::Affine3d measured_pose;
    robot.getEndeffectorState(measured_pose);
    std::cout << "Measured End effector Position \n";
    std::cout << measured_pose.translation() << "\n";

    return 0;
}

