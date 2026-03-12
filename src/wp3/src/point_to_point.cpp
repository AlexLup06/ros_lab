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
    const Vector3d end_position(0.25, 0.0, 0.1);
    const Eigen::Quaterniond end_quaternion(robot::convertRpyToRotMat(Vector3d(0.0,0.0,0.0)));

    // duration of robot motion
    double t_end = 10; // in seconds
    
    // move to the default position
    robot.setJointsDefault();

    // maximum joint velocity
    Vector5d max_joint_velocity(10.0, 10.0, 10.0, 10.0, 10.0);

    //Kinematics position error control -> calculates the joint velocities
    auto inverseKinematicsControl = [&](const Vector3d& x_e_position,
        const Vector3d& x_d_position,
        const Eigen::Quaterniond& x_e_orientation,
        const Eigen::Quaterniond& x_d_orientation,
        Vector5d& joint_velocity){

            // get the reduced Jacobian
            Matrix5d jacobian;
            robot.getJacobianReduced(jacobian);

            Matrix5d pseudo_inverse = jacobian.transpose()*(jacobian*jacobian.transpose()).inverse();

            // calculate position error
            Vector3d e_position = x_d_position-x_e_position;

            // calculate orientation error
            Eigen::Quaterniond e_quaternion = x_d_orientation*x_e_orientation.inverse();
            Vector3d e_orientation = e_quaternion.vec();
            if(e_quaternion.w() < 0){
                e_orientation = -e_orientation;
            }

            // combine position and orientation error
            Vector5d e(e_position(0), e_position(1), e_position(2), e_orientation(0), e_orientation(1));



            // publish the current control error
            std::vector<double> error_msg_array = {e(0), e(1), e(2), e(3), e(4)};
            std_msgs::msg::Float64MultiArray error_msg;
            error_msg.data = error_msg_array;
            publisher->publish(error_msg);


            // define Gain Matrix
            Matrix5d k = Matrix5d::Identity();
            k(0,0) = 1.5;
            k(1,1) = 1.5;
            k(2,2) = 1.5;
            k(3,3) = 1.0;
            k(4,4) = 1.0;

            // calculate joint velocity
            joint_velocity = pseudo_inverse*(k*e);

            std::cout << "Calculated Joint Velocities: \n" << joint_velocity <<"\n";

            for(int i=0; i<5; i++){
                if (std::abs(joint_velocity(i))> std::abs(max_joint_velocity(i)))
                {
                    // clamp joint velocity to a max value
                    if(joint_velocity(i) >=0){
                        joint_velocity(i) = max_joint_velocity(i);
                    }
                    else{
                        joint_velocity(i) = -max_joint_velocity(i);
                    }
                }
                
            }
    };

    
    int delta_t = 20; // in milliseconds
    
    for (double t = 0.0; t <= t_end; t+= delta_t/1000.0)
    {

        // get the current pose and convert it to a 5d vector
        Eigen::Affine3d current_pose_matrix;
        robot.getEndeffectorState(current_pose_matrix);
        Eigen::Quaterniond current_quaternion(current_pose_matrix.rotation());
        Vector3d current_position(current_pose_matrix(0,3), current_pose_matrix(1,3), current_pose_matrix(2,3));

        // get the current joint angles configuration
        Vector5d joint_angles;
        robot.getJointAngles(joint_angles);

        //check for collision and joint limits
        if(robot.checkSelfCollision(joint_angles)){
            robot.setJointVel(Vector5d::Zero());
            break;       
        }

        if(robot.isExceedingJointLimits(joint_angles)){
            robot.setJointVel(Vector5d::Zero());
            break;
        }

        // get the joint velocity through inverse kinematics
        Vector5d joint_velocity;
        inverseKinematicsControl(current_position, end_position, current_quaternion, end_quaternion, joint_velocity);


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

