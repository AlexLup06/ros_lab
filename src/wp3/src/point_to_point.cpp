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
    auto node_error = rclcpp::Node::make_shared("error_publisher");

    robot::SurrosControl robot(node);
    robot.initialize();

    // initialize the publisher for the error
    auto publisher = node_error->create_publisher<std_msgs::msg::Float64MultiArray>("/control_error", 10); //topic not found, message not displaying

    // desired pose
    const Vector3d end_position(0.0, 0.15, 0.0);
    const double end_psi = 0.0;
    const double end_phi = 0.0;
    //const Eigen::Affine3d end_pose = robot::createPoseFromPosAndPitch(end_position, end_psi, end_phi);
    //const Eigen::Quaterniond end_quaternion(robot::convertRpyToRotMat(Vector3d(0.0,0.0,-1.5)));

    // duration of robot motion
    double t_end = 15; // in seconds
    
    // move to the default position
    robot.setJointsDefault();
    //robot.setEndeffectorPose(Vector3d(0.2, 0.01, 0.03), 0.0, 0.0);

    // maximum joint velocity
    Vector5d max_joint_velocity(0.5, 0.5, 0.5, 0.5, 0.5);
    Vector5d min_joint_velocity(-0.5, -0.5, -0.5, -0.5, -0.5);

    //Kinematics position error control -> calculates the joint velocities
    auto inverseKinematicsControl = [&](const Vector5d& e,
        Vector5d& joint_velocity){

            // get the reduced Jacobian
            Matrix5d jacobian;
            robot.getJacobianReduced(jacobian);

            //Matrix5d pseudo_inverse = jacobian.transpose()*(jacobian*jacobian.transpose()).inverse();

            // TODO: publish the current control error
            std::vector<double> error_msg_array = {e(0), e(1), e(2), e(3), e(4)};
            std_msgs::msg::Float64MultiArray error_msg;
            error_msg.data = error_msg_array;
            publisher->publish(error_msg);


            // define Gain Matrix
            Matrix5d k = Matrix5d::Identity();
            k(0,0) = 1.0;
            k(1,1) = 1.0;
            k(2,2) = 1.0;
            k(3,3) = 0.5;
            k(4,4) = 0.5;

            // calculate joint velocity
            joint_velocity = jacobian.inverse()*(k*e);

            //std::cout << "Calculated Joint Velocities: \n" << joint_velocity <<"\n";

            // get the current joint angles configuration
            Vector5d joint_angles;
            robot.getJointAngles(joint_angles);

            //check for collision and joint limits
            /*if(robot.checkSelfCollision(joint_angles) || robot.isExceedingJointLimits(joint_angles)){
                std::cout << "The arm self collided or exceeded joint limits.\n";
                joint_velocity = Vector5d::Zero();
            }*/

            // limit joint velocities
            /*for(int i=0; i<5; i++){
                if (joint_velocity(i)> max_joint_velocity(i)){
                    joint_velocity(i) = max_joint_velocity(i);
                }
                if(joint_velocity(i)< min_joint_velocity(i)){
                    joint_velocity(i) = min_joint_velocity(i);
                }
                
            }*/
    };

    
    int delta_t = 20; // in milliseconds
    
    for (double t = 0.0; t <= t_end; t+= delta_t/1000.0)
    {

        // get the current pose and convert it to a 5d vector
        Eigen::Affine3d current_pose_matrix;
        robot.getEndeffectorState(current_pose_matrix);
        Vector3d current_position = current_pose_matrix.translation();
        //Eigen::Quaterniond current_quaternion(current_pose_matrix.rotation());

        // calculate position error
        Vector3d e_position = end_position-current_position;

        // calculate the current phi and psi
        Vector3d current_rpy = robot::convertRotMatToRpy(current_pose_matrix.rotation());
        double current_psi = current_rpy(0);
        double current_phi = current_rpy(2);
        //double current_psi = std::asin(current_pose_matrix(2,2));
        //double current_phi = std::atan2(current_pose_matrix(1,0), current_pose_matrix(0,0));
        std::cout << "Current psi: \n" << current_psi << "\n";
        std::cout << "Current phi: \n" << current_phi << "\n";


        // TODO: calculate orientation error
        //Eigen::Quaterniond e_quaternion = x_d_orientation*x_e_orientation.inverse();
        //Vector3d e_orientation = e_quaternion.vec();
        //if(e_quaternion.w() < 0){
        //    e_orientation = -e_orientation;
        //}
        // get the error vector between the desired and current orientation
        // first entry: rotation about x 
        // second entry: rotation about y
        // last entry: rotation about z => phi
        /*Vector3d e_orientation = robot::computeOrientationError(end_pose.rotation(),current_pose_matrix.rotation());
        double e_phi = e_orientation(2);
        Vector2d norm_current_xy(current_position(0), current_position(1));
        norm_current_xy = norm_current_xy.normalized();
        double e_psi = e_orientation(0)*norm_current_xy(0)+e_orientation(1)*norm_current_xy(1);*/
        double e_psi = end_psi-current_psi;
        double e_phi = end_phi-current_phi;

        // combine position and orientation error
        Vector5d e(e_position(0), e_position(1), e_position(2), e_psi, e_phi);

        std::cout << "Current Error: \n" << e <<"\n";

        // if pose is good enough: break
        double threshold_position = 0.005;
        double threshold_orientation = 0.005;
        if(std::abs(e(0)) < threshold_position && std::abs(e(1)) < threshold_position && std::abs(e(2)) < threshold_position && std::abs(e(3)) < threshold_orientation && std::abs(e(4)) < threshold_orientation){
            std::cout << "Error Threshold reached.\n";
            break;
        }

        // get the joint velocity through inverse kinematics
        Vector5d joint_velocity;
        //inverseKinematicsControl(current_position, end_position, current_quaternion, end_quaternion, joint_velocity);
        inverseKinematicsControl(e, joint_velocity);


        //set the current joint velocity that was computed
        robot.setJointVel(joint_velocity);
        //rclcpp::sleep_for(std::chrono::milliseconds(delta_t));
        std::this_thread::sleep_for(std::chrono::milliseconds(delta_t));

        
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

