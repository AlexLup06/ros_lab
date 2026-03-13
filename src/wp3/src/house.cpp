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

Matrix3x2 house_profile(double t, double t_end){
    // points of the house
    Vector3d p1(-0.02, 0.05, 0.01);
    Vector3d p2(-0.02, 0.10, 0.01);
    Vector3d p3(0.02, 0.05, 0.01);
    Vector3d p4(0.02, 0.10, 0.01);
    Vector3d p5(0.0, 0.13, 0.01);

    double t_line = t_end/8.0;
    Matrix3x2 result;

    if (t< 1*t_line){
        result.col(0) << linear_position_profile(p1, p2, t, 0.0, 1*t_line);
        result.col(1) << linear_velocity_profile(p1, p2, t, 0.0, 1*t_line);
    }
    else if (t< 2*t_line){
        result.col(0) << linear_position_profile(p2, p3, t, 1*t_line, 2*t_line);
        result.col(1) << linear_velocity_profile(p2, p3, t, 1*t_line, 2*t_line);
    }
    else if (t< 3*t_line){
        result.col(0) << linear_position_profile(p3, p4, t, 2*t_line, 3*t_line);
        result.col(1) << linear_velocity_profile(p3, p4, t, 2*t_line, 3*t_line);
    }
    else if (t< 4*t_line){
        result.col(0) << linear_position_profile(p4, p5, t, 3*t_line, 4*t_line);
        result.col(1) << linear_velocity_profile(p4, p5, t, 3*t_line, 4*t_line);
    }
    else if (t< 5*t_line){
        result.col(0) << linear_position_profile(p5, p2, t, 4*t_line, 5*t_line);
        result.col(1) << linear_velocity_profile(p5, p2, t, 4*t_line, 5*t_line);
    }
    else if (t< 6*t_line){
        result.col(0) << linear_position_profile(p2, p4, t, 5*t_line, 6*t_line);
        result.col(1) << linear_velocity_profile(p2, p4, t, 5*t_line, 6*t_line);
    }
    else if (t< 7*t_line){
        result.col(0) << linear_position_profile(p4, p1, t, 6*t_line, 7*t_line);
        result.col(1) << linear_velocity_profile(p4, p1, t, 6*t_line, 7*t_line);
    }
    else if (t< 8*t_line){
        result.col(0) << linear_position_profile(p1, p3, t, 7*t_line, 8*t_line);
        result.col(1) << linear_velocity_profile(p1, p3, t, 7*t_line, 8*t_line);
    }
    return result;
}


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
    // initialize the publisher for the desired path
    auto publisher_x_d = node->create_publisher<nav_msgs::msg::Path>("/desired_path", 10); //topic not found, message not displaying
    // initialize the publisher for the actual path
    auto publisher_x_e = node->create_publisher<nav_msgs::msg::Path>("/ee_path", 10); //topic not found, message not displaying

    // duration of robot motion
    double t_end = 60; // in seconds
    
    // move to the first point
    robot.setEndeffectorPose(Vector3d(-0.02, 0.05, 0.01), 0.0, 0.0);

    // maximum joint velocity
    Vector5d max_joint_velocity(1.0, 1.0, 1.0, 1.0, 1.0);

    //Kinematics position error control -> calculates the joint velocities
    auto inverseKinematicsControl = [&](const Vector5d& e,
        const Vector5d& desired_velocity,
        Vector5d& joint_velocity){

            // get the reduced Jacobian
            Matrix5d jacobian;
            robot.getJacobianReduced(jacobian);

            Matrix5d pseudo_inverse = jacobian.transpose()*(jacobian*jacobian.transpose()).inverse();

            // TODO: publish the current control error
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
            joint_velocity = pseudo_inverse*(desired_velocity+k*e);

            //std::cout << "Calculated Joint Velocities: \n" << joint_velocity <<"\n";

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
        Vector3d current_position = current_pose_matrix.translation();

        // get the desired position and velocity
        Matrix3x2 desired_position_velocity = house_profile(t, t_end);
        Vector3d desired_position = desired_position_velocity.col(0);
        const Eigen::Affine3d desired_pose = robot::createPoseFromPosAndPitch(desired_position, 0.0, 0.0);
        Vector5d desired_velocity(desired_position_velocity.col(1)(0), desired_position_velocity.col(1)(1), desired_position_velocity.col(1)(2), 0.0, 0.0);

        // publish desired path
        nav_msgs::msg::Path msg_x_d;
        msg_x_d.header.frame_id = "base_link";
        geometry_msgs::msg::PoseStamped pose_stamped_x_d;
        pose_stamped_x_d.header.frame_id = "base_link";
        pose_stamped_x_d.pose.position.x = desired_position(0);
        pose_stamped_x_d.pose.position.y = desired_position(1);
        pose_stamped_x_d.pose.position.z = desired_position(2);
        pose_stamped_x_d.pose.orientation.w = 0.0;

        msg_x_d.poses.push_back(pose_stamped_x_d);
        publisher_x_d->publish(msg_x_d);
        
        // publish end effector path
        nav_msgs::msg::Path msg_x_e;
        msg_x_e.header.frame_id = "base_link";
        geometry_msgs::msg::PoseStamped pose_stamped_x_e;
        pose_stamped_x_e.header.frame_id = "base_link";
        pose_stamped_x_e.pose.position.x = current_position(0);
        pose_stamped_x_e.pose.position.y = current_position(1);
        pose_stamped_x_e.pose.position.z = current_position(2);
        pose_stamped_x_e.pose.orientation.w = 0.0;

        msg_x_e.poses.push_back(pose_stamped_x_e);
        publisher_x_e->publish(msg_x_e);       

        

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

        // calculate position error
        Vector3d e_position = desired_position-current_position;

        // get the error vector between the desired and current orientation
        // first entry: rotation about x 
        // second entry: rotation about y
        // last entry: rotation about z => phi
        Vector3d e_orientation = robot::computeOrientationError(desired_pose.rotation(),current_pose_matrix.rotation());
        double e_phi = e_orientation(2);
        Vector2d norm_current_xy(current_position(0), current_position(1));
        norm_current_xy = norm_current_xy.normalized();
        double e_psi = e_orientation(0)*norm_current_xy(0)+e_orientation(1)*norm_current_xy(1);

        // combine position and orientation error
        Vector5d e(e_position(0), e_position(1), e_position(2), e_psi, e_phi);

        std::cout << "Current Error: \n" << e <<"\n";

        // if pose is good enough: break
        /*double threshold_position = 0.005;
        double threshold_orientation = 0.005;
        if(std::abs(e(0)) < threshold_position && std::abs(e(1)) < threshold_position && std::abs(e(2)) < threshold_position && std::abs(e(3)) < threshold_orientation && std::abs(e(4)) < threshold_orientation){
            std::cout << "Error Threshold reached.\n";
            break;
        }*/

        // get the joint velocity through inverse kinematics
        Vector5d joint_velocity;
        //inverseKinematicsControl(current_position, end_position, current_quaternion, end_quaternion, joint_velocity);
        inverseKinematicsControl(e, desired_velocity, joint_velocity);


        //set the current joint velocity that was computed
        robot.setJointVel(joint_velocity);
        rclcpp::sleep_for(std::chrono::milliseconds(delta_t));
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