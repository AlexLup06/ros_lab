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

// keep the angle between [-pi, pi]
// double angleLoop(double& angle){
//     while (angle > M_PI){
//         angle -= 2*M_PI;
//     }
//     while (angle < -M_PI){
//         angle += 2*M_PI;
//     }
//     return angle;   
// }

// Vector2d getQuatError(const Eigen::Matrix3d& error_rot){
//     Eigen::Quaterniond error_quat(error_rot);
//     std::cout << "Error Quat \n" << error_quat << "\n";
//     Vector2d error_2d;
//     if (error_quat.w()>=0){
//         error_2d = Vector2d(-std::abs(error_quat.x()), error_quat.z());
//     }
//     else{
//         error_2d = Vector2d(std::abs(error_quat.x()), -error_quat.z());
//     }
    
//     return error_2d;

//     // // get the error psi and phi
//     // Vector2d error_psi_phi = getPsiAndPhi(error_rot);
//     // // convert psi and phi to quaternions:
//     // // psi: rotation about y => [cos(psi/2), 0, sin(psi/2), 0] => use error: cos(psi/2)*sin(psi/2)
//     // // phi: rotation about x => [cos(phi/2), sin(phi/2), 0, 0] => use error: cos(phi/2)*sin(phi/2)
//     // return Vector2d(std::cos(error_psi_phi(0))*std::sin(error_psi_phi(0)), std::cos(error_psi_phi(1)));
// }

// Vector2d psiPhiToQuat(const double& psi, const double& phi){
//     Eigen::Affine3d pose = robot::createPoseFromPosAndPitch(Vector3d::Zero(), psi, phi);
//     Eigen::Quaterniond quat(pose.rotation());
//     return Vector2d(quat.x()*quat.w(), quat.z()*quat.w());
// }

// void limitJointVel(Vector5d& joint_velocity){
//     double upper_bound = 20.0;
//     double lower_bound = -20.0;
//     for(int i = 0; i <=4; i++){
//         if(joint_velocity(i) > upper_bound){
//             joint_velocity(i) = upper_bound;
//         }
//         else if(joint_velocity(i) < lower_bound){
//             joint_velocity(i) = lower_bound;
//         }
//     }
// }

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
Vector5d inverseKinematicsControl(const Vector5d& error, robot::SurrosControl& robot){
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

    return jacobian.inverse()*gain*error;
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

    // ------- methods I tried to calculate the orientation error -----
    //1: Quaternion error from rotation matrices
    // Eigen::Matrix3d error_rot = desired_pose.rotation()*current_pose.rotation();
    // Vector2d error_quat = getQuatError(error_rot);

    // 2: Quaternion error from quaternions
    // Eigen::Quaterniond current_quat(current_pose.rotation());
    // Eigen::Quaterniond desired_quat(desired_pose.rotation());
    // // std::cout << "Desired Quat: \n" << desired_quat << "\n";
    // // std::cout << "Current Quat: \n" << current_quat << "\n";
    // Eigen::Quaterniond error_quat = desired_quat*current_quat.inverse();

    // 3: Psi and Phi Error
    // Vector2d current_psi_phi = getPsiAndPhi(current_pose.rotation());
    // //std::cout << "Current Psi and Phi: \n" << current_psi_phi << "\n";
    // // double error_psi = current_psi_phi(0)-desired_psi;
    // // double error_phi = current_psi_phi(1)-desired_phi;
    // double error_psi = desired_psi-current_psi_phi(0);
    // double error_phi = desired_phi-current_psi_phi(1);
}

void publishError(const Vector5d& error, rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr& publisher){
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {100*error(0), 100*error(1), 100*error(2), error(3), error(4)};
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

    auto publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>("/control_error", 10);

    robot.setJointVel(Vector5d::Zero());

    // ----------------- Setting Poses, Fequencies, ... -----------------------
    // max time for the control loop in milliseconds
    double t_end = 20*1000; // 20s
    // time step until joint velocities are updated in milliseconds
    int delta_t = 20;

    /*
    Reachable (down) positions (for house):
    -0.03, 0.1, 0.04
    -0.03, 0.15, 0.04
    0.0, 0.18, 0.04
    0.03, 0.15, 0.04
    0.03, 0.1, 0.04
    */

    // setting the start pose
    robot.setEndeffectorPose(Vector3d(-0.03, 0.1, 0.0), 1.5, 0.0);
    rclcpp::sleep_for(std::chrono::seconds(2));

    // Get and Print the starting End Effector Pose
    Eigen::Affine3d eePose;
    robot.getEndeffectorState(eePose);
    std::cout << "Starting Translation: \n" << eePose.translation() << "\n";
    std::cout << "Starting Psi and Phi: \n" << getPsiAndPhi(eePose.rotation()) << "\n";
    
    // defining the desired pose
    Vector3d desired_position(-0.03, 0.15, 0.03);
    double desired_psi = 1.5;
    double desired_phi = 0.0;
    Eigen::Affine3d desired_pose = robot::createPoseFromPosAndPitch(desired_position, desired_psi, desired_phi);

    // -------------- Control Loop --------------------------------
    for(double t = 0.0; t < t_end; t+=delta_t){
        // get the current pose error between desired and actual pose
        Vector5d error = getPoseError(desired_pose, robot);
        //std::cout << "Current Pose Error: \n" << error << "\n";

        publishError(error, publisher);

        // check if the pose threshold was reached, if so stop the control loop
        double threshold_position = 0.005;
        double threshold_orientation = 0.05;
        if(std::abs(error(0)) < threshold_position && std::abs(error(1)) < threshold_position && std::abs(error(2)) < threshold_position && std::abs(error(3)) < threshold_orientation && std::abs(error(4)) < threshold_orientation){
            std::cout << "Pose Threshold Reached. \n";
            break;
        }
        
        // compute the joint velocities with the inverse kinematics control
        Vector5d joint_velocities = inverseKinematicsControl(error, robot);
        //std::cout << "Current Joint Velocities: \n" << joint_velocities << "\n";
        //limitJointVel(joint_velocities);

        // set the joint velocities
        robot.setJointVel(joint_velocities);
        // wait for delta_t befor updating to new joint velocities
        rclcpp::sleep_for(std::chrono::milliseconds(delta_t));
    }

    // stop the motion
    robot.setJointVel(Vector5d::Zero());

    // Get and Print the ending End Effector Pose
    robot.getEndeffectorState(eePose);
    std::cout << "Ending Translation: \n" << eePose.translation() << "\n";
    std::cout << "Ending Psi and Phi: \n" << getPsiAndPhi(eePose.rotation()) << "\n";

    std::cout << "Resulting Pose Error: \n" << getPoseError(desired_pose, robot) << "\n";
    

    return 0;
}
