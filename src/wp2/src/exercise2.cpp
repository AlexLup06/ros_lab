#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <surros_lib/kinematics.h>
#include <surros_lib/surros_interface.h>
#include <Eigen/Dense>

using robot::SurrosControl;

// Convert a rotation matrix to the angles Psi (return(0)) and Phi (return(1))
Eigen::Vector2d getPsiAndPhi(const Eigen::Matrix3d& rot){
    // change the orientation matrix to the base axis: z->x; x->y; y->z mapping in the world frame
    // if all euler angles are 0, the end effector coordinate system matches the base coordinate system
    Eigen::Matrix3d rotMat;
    rotMat.col(0) << rot.col(2);
    rotMat.col(1) << rot.col(0);
    rotMat.col(2) << rot.col(1);
    // get the ZYX euler angles from this rotation matrix ==> psi: rotation about y, phi: rotation about x
    Eigen::Vector3d euler = rotMat.eulerAngles(2,1,0);
    double psi = euler(1);
    double phi = euler(2);
    return Eigen::Vector2d(psi, phi);
}

int main(int argc, char **argv)
{
    // create the node and surros control to control the surros robot
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("exercise2");
    SurrosControl control = SurrosControl(node);

    // initialize the control
    control.initialize();

    //function given a specific pose executes it using the inverse kinematics method and measures the final executed pose
    auto setAndGetPose = [&] (double x = 0.0,
            double y = 0.0,
            double z = 0.0,
            double psi = 0.0,
            double phi = 0.0,
            const std::chrono::seconds duration = std::chrono::seconds(5)){

        // create the position vector
        Eigen::Vector3d desired_xyz(x,y,z);

        
        // publish the pose
        control.setEndeffectorPose(desired_xyz, psi, phi, rclcpp::Duration(duration));

        // wait for the robot to finish moving
        //rclcpp::sleep_for(duration);
        rclcpp::sleep_for(std::chrono::milliseconds(500));

        // get the robots rotation matrix and translation vector
        Eigen::Affine3d measured_pose;
        control.getEndeffectorState(measured_pose);

        // print the measured pose
        std::cout << "Measured End effector Position \n";
        std::cout << measured_pose.translation() << "\n";
        std::cout << "Ending Psi and Phi: \n" << getPsiAndPhi(measured_pose.rotation()) << "\n";

    };

    
    
    
    setAndGetPose(0.15, 0.05, 0.055, 1.5, 0.0);
    setAndGetPose(0.15, 0.05, 0.055, 1.0, 0.0);
    setAndGetPose(0.15, 0.05, 0.055, 1.2, 0.0);
    //setAndGetPose(0.25, 0.00, 0.05, 0.5, -0.5);
    //setAndGetPose(0.01, 0.25, 0.05, 0.00, 0.00);
    //setAndGetPose(0.00, 0.00, 0.3, 0.00, 0.00);
}
