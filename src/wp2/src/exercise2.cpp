#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <surros_lib/kinematics.h>
#include <surros_lib/surros_interface.h>
#include <Eigen/Dense>

using robot::SurrosControl;

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
        rclcpp::sleep_for(std::chrono::seconds(2));

        // get the robots rotation matrix and translation vector
        Eigen::Affine3d measured_pose;
        control.getEndeffectorState(measured_pose);

        // print the measured pose
        std::cout << "Measured End effector Position \n";
        std::cout << measured_pose.translation() << "\n";
        std::cout << "Measured End effector Rotation \n";
        std::cout << measured_pose.rotation() << "\n";

    };

    
    
    
    setAndGetPose(0.25, 0.00, 0.05, -0.5, 0.5);
    setAndGetPose(0.25, 0.00, 0.05, 0.5, -0.5);
    setAndGetPose(0.01, 0.25, 0.05, 0.00, 0.00);
    //setAndGetPose(0.00, 0.00, 0.3, 0.00, 0.00);
}
