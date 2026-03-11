#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <surros_lib/kinematics.h>
#include <surros_lib/surros_interface.h>
#include <Eigen/Dense>

using robot::SurrosControl;
//using JointVector = Eigen::Matrix<double, 5, 1>;

int main(int argc, char **argv)
{
    
    // create the node and surros control to control the surros robot
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("exercise1");
    SurrosControl control = SurrosControl(node);

    // initialize the control
    control.initialize();
    

    auto setAndGetJoints = [&] (double theta_1 = 0.0,
            double theta_2 = 0.0,
            double theta_3 = 0.0,
            double theta_4 = 0.0,
            double theta_5 = 0.0,
            const std::chrono::seconds duration = std::chrono::seconds(10)){
        

        // create the vector of joint angles
        robot::JointVector joint_angles = {theta_1, theta_2, theta_3, theta_4, theta_5};


        // checking for joint limits
        if(control.isExceedingJointLimits(joint_angles)){
            std::cout << "Joint Vector is exceeding joint limits";
            return;
        }
        // publish the joint angles
        control.setJoints(joint_angles, rclcpp::Duration(duration));

        // wait for the robot to finish moving
        rclcpp::sleep_for(duration);
        rclcpp::sleep_for(std::chrono::seconds(2));

        // get the robots joint angles
        std::vector<double> measured_joint_angles;

        control.getJointAngles(measured_joint_angles);

        // print the measured joint angles
        std::cout << "Measured Joint Angles \n";
        for(double angle: measured_joint_angles){
            std::cout << angle << "\n";
        }
    };

    setAndGetJoints(1.0, 1.0, 1.0, 1.0, 1.0);
    setAndGetJoints(0.0, 1.0, 0.0, 1.0, 0.0);
    setAndGetJoints(-1.0, 1.0, -1.0, 1.0, -1.0);
}
