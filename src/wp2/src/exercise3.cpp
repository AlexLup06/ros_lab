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
    auto node = rclcpp::Node::make_shared("exercise3");
    SurrosControl control = SurrosControl(node);

    // initialize the control
    control.initialize();
    

    //function given a specific pose executes it using the inverse kinematics method and measures the final executed pose
    auto pickAndPlace = [&] (Eigen::Vector3d& a,
            Eigen::Vector3d& b,
            double hover_dist = 0.05,
            int open_percentage = 10,
            int close_percentage = 80,
            const std::chrono::seconds duration = std::chrono::seconds(5)){

        // move to the default position
        control.setJointsDefault();

        // create positionns the hover above the targets
        Eigen::Vector3d a_above(a(0),a(1),a(2)+hover_dist);
        Eigen::Vector3d b_above(b(0),b(1),b(2)+hover_dist);

        // orientation towards the ground
        //double psi = -M_PI/2.0;
        double psi = 1.5;
        double phi = 0.0;
        
        // move the gripper to hover above A
        control.setEndeffectorPose(a_above, psi, phi, rclcpp::Duration(duration));

        std::cout << "Above a \n";

        // add buffer time
        rclcpp::sleep_for(std::chrono::milliseconds(500));

        // open the gripper fully
        control.setGripperJoint(open_percentage);
        std::cout << "Gripper open \n";

        // add buffer time
        rclcpp::sleep_for(std::chrono::milliseconds(500));

        // lowering the gripper
        control.setEndeffectorPose(a, psi, phi, rclcpp::Duration(std::chrono::seconds(1)));

        std::cout << "A \n";

        // add buffer time
        rclcpp::sleep_for(std::chrono::milliseconds(500));

        // close the gripper to grab the block
        control.setGripperJoint(close_percentage);
        std::cout << "Gripper close \n";

        // add buffer time
        rclcpp::sleep_for(std::chrono::milliseconds(500));

        // lift the block
        control.setEndeffectorPose(a_above, psi, phi, rclcpp::Duration(std::chrono::seconds(1)));
        std::cout << "Above a \n";

        // add buffer time
        rclcpp::sleep_for(std::chrono::milliseconds(500));

        // move the block above B
        control.setEndeffectorPose(b_above, psi, phi, rclcpp::Duration(duration));
        std::cout << "Above b \n";

        // add buffer time
        rclcpp::sleep_for(std::chrono::milliseconds(500));

        // lower the block
        control.setEndeffectorPose(b, psi, phi, rclcpp::Duration(std::chrono::seconds(1)));
        std::cout << "B \n";

        // add buffer time
        rclcpp::sleep_for(std::chrono::milliseconds(500));

        // open the gripper fully to release
        control.setGripperJoint(open_percentage);
        std::cout << "Gripper open \n";

        // add buffer time
        rclcpp::sleep_for(std::chrono::milliseconds(500));

        // lift the gripper
        control.setEndeffectorPose(b_above, psi, phi, rclcpp::Duration(std::chrono::seconds(1)));
        std::cout << "Above b \n";

        // add buffer time
        rclcpp::sleep_for(std::chrono::milliseconds(500));

        // move to the default position
        control.setJointsDefault();
        // close the gripper to grab the block
        control.setGripperJoint(close_percentage);
    };

    Eigen::Vector3d a(0.15, 0.0, 0.00);
    Eigen::Vector3d b(0.0, 0.15, 0.00);
    
    while(true){
        pickAndPlace(a, b);
        pickAndPlace(b, a);
    }
    
}
