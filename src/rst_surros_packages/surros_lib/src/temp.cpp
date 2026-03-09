#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <surros_lib/kinematics.h>
#include <surros_lib/surros_interface.h>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("robot_node");
  auto joint_pub = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

  auto node2 = std::make_shared<rclcpp::Node>("your_node_name");
  auto surros_control = std::make_shared<robot::SurrosControl>(node2);
  //surros_control->initialize();


  sensor_msgs::msg::JointState joint_state;
  joint_state.header.stamp = node->now();
  joint_state.name = {"theta_1", "theta_2", "theta_3", "theta_4", "theta_5", "theta_6"};
  joint_state.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // example values

  joint_pub->publish(joint_state);

  rclcpp::Rate loop_rate(0.5);
  double temp = 0.0;


  robot::JointVector joints;
  robot::JointVector joints_calc;
  joints << -M_PI/4, M_PI/4, -1.0, M_PI/4, -M_PI/4, 0.0;  // example values in radians
  //joints << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;  // zero values in radians
  std::cout << "input joint angles:" << std::endl;
  std::cout << joints << std::endl;
  Eigen::Affine3d T;
  robot::KinematicModel kin_model;
  T = kin_model.computeForwardKinematics(joints,6);
  std::cout << "forw. kin:" << std::endl << T.matrix() << std::endl;
  bool success;
  success =  kin_model.computeInverseKinematics(T, joints_calc);
  std::cout << "inv. kin:" << std::endl << "success: " << success << std::endl << joints_calc << std::endl;

  std::vector<double> desired_xyz = {
    T.translation().x(),
    T.translation().y(),
    T.translation().z()
  };
  //Eigen::Vector3d desired_xyz = T.translation();
  //std::cout << desired_xyz << std::endl << std::endl;
  double psi = 0;
  double phi = 0;
  success =  kin_model.computeInverseKinematics(desired_xyz, psi, phi, joints_calc);
  std::cout << "Inverse Kinematik mal anders: " <<std::endl;
  std::cout << "inv. kin:" << std::endl << "success: " << success << std::endl << joints_calc << std::endl;


  // Testing the computation of the Jacobian Matrix
  robot::RobotJacobian jacobian;
  kin_model.computeJacobian(joints, jacobian);
  std::cout << "Jacobian Matrix: "<< std::endl;
  std::cout << jacobian << std::endl;

  while (rclcpp::ok())
  {
    joint_state.header.stamp = node->now();
    //joint_state.position[0] = temp;
    joint_state.position[0] = joints_calc[0];// + M_PI/2;
    joint_state.position[1] = joints_calc[1];// + 1.435;
    joint_state.position[2] = joints_calc[2];// + 0.029;
    joint_state.position[3] = joints_calc[3];// - M_PI/2;
    joint_state.position[4] = joints_calc[4];
    joint_state.position[5] = joints_calc[5];

    temp += 0.01;

    joint_pub->publish(joint_state);

     rclcpp::spin_some(node);
    loop_rate.sleep();

    joint_state.header.stamp = node->now();
    joint_state.position[0] = joints[0];// + M_PI/2;
    joint_state.position[1] = joints[1];// + 1.435;
    joint_state.position[2] = joints[2];// + 0.029;
    joint_state.position[3] = joints[3];// - M_PI/2;
    joint_state.position[4] = joints[4];
    joint_state.position[5] = joints[5];

    joint_pub->publish(joint_state);

    rclcpp::spin_some(node);
    loop_rate.sleep();
   
  }

  rclcpp::shutdown();
  return 0;
}
