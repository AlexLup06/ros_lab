#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <surros_lib/kinematics.h>
#include <surros_lib/surros_interface.h>
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"

using TrajectoryPoint5d = std::tuple<Eigen::VectorXd, Eigen::VectorXd, double>;

void task_1_5(std::shared_ptr<robot::SurrosControl>& testrobo)
{
    robot::JointVector q0, q1, q2;
    q0 << 1.0, 0.5, 0.5, 0.57, 0.5;  
    q1 << -1.0, 0.5, 1.3, 1.34, 0.5;
    q2 << 0.0, 0.0, 0.0, 0.0, 0.0;

    // configuration 1
    testrobo->setJoints(q0, rclcpp::Duration(std::chrono::seconds(5)));
    RCLCPP_INFO_STREAM(testrobo->get_logger(), "q0: " << std::endl << q0);
    RCLCPP_INFO_STREAM(testrobo->get_logger(), "Robot: " << std::endl << testrobo->getJointAngles());
    RCLCPP_INFO_STREAM(testrobo->get_logger(), "Error: " << std::endl << (testrobo->getJointAngles() - q0).norm());

    // configuration 2
    testrobo->setJoints(q1, rclcpp::Duration(std::chrono::seconds(5)));
    RCLCPP_INFO_STREAM(testrobo->get_logger(), "q1: " << std::endl << q1);
    RCLCPP_INFO_STREAM(testrobo->get_logger(), "Robot: " << std::endl << testrobo->getJointAngles());
    RCLCPP_INFO_STREAM(testrobo->get_logger(), "Error: " << std::endl << (testrobo->getJointAngles() - q1).norm());

    // configuration 3
    testrobo->setJoints(q2, rclcpp::Duration(std::chrono::seconds(5)));
    RCLCPP_INFO_STREAM(testrobo->get_logger(), "q2: " << std::endl << q2);
    RCLCPP_INFO_STREAM(testrobo->get_logger(), "Robot: " << std::endl << testrobo->getJointAngles());
    RCLCPP_INFO_STREAM(testrobo->get_logger(), "Error: " << std::endl << (testrobo->getJointAngles() - q2).norm());
}

void task_2_1(std::shared_ptr<robot::SurrosControl>& testrobo)
{
    // define some positions
    Eigen::Vector3d t0, t1, t2;
    t0 << 0.15, 0.2, 0.05;
    t1 << 0.1, -0.1, 0.02;
    t2 << 0, 0, 0.384;

    // define some pitches
    double psi0 = 0; //-M_PI_2;    
    double psi1 = M_PI_2;
    double psi2 = -M_PI_2;

    // define some pitches
    double phi0 = 0.0;
    double phi1 = 0.0;
    double phi2 = 0.0;

    // memory for current pose
    Eigen::Affine3d T;
    Eigen::Vector3d euler;

    // pose 1
    testrobo->setEndeffectorPose(t0, psi0, phi0, rclcpp::Duration(std::chrono::milliseconds(500)));
    testrobo->setGripperJoint(50.0);
    testrobo->getEndeffectorState(T);
    euler = T.rotation().eulerAngles(2, 1, 0);

    RCLCPP_INFO_STREAM(testrobo->get_logger(), "Pose: " << std::endl << t0 << std::endl << std::endl << psi0);
    RCLCPP_INFO_STREAM(testrobo->get_logger(), "Robot: " << std::endl << T.translation() << std::endl << std::endl << euler(1));
    RCLCPP_INFO_STREAM(testrobo->get_logger(), "Position Error: " << std::endl << (t0 - T.translation()).norm());
    RCLCPP_INFO_STREAM(testrobo->get_logger(), "Orientation Error: " << std::endl << fabs(psi0-euler(1)));


    // // pose 2
    //testrobo->setEndeffectorPose(t1, psi1, phi1, rclcpp::Duration(std::chrono::seconds(5)));
    // testrobo->getEndeffectorState(T);
    // euler = T.rotation().eulerAngles(2, 1, 0);

    // RCLCPP_INFO_STREAM(testrobo->get_logger(), "Pose: " << std::endl << t1 << std::endl << std::endl << psi1);
    // RCLCPP_INFO_STREAM(testrobo->get_logger(), "Robot: " << std::endl << T.translation() << std::endl << std::endl << euler(1));
    // RCLCPP_INFO_STREAM(testrobo->get_logger(), "Position Error: " << std::endl << (t1 - T.translation()).norm());
    // RCLCPP_INFO_STREAM(testrobo->get_logger(), "Orientation Error: " << std::endl << fabs(psi1-euler(1)));


    // // pose 3
    // testrobo->setEndeffectorPose(t2, psi2, phi2, rclcpp::Duration(std::chrono::seconds(5)));
    // testrobo->getEndeffectorState(T);
    // euler = T.rotation().eulerAngles(2, 1, 0);

    // RCLCPP_INFO_STREAM(testrobo->get_logger(), "Pose: " << std::endl << t2 << std::endl << std::endl << psi2);
    // RCLCPP_INFO_STREAM(testrobo->get_logger(), "Robot: " << std::endl << T.translation() << std::endl << std::endl << euler(1));
    // RCLCPP_INFO_STREAM(testrobo->get_logger(), "Position Error: " << std::endl << (t2 - T.translation()).norm());
    // RCLCPP_INFO_STREAM(testrobo->get_logger(), "Orientation Error: " << std::endl << fabs(psi2-euler(1)));
}

void correct_initial_offset(std::shared_ptr<robot::SurrosControl>& robot)
{
    // correct initial offset
    Eigen::Affine3d T_ee;
    Eigen::Affine3d T_d;
    Eigen::VectorXd vel(5);
    Eigen::VectorXd vel0(5);
    Eigen::VectorXd e(5);
    Eigen::MatrixXd K(5, 5);
    robot::RobotJacobianReduced J;
    rclcpp::Rate loop_rate(100);
    rclcpp::Rate pause_rate(5);
    // after switching to velocity control, start with 0 velocities:
    vel0 << 0.0, 0.0, 0.0, 0.0, 0.0;
    vel << 0.0, 0.0, 0.0, 0.0, 0.1;
    K.setIdentity();
    K *= 0.1;  // gain for the diff kinematics control
    
    pause_rate.sleep();
    robot->getEndeffectorState(T_d);
    robot->setJointVel(vel);
    pause_rate.sleep();
    robot->setJointVel(vel0);
    pause_rate.sleep();
    // get the current endeffector pose:
    robot->getEndeffectorState(T_ee);
    // calculate initial position error:
    e.head<3>() = T_d.translation() - T_ee.translation();
    e(3) = 0.0;
    e(4) = 0.0;
    RCLCPP_INFO_STREAM(robot->get_logger(), "Initial Offset: " << std::endl << e.transpose());
    
    while (rclcpp::ok() && e.norm() > 1e-3)
    {
      // perform diff kinematics control:
      robot->getEndeffectorState(T_ee);
      e.head<3>() = T_d.translation() - T_ee.translation();
      robot->getJacobianReduced(J);
      vel = J.inverse() * e;
      robot->setJointVel(vel);
      loop_rate.sleep();
    }

    vel(0) = 0.0;
    vel(1) = 0.0;
    vel(2) = 0.0;
    vel(3) = 0.0;
    vel(4) = 0.0;
    robot->setJointVel(vel);
    RCLCPP_INFO_STREAM(robot->get_logger(), "Final Offset: " << std::endl << e.transpose());
    pause_rate.sleep();
}

void generateHorizontalLineTrajectory5D(
    const Eigen::Affine3d& start_pose,
    std::vector<TrajectoryPoint5d>& trajectory)
{
    const int N = 100;                   // number of points
    const double total_time = 20.0;      // seconds
    const double dt = total_time / (N-1);
    const double length = 0.05;          // 5 cm

    trajectory.clear();
    trajectory.resize(N);

    // direction of motion = x-axis of start_pose (horizontal line)
    Eigen::Vector3d direction = start_pose.rotation().col(0).normalized();
    Eigen::Vector3d start_pos = start_pose.translation();

    // extract orientation as yaw (psi) and pitch (phi)
    double psi = 0; //std::atan2(start_pose.rotation()(1,0), start_pose.rotation()(0,0));
    double phi = 0; //std::atan2(-start_pose.rotation()(2,0),
                            //std::sqrt(std::pow(start_pose.rotation()(2,1),2) + std::pow(start_pose.rotation()(2,2),2)));

    for (int i = 0; i < N; ++i)
    {
        double alpha = static_cast<double>(i) / (N-1);  // 0 → 1
        Eigen::Vector3d pos = start_pos + alpha * length * direction;

        Eigen::VectorXd pose(5);
        pose << pos.x(), pos.y(), pos.z(), psi, phi;

        Eigen::VectorXd vel = Eigen::VectorXd::Zero(5);
        if (i > 0) {
            Eigen::Vector3d prev_pos = start_pos + ((i-1.0)/(N-1)) * length * direction;
            Eigen::Vector3d diff = (pos - prev_pos) / dt;
            vel.head<3>() = diff;
            vel(3) = 0.0;  // psi velocity
            vel(4) = 0.0;  // phi velocity
        }

        trajectory[i] = std::make_tuple(pose, vel, i * dt);
    }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node_ = std::make_shared<rclcpp::Node>("software_node");
  auto surros_control = std::make_shared<robot::SurrosControl>(node_);
  Eigen::Affine3d endeffector_state;
  rclcpp::Rate wait_rate(10);
  rclcpp::Time start = node_->get_clock()->now();
  Eigen::VectorXd vel(5);
  Eigen::VectorXd vel0(5);
  Eigen::Affine3d T_ee;
  Eigen::Affine3d T_d;
  Eigen::VectorXd e(5);
  Eigen::MatrixXd K(5, 5);
  K.setIdentity();
  K *= 0.075;  // gain for the diff kinematics control
  robot::RobotJacobianReduced J;
  Eigen::Vector3d rpy_d;
  Eigen::Vector3d rpy_e;
  std::vector<TrajectoryPoint5d> trajectory;
  bool traj_gen = false;

  // Initialize the SurrosControl instance
  surros_control->initialize();

  // set initial velocity
  vel << 0.0, 0.0, 0.0, 0.0, 0.0;  // each joint has a different velocity
  vel0 << 0.05, 0.01, 0.2, 0.2, 0.0;
  // get the current joint angles and gripper state  

  rclcpp::Rate loop_rate(100);
  start = node_->get_clock()->now();
  surros_control->getEndeffectorState(T_d);

  auto traj_pub = node_->create_publisher<visualization_msgs::msg::Marker>(
    "target_trajectory_marker", rclcpp::QoS(10));
  // Create and fill marker once
  visualization_msgs::msg::Marker traj_marker;
  traj_marker.header.frame_id = "base_link";
  traj_marker.ns = "trajectory";
  traj_marker.id = 0;
  traj_marker.type = visualization_msgs::msg::Marker::LINE_STRIP; // use POINTS for discrete points
  traj_marker.action = visualization_msgs::msg::Marker::ADD;
  traj_marker.scale.x = 0.002; // line width in meters
  traj_marker.color.r = 1.0;
  traj_marker.color.g = 0.0;
  traj_marker.color.b = 0.0;
  traj_marker.color.a = 1.0;

  while (rclcpp::ok())
  {
    double elapsed = (node_->get_clock()->now() - start).seconds();

    // follow line trajectory:
    if (elapsed > 10.0) {
      if (!traj_gen) {
        surros_control->setJointVel(vel0);
        surros_control->getEndeffectorState(T_d); 
        generateHorizontalLineTrajectory5D(T_d, trajectory);
        traj_marker.points.resize(trajectory.size());
        for (size_t i = 0; i < trajectory.size(); ++i) {
            Eigen::VectorXd pose_d = std::get<0>(trajectory[i]);
            traj_marker.points[i].x = pose_d(0);
            traj_marker.points[i].y = pose_d(1);
            traj_marker.points[i].z = pose_d(2);
        }
        traj_gen = true;
      }

      // publish trajectory
      traj_marker.header.stamp = node_->get_clock()->now();
      traj_pub->publish(traj_marker);

      // find current target:
      TrajectoryPoint5d target;
      if (!trajectory.empty()) {
        // Clamp index to trajectory size
        int idx = static_cast<int>((elapsed - 10.0) / (15.0 / (trajectory.size()-1))); 
        if (idx >= static_cast<int>(trajectory.size()))
            idx = trajectory.size() - 1;
        target = trajectory[idx];
      }
      Eigen::VectorXd pose_d = std::get<0>(target); // [x, y, z, psi, phi]
      Eigen::VectorXd vel_d  = std::get<1>(target); // [vx, vy, vz, v_psi, v_phi]

      T_d = robot::createPoseFromPosAndPitch(pose_d.head<3>(), pose_d(3), pose_d(4));
      RCLCPP_INFO_STREAM(surros_control->get_logger(), "Target Pose: " << std::endl << T_d.translation() << std::endl << T_d.rotation());

      // perform diff kinematics control:
      surros_control->getEndeffectorState(T_ee);
      e.head<3>() = T_d.translation() - T_ee.translation();
      rpy_d = T_d.rotation().eulerAngles(0, 1, 2);  // roll-pitch-yaw (X-Y-Z)
      rpy_e = T_ee.rotation().eulerAngles(0, 1, 2);
      e(3) = 0 ;//rpy_d(0) - rpy_e(0); //roll error
      e(4) = 0; //rpy_d(2) - rpy_e(2); //yaw error
      surros_control->getJacobianReduced(J);
      vel = J.inverse() * K * e;
      RCLCPP_INFO_STREAM(surros_control->get_logger(), "Velocity Command: " << vel.transpose());
      surros_control->setJointVel(vel);
    }
    // drive into start position
    else if (elapsed > 6.0) {
      surros_control->setJointVel(vel);       
    }
    // change control mode and wait in home position:
    else {
      surros_control->setJointVel(vel0);
    }

    rclcpp::spin_some(node_);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
