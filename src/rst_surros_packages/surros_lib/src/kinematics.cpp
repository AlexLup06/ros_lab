#include <surros_lib/kinematics.h>

namespace robot {

void KinematicModel::getDHMatrix(const KinematicModel::DHParams& dh, Eigen::Affine3d& T) const
{
    // Precompute sin/cos
    double ct = std::cos(dh.theta);
    double st = std::sin(dh.theta);
    double ca = std::cos(dh.alpha);
    double sa = std::sin(dh.alpha);

    Eigen::Matrix4d mat;
    mat << ct,   -st * ca,  st * sa, dh.a * ct,
           st,    ct * ca, -ct * sa, dh.a * st,
           0.0,   sa,       ca,      dh.d,
           0.0,   0.0,      0.0,     1.0;

    T.matrix() = mat;
}


Eigen::Affine3d KinematicModel::computeForwardKinematics(const Eigen::Ref<const JointVector>& joint_values, int up_to_index) const
{

    Eigen::Affine3d transform = Eigen::Affine3d::Identity();

    if (up_to_index == 0)  // base
        return transform;

    // first joint
    Eigen::Affine3d T_1_to_0 = Eigen::Affine3d::Identity();
    // struct DHParams
    // {
    //     double theta;
    //     double alpha;
    //     double a;
    //     double d;
    // }
    DHParams dh1 = {M_PI/2 + joint_values[0], -M_PI/2, 0.0, 0.0563};
    getDHMatrix(dh1, T_1_to_0);
    transform = transform * T_1_to_0;

    if (up_to_index == 1)  // base to joint 1 only
        return transform;

    // second joint
    Eigen::Affine3d T_2_to_1 = Eigen::Affine3d::Identity();
    DHParams dh2 = {-1.435 + joint_values[1], 0.0, 0.109, 0.0};
    //DHParams dh2 = {-1.435 + joint_values[1], 0.0, -0.109, 0.0148};
    getDHMatrix(dh2, T_2_to_1);
    transform = transform * T_2_to_1;

    if (up_to_index == 2)  // base to joint 2 only
        return transform;

    // third joint
    Eigen::Affine3d T_3_to_2 = Eigen::Affine3d::Identity();
    DHParams dh3 = {-0.029 + joint_values[2], 0.0, 0.101, 0.0};
    getDHMatrix(dh3, T_3_to_2);
    transform = transform * T_3_to_2;

    if (up_to_index == 3)  // base to joint 3 only
        return transform;

    // 4. joint
    Eigen::Affine3d T_4_to_3 = Eigen::Affine3d::Identity();
    DHParams dh4 = {M_PI/2 + joint_values[3], M_PI/2, 0.000007, -0.001};
    getDHMatrix(dh4, T_4_to_3);
    transform = transform * T_4_to_3;

    if (up_to_index == 4)  // base to joint 4 only
        return transform;
    
    // 5. joint
    Eigen::Affine3d T_5_to_4 = Eigen::Affine3d::Identity();
    DHParams dh5 = {joint_values[4], 0.0, 0.0, 0.0};
    getDHMatrix(dh5, T_5_to_4);
    transform = transform * T_5_to_4;

    if (up_to_index == 5)  // base to joint 5 only
        return transform;

    // tcp
    Eigen::Affine3d T_6_to_5 = Eigen::Affine3d::Identity();
    DHParams dh6 = {M_PI/2, 0.0, -0.007, 0.114};
    getDHMatrix(dh6, T_6_to_5);
    transform = transform * T_6_to_5;
    return transform;
}



void KinematicModel::computeJacobian(const Eigen::Ref<const JointVector>& joint_values, RobotJacobian& jacobian, std::shared_ptr<tf2_ros::Buffer> _tf_buffer) const
{
    // Consider only revolute joints
    // translational part: z_{i-1} x p{i-1,E}  (p denotes the distance vector from
    // joint i-1 to the endeffector) rotational part: z_{i-1} Since our model
    // specifies the y-axis to be the axis of rotation, we have to substitute
    // z_{i-1} by y_{i-1}. Therefore our second column of the rotation matrix
    // represents the unit vector of the axis of rotation in our base frame.

    // In the Koch-Robot model, we are using the z-axis as rotation axis again
    // In addition to that, the robot uses 5 joints instead of 4. 
    // This means, that there has to be another column in the jacobian matrix.

    // Eigen::Affine3d tcp_frame = computeForwardKinematics(joint_values, 6);
    geometry_msgs::msg::TransformStamped tf_transform;
    Eigen::Affine3d tcp_frame;
    tf_transform = _tf_buffer->lookupTransform(
            "base_link", "tcp", rclcpp::Time(0), rclcpp::Duration::from_seconds(10.0));
    tcp_frame = tf2::transformToEigen(tf_transform);

    // joint 1
    // Eigen::Affine3d prev_frame = computeForwardKinematics(joint_values, 1);
    Eigen::Affine3d prev_frame;
    tf_transform = _tf_buffer->lookupTransform(
            "base_link", "link_1", rclcpp::Time(0), rclcpp::Duration::from_seconds(10.0));
    prev_frame = tf2::transformToEigen(tf_transform);

    jacobian.block(0, 0, 3, 1) = prev_frame.rotation().col(2).cross(tcp_frame.translation() - prev_frame.translation());  // position part joint 1
    jacobian.block(3, 0, 3, 1) = prev_frame.rotation().col(2);     
    

    // joint 2
    // prev_frame                 = computeForwardKinematics(joint_values, 2);
    tf_transform = _tf_buffer->lookupTransform(
            "base_link", "link_2", rclcpp::Time(0), rclcpp::Duration::from_seconds(10.0));
    prev_frame = tf2::transformToEigen(tf_transform);

    jacobian.block(0, 1, 3, 1) = prev_frame.rotation().col(2).cross(tcp_frame.translation() - prev_frame.translation());  // position part joint 2
    jacobian.block(3, 1, 3, 1) = prev_frame.rotation().col(2);     


    // joint 3
    // prev_frame                 = computeForwardKinematics(joint_values, 3);
    tf_transform = _tf_buffer->lookupTransform(
            "base_link", "link_3", rclcpp::Time(0), rclcpp::Duration::from_seconds(10.0));
    prev_frame = tf2::transformToEigen(tf_transform);

    jacobian.block(0, 2, 3, 1) = prev_frame.rotation().col(2).cross(tcp_frame.translation() - prev_frame.translation());  // position part joint 3
    jacobian.block(3, 2, 3, 1) = prev_frame.rotation().col(2);  

    // joint 4
    //prev_frame                 = computeForwardKinematics(joint_values, 4);
    tf_transform = _tf_buffer->lookupTransform(
            "base_link", "link_4", rclcpp::Time(0), rclcpp::Duration::from_seconds(10.0));
    prev_frame = tf2::transformToEigen(tf_transform);

    jacobian.block(0, 3, 3, 1) = prev_frame.rotation().col(2).cross(tcp_frame.translation() - prev_frame.translation());  // position part joint 4
    jacobian.block(3, 3, 3, 1) = prev_frame.rotation().col(2);     
    
    // std::stringstream ss_for;
    // ss_for << prev_frame.matrix();
    // RCLCPP_INFO_STREAM(rclcpp::get_logger("KinematicModel"), "T from inv. kin.:\n" << ss_for.str());
    // joint 5
    // prev_frame                 = computeForwardKinematics(joint_values, 5);
    tf_transform = _tf_buffer->lookupTransform(
            "base_link", "link_5", rclcpp::Time(0), rclcpp::Duration::from_seconds(10.0));
    prev_frame = tf2::transformToEigen(tf_transform);

    jacobian.block(0, 4, 3, 1) = prev_frame.rotation().col(2).cross(tcp_frame.translation() - prev_frame.translation());  // position part joint 5
    jacobian.block(3, 4, 3, 1) = prev_frame.rotation().col(2);   
}

void KinematicModel::computeJacobianReduced(const Eigen::Ref<const JointVector>& joint_values, RobotJacobianReduced& jacobian4d, std::shared_ptr<tf2_ros::Buffer> _tf_buffer) const
{
    // #TODO: Check if this is still correct after the change to 5 joints
    RobotJacobian jac_full; // I added a column to this type as we got another joint
    computeJacobian(joint_values, jac_full, _tf_buffer);
    jacobian4d.topRows<3>()   = jac_full.topRows<3>();
    //jacobian4d.bottomRows<2>()   = jac_full.bottomRows<2>();
    //changed to second to last rows
    jacobian4d.row(3) = jac_full.row(3);
    jacobian4d.row(4) = jac_full.row(4);
    // jacobian4d.coeffRef(3, 0) = 0.0;
    // jacobian4d.bottomRightCorner<1, 3>().setConstant(1.0);
}

bool KinematicModel::computeInverseKinematics(const Eigen::Affine3d& desired_pose, Eigen::Ref<JointVector> joint_values) const
{
    // Extract rotation and translation from desired pose
    Eigen::Matrix3d R_tcp = desired_pose.rotation();
    Eigen::Vector3d o_tcp = desired_pose.translation();

    // Compute wrist center (o_3)
    Eigen::Vector3d o_3 = o_tcp 
        - 0.114 * R_tcp * Eigen::Vector3d::UnitZ() 
        + 0.007 * R_tcp * Eigen::Vector3d::UnitX();
    double x_o3 = o_3(0);
    double y_o3 = o_3(1);
    double z_o3 = o_3(2);

    // calculate helper variables
    double r = std::sqrt(x_o3*x_o3 + y_o3*y_o3);
    double s = z_o3 - 0.0563;
    double A2 = 0.1093;
    double A3 = 0.100512;
    double W = (r*r + s*s - A2*A2 - A3*A3) / (2.0 * A2 * A3);

    if (W < -1.0 || W > 1.0) {
        // acos input invalid: point unreachable
        return false;
    }

    double theta1 = 0, theta2 = 0, theta3 = 0, theta4 = 0, theta5 = 0;
    double c1, s1, c23, s23, X, Y;
    Eigen::Affine3d T_temp;
    robot::JointVector joints_temp;
    Eigen::Vector3d y_temp, y_desired;
    double cos_theta5;

    // ##########################################################################################################
    // ###                                     candidate set 1                                                ###
    // ##########################################################################################################
    theta1 = std::atan2(y_o3, x_o3);
    theta3 = std::atan2(std::sqrt(1 - W*W), W);
    theta2 = - (std::atan2(s, r) - 
                       std::atan2(A3 * std::sin(-theta3),
                                  A2 + A3 * std::cos(-theta3)));

    c1 = std::cos(theta1);
    s1 = std::sin(theta1);
    c23 = std::cos(theta2 + theta3);
    s23 = std::sin(theta2 + theta3);
    Y = c1 * c23 * R_tcp(0,2) + s1 * c23 * R_tcp(1,2) - s23 * R_tcp(2,2);
    X = c1 * s23 * R_tcp(0,2) + s1 * s23 * R_tcp(1,2) + c23 * R_tcp(2,2);

    theta4 = std::atan2(Y, X);

    // Forward kinematics to estimate frame before joint 5
    joints_temp << theta1 - M_PI/2, theta2 + 1.435, theta3 + 0.029, theta4 - M_PI/2, 0.0;
    T_temp = computeForwardKinematics(joints_temp, 4);
    // Compute joint 5 angle using dot product between frame axes
    y_temp = T_temp.matrix().block<3,1>(0,1);
    y_desired = desired_pose.matrix().block<3,1>(0,1);
    cos_theta5 = y_temp.dot(y_desired);
    if (cos_theta5 < -1.0 || cos_theta5 > 1.0) {
        return false;
    }
    theta5 = std::acos(cos_theta5) - M_PI/2;

    // Assign results to joint_values_candidate1
    robot::JointVector joints_values_candidate1;
    joints_values_candidate1 << theta1 - M_PI/2, theta2 + 1.435, theta3 + 0.029, theta4 - M_PI/2, theta5;
    bool success_cand1        = isInsideInterval(_joint_lower_bounds[0], joints_values_candidate1[0], _joint_upper_bounds[0]) &&
                                isInsideInterval(_joint_lower_bounds[1], joints_values_candidate1[1], _joint_upper_bounds[1]) &&
                                isInsideInterval(_joint_lower_bounds[2], joints_values_candidate1[2], _joint_upper_bounds[2]) &&
                                isInsideInterval(_joint_lower_bounds[3], joints_values_candidate1[3], _joint_upper_bounds[3]) &&
                                isInsideInterval(_joint_lower_bounds[4], joints_values_candidate1[4], _joint_upper_bounds[4]);

    // ##########################################################################################################
    // ###                                     candidate set 2                                                ###
    // ##########################################################################################################
    theta1 = std::atan2(y_o3, x_o3);
    theta3 = std::atan2(-std::sqrt(1 - W*W), W);
    theta2 = - (std::atan2(s, r) - 
                       std::atan2(A3 * std::sin(-theta3),
                                  A2 + A3 * std::cos(-theta3)));

    c1 = std::cos(theta1);
    s1 = std::sin(theta1);
    c23 = std::cos(theta2 + theta3);
    s23 = std::sin(theta2 + theta3);
    Y = c1 * c23 * R_tcp(0,2) + s1 * c23 * R_tcp(1,2) - s23 * R_tcp(2,2);
    X = c1 * s23 * R_tcp(0,2) + s1 * s23 * R_tcp(1,2) + c23 * R_tcp(2,2);

    theta4 = std::atan2(Y, X);

    // Forward kinematics to estimate frame before joint 5
    joints_temp << theta1 - M_PI/2, theta2 + 1.435, theta3 + 0.029, theta4 - M_PI/2, 0.0;
    T_temp = computeForwardKinematics(joints_temp, 4);
    // Compute joint 5 angle using dot product between frame axes
    y_temp = T_temp.matrix().block<3,1>(0,1);
    y_desired = desired_pose.matrix().block<3,1>(0,1);
    cos_theta5 = y_temp.dot(y_desired);
    if (cos_theta5 < -1.0 || cos_theta5 > 1.0) {
        return false;
    }
    theta5 = std::acos(cos_theta5) - M_PI/2;
    // Assign results to joint_values_candidate1
    robot::JointVector joints_values_candidate2;
    joints_values_candidate2 << theta1 - M_PI/2, theta2 + 1.435, theta3 + 0.029, theta4 - M_PI/2, theta5;
    bool success_cand2        = isInsideInterval(_joint_lower_bounds[0], joints_values_candidate2[0], _joint_upper_bounds[0]) &&
                                isInsideInterval(_joint_lower_bounds[1], joints_values_candidate2[1], _joint_upper_bounds[1]) &&
                                isInsideInterval(_joint_lower_bounds[2], joints_values_candidate2[2], _joint_upper_bounds[2]) &&
                                isInsideInterval(_joint_lower_bounds[3], joints_values_candidate2[3], _joint_upper_bounds[3]) &&
                                isInsideInterval(_joint_lower_bounds[4], joints_values_candidate2[4], _joint_upper_bounds[4]);

    // ##########################################################################################################
    // ###                                     select candidate                                               ###
    // ##########################################################################################################
    if (success_cand1 && success_cand2)
    {
        // check forwards kinematics
        Eigen::Affine3d fwd1 = computeForwardKinematics(joints_values_candidate1);
        Eigen::Affine3d fwd2 = computeForwardKinematics(joints_values_candidate2);
        Eigen::Quaterniond quat_desired(desired_pose.rotation());
        Eigen::Quaterniond quat_cand1(fwd1.rotation());
        Eigen::Quaterniond quat_cand2(fwd2.rotation());
        // see http://math.stackexchange.com/questions/90081/quaternion-distance
        double angle1 = 1 - pow(quat_desired.dot(quat_cand1),
                                2);  // normalize_angle_rad(acos(quat_cand1.angularDistance(quat_desired)));
        double angle2 = 1 - pow(quat_desired.dot(quat_cand2),
                                2);  // normalize_angle_rad(acos(quat_cand2.angularDistance(quat_desired)));
        if (fabs(angle1) < fabs(angle2))
            joint_values = joints_values_candidate1;
        else
            joint_values = joints_values_candidate2;

        // check forwards kinematics
        Eigen::Affine3d fwd = computeForwardKinematics(joint_values);
        // verify translation
        if (!fwd.translation().isApprox(desired_pose.translation(), 3e-2))
        {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("KinematicModel"),
                "Inverse Kinematics: solution found, but mismatch in the "
                "translational part found.");
        }
        return true;
    }
    else if (success_cand1)
    {
        // check forwards kinematics
        Eigen::Affine3d fwd = computeForwardKinematics(joints_values_candidate1);
        // verify translation
        if (!fwd.translation().isApprox(desired_pose.translation(), 3e-2))
        {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("KinematicModel"),
                "Inverse Kinematics: solution found, but mismatch in the "
                "translational part detected.");
        }
        if (!Eigen::Quaterniond(fwd.rotation()).isApprox(Eigen::Quaterniond(desired_pose.rotation()), 1e-2))
        {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("KinematicModel"),
                "Inverse Kinematics: solution found, but mismatch in the "
                "orientation part detected.");
        }
        joint_values = joints_values_candidate1;
        return true;
    }
    else if (success_cand2)
    {
        // check forwards kinematics
        Eigen::Affine3d fwd = computeForwardKinematics(joints_values_candidate2);
        // verify translation
        if (!fwd.translation().isApprox(desired_pose.translation(), 3e-2))
        {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("KinematicModel"),
                "Inverse Kinematics: solution found, but mismatch in the "
                "translational part found.");
        }
        if (!Eigen::Quaterniond(fwd.rotation()).isApprox(Eigen::Quaterniond(desired_pose.rotation()), 1e-2))
        {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("KinematicModel"),
                "Inverse Kinematics: solution found, but mismatch in the "
                "orientation part found.");
        }
        joint_values = joints_values_candidate2;
        return true;
    }

    RCLCPP_ERROR_STREAM(rclcpp::get_logger("KinematicModel"),"InverseKinematics: No exact feasible solution found");
    return false;   
}

bool KinematicModel::computeInverseKinematics(const Eigen::Ref<const Eigen::Vector3d>& desired_xyz, double desired_psi, double desired_phi,
                                              Eigen::Ref<JointVector> joint_values) const
{
    return computeInverseKinematics(createPoseFromPosAndPitch(desired_xyz, desired_psi, desired_phi), joint_values);
}

bool KinematicModel::computeInverseKinematics(const std::vector<double>& desired_xyz, double desired_psi, double desired_phi,
                                              Eigen::Ref<JointVector> joint_values) const
{
    Eigen::Map<const Eigen::Vector3d> xyz_map(desired_xyz.data());
    return computeInverseKinematics(xyz_map, desired_psi, desired_phi, joint_values);
}
    

}  // end namespace robot