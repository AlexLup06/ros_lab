/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
 
#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#include <Eigen/Geometry>
#include <surros_lib/misc.h>
#include <surros_lib/types.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>

namespace robot {

/**
 * @class KinematicsModel
 * @brief Provides kinematic computations for a 5-DOF robot arm (Koch V1.1).
 *
 * This class allows simulation of end effector poses for varying joint states, inverse kinematics,
 * and differential kinematics. It is currently based on Denavit-Hartenberg parameters.
 *
 * @todo Make templated for arbitrary robots
 * @todo Extract transformations from URDF/Mesh files of the robot.
 * @remarks Only revolute joints are supported.
 */
class KinematicModel
{
 public:
    /**
     * @brief Default constructor.
     */
    KinematicModel(){};

    /**
     * @brief Destructor.
     */
    virtual ~KinematicModel(){};

    /**
     * @brief Denavit-Hartenberg parameters for a single joint.
     */
    struct DHParams
    {
        double theta; ///< Joint angle
        double alpha; ///< Link twist
        double a;     ///< Link length
        double d;     ///< Link offset
    };

    /**
     * @brief Compute the homogeneous transformation matrix from DH parameters.
     * @param params DH parameters for the joint.
     * @param[out] T Resulting transformation matrix.
     */
    void getDHMatrix(const DHParams& params, Eigen::Affine3d& T) const;

    /**
     * @brief Set lower and upper joint limits for the robot.
     * @param lower_bounds Vector of lower joint limits.
     * @param upper_bounds Vector of upper joint limits.
     */
    void setLowerAndUpperJointLimits(const Eigen::Ref<const JointVector> lower_bounds, const Eigen::Ref<const JointVector> upper_bounds)
    {
        _joint_lower_bounds = lower_bounds;
        _joint_upper_bounds = upper_bounds;
    }


    /**
     * @brief Compute the forward kinematics for the specified joint angles.
     * @param joint_values Vector of joint values q=[q1,q2,q3,q4,q5]^T
     * @param up_to_index Final frame index: 1=joint1, ..., 6=tcp frame.
     * @return Transformation from base_link to the specified frame.
     */
    Eigen::Affine3d computeForwardKinematics(const Eigen::Ref<const JointVector>& joint_values, int up_to_index = 6) const;

    /**
     * @brief Compute the geometric Jacobian of the end effector w.r.t. the base frame.
     *
     * The Jacobian relates joint velocities to end effector velocity: v = J*qdot.
     * The resulting matrix is 6x5 for a 5-DOF robot.
     * Rotational axes are assumed to be the z-axis of each joint frame.
     *
     * @param joint_values Joint configuration q=[q1,q2,q3,q4,q5].
     * @param[out] jacobian The 6x5 Jacobian matrix.
     * @param _tf_buffer Shared pointer to a tf2_ros::Buffer for frame lookups.
     */
    void computeJacobian(const Eigen::Ref<const JointVector>& joint_values, RobotJacobian& jacobian, std::shared_ptr<tf2_ros::Buffer> _tf_buffer) const;

    /**
     * @brief Compute reduced robot Jacobian (x, y, z, pitch, yaw) w.r.t. the base frame.
     *
     * The reduced Jacobian is 5x5, corresponding to the position and two orientation components.
     * @param joint_values Joint configuration q=[q1,q2,q3,q4,q5].
     * @param[out] jacobian4d The 5x5 reduced Jacobian matrix.
     * @param _tf_buffer Shared pointer to a tf2_ros::Buffer for frame lookups.
     */
    void computeJacobianReduced(const Eigen::Ref<const JointVector>& joint_values, RobotJacobianReduced& jacobian4d, std::shared_ptr<tf2_ros::Buffer> _tf_buffer) const;

    /**
     * @brief Compute joint angles for a given end effector pose (6D) w.r.t. the base frame.
     *
     * The inverse kinematics computes joint values for a desired pose, considering joint limits.
     * Two candidate solutions (elbow up/down) are evaluated, and the best match is selected.
     * If no solution is found, returns false.
     *
     * @param desired_pose Desired end effector pose (position and orientation).
     * @param[out] joint_values Resulting joint values. Should be initialized with current values for solution preference.
     * @return true if a solution was found within joint limits, false otherwise.
     */
    bool computeInverseKinematics(const Eigen::Affine3d& desired_pose, Eigen::Ref<JointVector> joint_values) const;

    /**
     * @brief Compute joint angles for a given 5D pose (position + pitch/yaw) w.r.t. the base frame.
     *
     * Overloads the generic function, limiting the pose to position and two orientation angles.
     * @param desired_xyz Desired [x,y,z] coordinates in the base frame.
     * @param desired_psi Desired pitch angle in the base frame.
     * @param desired_phi Desired yaw angle in the base frame.
     * @param[out] joint_values Resulting joint values.
     * @return true if a solution was found within joint limits, false otherwise.
     */
    bool computeInverseKinematics(const Eigen::Ref<const Eigen::Vector3d>& desired_xyz, double desired_psi, double desired_phi,
                                  Eigen::Ref<JointVector> joint_values) const;

    /**
     * @brief Compute joint angles for a given 5D pose (position + pitch/yaw) w.r.t. the robot base frame
     * @details This methods overloads a more generic function and limits the pose to the position part and a pitch angle.
     * @param desired_xyz Desired [x,y,z] coordinates in the base frame (std::vector< double >)
     * @param desired_psi Desired pitch angle in the base frame (<e> endeffector points upwards for -pi/2 [rad] and downwards for +pi/2 [rad] </e>)
     * @param desired_phi Desired yaw angle in the base frame (<e> endeffector points upwards for -pi/2 [rad] and downwards for +pi/2 [rad] </e>)
     * @param[out] joint_values the corresponding joint values. Initit them with the current joint values, in order to choose
     *                          either the elbow up or elbow down solution depending on the current angular distance.
     * @return \c true if a solution was found, \c false otherwise.
     */
    bool computeInverseKinematics(const std::vector<double>& desired_xyz, double desired_psi, double desired_phi, Eigen::Ref<JointVector> joint_values) const;


 protected:

 private:
    JointVector _joint_lower_bounds = JointVector::Constant(-M_PI); ///< Lower joint limits
    JointVector _joint_upper_bounds = JointVector::Constant(M_PI);  ///< Upper joint limits

 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace robot

#endif /* KINEMATICS_H_ */