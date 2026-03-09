#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
// #TODO: include
//#include <pxpincher_hardware/misc.h>  // conversions between ticks and angles in radiant
#include <surros_lib/surros_interface.h>
// #include <surros_msgs/srv/relax.hpp>
#include <signal.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <algorithm>
#include <sstream>

namespace robot {

SurrosControl::SurrosControl(std::shared_ptr<rclcpp::Node> node)
    : node_(node)
{}

SurrosControl::~SurrosControl()
{
    //if (_arm_action) _arm_action->cancelAllGoals();
}

void SurrosControl::initialize()
{
    
    if (_initialized)
    {
        RCLCPP_WARN(node_->get_logger(), "SurrosControl class already initialized. Skipping new initialization...");
        return;
    }

    // #################################################################################################
    // #                                 start action server clients                                   #
    // #################################################################################################
    node_->declare_parameter<std::string>("arm_action_topic", "/arm_controller/follow_joint_trajectory");
    std::string arm_action_topic = node_->get_parameter("arm_action_topic").as_string();

    RCLCPP_INFO(node_->get_logger(), "Waiting for arm action server to start.");
    _arm_action = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(node_, arm_action_topic);

    // Wait for the action server to be available
    while (!_arm_action->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_INFO(node_->get_logger(), "Waiting for arm action server...");
    }


    node_->declare_parameter<std::string>("gripper_action_topic", "/gripper_controller/gripper_cmd");
    std::string gripper_action_topic = node_->get_parameter("gripper_action_topic").as_string();

    RCLCPP_INFO(node_->get_logger(), "Arm action server found. Waiting for gripper action server to start.");
    _gripper_action = rclcpp_action::create_client<control_msgs::action::GripperCommand>(node_, gripper_action_topic);

    while (!_gripper_action->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_INFO(node_->get_logger(), "Waiting for gripper action server...");
    }
    RCLCPP_INFO(node_->get_logger(), "All action servers started.");

    // Marker publisher
    _marker_pub = node_->create_publisher<visualization_msgs::msg::Marker>("markers", 100);

    // setup subscriber (get joint_states in a separate thread)
    _joints_sub = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",
        10,
        std::bind(&robot::SurrosControl::jointStateCallback, this, std::placeholders::_1)
    );

    
    // #################################################################################################
    // #                                    setup joint names                                          #
    // #################################################################################################
    _joint_names_arm = {"theta_1", "theta_2", "theta_3", "theta_4","theta_5"}; // #TODO: rename in config file

    for (size_t i = 0; i < _joint_names_arm.size(); ++i)
    {
        _map_joint_to_index[_joint_names_arm.at(i)] = i;
    }
    
    // #################################################################################################
    // #                    get joint information (angle limits and max speed)                         #
    // #################################################################################################
    // #TODO: get limits from the parameter server of the hardware or controller manager node!
    // #TODO: remove this when above is read from the parameter server:
    for (size_t i = 0; i < _joint_names_arm.size(); ++i) {
        _joint_max_speeds[i]   = 2.0 * M_PI;
    }
    _joint_lower_bounds[0] = -270*M_PI/180.0;  // -270 degrees
    _joint_lower_bounds[1] = -45*M_PI/180.0;  // -30 degrees
    _joint_lower_bounds[2] = -10*M_PI/180.0;  // -10 degrees
    _joint_lower_bounds[3] = -100*M_PI/180.0;  // -100 degrees
    _joint_lower_bounds[4] = -270*M_PI/180.0;  // -270 degrees

    _joint_upper_bounds[0] = 270*M_PI/180.0;  // 270 degrees
    _joint_upper_bounds[1] = 90*M_PI/180.0;  // 90 degrees
    _joint_upper_bounds[2] = 120*M_PI/180.0;  // 110 degrees
    _joint_upper_bounds[3] = 100*M_PI/180.0;  // 100 degrees
    _joint_upper_bounds[4] = 270*M_PI/180.0;  // 270 degrees

    // #################################################################################################
    // #                                   setup further clients                                       #
    // #################################################################################################

    // setup service client for relaxing the servos
    _joint_relax_service = node_->create_client<surros_msgs::srv::Relax>("/surros/Relax");  // TODO param

    // setup service client for switching arm control mode (trajectory following or speed forwarding)
    _arm_control_mode_service = node_->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");
    
    // Optionally, wait for the service to be available
    if (!_arm_control_mode_service->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_WARN(node_->get_logger(),
            "'/controller_manager/switch_controller' not available. Cannot switch between joint interfaces, which is required by e.g. setJointVel");
    }

    // publisher for arm speed forwarding
    _arm_speed_forwarding_pub = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_speed_forwarder/commands", 1);

    //transform listener
    _tf_buffer = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);

    // #TODO: arm_controller starts in JointTrajectoryController mode, no initial switching required yet!
    switchArmControlMode(ArmControlMode::TRAJECTORY_FOLLOWING);

    // #################################################################################################
    // #                            get gripper information (angle limits)                             #
    // #################################################################################################
    // #TODO: get limits from the parameter server of the hardware or controller manager node!
    // #TODO: remove this assignment when above is read from the parameter server:
    _gripper_joint_name = "theta_6";
    _gripper_upper_bound = 0.0;
    _gripper_lower_bound = -1.4;

    // set gripper joint idx to 255 in our map (// TODO: ids from yaml file)
    _map_joint_to_index[_gripper_joint_name] = 255; 

    // #################################################################################################
    // #                                setup kinematic model                                          #
    // #################################################################################################
    // Wait for joint_state messages
    rclcpp::Rate rate(10);  // 10 Hz
    while (!_joint_values_received && rclcpp::ok())
    {
        RCLCPP_INFO_ONCE(node_->get_logger(), "Waiting for joint_states message...");
        rclcpp::spin_some(node_);
        rate.sleep();
    }
    // Drive into default position
    RCLCPP_INFO(node_->get_logger(), "Driving into default position in order to setup kinematic model...");
    setJointsDefault(0.1 * getSlowestMaxSpeed());
    // notify the kinematics model about lower and upper joint limits
    _kinematics.setLowerAndUpperJointLimits(_joint_lower_bounds, _joint_upper_bounds);

    // #################################################################################################
    // #                                      finish initialize                                        #
    // #################################################################################################
    _initialized = true;
    RCLCPP_INFO(node_->get_logger(), "Initialization completed.");
}

void robot::SurrosControl::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(_joints_mutex);
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        int joint_idx;
        try
        {
            joint_idx = _map_joint_to_index.at(msg->name[i]);  // throws except. if key is not included
        }
        catch (const std::out_of_range&)
        {
            // Everything ok since we've already specified our mapping table in the initialize method.
            // Just ignore / skip states of other joints...
            RCLCPP_INFO(node_->get_logger(), "Joint '%s' not found in mapping table. Skipping...", msg->name[i].c_str());
            continue;
        }
        if (joint_idx == 255)  // this must be our gripper! (see initialze())
            _gripper_value = msg->position[i];
        else
        {
            _joint_angles[joint_idx] = msg->position[i];
            if (!_joint_values_received) _joint_values_received = true;
        }
    }
}

// #TODO: is the SPEED_FORWARDING still required? If yes, a corresponding controller needs to be implemented!
bool robot::SurrosControl::switchArmControlMode(ArmControlMode mode)
{
    if (!_arm_control_mode_service->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(node_->get_logger(), "Cannot switch arm control mode, service not available");
        return false;
    }

    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();

    if (mode == ArmControlMode::TRAJECTORY_FOLLOWING)
    {
        //request->stop_controllers.push_back("arm_speed_forwarder");  // #TODO: implement
        //request->start_controllers.push_back("arm_controller");
        request->deactivate_controllers.push_back("arm_speed_forwarder");  // #TODO: implement
        request->activate_controllers.push_back("arm_controller");
    }
    else if (mode == ArmControlMode::SPEED_FORWARDING)
    {
        std::cout << "Switching to speed forwarding mode" << std::endl;
        request->deactivate_controllers.push_back("arm_controller");
        request->activate_controllers.push_back("arm_speed_forwarder"); // #TODO: implement
    }
    else
        return false;

    request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;

    // Call the service synchronously (for simplicity)
    auto future = _arm_control_mode_service->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(2)) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "Service call to switch controller timed out");
        return false;
    }

    bool ret_val = future.get()->ok;
    if (!ret_val)
        RCLCPP_ERROR(node_->get_logger(), "Failed to switch arm control mode");

    if (ret_val) _arm_control_mode = mode;

    return ret_val;
}

double SurrosControl::getSlowestMaxSpeed() const { return _joint_max_speeds.minCoeff(); }

void SurrosControl::setJointsDefault(const rclcpp::Duration& duration, bool blocking) {setJoints(JointVector::Zero(), duration, false, blocking); }

void SurrosControl::setJointsDefault(double speed, bool blocking) {setJoints(JointVector::Zero(), speed, false, blocking);}

void SurrosControl::setJointsDefault(const Eigen::Ref<const JointVector>& speed, bool blocking)
{
    setJoints(JointVector::Zero(), speed, false, blocking);
}

void SurrosControl::setJoints(const Eigen::Ref<const JointVector>& values, const rclcpp::Duration& duration, bool relative, bool blocking)
{
    trajectory_msgs::msg::JointTrajectoryPoint state;
    state.positions.assign(values.data(), values.data() + values.rows());
    state.time_from_start.sec = static_cast<int32_t>(duration.seconds());
    state.time_from_start.nanosec = static_cast<uint32_t>(duration.nanoseconds() % 1000000000);
    setJoints(state, relative, blocking);
}

void SurrosControl::setJoints(const std::vector<double>& values, const rclcpp::Duration& duration, bool relative, bool blocking)
{
    if (values.size() != static_cast<size_t>(_joint_lower_bounds.size()))
    {
        RCLCPP_ERROR(node_->get_logger(), "Number of joint values provided does not match number of joints");
        return;
    }
    Eigen::Map<const JointVector> values_map(values.data());
    setJoints(values_map, duration, relative, blocking);
}

void SurrosControl::setJoints(const Eigen::Ref<const JointVector>& values, double speed, bool relative, bool blocking)
{
    if (speed <= 0)
    {
        RCLCPP_ERROR(node_->get_logger(), "You cannot command a zero or negative velocity.");
        return;
    }

    JointVector current_states;
    getJointAngles(current_states);

    double max_diff = (values - current_states).cwiseAbs().maxCoeff();
    if (max_diff < 0.001) return;

    if (speed >= MaxSpeed)
        speed = _joint_max_speeds.minCoeff();

    double duration = max_diff / speed;
    if (duration < 0)
    {
        RCLCPP_ERROR(node_->get_logger(), "SurrosControl::setJoints(): obtained an invalid (negative) duration. Cannot set new joint values.");
        return;
    }
    setJoints(values, rclcpp::Duration::from_seconds(duration), relative, blocking);
}

void SurrosControl::setJoints(const std::vector<double>& values, double speed, bool relative, bool blocking)
{
    if (values.size() != static_cast<size_t>(_joint_lower_bounds.size()))
    {
        RCLCPP_ERROR(node_->get_logger(), "Number of joint values provided does not match number of joints");
        return;
    }
    Eigen::Map<const JointVector> values_map(values.data());
    setJoints(values_map, speed, relative, blocking);
}

void SurrosControl::setJoints(const Eigen::Ref<const JointVector>& values, const Eigen::Ref<const JointVector>& speed, bool relative, bool blocking)
{
    JointVector current_states;
    getJointAngles(current_states);

    JointVector act_speed = (speed.array() >= MaxSpeed).select(_joint_max_speeds, speed);

    trajectory_msgs::msg::JointTrajectory traj;
    createP2PTrajectoryWithIndividualVel(current_states, values, act_speed, traj);
    setJointTrajectory(traj, blocking);
}

void SurrosControl::setJoints(const std::vector<double>& values, const std::vector<double>& speed, bool relative, bool blocking)
{
    if (values.size() != static_cast<size_t>(_joint_lower_bounds.size()))
    {
        RCLCPP_ERROR(node_->get_logger(), "Number of joint values provided does not match number of joints");
        return;
    }
    Eigen::Map<const JointVector> values_map(values.data());
    Eigen::Map<const JointVector> vel_map(speed.data());
    setJoints(values_map, vel_map, relative, blocking);
}

void SurrosControl::setJoints(const trajectory_msgs::msg::JointTrajectoryPoint& joint_state, bool relative, bool blocking)
{
    bool sync_duration_mode = joint_state.time_from_start.sec != 0 || joint_state.time_from_start.nanosec != 0;
    if ((sync_duration_mode && !joint_state.velocities.empty()) || (!sync_duration_mode && joint_state.velocities.empty()))
    {
        RCLCPP_ERROR(node_->get_logger(),
            "SurrosControl::setJoints(): you must either specify a total duration (time_from_start) or individual velocities. Do not choose both.");
        return;
    }

    // create trajectory to the single goal
    control_msgs::action::FollowJointTrajectory_Goal goal;
    goal.trajectory.joint_names  = _joint_names_arm;
    // goal.trajectory.header.stamp = node_->now();
    goal.trajectory.header.stamp.sec = 0;
    goal.trajectory.header.stamp.nanosec = 0;
    goal.trajectory.points.push_back(joint_state);

    trajectory_msgs::msg::JointTrajectoryPoint& new_state = goal.trajectory.points.front();

    JointVector current_states;
    getJointAngles(current_states);

    Eigen::Map<JointVector> values(new_state.positions.data());

    if (relative) values += current_states;

    if (isExceedingJointLimits(values))
    {
        std::stringstream ss;
        ss << "SurrosControl::setJoints(): cannot set new joint values since they exceed joint limits.\ndesired angles: ["
           << values.transpose() << "]\nlower bounds: [" << _joint_lower_bounds.transpose() << "]\nupper bounds: ["
           << _joint_upper_bounds.transpose() << "]";
        RCLCPP_WARN(node_->get_logger(), "%s", ss.str().c_str());
        return;
    }

    for (size_t i = 0; i < new_state.velocities.size(); ++i)
    {
        new_state.velocities[i] = fabs(new_state.velocities[i]);
        if (new_state.velocities[i] > _joint_max_speeds[i])
        {
            RCLCPP_WARN(node_->get_logger(), "SurrosControl::setJoints(): Velocity of joint %zu exceeds limits. Bounding...", i);
            new_state.velocities[i] = _joint_max_speeds.coeffRef(i);
        }
    }
    if (sync_duration_mode)
    {
        double max_diff = (values - current_states).cwiseAbs().maxCoeff();
        double duration_bounded = std::max(max_diff / getSlowestMaxSpeed(),
                                           rclcpp::Duration(new_state.time_from_start).seconds());
        if (duration_bounded > rclcpp::Duration(new_state.time_from_start).seconds())
        {
            RCLCPP_WARN(node_->get_logger(),
                        "SurrosControl::setJoints(): desired speed is too high in order to drive all joints at this speed. Bounding...");
        }
        rclcpp::Duration bounded_duration = rclcpp::Duration::from_seconds(duration_bounded);
        new_state.time_from_start.sec = static_cast<int32_t>(bounded_duration.seconds());
        new_state.time_from_start.nanosec = static_cast<uint32_t>(bounded_duration.nanoseconds() % 1000000000);
        // new_state.time_from_start = rclcpp::Duration::from_seconds(duration_bounded).to_builtin();
    }
    setJointTrajectory(goal, blocking);
}

void SurrosControl::getJointAngles(Eigen::Ref<JointVector> values_out)
{
    std::lock_guard<std::mutex> lock(_joints_mutex);
    values_out = _joint_angles;
}

void SurrosControl::getJointAngles(std::vector<double>& values_out)
{
    std::lock_guard<std::mutex> lock(_joints_mutex);
    values_out.assign(_joint_angles.data(), _joint_angles.data() + _joint_angles.rows());
}

JointVector SurrosControl::getJointAngles()
{
    JointVector q;
    getJointAngles(q);
    return q;
}

bool SurrosControl::isExceedingJointLimits(const Eigen::Ref<const JointVector>& joint_values)
{
    return (joint_values.array() < _joint_lower_bounds.array()).any() || (joint_values.array() > _joint_upper_bounds.array()).any();
}

void SurrosControl::createP2PTrajectoryWithIndividualVel(
    const std::vector<double>& start_conf,
    const std::vector<double>& goal_conf,
    const std::vector<double>& speed,
    trajectory_msgs::msg::JointTrajectory& trajectory)
{
    Eigen::Map<const JointVector> start_map(start_conf.data());
    Eigen::Map<const JointVector> goal_map(goal_conf.data());
    Eigen::Map<const JointVector> speed_map(speed.data());
    createP2PTrajectoryWithIndividualVel(start_map, goal_map, speed_map, trajectory);
}

void SurrosControl::createP2PTrajectoryWithIndividualVel(
    const Eigen::Ref<const JointVector>& start_conf,
    const Eigen::Ref<const JointVector>& goal_conf,
    const Eigen::Ref<const JointVector>& speed,
    trajectory_msgs::msg::JointTrajectory& trajectory)
{
    JointVector new_speed = (speed.array() != 0).select(speed.cwiseAbs(), 1e-2);
    JointVector diff = goal_conf - start_conf;
    new_speed = (diff.array() < 0).select(-new_speed, new_speed);
    JointVector durations = diff.cwiseQuotient(new_speed).cwiseAbs();
    durations = (durations.array() < 20.0).select(durations, 20.0);

    // #TODO: not needed until now:
    // double max_duration = durations.maxCoeff();

    std::vector<int> indices(durations.rows());
    std::iota(indices.begin(), indices.end(), 0);

    std::sort(indices.begin(), indices.end(),
        [&durations](std::size_t i, std::size_t j) { return durations.coeffRef(i) < durations.coeffRef(j); });

    int n_joints = durations.rows();
    trajectory.header.stamp = node_->now();
    trajectory.joint_names = getJointNames();
    trajectory.points.reserve(indices.size());

    for (int i = 0; i < static_cast<int>(indices.size()); ++i)
    {
        if (durations.coeffRef(indices[i]) > 1e-2)
        {
            trajectory.points.emplace_back();
            trajectory.points.back().time_from_start.sec = static_cast<int32_t>(durations.coeffRef(indices[i]));
            trajectory.points.back().time_from_start.nanosec = static_cast<uint32_t>((durations.coeffRef(indices[i]) - static_cast<int32_t>(durations.coeffRef(indices[i]))) * 1e9);
            trajectory.points.back().positions.resize(n_joints);
            trajectory.points.back().velocities.resize(n_joints);
        }
    }

    int n_points = static_cast<int>(trajectory.points.size());
    for (int i = 0; i < n_joints; ++i)
    {
        int curr_joint = indices[i];
        for (int j = 0; j < n_points; ++j)
        {
            trajectory_msgs::msg::JointTrajectoryPoint& pt = trajectory.points[j];
            if (j == 0)
            {
                if (fabs(diff.coeffRef(curr_joint)) > 1e-2)
                {
                    double t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9;
                    pt.positions[curr_joint] = start_conf[curr_joint] + t * new_speed[curr_joint];
                    pt.velocities[curr_joint] = new_speed[curr_joint];
                }
                else
                {
                    pt.positions[curr_joint] = start_conf[curr_joint];
                    pt.velocities[curr_joint] = 0.0;
                }
            }
            else
            {
                trajectory_msgs::msg::JointTrajectoryPoint& last_pt = trajectory.points[j - 1];
                double t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9;
                double t_last = last_pt.time_from_start.sec + last_pt.time_from_start.nanosec * 1e-9;
                if (!is_approx(last_pt.positions[curr_joint], goal_conf[curr_joint], 1e-2))
                {
                    pt.positions[curr_joint] =
                        last_pt.positions[curr_joint] + (t - t_last) * new_speed[curr_joint];
                    pt.velocities[curr_joint] = new_speed[curr_joint];
                }
                else
                {
                    pt.positions[curr_joint] = goal_conf[curr_joint];
                    pt.velocities[curr_joint] = 0.0;
                }
            }
        }
    }

    // Delete duplicates
    auto start_it = trajectory.points.begin();
    start_it = std::next(start_it);
    while (start_it != trajectory.points.end())
    {
        if (start_it->time_from_start == std::prev(start_it)->time_from_start)
        {
            start_it = trajectory.points.erase(start_it);
        }
        else
        {
            ++start_it;
        }
    }
}

void SurrosControl::setJointTrajectory(trajectory_msgs::msg::JointTrajectory& trajectory, bool blocking)
{
    control_msgs::action::FollowJointTrajectory_Goal goal;
    goal.trajectory = trajectory;

    setJointTrajectory(goal, blocking);
}

void SurrosControl::setJointTrajectory(control_msgs::action::FollowJointTrajectory_Goal& goal, bool blocking)
{
    if (!_arm_action) {
        RCLCPP_ERROR(node_->get_logger(), "PhantomXControl: class not initialized, call initialize().");
        return;
    }

    if (_arm_control_mode != ArmControlMode::TRAJECTORY_FOLLOWING)
        switchArmControlMode(ArmControlMode::TRAJECTORY_FOLLOWING);

    bool feasible = verifyTrajectory(goal.trajectory);
    if (!feasible)
    {
        RCLCPP_ERROR(node_->get_logger(), "The trajectory is not feasible. Printing trajectory now: ");
        printTrajectory(goal.trajectory);
        return;
    }

    auto send_goal_options = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();
    // Optionally, set up feedback/callbacks here

    if (blocking)
    {
        auto future_goal_handle = _arm_action->async_send_goal(goal, send_goal_options);
        if (rclcpp::spin_until_future_complete(node_, future_goal_handle, std::chrono::seconds(30)) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(), "Timed out waiting for goal handle from action server...");
            return;
        }
        auto goal_handle = future_goal_handle.get();
        if (!goal_handle) {
            RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by the action server.");
            return;
        }
        auto result_future = _arm_action->async_get_result(goal_handle);
        if (rclcpp::spin_until_future_complete(node_, result_future, std::chrono::seconds(30)) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(), "Timed out waiting for action result.");
            return;
        }
        // Optionally, check result_future.get()->result
    }
    else
    {
        _arm_action->async_send_goal(goal, send_goal_options);
    }
}

bool SurrosControl::verifyTrajectory(trajectory_msgs::msg::JointTrajectory& trajectory)
{
    // Get current joint angles
    JointVector current_states;
    getJointAngles(current_states);

    // Store previous angle positions as Eigen type (required inside the loop)
    Eigen::Map<JointVector> prev_pos(current_states.data());

    int pt_idx = 0;
    for (trajectory_msgs::msg::JointTrajectoryPoint& state : trajectory.points)
    {
        Eigen::Map<const JointVector> values(state.positions.data());

        // check joint angle limits
        if (isExceedingJointLimits(values))
        {
            std::stringstream ss;
            ss << "SurrosControl::verifyTrajectory(): cannot set new joint values since they exceed joint limits at point "
               << pt_idx << ".\ndesired angles: [" << values.transpose() << "]\nlower bounds: [" << _joint_lower_bounds.transpose()
               << "]\nupper bounds: [" << _joint_upper_bounds.transpose() << "]";
            RCLCPP_WARN(node_->get_logger(), "%s", ss.str().c_str());
            return false;
        }

        // check velocities and/or duration before adding
        for (int i = 0; i < static_cast<int>(state.velocities.size()); ++i)
        {
            state.velocities[i] = fabs(state.velocities[i]);
            if (state.velocities[i] > _joint_max_speeds[i])
            {
                RCLCPP_WARN(node_->get_logger(), "SurrosControl::verifyTrajectory(): Velocity of joint %d at point %d exceeds limits. Bounding...", i, pt_idx);
                state.velocities[i] = _joint_max_speeds.coeffRef(i);
            }
        }

        // check transition mode (synchronous transition or individual velocities)
        bool sync_duration_mode = state.time_from_start.sec != 0 || state.time_from_start.nanosec != 0;
        if (sync_duration_mode)
        {
            // get maximum absolute angle difference
            double max_diff = (values - prev_pos).cwiseAbs().maxCoeff();

            // check and bound velocity
            // get maximum allowed duration (w.r.t. min of all max joint speeds)
            double time_from_start_sec = state.time_from_start.sec + state.time_from_start.nanosec * 1e-9;
            double duration_bounded = std::max(max_diff / getSlowestMaxSpeed(), time_from_start_sec);

            if (duration_bounded > time_from_start_sec)
            {
                RCLCPP_WARN(node_->get_logger(),
                    "SurrosControl::verifyTrajectory(): desired speed at point %d is too high in order to drive all joints at this speed. Bounding...",
                    pt_idx);
            }
            state.time_from_start.sec = static_cast<int32_t>(duration_bounded);
            state.time_from_start.nanosec = static_cast<uint32_t>((duration_bounded - static_cast<int32_t>(duration_bounded)) * 1e9);
        }

        // check self-collision
        if (_collision_check_enabled && checkSelfCollision(values))
        {
            RCLCPP_ERROR(node_->get_logger(), "SurrosControl::verifyTrajectory(): Self-collision detected at point %d.", pt_idx);
            return false;
        }

        // Store pos vector for the subsequent iteration (using C++ placement new operator)
        new (&prev_pos) Eigen::Map<JointVector>(state.positions.data());

        ++pt_idx;
    }
    // set check enabled for next time
    _collision_check_enabled = true;
    return true;
}

bool SurrosControl::checkSelfCollision(const Eigen::Ref<const JointVector>& joint_values)
{
    if (!_initialized)
        return false;  // Only check after initialization

    // WORKAROUND: Deactivate collision check for velocity control
    int count_equal = 0;
    for (int i = 0; i < joint_values.rows(); ++i)
    {
        if (is_approx(joint_values.coeffRef(i), _joint_lower_bounds.coeffRef(i)) ||
            is_approx(joint_values.coeffRef(i), _joint_upper_bounds.coeffRef(i)) || joint_values.coeffRef(i) == 0)
            count_equal = count_equal + 1;
    }
    if (count_equal == joint_values.rows()) return false;  // velocity control detected

    // Get end-effector position
    Eigen::Affine3d transform = _kinematics.computeForwardKinematics(joint_values);
    transform = _j1_T_base * transform;

    const Eigen::Vector3d& point = transform.translation();

    // Check bounding box (parameters can be tuned)
    bool inside = point.z() > -0.13 && point.z() < 0.07 && point.x() > -0.11 && point.x() < 0.11;
    if (inside)
    {
        if (point.y() < 0.1) return true;
    }

    // Check the fourth joint
    transform = _kinematics.computeForwardKinematics(joint_values, 3);
    transform = _j1_T_base * transform;
    const Eigen::Vector3d& point2 = transform.translation();
    inside = point2.z() > -0.13 && point2.z() < 0.07 && point2.x() > -0.11 && point2.x() < 0.11;
    if (inside)
    {
        if (point2.y() < 0)  // threshold for the fourth joint
            return true;
    }

    return false;
}

void SurrosControl::printTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory)
{
    if (trajectory.points.empty())
    {
        RCLCPP_WARN(node_->get_logger(), "SurrosControl::printTrajectory() - cannot print empty trajectory.");
        return;
    }

    int no_joints = trajectory.points.front().positions.size();
    int no_points = trajectory.points.size();
    int no_vel    = trajectory.points.front().velocities.size();

    if (no_joints == 0) RCLCPP_INFO(node_->get_logger(), "Position profile empty.");
    if (no_vel == 0) RCLCPP_INFO(node_->get_logger(), "Velocity profile empty.");

    RCLCPP_INFO(node_->get_logger(), "");  // add blank line

    // Position:
    // print header
    std::stringstream header;
    for (int i = 0; i < no_points; ++i)
    {
        header << "p" << i << "\t";
        assert(trajectory.points[i].positions.size() == static_cast<size_t>(no_joints));
        assert(trajectory.points[i].velocities.size() == static_cast<size_t>(no_vel));
    }
    RCLCPP_INFO(node_->get_logger(), "%s", header.str().c_str());

    for (int i = 0; i < no_joints; ++i)
    {
        std::stringstream row;
        for (int j = 0; j < no_points; ++j)
            row << std::fixed << std::setprecision(2) << trajectory.points[j].positions[i] << "\t";
        RCLCPP_INFO(node_->get_logger(), "%s", row.str().c_str());
    }

    RCLCPP_INFO(node_->get_logger(), "");  // add blank line

    // Velocity:
    header.str("");
    header.clear();
    for (int i = 0; i < no_points; ++i) header << "v" << i << "\t";
    RCLCPP_INFO(node_->get_logger(), "%s", header.str().c_str());

    for (int i = 0; i < no_vel; ++i)
    {
        std::stringstream row;
        for (int j = 0; j < no_points; ++j)
            row << std::fixed << std::setprecision(2) << trajectory.points[j].velocities[i] << "\t";
        RCLCPP_INFO(node_->get_logger(), "%s", row.str().c_str());
    }

    RCLCPP_INFO(node_->get_logger(), "");  // add blank line

    // Time:
    header.str("");
    header.clear();
    for (int i = 0; i < no_points; ++i) header << "t" << i << "\t";
    RCLCPP_INFO(node_->get_logger(), "%s", header.str().c_str());
    std::stringstream row;
    for (int j = 0; j < no_points; ++j)
    {
        double t = trajectory.points[j].time_from_start.sec + trajectory.points[j].time_from_start.nanosec * 1e-9;
        row << std::fixed << std::setprecision(2) << t << "\t";
    }
    RCLCPP_INFO(node_->get_logger(), "%s", row.str().c_str());
}

void SurrosControl::getJointVelocities(Eigen::Ref<JointVector> velocities_out)
{
    std::lock_guard<std::mutex> lock(_joints_mutex);
    velocities_out = _joint_velocities;
}

void SurrosControl::setJointVel(const Eigen::Ref<const JointVector>& velocities)
{
    if (_arm_speed_forwarding_pub->get_subscription_count() == 0)
    {
       RCLCPP_WARN(node_->get_logger(),"setJointVel(): no subscribers on topic '/arm_speed_forwarder/command'");
       return;
    }

    // cancel any previous goals
    // stopMoving(); // TODO: required?

    if (_arm_control_mode != ArmControlMode::SPEED_FORWARDING) switchArmControlMode(ArmControlMode::SPEED_FORWARDING);

    std_msgs::msg::Float64MultiArray joint_vels;
    joint_vels.data.resize(JointVector::RowsAtCompileTime);
    Eigen::Map<JointVector> vel_bounded(joint_vels.data.data());

    // Bound velocities
    vel_bounded = robot::bound(-_joint_max_speeds, velocities, _joint_max_speeds);

    _arm_speed_forwarding_pub->publish(joint_vels);

    /*

    // Select lower and upper bound values for each joint as goal w.r.t. the direction of the velocity
    // For zero velocities stay at the current position
    JointVector current;
    getJointAngles(current);
    JointVector goal = (vel_bounded.array()==0).select( current, (vel_bounded.array()<0).select(_joint_lower_bounds,_joint_upper_bounds) );
    _collision_check_enabled = false; // Disable collision check, be careful!!
    setJoints(goal, vel_bounded, false, false); // we do not want to block in case of commanding velocities

    */
}

void SurrosControl::setJointVel(const std::vector<double>& velocities)
{
    if (velocities.size() != static_cast<std::size_t>(_joint_lower_bounds.size()))
    {
        RCLCPP_ERROR(node_->get_logger(),"Number of joint velocities provided does not match number of joints");
        return;
    }
    Eigen::Map<const JointVector> vel_map(velocities.data());
    setJointVel(vel_map);
}

void SurrosControl::getEndeffectorState(Eigen::Affine3d& base_T_gripper)
{
    geometry_msgs::msg::TransformStamped tf_transform;
    getEndeffectorState(tf_transform);
    base_T_gripper = tf2::transformToEigen(tf_transform);
}

void SurrosControl::getEndeffectorState(geometry_msgs::msg::TransformStamped& base_T_gripper)
{
    try
    {
        // Use tf2_ros::Buffer for ROS 2
        base_T_gripper = _tf_buffer->lookupTransform(
            "base_link", "tcp", rclcpp::Time(0), rclcpp::Duration::from_seconds(10.0));
    }
    catch (const tf2::TransformException& ex)
    {
        RCLCPP_ERROR(node_->get_logger(), "%s", ex.what());
    }
}

void SurrosControl::setEndeffectorPose(const Eigen::Affine3d& desired_pose, const rclcpp::Duration& duration, bool relative, bool blocking)
{
    JointVector joint_angles;
    getJointAngles(joint_angles);  // use current joint angles as initialization
    bool success = false;
    if (relative)
    {
        Eigen::Affine3d current_tcp = _kinematics.computeForwardKinematics(joint_angles);
        success                     = _kinematics.computeInverseKinematics(desired_pose * current_tcp, joint_angles);
    }
    else
        success = _kinematics.computeInverseKinematics(desired_pose, joint_angles);
    if (success) setJoints(joint_angles, duration, false, blocking);
}

void SurrosControl::setEndeffectorPose(const Eigen::Affine3d& desired_pose, double speed, bool relative, bool blocking)
{
    JointVector joint_angles;
    getJointAngles(joint_angles);  // use current joint angles as initialization
    bool success = false;
    if (relative)
    {
        Eigen::Affine3d current_tcp = _kinematics.computeForwardKinematics(joint_angles);
        success                     = _kinematics.computeInverseKinematics(desired_pose * current_tcp, joint_angles);
    }
    else
        success = _kinematics.computeInverseKinematics(desired_pose, joint_angles);
    if (success) setJoints(joint_angles, speed, false, blocking);
}

bool SurrosControl::setEndeffectorPose(const Eigen::Ref<const Eigen::Vector3d>& desired_xyz,
    double psi, double phi,
    const rclcpp::Duration& duration,
    bool relative,
    bool blocking)
{
    JointVector joint_angles;
    getJointAngles(joint_angles);
    bool success = false;
    if (relative)
    {
        Eigen::Affine3d current_tcp = _kinematics.computeForwardKinematics(joint_angles);
        Eigen::Affine3d target_pose = createPoseFromPosAndPitch(desired_xyz, psi, phi) * current_tcp;
        success = _kinematics.computeInverseKinematics(target_pose, joint_angles);
    }
    else
    {
        success = _kinematics.computeInverseKinematics(desired_xyz, psi, phi, joint_angles);
    }
    if (success)
    {
        setJoints(joint_angles, duration, false, blocking);
    }
    return success;
}

bool SurrosControl::setEndeffectorPose(const Eigen::Ref<const Eigen::Vector3d>& desired_xyz, double psi, double phi, double speed, bool relative,
                                         bool blocking)
{
    JointVector joint_angles;
    getJointAngles(joint_angles);  // use current joint angles as initialization
    bool success = false;
    if (relative)
    {
        Eigen::Affine3d current_tcp = _kinematics.computeForwardKinematics(joint_angles);
        success = _kinematics.computeInverseKinematics(createPoseFromPosAndPitch(desired_xyz, psi, phi) * current_tcp, joint_angles);
    }
    else
        success = _kinematics.computeInverseKinematics(desired_xyz, psi, phi, joint_angles);
    if (success) setJoints(joint_angles, speed, false, blocking);
    return success;
}

bool SurrosControl::setEndeffectorPose(const std::vector<double>& desired_xyz, double psi, double phi, double speed, bool relative, bool blocking)
{
    if (desired_xyz.size() != 3)
    {
        RCLCPP_ERROR(node_->get_logger(),"Endeffector position must be a 3d vector.");
        return false;
    }
    return setEndeffectorPose(Eigen::Map<const Eigen::Vector3d>(desired_xyz.data()), psi, phi, speed, relative, blocking);
}

void SurrosControl::getJacobian(RobotJacobian& jacobian)
{
    JointVector joint_angles;
    getJointAngles(joint_angles);
    _kinematics.computeJacobian(joint_angles, jacobian, _tf_buffer);
}

void SurrosControl::getJacobianReduced(RobotJacobianReduced& jacobian)
{
    JointVector joint_angles;
    getJointAngles(joint_angles);
    _kinematics.computeJacobianReduced(joint_angles, jacobian, _tf_buffer);
}

void SurrosControl::stopMoving()
{
    if (_arm_action) {
        _arm_action->async_cancel_all_goals();
    }

    // also send stop topic
    std_msgs::msg::Float64MultiArray data;
    data.data.resize(JointVector::RowsAtCompileTime, 0.0);
    for (int i = 0; i < 10; ++i) _arm_speed_forwarding_pub->publish(data);
}

bool robot::SurrosControl::relaxServos(bool relaxed)
{
    bool ret_val = true;

    // ROS 2: Use service client with request/response
    auto request = std::make_shared<surros_msgs::srv::Relax::Request>();
    request->relaxed = relaxed;
    
    if (!_joint_relax_service->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_WARN(node_->get_logger(), "Relax service not available");
        return false;
    }

    auto future = _joint_relax_service->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
        auto response = future.get();
        ret_val = response->success;
        if (!ret_val) {
            RCLCPP_WARN(node_->get_logger(), "Not all servos could be relaxed");
        }
    } else {
        ret_val = false;
        RCLCPP_WARN(node_->get_logger(), "Failed to relax servos");
    }

    return ret_val;
}

void SurrosControl::setGripperJoint(int percent_open, bool blocking)
{
    double joint_value = _gripper_lower_bound + 0.01 * double(percent_open) * (_gripper_upper_bound - _gripper_lower_bound);
    setGripperRawJointAngle(joint_value, blocking);
}

void SurrosControl::setGripperRawJointAngle(double joint_value, bool blocking)
{
    // bound value
    joint_value = bound(_gripper_lower_bound, joint_value, _gripper_upper_bound);

    control_msgs::action::GripperCommand_Goal goal;
    goal.command.position = joint_value;

    if (!_gripper_action) {
        RCLCPP_ERROR(node_->get_logger(), "Gripper action client not initialized.");
        return;
    }

    auto send_goal_options = rclcpp_action::Client<control_msgs::action::GripperCommand>::SendGoalOptions();
    // Optionally, set up feedback/callbacks here

    if (blocking)
    {
        auto future_goal_handle = _gripper_action->async_send_goal(goal, send_goal_options);
        // Wait for the result (timeout: 10s)
        if (rclcpp::spin_until_future_complete(node_, future_goal_handle, std::chrono::seconds(1)) !=
            rclcpp::FutureReturnCode::SUCCESS){
            RCLCPP_ERROR(node_->get_logger(), "Timed out waiting for gripper action result.");
            return;
        }
        auto goal_handle = future_goal_handle.get();
        if (!goal_handle) {
            RCLCPP_ERROR(node_->get_logger(), "Gripper goal was rejected by the action server.");
            return;
        }
        auto result_future = _gripper_action->async_get_result(goal_handle);
        if (rclcpp::spin_until_future_complete(node_, result_future, std::chrono::seconds(1)) !=
            rclcpp::FutureReturnCode::SUCCESS){
            RCLCPP_ERROR(node_->get_logger(), "Timed out waiting for gripper action result.");
            return;
        }
        // Optionally, check result_future.get()->result
    }
    else
    {
        _gripper_action->async_send_goal(goal, send_goal_options);
    }
}

double SurrosControl::getGripperRawJointAngle()
{
    std::lock_guard<std::mutex> lock(_joints_mutex);  // TODO: makes no sense here?
    double gripper_value = _gripper_value;
    return gripper_value;  // avoid race conditions
}

int SurrosControl::getGripperJointPercentage()
{
    double raw_angle    = getGripperRawJointAngle();
    double percent_open = std::round((raw_angle - _gripper_lower_bound) / (_gripper_upper_bound - _gripper_lower_bound) / 0.01);
    return (int)percent_open;
}

}  // end namespace robot
