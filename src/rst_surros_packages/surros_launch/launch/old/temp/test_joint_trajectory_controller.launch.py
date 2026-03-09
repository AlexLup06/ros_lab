# This launch file is used to test the joint trajectory controller by publishing position goals.
# It uses the `ros2_controllers_test_nodes` package to publish goals defined in a YAML configuration file. 
# Start using: ros2 launch surros_launch robot_with_controllers.launch.py and ros2 launch surros_launch test_joint_trajectory_controller.launch.py

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    position_goals = PathJoinSubstitution(
        [FindPackageShare("surros_config"), "config", "test_goal_publishers_config.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="ros2_controllers_test_nodes",
                executable="publisher_joint_trajectory_controller",
                name="publisher_gripper_controller",
                parameters=[position_goals],
                output="both",
            ),
            Node(
                package="ros2_controllers_test_nodes",
                executable="publisher_joint_trajectory_controller",
                name="publisher_arm_controller",
                parameters=[position_goals],
                output="both",
            )
        ]
    )