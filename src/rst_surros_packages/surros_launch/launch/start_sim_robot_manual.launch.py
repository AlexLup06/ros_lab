import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    surros_description = "surros_description"
    surros_config = "surros_config"
    rviz_config = os.path.join(get_package_share_directory(
        surros_config), "config", "surros_rviz.rviz")
    robot_description = os.path.join(get_package_share_directory(
        surros_description), "urdf", "robot.urdf.xacro")
    # robot_description_config = xacro.process_file(robot_description)
    robot_description_config = xacro.process_file(
        robot_description,
        mappings={
            "use_mock_hardware": "true"
        }
    )
    controller_config = os.path.join(get_package_share_directory(
        surros_config), "config", "controllers.yaml")

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                {"robot_description": robot_description_config.toxml()}],
            output="screen",
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),

        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
        )
    ])
