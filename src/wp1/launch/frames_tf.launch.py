from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='workpackage_1',  # replace with your ROS2 package
            executable='frames_tf',       # executable name (after building)
            name='frames_tf',
            output='screen'
        )
    ])
