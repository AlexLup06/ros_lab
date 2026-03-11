from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='workpackage_1',
            executable='frames_eigen',
            name='frames_eigen',
            output='screen'
        )
    ])
