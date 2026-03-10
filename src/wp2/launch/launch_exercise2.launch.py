from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():

    # Launch the exercise2 node from workpackage_2 package
    workpackage_2_node = Node(
        package='workpackage_2',
        executable='exercise2',
        name='wp2_exercise2',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        workpackage_2_node
    ])
