from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    image_path = LaunchConfiguration('image_path')

    # Launch the robot_control node from workpackage_4 package
    workpackage_4_node = Node(
        package='workpackage_4',
        executable='exercise1',
        name='wp4_exercise1',
        output='screen',
        parameters=[{'image_path': image_path}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'image_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('workpackage_4'),
                'test_image.jpeg',
            ]),
            description='Path to the image used for block detection'),
        workpackage_4_node
    ])
