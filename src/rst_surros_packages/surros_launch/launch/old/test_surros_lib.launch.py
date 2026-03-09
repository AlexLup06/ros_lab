from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time',default='false')
    # Declare launch argument for xacro file
    xacro_file_arg = DeclareLaunchArgument(
        'xacro_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('surros_description'),
            'urdf',
            'follower_arm.urdf.xacro'
        ]),
        description='Path to the xacro file'
    )

    # Load robot_description using xacro
    robot_description = Command([
        'xacro ',
        LaunchConfiguration('xacro_file')
    ])

    # Node to publish robot_description
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time':use_sim_time, 'robot_description': robot_description}],
        arguments=[robot_description],
        output='screen'
    )

    robot_node = Node(
        package='surros_lib',
        executable='robot_node',
        name='robot_node',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        xacro_file_arg,
        robot_state_publisher_node,
        robot_node
    ])
