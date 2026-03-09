from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='Run in simulation mode'
    )

    # Static transform publisher (commented out in original)
    # static_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='gripper_base_link',
    #     arguments=['0', '0', '-0.01', '0.5', '-0.5', '-0.5', '0.5', 'gripper_finger_base_link', 'gripper_link', '50']
    # )


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

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_pub',
        parameters=[{'robot_description': robot_description}],
        arguments=[robot_description],
        output='screen'
    )

    surros_node = Node(
        package='surros_hardware',
        executable='surros_hardware_node',  # Fixed executable name
        name='surros',
        output='screen',
        parameters=[{'simulation': LaunchConfiguration('sim')}],
        # For YAML loading, see ROS 2 documentation for parameter file usage
    )

    return LaunchDescription([
        sim_arg,
        # static_tf,  # Uncomment if needed
        xacro_file_arg,
        robot_state_pub,
        surros_node
    ])
