from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='Run in simulation mode'
    )

    xacro_file_arg = DeclareLaunchArgument(
        'xacro_file',
        default_value=PathJoinSubstitution([
            get_package_share_directory('surros_description'),
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

    # Node to publish robot_description parameter
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # Include RViz display launch file (if you have a ROS 2 version of it)
    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('urdf_tutorial'),
                'launch',
                'display.launch.py'
            ])
        ]),
        launch_arguments={'model': LaunchConfiguration('xacro_file')}.items()
    )

    # Your C++ node
    #robot_node = Node(
    #    package='robot_lib',
    #    executable='robot_node',
    #    name='robot_node',
    #    output='screen'
    #)

    return LaunchDescription([
        sim_arg,
        xacro_file_arg,
        robot_state_publisher_node,
        display_launch,
        #robot_node
    ])
