from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    #surros_arm_yaml = PathJoinSubstitution([
    #    FindPackageShare('surros_config'), 'config', 'surros_arm.yaml'
    #])
    controllers_yaml = PathJoinSubstitution([
        FindPackageShare('surros_config'), 'config', 'controllers.yaml'
    ])
    return LaunchDescription([
        # Start the controller manager with your hardware interface and controller configs
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[controllers_yaml],
            output='screen',
        ),
        # Spawner for arm controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['arm_controller'],
            output='screen',
        ),
        # Spawner for gripper controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['gripper_controller'],
            output='screen',
        ),
        # Spawner for speed forwarder (optional)
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['arm_speed_forwarder'],
            output='screen',
        ),
    ])
