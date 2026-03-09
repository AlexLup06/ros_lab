from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    surros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('surros_launch'), '/launch/surros.launch.py'
        ]),
        launch_arguments={'sim': 'true'}.items()
    )

    return LaunchDescription([
        surros_launch
    ])