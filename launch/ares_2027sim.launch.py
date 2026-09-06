"""Launch the ABU Robocon 2027 field in Gazebo Harmonic."""

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Start Gazebo with the generated Robocon 2027 world."""
    world = PathJoinSubstitution([
        FindPackageShare('ares_2027sim'),
        'worlds',
        'robocon_2027.sdf',
    ])
    gz_launch = PathJoinSubstitution([
        FindPackageShare('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py',
    ])
    verbosity = LaunchConfiguration('verbosity')

    return LaunchDescription([
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            PathJoinSubstitution([FindPackageShare('ares_2027sim'), '..']),
        ),
        DeclareLaunchArgument(
            'verbosity',
            default_value='3',
            description='Gazebo console verbosity from 0 to 4',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={
                'gz_args': ['-r -v ', verbosity, ' ', world],
                'on_exit_shutdown': 'true',
            }.items(),
        ),
    ])
