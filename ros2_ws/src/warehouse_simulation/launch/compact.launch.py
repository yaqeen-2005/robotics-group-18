"""Use the team's SDF world with Clearpath's generators, bridges and robot spawn."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    clearpath = Path(get_package_share_directory('clearpath_gz')) / 'launch'
    world = Path(__file__).resolve().parents[1] / 'worlds' / 'cargo_warehouse'

    def include(filename, arguments):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(clearpath / filename)),
            launch_arguments=arguments.items())

    setup = LaunchConfiguration('setup_path')
    return LaunchDescription([
        DeclareLaunchArgument('setup_path'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='-3.0'),
        DeclareLaunchArgument('z', default_value='0.3'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('on_exit_shutdown', default_value='true'),
        include('gz_sim.launch.py', {'world': str(world), 'setup_path': setup}),
        include('robot_spawn.launch.py', {
            'world': 'cargo_warehouse', 'setup_path': setup, 'use_sim_time': 'true',
            'rviz': LaunchConfiguration('rviz'),
            **{axis: LaunchConfiguration(axis) for axis in ('x', 'y', 'z', 'yaw')},
        }),
    ])
