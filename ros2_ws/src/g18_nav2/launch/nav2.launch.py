"""Nav2 in an unknown warehouse: slam_toolbox builds the map while Nav2 drives.

Start the simulator first (robot_bringup webots_bridge.launch.py), then:

    ros2 launch g18_nav2 nav2.launch.py              # SLAM + Nav2 (+ RViz)
    ros2 launch g18_nav2 nav2.launch.py rviz:=false
    ros2 launch g18_nav2 nav2.launch.py planner:=astar   # plan with Yi's A* instead of Smac

The map frame starts at the robot's pose when SLAM starts (its parking bay).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg = get_package_share_directory('g18_nav2')
    nav2 = get_package_share_directory('nav2_bringup')
    slam = get_package_share_directory('slam_toolbox')

    params = LaunchConfiguration('params_file')
    slam_params = LaunchConfiguration('slam_params_file')

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam, 'launch', 'online_async_launch.py')),
        condition=IfCondition(LaunchConfiguration('slam')),
        launch_arguments={'use_sim_time': 'true', 'slam_params_file': slam_params,
                          'use_lifecycle_manager': 'true'}.items())

    # slam_toolbox's own launch activates the node from a launch event, which is
    # occasionally missed (the node then stays "inactive": no map->odom TF and Nav2
    # waits forever). A Nav2 lifecycle manager configures and activates it reliably.
    slam_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_slam', output='screen',
        condition=IfCondition(LaunchConfiguration('slam')),
        parameters=[{'use_sim_time': True, 'autostart': True,
                     'node_names': ['slam_toolbox'], 'bond_timeout': 0.0}])

    def navigation(context):
        # planner:=astar swaps the navigate_to_pose behaviour tree for one whose
        # ComputePathToPose calls astar_planner (Yi's A*) instead of planner_server.
        params_file = params
        if LaunchConfiguration('planner').perform(context) == 'astar':
            params_file = RewrittenYaml(
                source_file=params, root_key='', convert_types=True,
                param_rewrites={'default_nav_to_pose_bt_xml':
                                os.path.join(pkg, 'behavior_trees', 'navigate_to_pose_astar.xml')})
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2, 'launch', 'navigation_launch.py')),
            launch_arguments={'use_sim_time': 'true', 'params_file': params_file,
                              'autostart': 'true'}.items())]

    # The costmaps' KeepoutFilter waits for a mask; without this node they never
    # become current and no path is planned. It publishes an empty mask at start.
    spill_keepout = Node(package='g18_nav2', executable='spill_keepout', output='screen',
                         parameters=[{'use_sim_time': True}])

    astar = Node(package='g18_nav2', executable='astar_planner', output='screen',
                 condition=IfCondition(EqualsSubstitution(LaunchConfiguration('planner'), 'astar')),
                 parameters=[{'use_sim_time': True}])

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2, 'launch', 'rviz_launch.py')),
        condition=IfCondition(LaunchConfiguration('rviz')),
        launch_arguments={'use_sim_time': 'true'}.items())

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=os.path.join(pkg, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('slam_params_file', default_value=os.path.join(pkg, 'config', 'slam_mapping.yaml')),
        DeclareLaunchArgument('slam', default_value='true', description='Start slam_toolbox'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('planner', default_value='smac', choices=['smac', 'astar'],
                              description="Global planner: Smac Hybrid-A* or Yi's A*"),
        slam_launch,
        slam_lifecycle,
        # Give SLAM a moment to publish map->odom before the costmaps start.
        TimerAction(period=5.0, actions=[OpaqueFunction(function=navigation)]),
        spill_keepout,
        astar,
        rviz,
    ])
