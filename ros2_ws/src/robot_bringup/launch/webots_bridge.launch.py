"""Start a team Webots world with the pallet robot driven from ROS 2.

The world, PROTOs and facility controllers are copied unchanged to
``<repo>/.runtime/ros_bridge/`` (ignored by Git); only the copy's
TRANSPORT_ROBOT gets ``controller "<extern>"``. Webots runs on Windows and
opens that copy, so the facility supervisor (people, forklifts, orders, spills)
still finds its controllers. webots_ros2_driver connects from WSL over TCP.

    ros2 launch robot_bringup webots_bridge.launch.py world:=final_warehouse.wbt
"""
import os
import shutil
import subprocess
from pathlib import Path

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.utils import is_wsl
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import Ros2SupervisorLauncher

ROBOT_NAME = 'group18_pallet_robot'
ROBOT_NODE = 'DEF TRANSPORT_ROBOT PalletRobot {'
PORT = '1234'


def find_repo(override):
    if override:
        return Path(override).resolve()
    if os.environ.get('G18_REPO'):
        return Path(os.environ['G18_REPO']).resolve()
    # With --symlink-install this file resolves into the source tree.
    for parent in Path(os.path.realpath(__file__)).parents:
        if (parent / 'webots' / 'worlds').is_dir():
            return parent
    raise RuntimeError('Cannot find the repository; pass repo:=/path/to/robotics-group-18 or set G18_REPO.')


def prepare_world(repo, world_name, runtime=''):
    source = repo / 'webots'
    # Windows paths are limited to 260 characters; pass a short runtime:= if the
    # facility controller fails to open results/.../ground_truth.csv.
    runtime = Path(runtime) if runtime else repo / '.runtime' / 'ros_bridge'
    runtime.mkdir(parents=True, exist_ok=True)
    # Refresh everything except previous facility logs.
    for child in runtime.iterdir():
        if child.name != 'results':
            shutil.rmtree(child) if child.is_dir() else child.unlink()
    for child in source.iterdir():
        if child.name in ('results', 'docs'):
            continue
        target = runtime / child.name
        if child.is_dir():
            shutil.copytree(child, target, ignore=shutil.ignore_patterns('__pycache__', '.*.wbproj'))
        else:
            shutil.copy2(child, target)

    world = runtime / 'worlds' / world_name
    if not world.is_file():
        raise RuntimeError(f'World {world_name} not found in {source / "worlds"}')
    text = world.read_text(encoding='utf-8')
    if ROBOT_NODE not in text:
        raise RuntimeError(f'{world_name} has no "{ROBOT_NODE}" node to hand over to ROS.')
    text = text.replace(ROBOT_NODE, ROBOT_NODE + ' controller "<extern>"', 1)
    # webots_ros2's Ros2Supervisor publishes /clock, which steps ros2_control.
    text += '\nRobot { name "Ros2Supervisor" controller "<extern>" supervisor TRUE }\n'
    world.write_text(text, encoding='utf-8')
    return world


def launch_webots(context):
    repo = find_repo(LaunchConfiguration('repo').perform(context))
    world = prepare_world(repo, LaunchConfiguration('world').perform(context),
                          LaunchConfiguration('runtime').perform(context))
    mode = LaunchConfiguration('mode').perform(context)

    webots_home = os.environ.get('WEBOTS_HOME')
    if not webots_home:
        raise RuntimeError('Set WEBOTS_HOME to the Webots installation folder.')
    if is_wsl():
        executable = os.path.join(webots_home, 'msys64', 'mingw64', 'bin', 'webots.exe')
        world_arg = subprocess.check_output(['wslpath', '-w', str(world)]).decode().strip()
    else:
        executable = os.path.join(webots_home, 'webots')
        world_arg = str(world)

    webots = ExecuteProcess(
        cmd=[executable, '--batch', '--stdout', '--stderr', f'--mode={mode}', f'--port={PORT}', world_arg],
        name='webots', output='screen')
    shutdown_with_webots = RegisterEventHandler(launch.event_handlers.OnProcessExit(
        target_action=webots, on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())]))
    return [webots, shutdown_with_webots]


def generate_launch_description():
    package_dir = get_package_share_directory('robot_bringup')
    robot_description_path = os.path.join(package_dir, 'resource', 'pallet_robot.urdf')
    ros2_control_params = os.path.join(package_dir, 'config', 'ros2_control.yaml')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': '<robot name=""><link name=""/></robot>',
                     'use_sim_time': True}],
    )

    # robot_state_publisher receives the URDF that Webots generates from the PROTO
    # (base_link on the axle, front_lidar, wheels), so add the ground frame here.
    # Nav2, AMCL and slam_toolbox default to base_footprint.
    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['--z', '-0.15', '--frame-id', 'base_link', '--child-frame-id', 'base_footprint'],
        parameters=[{'use_sim_time': True}],
    )

    # Webots publishes the scan left to right (negative increment); slam_toolbox
    # needs it right to left. The driver writes /scan_raw, this node /scan.
    scan_normalizer = Node(package='robot_bringup', executable='scan_normalizer', output='screen',
                           parameters=[{'use_sim_time': True}])

    # Jazzy's diff_drive_controller takes geometry_msgs/TwistStamped on /cmd_vel.
    driver = WebotsController(
        robot_name=ROBOT_NAME,
        port=PORT,
        parameters=[
            {'robot_description': robot_description_path,
             'use_sim_time': True,
             'set_robot_state_publisher': True},
            ros2_control_params,
        ],
        remappings=[('/diffdrive_controller/cmd_vel', '/cmd_vel'),
                    ('/diffdrive_controller/odom', '/odom')],
        respawn=True,
    )

    spawners = [
        Node(package='controller_manager', executable='spawner', output='screen',
             arguments=[name, '--controller-manager-timeout', '60'])
        for name in ('joint_state_broadcaster', 'diffdrive_controller')
    ]

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='final_warehouse.wbt',
                              description='World file in webots/worlds/'),
        DeclareLaunchArgument('mode', default_value='realtime',
                              description='Webots start mode: realtime, fast or pause'),
        DeclareLaunchArgument('repo', default_value='',
                              description='Repository root (auto-detected if empty)'),
        DeclareLaunchArgument('runtime', default_value='',
                              description='Folder for the world copy (default <repo>/.runtime/ros_bridge); '
                                          'must be on a Windows drive when Webots runs on Windows'),
        OpaqueFunction(function=launch_webots),
        Ros2SupervisorLauncher(port=PORT),
        robot_state_publisher,
        footprint_publisher,
        scan_normalizer,
        driver,
        WaitForControllerConnection(target_driver=driver, nodes_to_start=spawners),
    ])
