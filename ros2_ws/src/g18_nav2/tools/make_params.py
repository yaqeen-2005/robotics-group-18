#!/usr/bin/env python3
"""Generate config/nav2_params.yaml from the Jazzy nav2_bringup defaults.

Every change from the defaults is made here, with the reason next to it, so the
YAML can be regenerated after a Nav2 upgrade:

    python3 src/g18_nav2/tools/make_params.py
"""
from pathlib import Path

import yaml

DEFAULTS = Path('/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml')
OUT = Path(__file__).resolve().parents[1] / 'config' / 'nav2_params.yaml'

# Robot outline in base_link (drive axle, x forward), from PalletRobot.proto:
# body 0.99 x 0.68 m plus the 0.94 m wide wheel track, LiDAR at the front.
FOOTPRINT = '[[0.56, 0.47], [0.56, -0.47], [-0.52, -0.47], [-0.52, 0.47]]'

p = yaml.safe_load(DEFAULTS.read_text())


def params(node):
    return p[node]['ros__parameters']


# Simulation time everywhere (Webots publishes /clock through Ros2Supervisor).
for node in p.values():
    if isinstance(node, dict) and 'ros__parameters' in node:
        node['ros__parameters']['use_sim_time'] = True

# Jazzy's diff_drive_controller only accepts geometry_msgs/TwistStamped.
for node in ('controller_server', 'behavior_server', 'velocity_smoother',
             'collision_monitor', 'docking_server'):
    params(node)['enable_stamped_cmd_vel'] = True

# --- Controller: Regulated Pure Pursuit, forward only (the LiDAR looks forward).
c = params('controller_server')
c['controller_frequency'] = 20.0
c['progress_checker']['required_movement_radius'] = 0.3
c['progress_checker']['movement_time_allowance'] = 15.0
c['general_goal_checker']['xy_goal_tolerance'] = 0.3
c['general_goal_checker']['yaw_goal_tolerance'] = 0.4
c['FollowPath'] = {
    'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
    'desired_linear_vel': 0.5,
    'lookahead_dist': 1.0,
    'min_lookahead_dist': 0.7,
    'max_lookahead_dist': 1.8,
    'lookahead_time': 1.5,
    'use_velocity_scaled_lookahead_dist': True,
    'rotate_to_heading_angular_vel': 0.6,
    'use_rotate_to_heading': True,
    'rotate_to_heading_min_angle': 0.6,
    'max_angular_accel': 1.5,
    'allow_reversing': False,
    'transform_tolerance': 0.2,
    'min_approach_linear_velocity': 0.05,
    'approach_velocity_scaling_dist': 0.8,
    'use_collision_detection': True,
    'max_allowed_time_to_collision_up_to_carrot': 1.5,
    'use_regulated_linear_velocity_scaling': True,
    'regulated_linear_scaling_min_radius': 1.0,
    'regulated_linear_scaling_min_speed': 0.2,
    'use_cost_regulated_linear_velocity_scaling': True,
    'cost_scaling_dist': 0.8,
    'cost_scaling_gain': 1.0,
    'inflation_cost_scaling_factor': 3.0,
    'use_fixed_curvature_lookahead': False,
    'interpolate_curvature_after_goal': False,
}

# --- Costmaps: rectangular footprint, LiDAR obstacles, unknown space is allowed.
scan = {
    'topic': '/scan',
    'data_type': 'LaserScan',
    'clearing': True,
    'marking': True,
    'max_obstacle_height': 2.0,
    'min_obstacle_height': 0.0,
    'raytrace_max_range': 12.0,
    'raytrace_min_range': 0.0,
    'obstacle_max_range': 11.5,
    'obstacle_min_range': 0.0,
}
inflation = {
    'plugin': 'nav2_costmap_2d::InflationLayer',
    'inflation_radius': 1.2,       # keep ~0.7 m from walls where there is room
    'cost_scaling_factor': 2.5,
}
for name in ('local_costmap', 'global_costmap'):
    cm = p[name][name]['ros__parameters']
    cm.pop('robot_radius', None)
    cm['footprint'] = FOOTPRINT
    cm['footprint_padding'] = 0.05
    cm['inflation_layer'] = dict(inflation)
    cm['obstacle_layer'] = {
        'plugin': 'nav2_costmap_2d::ObstacleLayer',
        'enabled': True,
        'observation_sources': 'scan',
        'scan': dict(scan),
    }
    cm.pop('voxel_layer', None)

local = p['local_costmap']['local_costmap']['ros__parameters']
local.update({'width': 8, 'height': 8, 'resolution': 0.05,
              'plugins': ['obstacle_layer', 'inflation_layer']})
local.pop('static_layer', None)

glob = p['global_costmap']['global_costmap']['ros__parameters']
# Unknown environment: the SLAM map starts as the patch in front of the robot,
# so a map-sized costmap would leave the robot and the goal outside it. Use a
# large window that follows the robot (covers the 96 x 64 m hall from anywhere);
# cells SLAM has not seen stay "unknown" and the planner may cross them.
glob.update({'resolution': 0.1, 'track_unknown_space': True,
             'rolling_window': True, 'width': 160, 'height': 160,
             'plugins': ['static_layer', 'obstacle_layer', 'inflation_layer']})
# The static layer follows slam_toolbox's growing /map.
glob['static_layer'] = {'plugin': 'nav2_costmap_2d::StaticLayer',
                        'map_subscribe_transient_local': True}

# --- Planner: plan through unexplored (unknown) cells; replanning fixes it later.
# NavFn plans for a point robot and only keeps distance through inflation, so a
# rectangular body (worse with a wide or long pallet) can clip racks at corners;
# RPP then stops with "collision ahead". Smac Hybrid-A* checks the full footprint
# (it follows cargo_manager's footprint changes) and plans forward-only arcs.
params('planner_server')['GridBased'] = {
    'plugin': 'nav2_smac_planner::SmacPlannerHybrid',
    'downsample_costmap': False,
    'tolerance': 0.5,
    'allow_unknown': True,
    'max_iterations': 1000000,
    'max_on_approach_iterations': 1000,
    'max_planning_time': 5.0,
    'motion_model_for_search': 'DUBIN',      # forward only: the LiDAR looks forward
    'angle_quantization_bins': 72,
    'analytic_expansion_ratio': 3.5,
    'analytic_expansion_max_length': 3.0,
    'minimum_turning_radius': 0.4,           # RPP can also turn in place
    'reverse_penalty': 2.0,
    'change_penalty': 0.0,
    'non_straight_penalty': 1.2,
    'cost_penalty': 2.0,
    'retrospective_penalty': 0.015,
    'lookup_table_size': 20.0,
    'cache_obstacle_heuristic': False,
    'smooth_path': True,
    'debug_visualizations': False,
}

# --- Behaviour tree: name the default tree explicitly so nav2.launch.py
# (planner:=astar) can swap it; RewrittenYaml only rewrites keys that exist.
params('bt_navigator')['default_nav_to_pose_bt_xml'] = (
    '/opt/ros/jazzy/share/nav2_bt_navigator/behavior_trees/'
    'navigate_to_pose_w_replanning_and_recovery.xml')


# --- Behaviours: never back up blind more than a little.
params('behavior_server')['max_rotational_vel'] = 0.6
params('behavior_server')['min_rotational_vel'] = 0.2
params('behavior_server')['rotational_acc_lim'] = 1.5

# --- Velocity limits (within robot_bringup/config/ros2_control.yaml limits).
v = params('velocity_smoother')
v.update({'max_velocity': [0.6, 0.0, 0.8], 'min_velocity': [0.0, 0.0, -0.8],
          'max_accel': [0.6, 0.0, 1.5], 'max_decel': [-1.0, 0.0, -1.5]})

# --- Collision monitor: slow down, then stop, when something is right ahead.
cmon = params('collision_monitor')
cmon['polygons'] = ['SlowZone', 'StopZone']
cmon['SlowZone'] = {
    'type': 'polygon',
    'points': '[[1.8, 0.6], [1.8, -0.6], [0.56, -0.6], [0.56, 0.6]]',  # 1.2 m ahead of the bumper
    'action_type': 'slowdown',
    'slowdown_ratio': 0.4,
    'min_points': 4,            # need 4 LiDAR points inside, so single noisy returns do not trigger it
    'visualize': True,
    'polygon_pub_topic': 'slow_zone',
    'enabled': True,
}
cmon['StopZone'] = {
    'type': 'velocity_polygon',
    'action_type': 'stop',
    'min_points': 4,
    'visualize': True,
    'polygon_pub_topic': 'stop_zone',
    'enabled': True,
    'holonomic': False,
    # checked in order; the first sub-polygon matching the current velocity applies
    'velocity_polygons': ['rotation', 'forward'],
    'rotation': {   # turning in place: circle swept by the body corners (0.73 m + margin)
        'points': '[[0.8, 0.0], [0.57, 0.57], [0.0, 0.8], [-0.57, 0.57], '
                  '[-0.8, 0.0], [-0.57, -0.57], [0.0, -0.8], [0.57, -0.57]]',
        'linear_min': -0.05, 'linear_max': 0.05,
        'theta_min': -2.0, 'theta_max': 2.0,
    },
    'forward': {    # driving forward: 0.4 m ahead of the bumper
        'points': '[[0.95, 0.5], [0.95, -0.5], [0.56, -0.5], [0.56, 0.5]]',
        'linear_min': 0.05, 'linear_max': 1.0,
        'theta_min': -2.0, 'theta_max': 2.0,
    },
}

# --- Keepout filter: reported spills (published by spill_keepout) are lethal.
for name in ('local_costmap', 'global_costmap'):
    cm = p[name][name]['ros__parameters']
    cm['filters'] = ['keepout_filter']
    cm['keepout_filter'] = {
        'plugin': 'nav2_costmap_2d::KeepoutFilter',
        'enabled': True,
        'filter_info_topic': '/costmap_filter_info',
    }

header = ('# GENERATED by tools/make_params.py from nav2_bringup defaults. '
          'Edit the script, not this file.\n')
OUT.write_text(header + yaml.safe_dump(p, sort_keys=False, default_flow_style=None, width=100))
print(f'wrote {OUT}')
