#!/usr/bin/env python3
"""
bringup_circuit1.launch.py  —  nav_rtabmap
==========================================
One-command bringup: Stonefish simulator (rtabmap RViz config) +
RTAB-Map SLAM + scan_relay + arm_tuck + RRT* + DWA + Frontier explorer.

Scenario: turtlebot_hoi_circuit1.scn  (small/medium environment)

HOW TO RUN
----------
  source ~/ros2_ws/install/setup.bash
  ros2 launch nav_rtabmap bringup_circuit1.launch.py

  # Then start exploration:
  ros2 topic pub /explorer_start std_msgs/msg/Bool "data: true" --once

CIRCUIT1 vs INTEGRATION (bigger map) DIFFERENCES
-------------------------------------------------
  Icp/MaxCorrespondenceDistance : 0.20  (tight — small map, less drift)
  Icp/MaxTranslation            : 0.5   (more tolerant on short moves)
  Mem/NotLinkedNodesKept        : false (small map — no need to keep disconnected nodes)
  rrt_step_size                 : 0.3   (smaller steps for tighter corridors)
  rrt_max_iter                  : 5000
  robot_radius                  : 0.17
  v_max                         : 0.12  (slower — safer in narrow circuit1 corridors)
  sim_time                      : 3.0
  w_clearance                   : 0.8   (strong wall-clearance preference)
  dbscan_eps                    : 3.0   (tight clustering for small frontiers)
  entropy_radius                : 5
  distance_threshold            : 0.3

KEY FIXES (all applied, same as bringup.launch.py)
---------------------------------------------------
1. QoS: map subscribers use RELIABLE + TRANSIENT_LOCAL — in Python nodes
2. global_planner map_frame = 'map' (was defaulting to 'world_enu')
3. dwa_planner    map_frame = 'map'
4. replan_interval = 0.5s + lookahead_waypoints = 4 for fast wall detection
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

MAP_TOPIC    = '/rtabmap/grid_map'
ROBOT_RADIUS = 0.17   # metres — tight corridors: smaller than circuit2


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── 0. Stonefish simulator + RViz (rtabmap config) ────────────────────
    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot_simulation'),
                'launch', 'turtlebot_basic.launch.py'
            ])
        ]),
        launch_arguments={
            'scenario_description': PathJoinSubstitution([
                FindPackageShare('turtlebot_simulation'),
                'scenarios', 'turtlebot_hoi_circuit1.scn'
            ]),
            'rviz_config': PathJoinSubstitution([
                FindPackageShare('turtlebot_description'),
                'rviz', 'turtlebot_rtabmap.rviz'
            ]),
        }.items()
    )

    # ── 1. scan_relay — fix Stonefish clockwise LiDAR ─────────────────────
    scan_relay = Node(
        package='nav_rtabmap',
        executable='scan_relay',
        name='scan_relay',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── 1.5. arm_tuck — tuck SwiftPro arm ────────────────────────────────
    arm_tuck = Node(
        package='nav_rtabmap',
        executable='arm_tuck',
        name='arm_tuck',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'target_joint2': -0.25,
            'target_joint3': -1.5,
        }],
    )

    # ── 2. RTAB-Map SLAM — tuned for small circuit1 environment ──────────
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time':                  use_sim_time,
            'frame_id':                      'turtlebot/base_link',
            'map_frame_id':                  'map',
            'odom_frame_id':                 'world_enu',
            'subscribe_scan':                True,
            'subscribe_depth':               False,
            'subscribe_rgb':                 False,
            'approx_sync':                   True,
            'Reg/Strategy':                  '1',
            'Reg/Force3DoF':                 'true',
            'Grid/RangeMin':                 '0.2',
            # circuit1: tighter ICP — smaller map means less odometry drift
            # ── anti-clutter: free-space ray-tracing + noise filter + finer grid for circuit1 ──
            'Grid/RayTracing':                   'true',   # erase free cells along each ray — prevents ghost obstacles
            'Grid/NoiseFilteringMinNeighbors':   '5',      # remove isolated occupied cells
            'Grid/NoiseFilteringRadius':         '0.5',    # search radius (m) for noise-neighbour check
            'Grid/CellSize':                     '0.05',   # 5 cm cells — fine resolution for narrow corridors
            'Mem/LaserScanVoxelSize':            '0.03',   # tighter voxel for small map — keeps sharp corner detail
            'Mem/NotLinkedNodesKept':        'false',   # small map — don't keep dead nodes
            'RGBD/NeighborLinkRefining':     'true',
            'RGBD/LoopClosureReextractFeatures': 'true',
            'Optimizer/GravitySigma':        '0',
            'Optimizer/Robust':              'true',
            'Icp/MaxTranslation':            '0.5',     # tolerant on short moves
            'Icp/MaxRotation':               '1.0',
            'Icp/MaxCorrespondenceDistance': '0.20',    # tight — less drift in small map
            'Icp/PointToPlane':              'false',
        }],
        remappings=[
            ('scan',  '/turtlebot/scan_rtabmap'),
            ('odom',  '/turtlebot/odom'),
            ('map',   MAP_TOPIC),
        ],
        arguments=['-d'],
    )

    # ── 3. RRT* global planner — tuned for small circuit1 environment ─────
    global_planner = Node(
        package='nav_rtabmap',
        executable='global_planner',
        name='global_planner',
        output='screen',
        parameters=[{
            'use_sim_time':         use_sim_time,
            'use_sim':              True,
            'use_saved_map':        False,
            'map_topic':            MAP_TOPIC,
            'map_frame':            'map',       # FIX: was defaulting to 'world_enu'
            'robot_radius':         ROBOT_RADIUS,
            'rrt_max_iter':         5000,        # smaller map needs fewer iters
            'rrt_step_size':        0.3,         # smaller steps for tighter corridors
            'replan_interval':      0.5,         # check every 0.5 s — fast wall detection
            'waypoint_tol':         0.25,
            'replan_confirmations': 2,           # 2 × 0.5 s = 1 s before replan
            'replan_cooldown':      2.0,         # recover quickly after replan
            'lookahead_waypoints':  4,           # look 4 wp = ~1.2 m ahead
        }],
    )

    # ── 4. DWA local planner — tuned for small circuit1 environment ───────
    dwa_planner = Node(
        package='nav_rtabmap',
        executable='dwa_planner',
        name='dwa_planner',
        output='screen',
        parameters=[{
            'use_sim_time':              use_sim_time,
            'use_sim':                   True,
            'map_frame':                 'map',   # must match RTAB-Map map_frame_id
            'robot_radius':              0.12,    # smaller for tight circuit1 corridors
            'v_max':                     0.12,    # slower — safer in narrow passages
            'w_max':                     1.2,
            'a_w_max':                   1.5,
            'a_lin_max':                 2.0,
            'v_samples':                 8,
            'w_samples':                 21,
            'sim_time':                  3.0,
            'sim_steps':                 20,
            'waypoint_tol':              0.25,
            'w_heading':                 2.5,
            'w_clearance':               0.8,    # strong wall-clearance — vital in narrow corridors
            'w_velocity':                0.5,
            'dt':                        0.1,
            'k_w':                       1.4,
            'angle_threshold':           0.65,
            'angle_exit_ratio':          0.70,
            'path_interp_spacing':       0.10,
            'waypoint_lookahead_dist':   0.35,
            'stuck_lin_speed_eps':       0.01,
            'stuck_ticks_before_replan': 30,
            'plan_max_age':              3.0,
            'use_scan_collision':        True,
            'scan_check_radius':         0.25,   # tighter — circuit1 has narrow turns
        }],
        remappings=[
            ('/projected_map', MAP_TOPIC),
            ('/scan_fixed', '/turtlebot/scan_rtabmap'),
        ],
    )

    # ── 5. Frontier explorer — tuned for small circuit1 environment ───────
    frontier_explorer = Node(
        package='nav_rtabmap',
        executable='frontier_explorer',
        name='frontier_explorer',
        output='screen',
        parameters=[{
            'use_sim_time':        use_sim_time,
            'use_sim':             True,
            'map_topic':           MAP_TOPIC,
            'distance_threshold':  0.3,    # tight clustering for small frontiers
            'map_update_interval': 1.0,
            'explore_interval':    1.0,
            'length_weight':       0.6,
            'distance_weight':     0.4,
            'entropy_radius':      5,      # small window — circuit1 is compact
            'entropy_rays':        12,
            'entropy_max_cells':   60,
            'dbscan_eps':          3.0,    # tight clustering
            'dbscan_min_samples':  4,
            'goal_tolerance':      0.40,
            'max_nav_attempts':    10,
            'lookahead_distance':  1.5,    # pre-compute next goal 1.5 m before arrival
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Stonefish does NOT publish /clock — keep false'),

        simulator,
        scan_relay,
        arm_tuck,
        rtabmap,
        global_planner,
        dwa_planner,
        frontier_explorer,
    ])
