#!/usr/bin/env python3
"""
bringup.launch.py  —  nav_rtabmap
====================================
One-command bringup: Stonefish simulator (rtabmap RViz config) +
RTAB-Map SLAM + scan_relay + arm_tuck + RRT* + DWA + Frontier explorer.

Scenario: turtlebot_integration.scn  (large integration environment)

HOW TO RUN
----------
  source ~/ros2_ws/install/setup.bash
  ros2 launch nav_rtabmap bringup.launch.py

  # Then start exploration:
  ros2 topic pub /explorer_start std_msgs/msg/Bool "data: true" --once

RViz opens with Fixed Frame = map and all planning displays:
  /rtabmap/grid_map OccupancyGrid  SLAM map
  /plan             Path           RRT* global path
  /local_path       Path           DWA arc
  /dwa_trajectories MarkerArray    DWA fan (blue/green)

KEY FIXES APPLIED
-----------------
1. QoS: all map subscribers now use RELIABLE + TRANSIENT_LOCAL (depth=1)
   to match RTAB-Map's latched publisher — fixes "no map yet" in global_planner.
2. global_planner: map_frame set to 'map' (was defaulting to 'world_enu').
3. dwa_planner:    map_frame = 'map' (was 'world_enu' in earlier versions).
4. RTAB-Map:       Mem/NotLinkedNodesKept=true, wider ICP, loop-closure
   re-extraction — all tuned for large-map stability.
5. Scenario:       turtlebot_integration.scn (large environment).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

MAP_TOPIC    = '/rtabmap/grid_map'
ROBOT_RADIUS = 0.14   # metres — TurtleBot body radius (used by RRT* inflation + DWA)


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── 0. Stonefish simulator + RViz (rtabmap config) 
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
                'scenarios', 'turtlebot_integration.scn'
            ]),
            # override rviz_config directly — loads rtabmap displays
            # (Fixed Frame = map, /rtabmap/grid_map, /dwa_trajectories, /plan)
            'rviz_config': PathJoinSubstitution([
                FindPackageShare('turtlebot_description'),
                'rviz', 'turtlebot_rtabmap.rviz'
            ]),
        }.items()
    )

    # ── 1. scan_relay — fix Stonefish clockwise LiDAR 
    scan_relay = Node(
        package='nav_rtabmap',
        executable='scan_relay',
        name='scan_relay',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )


    # ── 1.5. arm_tuck — tuck SwiftPro arm 
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
    # ── 2. RTAB-Map SLAM — tuned for large integration environment 
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time':                      use_sim_time,
            'frame_id':                          'turtlebot/base_link',
            'map_frame_id':                      'map',
            'odom_frame_id':                     'world_enu',
            'subscribe_scan':                    True,
            'subscribe_depth':                   False,
            'subscribe_rgb':                     False,
            'approx_sync':                       True,
            'Reg/Strategy':                      '1',      # ICP registration (not visual)
            'Reg/Force3DoF':                     'true',   # force 2D (x, y, yaw) — ground robot

            # ── scan range 
            'Grid/RangeMin':                     '0.2',    # ignore returns < 20 cm (scanner blind zone)
            'Grid/RangeMax':                     '10.0',   # full LiDAR range — room is ~25 m, 10 m gives good overlap

            # ── map resolution 
            # 10 cm cells for a 25 m room = 250×250 grid. Fine enough for navigation,
            # coarse enough that small ICP drift doesn't scatter cells across multiple rows.
            'Grid/CellSize':                     '0.10',

            # ── anti-clutter: ray-tracing + aggressive noise filter 
            'Grid/RayTracing':                   'true',   # erase free cells along each ray — prevents ghost-wall buildup
            # Raise MinNeighbors to 7 (was 5): in a cluttered map with many objects,
            # noise cells tend to cluster in groups of 2-4. Requiring 7 neighbours
            # removes those blobs while keeping real thin walls (which have 10+ neighbours).
            'Grid/NoiseFilteringMinNeighbors':   '7',
            'Grid/NoiseFilteringRadius':         '0.5',    # 50 cm search radius for neighbour check

            # ── scan downsampling & ICP 
            'Mem/LaserScanVoxelSize':            '0.05',   # 1 point per 5 cm voxel before ICP
            'Mem/NotLinkedNodesKept':            'true',   # large map: keep isolated nodes for future loop closure

            # ── ICP scan matching 
            # MaxCorrespondenceDistance 0.50 was too loose — accepting poor scan
            # matches caused the thick noisy walls visible in RViz. Tighten to 0.30:
            # scan points must land within 30 cm of their map match.
            'Icp/MaxCorrespondenceDistance':     '0.30',
            # MaxTranslation: at v_max=0.18 m/s with a ~0.2 s scan period the
            # robot moves at most ~0.04 m between scans. Keep 0.5 m as headroom.
            'Icp/MaxTranslation':                '0.5',
            'Icp/MaxRotation':                   '1.0',    # allow up to ~57° yaw between scans
            'Icp/PointToPlane':                  'false',  # point-to-point ICP — stable for 2D LiDAR
            # Require at least 60% of scan points to have a valid ICP correspondence
            # before accepting the match. Rejects scans when robot is in a doorway
            # or near a boundary where half the scan sees an unknown area.
            'Icp/CorrespondenceRatio':           '0.6',

            # ── loop closure & optimisation 
            'RGBD/NeighborLinkRefining':         'true',
            'RGBD/LoopClosureReextractFeatures': 'true',
            'Optimizer/GravitySigma':            '0',
            'Optimizer/Robust':                  'true',   # discard outlier loop closures
        }],
        remappings=[
            ('scan',  '/turtlebot/scan_rtabmap'),
            ('odom',  '/turtlebot/odom'),
            ('map',   MAP_TOPIC),
        ],
        arguments=['-d'],
    )

    # ── 3. RRT* global planner — tuned for large integration environment ──
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
            'map_frame':            'map',        # FIX: was defaulting to 'world_enu'
            'robot_radius':         ROBOT_RADIUS, # 0.17 m — used for path inflation
            'rrt_max_iter':         10000,        # 10 k iterations — needed for complex 25 m map
            'rrt_step_size':        0.4,          # 40 cm branches — threads through 1.5 m corridors safely
            'replan_interval':      0.5,          # check path validity every 0.5 s — fast detection
            'waypoint_tol':         0.35,         # waypoint reached within 35 cm

            # ── replanning debounce — KEY ANTI-THRASH PARAMS 
            # Root cause of continuous replanning:
            #   inflate_map() pads each occupied cell by r=ceil(0.17/0.10)=2 cells.
            #   Noisy map cells each create a 40×40 cm exclusion zone.
            #   _replan_check samples every 10 cm along lookahead_waypoints ahead —
            #   a noisy map almost always has a noise cell in that scan window.
            #   With replan_confirmations=2, one noise cell appearing for 1 s fired a replan.
            #
            # Fix 1 — replan_confirmations: 8
            #   Need 8 consecutive blocked detections (8 × 0.5 s = 4.0 s sustained).
            #   Real obstacles persist through all 8 checks.
            #   Noise cells are transient — they vanish in 1-2 map updates.
            'replan_confirmations': 8,

            # Fix 2 — replan_cooldown: 8.0 s
            #   After a replan fires, the new path is immune from another replan for 8 s.
            #   Was 6.0 s — not enough time for RRT* to finish AND robot to start moving
            #   away from the noisy area that triggered the last replan.
            'replan_cooldown':      8.0,

            # Fix 3 — lookahead_waypoints: 3
            #   Check 3 WPs × 0.4 m = 1.2 m ahead instead of 5 × 0.4 m = 2.0 m.
            #   Fewer sample points per check = less chance of a noise cell in the window.
            #   Still enough warning: at 0.18 m/s, 1.2 m = ~6.7 s of travel time.
            'lookahead_waypoints':  2,

            # Fix 4 — check_inflation_radius: 0.05 m
            #   _replan_check/_cell_free now uses a SEPARATE lightly-inflated map.
            #   robot_radius (0.15 m) is still used by RRT* for safe path planning.
            #   The checker only fires when the path is within 5 cm of a real wall
            #   cell — NOT within the large safety padding of a transient noise cell.
            #   This is the primary fix for "valid path but constant replanning".
            'check_inflation_radius': 0.05,
        }],
    )

    # ── 4. DWA local planner — tuned for large integration environment ───
    dwa_planner = Node(
        package='nav_rtabmap',
        executable='dwa_planner',
        name='dwa_planner',
        output='screen',
        parameters=[{
            'use_sim_time':              use_sim_time,
            'use_sim':                   True,
            'map_frame':                 'map',    # must match RTAB-Map map_frame_id

            # ── robot geometry 
            'robot_radius':              0.16,  # 0.17 m — match RRT* inflation radius

            # ── speed limits 
            'v_max':                     0.14,    # 18 cm/s — safe for this corridor width
            'w_max':                     0.8,     # max yaw rate — lower prevents overshoot
            'a_w_max':                   1.5,     # angular acceleration
            'a_lin_max':                 2.0,     # linear acceleration

            # ── DWA sampling 
            'v_samples':                 10,      # 10 forward speed samples
            'w_samples':                 21,      # 21 angular speed samples — finer steering
            'sim_time':                  3.0,     # simulate 3.0 s ahead for each candidate
            'sim_steps':                 20,
            'dt':                        0.1,

            # ── scoring weights 
            # w_clearance was 0.4 — robot barely avoided obstacles → direct wall hits
            # This map has many scattered objects; clearance MUST be the dominant weight
            'w_heading':                 1.5,     # reward facing the next waypoint
            'w_clearance':               0.8,     # CRITICAL: reward staying away from walls/objects
            'w_velocity':                0.6,     # reward forward progress (lower than clearance on purpose)

            # ── waypoint tracking 
            'waypoint_tol':              0.35,    # waypoint reached within 35 cm
            'waypoint_lookahead_dist':   0.35,    # aim 35 cm ahead on path for heading error
            'path_interp_spacing':       0.10,

            # ── rotation control 
            'k_w':                       2.5,
            'angle_threshold':           0.70,    # rotate-in-place if heading error > 40°
            'angle_exit_ratio':          0.30,

            # ── stuck / collision detection 
            'stuck_lin_speed_eps':       0.01,
            'stuck_ticks_before_replan': 50,      # ~5.0 s of no movement before declaring stuck
            'plan_max_age':              3.0,
            'use_scan_collision':        True,
            'scan_check_radius':         0.38,    # immediate collision check radius (>= robot_radius)
            'dwa_check_radius':          0.06,    # lighter inflation for clearance scoring only (Option B fix)
        }],
        remappings=[
            ('/projected_map', MAP_TOPIC),
            ('/scan_fixed', '/turtlebot/scan_rtabmap'),
        ],
    )

    # ── 5. Frontier explorer — tuned for large integration environment ───
    frontier_explorer = Node(
        package='nav_rtabmap',
        executable='frontier_explorer',
        name='frontier_explorer',
        output='screen',
        parameters=[{
            'use_sim_time':        use_sim_time,
            'use_sim':             True,
            'map_topic':           MAP_TOPIC,

            # ── frontier scoring 
            # entropy_radius: cells of ray-cast window around each frontier centroid.
            # With 0.10 m cells and entropy_max_cells=80: rays reach 8 m out.
            'entropy_radius':      8,
            'entropy_rays':        12,
            'entropy_max_cells':   80,

            # Scoring: score = norm_size × length_weight + inv_norm_dist × distance_weight
            # inv_norm_dist is HIGHER for CLOSER clusters → distance_weight biases robot toward NEARBY frontiers.
            # This is WHY the robot was always going to the nearest "Best Viewpoint".
            # Fix: length_weight=1.0, distance_weight=0.0 → score = cluster SIZE only.
            # The robot now always picks the LARGEST unexplored frontier region,
            # which is typically deeper into unmapped territory.
            'length_weight':       1.0,
            'distance_weight':     0.0,

            # ── clustering 
            'dbscan_eps':          5.0,    # group frontier cells within 5 cells (~50 cm)
            'dbscan_min_samples':  4,      # need at least 4 cells to form a valid cluster

            # ── navigation thresholds 
            'distance_threshold':  0.5,
            'goal_tolerance':      0.55,   # frontier reached when within 55 cm of centroid
            'max_nav_attempts':    15,

            # lookahead_distance: how far from the CURRENT goal the robot must be
            # before pre-computing the NEXT goal in the background.
            # Was 3.0 m — if goal is only 2 m away the pre-compute fires immediately
            # before the robot even moves, then goes stale by arrival time.
            # Set to 1.5 m: pre-compute starts in the last 1.5 m of each leg,
            # giving ~8 s of planning time at 0.18 m/s.
            'lookahead_distance':  1.5,

            # ── timing 
            'map_update_interval': 1.0,
            # explore_interval was 1.5 s: robot stopped at each goal and waited up
            # to 1.5 s before the next tick even computed a new frontier.
            # 0.5 s makes the transition nearly instantaneous.
            'explore_interval':    0.5,
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
