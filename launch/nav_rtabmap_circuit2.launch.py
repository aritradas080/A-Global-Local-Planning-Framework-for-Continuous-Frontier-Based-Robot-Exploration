#!/usr/bin/env python3
"""
nav_rtabmap_circuit2.launch.py
==============================
RTAB-Map 2-D LiDAR SLAM  +  RRT* global planner  +  DWA local planner
                          +  Frontier-based exploration

TUNED FOR: hoi_circuit2  (large environment)
  - Looser ICP tolerances to handle longer odometry drift
  - Larger RRT* step size so planning is fast over a big map
  - Higher v_max and longer sim_time for faster exploration
  - Wider DBSCAN clustering and entropy radius for large frontiers

HOW TO RUN
----------
  Terminal 1 — simulator + RViz:
    source ~/ros2_ws/install/setup.bash
    ros2 launch turtlebot_simulation turtlebot_hoi_circuit2.launch.py \
      rviz_config:=$(ros2 pkg prefix turtlebot_description)/share/turtlebot_description/rviz/turtlebot_rtabmap.rviz

  Terminal 2 — this launch:
    source ~/ros2_ws/install/setup.bash
    ros2 launch nav_rtabmap nav_rtabmap_circuit2.launch.py

  Terminal 3 — (optional) start autonomous frontier exploration:
    ros2 topic pub /explorer_start std_msgs/msg/Bool "data: true" --once

  OR just click "2D Goal Pose" in RViz for a manual goal.

FRAME TREE
----------
  map  ──(dynamic, rtabmap)──►  world_enu
  world_enu  ──────────────►  turtlebot/base_footprint ──► turtlebot/base_link

DIFFERENCE vs nav_rtabmap.launch.py (circuit1)
-----------------------------------------------
  ICP/MaxCorrespondenceDistance : 0.2  → 0.40   (tolerates more drift in big map)
  ICP/MaxTranslation            : 0.5  → 0.3    (reject wild ICP jumps)
  Mem/NotLinkedNodesKept        : false → true   (keep nodes for loop closure)
  rrt_step_size                 : 0.3  → 0.5    (faster planning over large space)
  rrt_max_iter                  : 5000 → 8000   (more iterations for longer paths)
  robot_radius (global)         : 0.17 → 0.20   (slightly more conservative in big space)
  v_max                         : 0.12 → 0.18   (faster travel in open areas)
  sim_time                      : 3.0  → 5.0    (longer lookahead for higher speed)
  waypoint_lookahead_dist       : 0.35 → 0.50   (smoother tracking at higher speed)
  dbscan_eps                    : 3.0  → 5.0    (wider frontier clustering)
  entropy_radius                : 5    → 8      (larger viewpoint info gain radius)
  distance_threshold            : 0.3  → 0.5    (merge closer frontiers in big map)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ── constants ─────────────────────────────────────────────────────────────────
MAP_TOPIC    = '/rtabmap/grid_map'
ROBOT_RADIUS = 0.20   # metres — slightly larger for circuit2 open corridors


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── 1. scan_relay — fix Stonefish clockwise LiDAR convention ─────────────
    scan_relay = Node(
        package='nav_rtabmap',
        executable='scan_relay',
        name='scan_relay',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── 2. RTAB-Map SLAM — tuned for large environment ────────────────────────
    # Key changes vs circuit1:
    #   Icp/MaxCorrespondenceDistance 0.40  — tolerates ~20–30 cm odometry drift
    #   Icp/MaxTranslation            0.30  — rejects wild ICP corrections
    #   Mem/NotLinkedNodesKept        true  — keeps all nodes so loop closure has
    #                                         more candidates when revisiting distant areas
    #   RGBD/LoopClosureReextractFeatures   — re-extracts features at loop closure
    #                                         for more reliable long-range recognition
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
            'Reg/Strategy':                      '1',      # ICP
            'Reg/Force3DoF':                     'true',
            'Grid/RangeMin':                     '0.2',
            'Mem/NotLinkedNodesKept':            'true',   # was false — keep all nodes for loop closure
            'RGBD/NeighborLinkRefining':         'true',
            'RGBD/LoopClosureReextractFeatures': 'true',   # better long-range loop closure
            'Optimizer/GravitySigma':            '0',
            'Optimizer/Robust':                  'true',
            # ICP tolerances — looser for large environment with more drift
            'Icp/MaxTranslation':                '0.3',    # was 0.5 — reject wild jumps
            'Icp/MaxRotation':                   '1.0',
            'Icp/MaxCorrespondenceDistance':     '0.40',   # was 0.2 — handles ~20-30cm drift
            'Icp/PointToPlane':                  'false',
        }],
        remappings=[
            ('scan', '/turtlebot/scan_rtabmap'),
            ('odom', '/turtlebot/odom'),
            ('map',  MAP_TOPIC),
        ],
        arguments=['-d'],   # delete db on every launch for clean map
    )

    # ── 3. RRT* global planner — tuned for large environment ─────────────────
    # Key changes vs circuit1:
    #   rrt_step_size  0.5  — larger steps cover more ground per iteration
    #   rrt_max_iter   8000 — more iterations needed for longer paths
    #   robot_radius   0.20 — slightly more conservative inflation
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
            'robot_radius':         ROBOT_RADIUS,
            'rrt_max_iter':         8000,    # was 5000
            'rrt_step_size':        0.5,     # was 0.3 — faster planning in large map
            'replan_interval':      2.0,
            'waypoint_tol':         0.30,    # was 0.25 — slightly more tolerance
            'replan_confirmations': 3,
            'replan_cooldown':      5.0,
        }],
    )

    # ── 4. DWA local planner — tuned for large environment ───────────────────
    # Key changes vs circuit1:
    #   v_max                   0.18  — faster in open circuit2 corridors
    #   sim_time                5.0   — longer lookahead matches higher speed
    #   waypoint_lookahead_dist 0.50  — smoother tracking at higher speed
    #   robot_radius            0.15  — slightly larger collision radius
    dwa_planner = Node(
        package='nav_rtabmap',
        executable='dwa_planner',
        name='dwa_planner',
        output='screen',
        parameters=[{
            'use_sim_time':              use_sim_time,
            'use_sim':                   True,
            'map_frame':                 'world_enu',
            'robot_radius':              0.15,    # slightly larger for circuit2
            'v_max':                     0.18,    # was 0.12 — faster in open areas
            'w_max':                     1.2,
            'a_w_max':                   1.5,
            'v_samples':                 8,
            'w_samples':                 21,
            'sim_time':                  5.0,     # was 3.0 — longer lookahead at higher speed
            'sim_steps':                 20,
            'waypoint_tol':              0.30,    # was 0.25
            'w_heading':                 2.5,
            'w_clearance':               0.8,
            'w_velocity':                0.5,
            'dt':                        0.1,
            'k_w':                       1.4,
            'angle_threshold':           0.65,
            'angle_exit_ratio':          0.70,
            'path_interp_spacing':       0.10,
            'waypoint_lookahead_dist':   0.50,    # was 0.35 — smoother at higher speed
            'stuck_lin_speed_eps':       0.02,
            'stuck_ticks_before_replan': 25,
            'plan_max_age':              3.0,
        }],
        remappings=[('/projected_map', MAP_TOPIC)],
    )

    # ── 5. Frontier explorer — tuned for large environment ───────────────────
    # Key changes vs circuit1:
    #   dbscan_eps        5.0  — wider clustering for large frontier regions
    #   entropy_radius    8    — larger information gain radius
    #   distance_threshold 0.5 — merge frontiers that are within 50 cm
    #   goal_tolerance    0.50 — more tolerant goal arrival in big space
    frontier_explorer = Node(
        package='nav_rtabmap',
        executable='frontier_explorer',
        name='frontier_explorer',
        output='screen',
        parameters=[{
            'use_sim_time':         use_sim_time,
            'use_sim':              True,
            'map_topic':            MAP_TOPIC,
            'distance_threshold':   0.5,     # was 0.3 — merge closer frontiers
            'map_update_interval':  1.0,
            'explore_interval':     1.0,
            'length_weight':        0.6,
            'distance_weight':      0.4,
            'entropy_radius':       8,       # was 5 — larger info gain radius
            'dbscan_eps':           5.0,     # was 3.0 — wider frontier clustering
            'dbscan_min_samples':   4,
            'goal_tolerance':       0.50,    # was 0.40
            'max_nav_attempts':     10,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use /clock — set true only if simulator publishes /clock (Stonefish does NOT)'),

        scan_relay,
        rtabmap,
        global_planner,
        dwa_planner,
        frontier_explorer,
    ])
