#!/usr/bin/env python3
"""
nav_rtabmap.launch.py
=====================
RTAB-Map 2-D LiDAR SLAM  +  RRT* global planner  +  DWA local planner
                          +  Frontier-based exploration

HOW TO RUN
----------
  Terminal 1 — simulator + RViz (rtabmap config):
    source ~/ros2_ws/install/setup.bash
    ros2 launch turtlebot_simulation turtlebot_hoi_circuit1.launch.py \
      rviz_config:=$(ros2 pkg prefix turtlebot_description)/share/turtlebot_description/rviz/turtlebot_rtabmap.rviz

  Terminal 2 — this launch:
    source ~/ros2_ws/install/setup.bash
    ros2 launch nav_rtabmap nav_rtabmap.launch.py

  Terminal 3 — (optional) start autonomous frontier exploration:
    ros2 topic pub /explorer_start std_msgs/msg/Bool "data: true" --once

  OR just click "2D Goal Pose" in RViz for a manual goal.

FRAME TREE
----------
  map  ─(static 0,0,0)─►  world_enu          (published by this launch)
  world_enu  ──────────►  turtlebot/base_footprint ──► turtlebot/base_link
                                              (Stonefish diff_drive_odometry)

  rtabmap map_frame_id = world_enu, odom_frame_id = world_enu
  → map and world_enu are the same physical origin; the static TF is
    needed so RViz (Fixed Frame: map) can render topics in world_enu.

TOPICS
------
  /rtabmap/grid_map     OccupancyGrid   SLAM map (from rtabmap, remapped)
  /plan                 Path            RRT* global path
  /local_path           Path            DWA arc
  /dwa_trajectories     MarkerArray     DWA sample fan
  /goal_pose            PoseStamped     goal input  (from RViz 2D Goal Pose)
  /explorer_start       Bool            start/stop frontier exploration
  /turtlebot/cmd_vel    Twist           velocity output to robot
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ── constants ─────────────────────────────────────────────────────────────────
MAP_TOPIC    = '/rtabmap/grid_map'
ROBOT_RADIUS = 0.17   # metres — safe for straight corridors (0.03 for S-bends)


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    # NOTE: No static TF needed here.
    # rtabmap with map_frame_id='map', odom_frame_id='world_enu' publishes the
    # TF  map → world_enu  dynamically as SLAM corrects the pose estimate.
    # A static TF would conflict with that dynamic transform.

    # ── 1. scan_relay — fix Stonefish clockwise LiDAR convention ─────────────
    # Stonefish mounts the RPLidar with roll=-180°, making angles run clockwise.
    # rtabmap expects standard CCW.  This node re-publishes the scan with
    # negated angles and reversed ranges, in the turtlebot/base_link frame.
    scan_relay = Node(
        package='nav_rtabmap',
        executable='scan_relay',
        name='scan_relay',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── 2. RTAB-Map SLAM ──────────────────────────────────────────────────────
    # Wheel odometry (/turtlebot/odom) is used directly as the pose source.
    # map_frame_id = odom_frame_id = world_enu  → no separate map→odom TF is
    # published; world_enu acts as both map and odom reference.
    # -d  deletes the database on every launch for a clean map each session.
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time':                  use_sim_time,
            'frame_id':                      'turtlebot/base_link',
            # map_frame_id MUST differ from odom_frame_id.
            # rtabmap publishes the TF  map → odom_frame_id (world_enu) dynamically.
            # Setting both to world_enu causes TF_SELF_TRANSFORM errors and no map.
            'map_frame_id':                  'map',
            'odom_frame_id':                 'world_enu',
            'subscribe_scan':                True,
            'subscribe_depth':               False,
            'subscribe_rgb':                 False,
            'approx_sync':                   True,
            # scan-matching registration
            'Reg/Strategy':                  '1',     # ICP
            'Reg/Force3DoF':                 'true',
            'Grid/RangeMin':                 '0.2',
            'Mem/NotLinkedNodesKept':        'false',
            'RGBD/NeighborLinkRefining':     'true',
            'Optimizer/GravitySigma':        '0',
            'Optimizer/Robust':              'true',
            # ICP tolerances tuned for 5 Hz Stonefish scan
            'Icp/MaxTranslation':            '0.5',
            'Icp/MaxRotation':               '1.0',
            'Icp/MaxCorrespondenceDistance': '0.2',
            'Icp/PointToPlane':              'false',
        }],
        remappings=[
            ('scan', '/turtlebot/scan_rtabmap'),   # corrected scan from relay
            ('odom', '/turtlebot/odom'),
            # rtabmap publishes OccupancyGrid on relative topic "map"
            # → remap so planners and RViz both see /rtabmap/grid_map
            ('map',  MAP_TOPIC),
        ],
        arguments=['-d'],
    )

    # ── 3. RRT* global planner ────────────────────────────────────────────────
    # Receives goal from /goal_pose, plans through the SLAM map, publishes
    # waypoint path to /plan.  Replans automatically if the path is blocked.
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
            'rrt_max_iter':         5000,
            'rrt_step_size':        0.3,
            'replan_interval':      2.0,
            'waypoint_tol':         0.25,
            'replan_confirmations': 3,
            'replan_cooldown':      5.0,
        }],
    )

    # ── 4. DWA local planner ──────────────────────────────────────────────────
    # Converts the RRT* waypoint path into smooth velocity commands using
    # Dynamic Window Approach.  Publishes cmd_vel to the robot.
    dwa_planner = Node(
        package='nav_rtabmap',
        executable='dwa_planner',
        name='dwa_planner',
        output='screen',
        parameters=[{
            'use_sim_time':              use_sim_time,
            'use_sim':                   True,
            'map_frame':                 'world_enu',
            'robot_radius':              0.12,    # inflated collision radius — smaller than
                                                  # global_planner (0.17) to let DWA manoeuvre
                                                  # along the planned path, but large enough to
                                                  # match the physical TurtleBot body (~0.17 m)
            'v_max':                     0.12,
            'w_max':                     1.2,
            'a_w_max':                   1.5,
            'v_samples':                 8,
            'w_samples':                 21,
            'sim_time':                  3.0,   # 3 s lookahead → arcs up to 0.36 m at v_max=0.12
            'sim_steps':                 20,    # 0.15 s per step, same as v1_handson
            'waypoint_tol':              0.25,
            'w_heading':                 2.5,
            'w_clearance':               0.8,    # strong clearance preference — keeps robot away from walls
            'w_velocity':                0.5,
            'dt':                        0.1,
            'k_w':                       1.4,
            'angle_threshold':           0.65,
            'angle_exit_ratio':          0.70,
            'path_interp_spacing':       0.10,
            'waypoint_lookahead_dist':   0.35,
            'stuck_lin_speed_eps':       0.02,
            'stuck_ticks_before_replan': 25,
            'plan_max_age':              3.0,
        }],
        # DWA subscribes to /projected_map by default — remap to rtabmap topic
        remappings=[('/projected_map', MAP_TOPIC)],
    )

    # ── 5. Frontier explorer ──────────────────────────────────────────────────
    # Autonomously picks unexplored frontier regions and sends goals to the
    # global planner.  Send Bool(true) to /explorer_start to activate.
    frontier_explorer = Node(
        package='nav_rtabmap',
        executable='frontier_explorer',
        name='frontier_explorer',
        output='screen',
        parameters=[{
            'use_sim_time':         use_sim_time,
            'use_sim':              True,
            'map_topic':            MAP_TOPIC,
            'distance_threshold':   0.3,
            'map_update_interval':  1.0,
            'explore_interval':     1.0,
            'length_weight':        0.6,
            'distance_weight':      0.4,
            'entropy_radius':       5,
            'dbscan_eps':           3.0,
            'dbscan_min_samples':   4,
            'goal_tolerance':       0.40,
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
