#!/usr/bin/env python3
"""
bringup_circuit2.launch.py  —  nav_rtabmap
==========================================
One-command bringup: Stonefish simulator (rtabmap RViz config) +
RTAB-Map SLAM + scan_relay + arm_tuck + RRT* + DWA + Frontier explorer.

Tuned for hoi_circuit2 (large environment).

HOW TO RUN
----------
  source ~/ros2_ws/install/setup.bash
  ros2 launch nav_rtabmap bringup_circuit2.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

MAP_TOPIC    = '/rtabmap/grid_map'
ROBOT_RADIUS = 0.20


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
                'scenarios', 'turtlebot_hoi_circuit2.scn'
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

    # ── 1.5. arm_tuck — tuck SwiftPro arm ─────────────────────────────────
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

    # ── 2. RTAB-Map SLAM — tuned for large environment ────────────────────
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
            'Reg/Strategy':                      '1',
            'Reg/Force3DoF':                     'true',
            'Grid/RangeMin':                     '0.2',
            'Mem/NotLinkedNodesKept':            'true',    # was false
            'RGBD/NeighborLinkRefining':         'true',
            'RGBD/LoopClosureReextractFeatures': 'true',    # added
            'Optimizer/GravitySigma':            '0',
            'Optimizer/Robust':                  'true',
            'Icp/MaxTranslation':                '0.3',     # was 0.5
            'Icp/MaxRotation':                   '1.0',
            'Icp/MaxCorrespondenceDistance':     '0.40',    # was 0.2
            'Icp/PointToPlane':                  'false',
        }],
        remappings=[
            ('scan',  '/turtlebot/scan_rtabmap'),
            ('odom',  '/turtlebot/odom'),
            ('map',   MAP_TOPIC),
        ],
        arguments=['-d'],
    )

    # ── 3. RRT* global planner — tuned for large environment ─────────────
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
            'rrt_step_size':        0.5,     # was 0.3
            'replan_interval':      2.0,
            'waypoint_tol':         0.30,    # was 0.25
            'replan_confirmations': 3,
            'replan_cooldown':      5.0,
        }],
    )

    # ── 4. DWA local planner — tuned for large environment ───────────────
    dwa_planner = Node(
        package='nav_rtabmap',
        executable='dwa_planner',
        name='dwa_planner',
        output='screen',
        parameters=[{
            'use_sim_time':              use_sim_time,
            'use_sim':                   True,
            'map_frame':                 'world_enu',
            'robot_radius':              0.15,    # was 0.12
            'v_max':                     0.18,    # was 0.12
            'w_max':                     1.2,
            'a_w_max':                   1.5,
            'v_samples':                 8,
            'w_samples':                 21,
            'sim_time':                  3.0,
            'sim_steps':                 20,
            'waypoint_tol':              0.30,    # was 0.25
            'w_heading':                 3.0,
            'w_clearance':               0.5,
            'w_velocity':                0.5,
            'dt':                        0.1,
            'k_w':                       1.4,
            'angle_threshold':           0.40,
            'angle_exit_ratio':          0.70,
            'path_interp_spacing':       0.10,
            'waypoint_lookahead_dist':   0.50,    # was 0.35
            'stuck_lin_speed_eps':       0.02,
            'stuck_ticks_before_replan': 25,
            'plan_max_age':              3.0,
        }],
        remappings=[('/projected_map', MAP_TOPIC)],
    )

    # ── 5. Frontier explorer — tuned for large environment ───────────────
    frontier_explorer = Node(
        package='nav_rtabmap',
        executable='frontier_explorer',
        name='frontier_explorer',
        output='screen',
        parameters=[{
            'use_sim_time':        use_sim_time,
            'use_sim':             True,
            'map_topic':           MAP_TOPIC,
            'distance_threshold':  0.5,     # was 0.3
            'map_update_interval': 1.0,
            'explore_interval':    1.0,
            'length_weight':       0.6,
            'distance_weight':     0.4,
            'entropy_radius':      8,       # was 5
            'dbscan_eps':          5.0,     # was 3.0
            'dbscan_min_samples':  4,
            'goal_tolerance':      0.50,    # was 0.40
            'max_nav_attempts':    10,
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
