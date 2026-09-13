#!/usr/bin/env python3
"""
turtlebot4_real.launch.py  --  nav_rtabmap
==========================================
Real TurtleBot 4 bringup for RTAB-Map 2-D LiDAR SLAM + RRT* + DWA +
Frontier exploration.

This launch file intentionally does not start simulation or robot bringup.
Run it after the TurtleBot 4 is already publishing:
  /scan       sensor_msgs/LaserScan
  /odom       nav_msgs/Odometry
  /tf         odom -> base_link
  /cmd_vel    geometry_msgs/TwistStamped

HOW TO RUN
----------
  source ~/ros2_ws/install/setup.bash
  ros2 launch nav_rtabmap turtlebot4_real.launch.py

Manual goal:
  Publish/click a PoseStamped goal on /goal_pose.

Autonomous frontier exploration:
  ros2 topic pub /explorer_start std_msgs/msg/Bool "data: true" --once
"""

from launch import LaunchDescription
from launch_ros.actions import Node


MAP_TOPIC = '/rtabmap/grid_map'
ROBOT_RADIUS = 0.10   # TB4 footprint ~0.17m — keep at 0.10 minimum


def generate_launch_description():

    use_sim_time = False
    scan_topic = '/scan'
    odom_topic = '/odom'
    base_frame = 'base_link'
    odom_frame = 'odom'
    map_frame = 'map'
    v_max = 0.15
    w_max = 0.5

    # 1. RTAB-Map SLAM
    # TurtleBot 4 publishes a standard CCW /scan, so the Stonefish-only
    # scan_relay from bringup.launch.py is not used here.
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time':                  use_sim_time,
            'frame_id':                      base_frame,
            'map_frame_id':                  map_frame,
            'odom_frame_id':                 odom_frame,
            'subscribe_scan':                True,
            'subscribe_depth':               False,
            'subscribe_rgb':                 False,
            'approx_sync':                   True,
            'tf_buffer_size':                60,
            'Reg/Strategy':                  '1',
            'Grid/RangeMin':                 '0.05',
            'Grid/3D':                       'false',
            'Grid/FromDepth':                'false',
            'Grid/MapFrameProjection':       'true',
            'Mem/InitWMWithAllNodes':        'true',
            'RGBD/StartAtOrigin':            'true',
            'RGBD/NeighborLinkRefining':     'true',
            'Optimizer/GravitySigma':        '0',
            'Optimizer/Robust':              'true',
            'Icp/MaxTranslation':            '0.5',
            'Icp/MaxRotation':               '1.0',
            'Icp/MaxCorrespondenceDistance': '0.2',
            'Icp/PointToPlane':              'false',
        }],
        remappings=[
            ('scan', scan_topic),
            ('odom', odom_topic),
            ('map', MAP_TOPIC),
        ],
        arguments=['-d'],
    )

    # 2. RRT* global planner
    global_planner = Node(
        package='nav_rtabmap',
        executable='global_planner',
        name='global_planner',
        output='screen',
        parameters=[{
            'use_sim_time':         use_sim_time,
            'use_sim':              False,
            'use_saved_map':        False,
            'map_topic':            MAP_TOPIC,
            'map_frame':            odom_frame,
            'odom_topic':           odom_topic,
            'robot_radius':         ROBOT_RADIUS,
            'rrt_max_iter':         5000,
            'rrt_step_size':        0.3,
            'replan_interval':      2.0,
            'waypoint_tol':         0.25,
            'replan_confirmations': 3,
            'replan_cooldown':      5.0,
        }],
    )

    # 3. DWA local planner
    # With use_sim=False, this node publishes geometry_msgs/TwistStamped to
    # /cmd_vel, which matches your TurtleBot 4 topic info.
    dwa_planner = Node(
        package='nav_rtabmap',
        executable='dwa_planner',
        name='dwa_planner',
        output='screen',
        parameters=[{
            'use_sim_time':              use_sim_time,
            'use_sim':                   False,  # False = publish TwistStamped — confirmed required by this TB4
            'map_frame':                 odom_frame,
            'odom_frame_id':             odom_frame,
            'odom_topic':                odom_topic,
            'robot_radius':              0.12,
            'v_max':                     v_max,
            'w_max':                     w_max,
            'a_w_max':                   0.8,
            'v_samples':                 8,
            'w_samples':                 21,
            'sim_time':                  2.0,
            'sim_steps':                 20,
            'waypoint_tol':              0.25,
            'w_heading':                 2.5,
            'w_clearance':               0.2,
            'w_velocity':                0.6,
            'dt':                        0.1,
            'k_w':                       1.2,
            'angle_threshold':           0.45,
            'angle_exit_ratio':          0.70,
            'path_interp_spacing':       0.10,
            'waypoint_lookahead_dist':   0.35,
            'stuck_lin_speed_eps':       0.02,
            'stuck_ticks_before_replan': 25,
            'plan_max_age':              3.0,
        }],
        remappings=[('/projected_map', MAP_TOPIC)],
    )

    # 4. Frontier explorer
    frontier_explorer = Node(
        package='nav_rtabmap',
        executable='frontier_explorer',
        name='frontier_explorer',
        output='screen',
        parameters=[{
            'use_sim_time':        use_sim_time,
            'use_sim':             False,
            'map_topic':           MAP_TOPIC,
            'odom_topic':          odom_topic,
            'goal_frame':          odom_frame,
            'distance_threshold':  0.3,
            'map_update_interval': 1.0,
            'explore_interval':    1.0,
            'length_weight':       0.6,
            'distance_weight':     0.4,
            'entropy_radius':      5,
            'dbscan_eps':          3.0,
            'dbscan_min_samples':  4,
            'goal_tolerance':      0.40,
            'max_nav_attempts':    10,
        }],
    )

    return LaunchDescription([
        rtabmap,
        global_planner,
        dwa_planner,
        frontier_explorer,
    ])
