#!/usr/bin/env python3
"""
bringup_real.launch.py  —  nav_rtabmap  (REAL TURTLEBOT4)
==========================================================
Single command to start RViz + SLAM + planning on the real TurtleBot4.
TurtleBot4 hardware (Create3 base, RPLidar, OAK-D) is already running.

HOW TO RUN
----------
  source ~/ros2_ws/install/setup.bash
  ros2 launch nav_rtabmap bringup_real.launch.py

  Optional — start autonomous exploration:
    ros2 topic pub /explorer_start std_msgs/msg/Bool "data: true" --once

CONFIRMED REAL ROBOT TOPICS (Apr 2026)
---------------------------------------
  /scan          RPLidar
  /odom          wheel odometry
  /cmd_vel       velocity command (Twist)
  /joint_states  arm joints
  TF: odom → base_link → rplidar_link

TUNE IF NAVIGATION FAILS
-------------------------
  ROBOT_RADIUS  — map inflation. Start at 0.12.
    Corridors impassable → decrease to 0.10
    Robot scrapes walls  → increase to 0.15
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# ── REAL ROBOT CONFIRMED VALUES 
MAP_TOPIC      = '/rtabmap/grid_map'
SCAN_TOPIC     = '/scan'
ODOM_TOPIC     = '/odom'
ODOM_FRAME     = 'odom'
BASE_FRAME     = 'base_link'
ROBOT_RADIUS   = 0.12        # ← TUNE THIS if corridors blocked
USE_SCAN_RELAY = False       # set True only if angle_min ≈ +3.14 (inverted lidar)



ROBOT_DESCRIPTION = """<?xml version="1.0"?>
<robot name="turtlebot_rviz_visual">
  <link name="base_link">
    <visual>
      <geometry>
        <mesh filename="package://kobuki_description/resources/meshes/main_body.dae"/>
      </geometry>
    </visual>
  </link>

  <link name="robot_visual_plates">
    <visual>
      <geometry>
        <mesh filename="package://kobuki_description/resources/meshes/plates.dae"/>
      </geometry>
    </visual>
  </link>
  <joint name="robot_visual_base_to_plates" type="fixed">
    <parent link="base_link"/>
    <child link="robot_visual_plates"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>

  <link name="robot_visual_left_wheel">
    <visual>
      <geometry>
        <mesh filename="package://kobuki_description/resources/meshes/wheel.dae"/>
      </geometry>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
    </visual>
  </link>
  <joint name="robot_visual_left_wheel_joint" type="fixed">
    <parent link="base_link"/>
    <child link="robot_visual_left_wheel"/>
    <origin xyz="-0.014 -0.115 0.163" rpy="0 0 3.14159"/>
  </joint>

  <link name="robot_visual_right_wheel">
    <visual>
      <geometry>
        <mesh filename="package://kobuki_description/resources/meshes/wheel.dae"/>
      </geometry>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
    </visual>
  </link>
  <joint name="robot_visual_right_wheel_joint" type="fixed">
    <parent link="base_link"/>
    <child link="robot_visual_right_wheel"/>
    <origin xyz="-0.014 0.115 0.163" rpy="0 0 3.14159"/>
  </joint>

  <link name="robot_visual_rplidar">
    <visual>
      <geometry>
        <mesh filename="package://kobuki_description/resources/meshes/rplidar.dae"/>
      </geometry>
    </visual>
  </link>
  <joint name="robot_visual_lidar_joint" type="fixed">
    <parent link="robot_visual_plates"/>
    <child link="robot_visual_rplidar"/>
    <origin xyz="-0.02 0.0 0.01918" rpy="0 0 3.14159"/>
  </joint>

  <link name="robot_visual_camera">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.09 0.025 0.025"/>
      </geometry>
      <material name="camera_black">
        <color rgba="0.02 0.02 0.02 1.0"/>
      </material>
    </visual>
  </link>
  <joint name="robot_visual_camera_joint" type="fixed">
    <parent link="robot_visual_plates"/>
    <child link="robot_visual_camera"/>
    <origin xyz="0.1115 0 0.0945" rpy="0 0 0"/>
  </joint>
</robot>
"""


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_description = ParameterValue(ROBOT_DESCRIPTION, value_type=str)

    nodes = []

    # ── -1. Robot model for RViz 
    # RViz config subscribes to /turtlebot/robot_description. Keep the model
    # rooted at the real base_link so it follows the robot's existing TF tree.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='turtlebot',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }],
    )
    nodes.append(robot_state_publisher)

    # ── 0. RViz with rtabmap config 
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('turtlebot_description'),
            'rviz', 'turtlebot_rtabmap.rviz'
        ])],
        output='screen',
    )
    nodes.append(rviz)

    # ── 1. scan_relay (only if RPLidar is physically inverted) 
    # Check: ros2 topic echo /scan --once | grep angle_min
    # angle_min ≈ -3.14 → normal, leave USE_SCAN_RELAY = False
    # angle_min ≈ +3.14 → inverted, set USE_SCAN_RELAY = True
    if USE_SCAN_RELAY:
        scan_relay = Node(
            package='nav_rtabmap',
            executable='scan_relay',
            name='scan_relay',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('/turtlebot/scan',         SCAN_TOPIC),
                ('/turtlebot/scan_rtabmap', '/scan_rtabmap'),
            ],
        )
        nodes.append(scan_relay)
        rtabmap_scan = '/scan_rtabmap'
    else:
        rtabmap_scan = SCAN_TOPIC
    
    # ── 1.2. scan_restamper (fixes lidar clock mismatch) 
    scan_restamper = Node(
        package='nav_rtabmap',
        executable='scan_restamper',
        name='scan_restamper',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('scan_in',  rtabmap_scan),
            ('scan_out', '/scan_fixed'),
        ],
    )
    nodes.append(scan_restamper)
    

    # # ── 1.5. arm_tuck 
    # arm_tuck = Node(
    #     package='nav_rtabmap',
    #     executable='arm_tuck',
    #     name='arm_tuck',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time':  use_sim_time,
    #         'target_joint2': -0.25,
    #         'target_joint3': -1.5,
    #     }],
    # )
    # nodes.append(arm_tuck)

    # ── 2. RTAB-Map SLAM (ICP / LiDAR focus) 
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time':                      use_sim_time,
            'frame_id':                          BASE_FRAME,
            'map_frame_id':                      'map',
            'odom_frame_id':                     ODOM_FRAME,
            'subscribe_scan':                    True,
            'subscribe_depth':                   False,
            'subscribe_rgb':                     False,
            'approx_sync':                       True,
            'queue_size':                        30,
            'wait_for_transform':                0.5,
            'Mem/IncrementalMemory':             'true',
            'Mem/InitWMWithAllNodes':            'true',
            'Mem/NotLinkedNodesKept':            'false',   # narrow map — save memory
            'Reg/Strategy':                      '1',       # ICP
            'Reg/Force3DoF':                     'true',
            'RGBD/StartAtOrigin':                'true',
            'RGBD/NeighborLinkRefining':         'true',
            'RGBD/LinearUpdate':                 '0.05',
            'RGBD/AngularUpdate':                '0.05',
            'Icp/VoxelSize':                     '0.03',    # finer for tight corridors
            'Icp/MaxTranslation':                '0.5',     # circuit1 value
            'Icp/MaxRotation':                   '1.0',
            'Icp/MaxCorrespondenceDistance':     '0.2',     # tight — precise in corridors
            'Icp/PointToPlane':                  'false',
            'Mem/LaserScanVoxelSize':            '0.03',    # circuit1: finer scan downsampling
            'Grid/CellSize':                     '0.05',    # circuit1: 5 cm resolution for corridors
            'Grid/RangeMin':                     '0.05',
            'Grid/RangeMax':                     '8.0',     # circuit1: match room size not lidar max
            'Grid/RayTracing':                   'true',    # CRITICAL: erases free space along rays
            'Grid/3D':                           'false',
            'Grid/FromDepth':                    'false',
            'Grid/MapFrameProjection':           'true',
            'Grid/NoiseFilteringMinNeighbors':   '5',       # remove isolated noise cells
            'Grid/NoiseFilteringRadius':         '0.5',
            'Optimizer/GravitySigma':            '0',
            'Optimizer/Robust':                  'true',
        }],
        remappings=[
            # Use the restamped laser scan so RTAB-Map publishes map->odom on
            # the same clock as the live odom->base_link TF tree.
            ('scan', '/scan_fixed'),
            ('odom', ODOM_TOPIC),
            ('map',  MAP_TOPIC),
        ],
        arguments=['--delete_db_on_start'],
    )
    nodes.append(rtabmap)

    # ── 3. RRT* global planner 
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
            # map_frame MUST match frontier_explorer's goal_frame.
            # frontier_explorer defaults to 'odom' for real robot (use_sim=False).
            # global_planner reads goal x,y raw — no TF conversion — so frames must match.
            'map_frame':            ODOM_FRAME,   # = 'odom'  ← reverted from 'map'
            'odom_topic':           ODOM_TOPIC,
            'robot_radius':         ROBOT_RADIUS,
            'rrt_max_iter':         5000,
            'rrt_step_size':        0.20, #0.20
            'replan_interval':      0.5, #1.0
            'waypoint_tol':         0.35, #0.30
            'replan_confirmations': 4, #8
            'replan_cooldown':      2.5, #5.0
            'lookahead_waypoints':  5, #3
            'check_inflation_radius': 0.07, #0.05
        }],
        remappings=[('/turtlebot/odom', ODOM_TOPIC)],
    )
    nodes.append(global_planner)

    # ── 4. DWA local planner 
    dwa_planner = Node(
        package='nav_rtabmap',
        executable='dwa_planner',
        name='dwa_planner',
        output='screen',
        parameters=[{
            'use_sim_time':              use_sim_time,
            # use_sim=False → DWA publishes geometry_msgs/TwistStamped to /cmd_vel
            # CONFIRMED: this TB4 requires TwistStamped (teleop stamped:=true works)
            'use_sim':                   False,
            'odom_topic':                ODOM_TOPIC,
            'odom_frame_id':             ODOM_FRAME,
            'map_frame':                 ODOM_FRAME,

            # ── robot footprint 
            'robot_radius':              0.11,   # circuit1: 0.12 m  (spreadsheet Critical row)

            # ── speed limits 
            'v_max':                     0.12,   # circuit1: 0.12 m/s — slow & safe in tight corridors
            'w_max':                     0.45, #0.40  # circuit1: 0.70 rad/s — don't overshoot turns
            'a_w_max':                   1.0,

            # ── DWA sampling 
            'v_samples':                 10,     # circuit1: 10
            'w_samples':                 21,  #20   # circuit1: 20
            'sim_time':                  3.0, #2.5   # 3 s lookahead — balanced CPU vs foresight
            'sim_steps':                 15, #10
            'dt':                        0.1,

            # ── scoring weights 
            # FIX excessive rotation: lower w_heading so robot doesn't spin to face
            # each waypoint perfectly. Raise w_velocity so it prefers moving forward.
            'w_heading':                 0.75, #0.9   # spreadsheet High: 1.8
            'w_clearance':               1.35, #1.0  # corridors: moderate (Option B fix reduces false penalty)
            'w_velocity':                1.10, #1.0   # spreadsheet High: 1.2

            # ── waypoint tracking 
            'waypoint_tol':              0.35, #0.30  # reach waypoint within 25 cm
            'waypoint_lookahead_dist':   0.55, #0.40  # circuit1: 0.35 m ahead for heading target
            'path_interp_spacing':       0.12, #0.10

            # ── rotation control — FIX excessive rotation 
            # angle_threshold: only rotate-in-place if heading error > this value
            # Spreadsheet says 0.55 rad (~31°) for circuit1
            # Lower than 0.55 → robot rotates less often (good for narrow maps)
            'k_w':                       0.70, #0.85           # gentler angular correction
            'angle_threshold':           0.90,  #1.0         # ~57° — DWA runs continuously, smooth motion
            'angle_exit_ratio':          0.55, #0.30          # stay in rotation until nearly aligned

            # ── stuck / replan detection 
            'stuck_lin_speed_eps':       0.015, #0.02
            'stuck_ticks_before_replan': 25,  #35          # ~5 s of no movement
            'plan_max_age':              6.0,            # give planner more time in tight spaces

            # ── scan collision 
            # DISABLED for real robot: in narrow corridors walls are within
            # 0.25 m of the robot, causing ALL forward DWA trajectories to be
            # rejected by the scan check → robot only rotates, never moves.
            # Re-enable only after confirming open space navigation works.
            'use_scan_collision':        True,
            'scan_check_radius':         0.30, #0.30  # tight — only if re-enabled

            # ── Option B: lighter inflation for clearance SCORING ─────────
            # map_inf (robot_radius=0.12 m → 3-cell at 5 cm res) stays in
            # _simulate() as hard collision guard.
            # map_check (dwa_check_radius=0.05 m → 1-cell) is used ONLY by
            # _min_clearance() for trajectory scoring.  Without this, noisy
            # early scans inflate to 30 cm exclusion zones that score all
            # forward arcs near-zero → robot freezes in Phase C.
            'dwa_check_radius':          0.06, #0.05

            # ── velocity window 
            # Default a_lin_max=0.3 gives v_hi=0.03 m/s on the first Phase B
            # tick when coming out of a Phase A rotate (_current_v stays 0).
            # 1.5 opens the full 0..v_max window immediately so DWA always
            # samples the full velocity range even after rotate-in-place.
            'a_lin_max':                 0.8,
            'a_ang_max':                 0.8,
        }],
        remappings=[
            # use_sim=False: DWA publishes directly to /cmd_vel
            ('/projected_map', MAP_TOPIC),
        ],
    )
    nodes.append(dwa_planner)

    # ── 5. Frontier explorer 
    frontier_explorer = Node(
        package='nav_rtabmap',
        executable='frontier_explorer',
        name='frontier_explorer',
        output='screen',
        parameters=[{
            'use_sim_time':        use_sim_time,
            'use_sim':             False,
            'map_topic':           MAP_TOPIC,
            'odom_topic':          ODOM_TOPIC,    # explicit — avoids default /turtlebot/odom
            'goal_frame':          ODOM_FRAME,    # explicit — must match global_planner map_frame
            'distance_threshold':  0.3,
            'map_update_interval': 1.0,
            'explore_interval':    0.5,    # was 1.0; faster goal transitions
            # Scoring: pure cluster-size scoring (same fix as sim).
            # distance_weight=0.4 was biasing robot toward nearby small frontiers
            # instead of large unexplored regions deeper in the map.
            'length_weight':       1.0,    # was 0.6
            'distance_weight':     0.0,    # was 0.4
            'entropy_radius':      5,
            # DBSCAN: eps=3 cells = 15 cm at 5 cm/cell — too tight.
            # Frontier points > 15 cm apart each become singletons (< min_samples)
            # and are discarded, leaving few or no valid clusters.
            # eps=5 cells = 25 cm groups the same boundary into proper clusters.
            'dbscan_eps':          5.0,    # was 3.0
            'dbscan_min_samples':  4,
            'goal_tolerance':      0.45,
            'max_nav_attempts':    12,
            'lookahead_distance':  1.0,
            'nav_timeout':        45.0,
        }],
    )
    nodes.append(frontier_explorer)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Real robot — always false'),
        *nodes,
    ])
