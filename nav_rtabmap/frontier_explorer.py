#!/usr/bin/env python3
"""
frontier_explorer.py  —  Frontier-Based Autonomous Exploration
==============================================================

Ported from Sazid669/Autonomus-Exploration (ROS1) to ROS2 Jazzy.
Core algorithms preserved exactly:
  - DBSCAN clustering  (eps=3 cells, min_samples=4)
  - Maximum Information Gain via Shannon entropy window
  - Score = 0.6 × cluster_length + 0.4 × min_distance
  - Fallback: next point in same cluster → next cluster → replan

StateValidityChecker is reimplemented inline (map_to_position +
is_valid_frontier) — no external dependency needed.

INTEGRATION
-----------
  /projected_map   ──→  frontier_explorer  ──→  /goal_pose  ──→  global_planner
  /turtlebot/odom  ──→  frontier_explorer
  /dwa_status      ──→  frontier_explorer  (goal reached signal from dwa_planner)
  /controller_status ─→ frontier_explorer  (goal reached from control_tb)

RViz topics:
  /frontier_points   PoseArray    all raw frontier cells
  /frontier_clusters MarkerArray  coloured clusters + labels
  /frontiers         MarkerArray  chosen goal marker

START: ros2 topic pub /explorer_start std_msgs/msg/Bool "data: true"  --once
STOP:  ros2 topic pub /explorer_start std_msgs/msg/Bool "data: false" --once

PARAMETERS
----------
  use_sim             bool    True = /turtlebot/odom, False = /odom
  distance_threshold  float   validity check radius in metres (default 0.3)
  map_update_interval float   min seconds between map callbacks processed
  explore_interval    float   seconds between explore loop ticks
  length_weight       float   cluster size score weight  (default 0.6)
  distance_weight     float   distance score weight      (default 0.4)
  entropy_radius      int     window radius in cells for information gain
  dbscan_eps          float   DBSCAN neighbourhood radius in cells
  dbscan_min_samples  int     DBSCAN min points to form cluster
  goal_tolerance      float   metres — distance to consider goal reached
  max_nav_attempts    int     attempts per cluster before moving to next
"""

import math
import rclpy
import numpy as np
import scipy.stats as stats
from collections import deque
from sklearn.cluster import DBSCAN
import threading

from rclpy.node        import Node
from rclpy.qos         import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg      import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Point
from std_msgs.msg      import String, Bool
from visualization_msgs.msg import Marker, MarkerArray


class StateValidityChecker:
    """
    Inline reimplementation of utils_lib.StateValidityChecker.

    The original repo stores the map TRANSPOSED (.T), so grid indices are
    (col, row) — i.e. point = (i, j) where i is the column index.
    We keep the SAME convention here so all algorithm logic is identical.

    self.map  shape: (width, height)  — matches original .T convention
    """

    def __init__(self, distance_threshold: float = 0.3):
        self.map        = None
        self.resolution = None
        self.origin     = None          # [ox, oy] world coords of cell (0,0)
        self._thresh    = distance_threshold

    def set(self, env_transposed: np.ndarray, resolution: float, origin: list):
        """
        env_transposed: occupancy grid ALREADY transposed (.T), shape (W, H)
        resolution: metres per cell
        origin: [ox, oy] world position of cell (0, 0)
        """
        self.map        = env_transposed
        self.resolution = resolution
        self.origin     = origin

    def map_to_position(self, point) -> np.ndarray:
        """
        Convert (col, row) grid index → world (x, y).
        col = point[0], row = point[1]   (matches transposed convention)
        """
        x = self.origin[0] + point[0] * self.resolution
        y = self.origin[1] + point[1] * self.resolution
        return np.array([x, y])

    def position_to_map(self, world_xy) -> tuple:
        """World (x, y) → (col, row) in transposed map."""
        col = int((world_xy[0] - self.origin[0]) / self.resolution)
        row = int((world_xy[1] - self.origin[1]) / self.resolution)
        return col, row

    def is_valid_frontier(self, world_xy) -> bool:
        """
        True if the world point maps to a FREE cell (value 0).
        Unknown (-1) and occupied (100) are both invalid navigation targets.
        """
        if self.map is None:
            return False
        col, row = self.position_to_map(world_xy)
        w, h = self.map.shape
        if not (0 <= col < w and 0 <= row < h):
            return False
        return self.map[col, row] == 0



class FrontierExplorer(Node):


    def __init__(self):
        super().__init__('frontier_explorer')

        # parameters 
        self.declare_parameter('use_sim',             True)
        self.declare_parameter('distance_threshold',  0.3)
        self.declare_parameter('map_update_interval', 1.0)   # s
        self.declare_parameter('explore_interval',    1.0)   # s
        self.declare_parameter('length_weight',       0.6)
        self.declare_parameter('distance_weight',     0.4)
        self.declare_parameter('entropy_radius',      5)     # cells
        self.declare_parameter('entropy_rays',      12)   # number of rays (10–16 per prof)
        self.declare_parameter('entropy_max_cells', 60)   # ray length in cells (60 ≈ 3 m at 0.05 res)
        self.declare_parameter('dbscan_eps',          3.0)   # cells
        self.declare_parameter('dbscan_min_samples',  4)
        self.declare_parameter('goal_tolerance',      0.40)  # m
        self.declare_parameter('max_nav_attempts',    10)
        self.declare_parameter('lookahead_distance', 0.5)  # metres — trigger pre-compute
        self.declare_parameter('map_topic',           '/projected_map')
        self.declare_parameter('odom_topic',          '')
        self.declare_parameter('goal_frame',          '')

        def g(n): return self.get_parameter(n).value
        self.use_sim             = g('use_sim')
        dist_thresh              = g('distance_threshold')
        self.map_update_interval = g('map_update_interval')
        self.explore_interval    = g('explore_interval')
        self.length_weight       = g('length_weight')
        self.distance_weight     = g('distance_weight')
        self.entropy_radius      = int(g('entropy_radius'))
        self.entropy_rays      = int(g('entropy_rays'))
        self.entropy_max_cells = int(g('entropy_max_cells'))
        self.dbscan_eps          = g('dbscan_eps')
        self.dbscan_min_samples  = int(g('dbscan_min_samples'))
        self.goal_tolerance      = g('goal_tolerance')
        self.max_nav_attempts    = int(g('max_nav_attempts'))
        self.lookahead_distance  = g('lookahead_distance')
        map_topic                = g('map_topic')
        _goal_frame              = g('goal_frame')
        default_goal_frame       = 'world_enu' if self.use_sim else 'odom'
        self.goal_frame          = _goal_frame if _goal_frame else default_goal_frame

        # state validity checker (inline) 
        self.svc = StateValidityChecker(dist_thresh)

        # robot / map state 
        self.current_pose  = None          # np.array [x, y, yaw]
        self.map           = None          # transposed grid (W, H) int8
        self.resolution    = None
        self.origin        = None
        self.last_map_time = 0.0           # monotonic float

        # exploration state 
        self.goal_reached  = True          # True = ready for next goal
        self.goal          = None          # current (gx, gy) world target
        self.clusters      = {}            # {label: [grid_points]}
        self.sorted_clusters = None
        self.exploring     = False

        # pre-emptive planning state 
        self._next_goal        = None    # pre-computed next goal (x, y) or None
        self._precompute_done  = False   # True when next goal is ready to publish
        self._precomputing     = False   # True while background thread is running
        self._lookahead_armed  = False   # True once we have entered lookahead zone
        self._ignore_status_until = 0.0  # epoch time; ignore DWA "reached" signals before this
        self._precompute_gen   = 0       # monotonically increasing; cancels stale threads

        # ROS topics 
        _odom_override = g('odom_topic')
        default_odom   = '/turtlebot/odom' if self.use_sim else '/odom'
        odom_t = _odom_override if _odom_override else default_odom

        # RTAB-Map publishes the grid map with QoS(1) RELIABLE TRANSIENT_LOCAL
        # (latching=true, MapsManager.cpp default).  Subscribe with the same
        # profile so the retained latched map is delivered on node startup.
        _map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(OccupancyGrid, map_topic,            self._map_cb,    _map_qos)
        self.create_subscription(Odometry,      odom_t,               self._odom_cb,   10)
        self.create_subscription(String,        '/dwa_status',        self._status_cb, 10)
        self.create_subscription(String,        '/controller_status', self._status_cb, 10)
        self.create_subscription(Bool,          '/explorer_start',    self._start_cb,  10)

        self.goal_pub      = self.create_publisher(PoseStamped, '/goal_pose',           10)
        self.frontier_pub  = self.create_publisher(PoseArray,   '/frontier_points',     10)
        self.cluster_pub   = self.create_publisher(MarkerArray, '/frontier_clusters',   10)
        self.marker_pub    = self.create_publisher(Marker,      '/frontier_best_marker',10)
        self.status_pub    = self.create_publisher(String,      '/explorer_status',     10)

        self.create_timer(self.explore_interval, self._explore_tick)

        self.get_logger().info('=' * 60)
        self.get_logger().info('🧭  FrontierExplorer ready (DBSCAN + InfoGain)')
        self.get_logger().info(f'   DBSCAN eps={self.dbscan_eps} cells  '
                               f'min_samples={self.dbscan_min_samples}')
        self.get_logger().info(f'   score weights: length={self.length_weight}  '
                               f'dist={self.distance_weight}')
        self.get_logger().info(f'   entropy radius={self.entropy_radius} cells')
        self.get_logger().info('   → publish Bool(true) to /explorer_start to begin')
        self.get_logger().info('=' * 60)

    # callbacks 

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y),
                         1.0 - 2.0*(q.y*q.y + q.z*q.z))
        self.current_pose = np.array([p.x, p.y, yaw])

    def _map_cb(self, msg):
        """
        Process map at most once per map_update_interval seconds.
        Transposes the grid to match the original repo's (W,H) convention.
        """
        import time as _time
        now = _time.monotonic()
        if now - self.last_map_time < self.map_update_interval:
            return
        self.last_map_time = now

        raw = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)
        # Transpose to (W, H) — same as original repo's env.T convention
        self.map       = raw.T
        self.origin    = [msg.info.origin.position.x,
                          msg.info.origin.position.y]
        self.resolution = msg.info.resolution
        self.svc.set(self.map, self.resolution, self.origin)

    def _status_cb(self, msg: String):
        """Handle goal-reached signals from dwa_planner or control_tb."""
        import time as _time
        txt = msg.data.lower()
        if 'goal reached' in txt or 'reached' in txt:
            if _time.time() < self._ignore_status_until:
                self.get_logger().debug(
                    '[EXPLORE] ignoring stale goal-reached signal (mid goal-switch)')
                return
            if not self.goal_reached:
                self.get_logger().info('[EXPLORE] goal reached signal received')
                self.goal_reached = True

    def _start_cb(self, msg: Bool):
        if msg.data:
            self.exploring    = True
            self.goal_reached = True   # kick off immediately
            self.get_logger().info('[EXPLORE] ▶ exploration STARTED')
            self._pub_status('exploring')
        else:
            self.exploring = False
            self.goal      = None
            self.get_logger().info('[EXPLORE] ⏸ exploration PAUSED')
            self._pub_status('paused')

    # main tick 

    def _explore_tick(self):
        """
        Main exploration loop, called at explore_interval Hz.

        States:
          - goal_reached=True, _precompute_done=False → find & go to next goal normally
          - goal_reached=False, within lookahead → trigger pre-compute in background
          - goal_reached=False, _precompute_done=True → publish pre-computed next goal
          - goal_reached=False, far from goal → keep driving, do nothing
        """
        if not self.exploring:
            return
        if self.current_pose is None or self.map is None:
            return

        # compute distance to current goal 
        dist_to_goal = None
        if self.goal is not None:
            dist_to_goal = math.hypot(
                self.goal[0] - self.current_pose[0],
                self.goal[1] - self.current_pose[1])

        # hard goal-reached: robot is at (or past) the goal 
        if dist_to_goal is not None and dist_to_goal < self.goal_tolerance:
            if not self.goal_reached:
                self.get_logger().info(
                    f'[EXPLORE] goal reached (dist={dist_to_goal:.2f}m)')
                self.goal_reached = True

        # signal-based goal reached (from DWA status topic) 
        # (already handled in _status_cb — self.goal_reached may be True)

        # PRE-EMPTIVE TRIGGER: start computing next goal early 
        if (dist_to_goal is not None
                and not self.goal_reached
                and not self._precomputing
                and not self._precompute_done
                and dist_to_goal < self.lookahead_distance
                and not self._lookahead_armed):
            self._lookahead_armed = True
            self.get_logger().info(
                f'[EXPLORE] lookahead triggered at {dist_to_goal:.2f}m from goal '
                f'(threshold={self.lookahead_distance:.1f}m) — pre-computing next')
            threading.Thread(
                target=self._precompute_next_goal,
                args=(self._precompute_gen,),
                daemon=True).start()

        # PRE-COMPUTED GOAL READY: validate then publish without stopping ─
        if (self._precompute_done
                and self._next_goal is not None
                and not self.goal_reached):
            gx, gy = self._next_goal
            if not self.svc.is_valid_frontier(np.array([gx, gy])):
                self.get_logger().warn(
                    f'[EXPLORE] pre-computed goal ({gx:.2f},{gy:.2f}) '
                    'no longer valid in current map — falling back to normal mode')
                self._precompute_done = False
                self._lookahead_armed = False
                self._next_goal       = None
                self.goal_reached     = True   # triggers normal-mode on next tick
            else:
                import time as _time
                self.get_logger().info(
                    f'[EXPLORE] publishing pre-computed goal '
                    f'({gx:.2f}, {gy:.2f}) — robot keeps moving')
                # Suppress stale DWA "goal reached" for old goal for 10 s
                self._ignore_status_until = _time.time() + 10.0
                # Reset pre-compute state BEFORE publishing so tick doesn't re-trigger
                self._precompute_done  = False
                self._lookahead_armed  = False
                self._next_goal        = None
                # Switch goal (do not set goal_reached=True — robot never stops)
                self.goal         = (gx, gy)
                self.goal_reached = False
                self._publish_goal_xy(gx, gy)
                return

        # PRE-COMPUTED BUT INVALID: fall through to normal mode 
        if self._precompute_done and self._next_goal is None:
            self.get_logger().warn(
                '[EXPLORE] pre-compute found no valid next goal — '
                'will use normal mode after current goal reached')
            self._precompute_done = False
            self._lookahead_armed = False

        # NORMAL MODE: wait for goal_reached, then find next frontier 
        if not self.goal_reached:
            return   # still driving, pre-compute is running or not needed yet

        # Robot has reached the goal (or this is the very first tick).
        # Reset state and find next frontier normally.
        # Bump generation so any still-running precompute thread discards its result.
        self._precompute_gen += 1
        self._precompute_done = False
        self._lookahead_armed = False
        self._next_goal       = None
        self.goal_reached     = False   # will be set True when we reach next goal

        frontiers = self._get_frontiers(self.map)
        if not frontiers:
            self.get_logger().info(
                '[EXPLORE] no frontiers remaining — exploration complete!')
            self._pub_status('complete')
            self.exploring = False
            return

        self._frontier_publish(frontiers)
        clusters = self._cluster_frontiers(frontiers)
        self._publish_clusters(clusters)
        self._explore(clusters)

    # frontier detection 

    def _get_frontiers(self, gridmap: np.ndarray) -> list:
        """
        Find all frontier cells.
        gridmap shape: (W, H) — transposed convention from original repo.
        A cell is a frontier if it is FREE (0) and has at least one
        UNKNOWN (-1) neighbour in 8-connectivity.
        Returns list of (col, row) grid indices.
        """
        frontiers = []
        W, H = gridmap.shape

        for i in range(W):
            for j in range(H):
                if gridmap[i, j] != 0:
                    continue
                # check 8 neighbours
                for di in [-1, 0, 1]:
                    for dj in [-1, 0, 1]:
                        if di == 0 and dj == 0:
                            continue
                        ni, nj = i + di, j + dj
                        if 0 <= ni < W and 0 <= nj < H:
                            if gridmap[ni, nj] == -1:
                                frontiers.append((i, j))
                                break
                    else:
                        continue
                    break
        return frontiers

    # clustering (DBSCAN — from original repo) 

    def _cluster_frontiers(self, frontier_points: list) -> dict:
        """
        DBSCAN clustering — identical parameters to original repo.
        eps=3 cells, min_samples=4.
        Returns {label: [list of (col,row) points]}, noise points excluded.
        """
        if not frontier_points:
            return {}

        pts    = np.array(frontier_points)
        dbscan = DBSCAN(eps=self.dbscan_eps, min_samples=self.dbscan_min_samples)
        dbscan.fit(pts)
        labels = dbscan.labels_

        clusters = {}
        for label, point in zip(labels, pts):
            if label == -1:
                continue   # noise
            clusters.setdefault(int(label), []).append(tuple(point))
        return clusters

    # exploration logic 

    def _explore(self, clusters: dict):
        if not clusters:
            self.get_logger().warn('[EXPLORE] no clusters — reclustering')
            self._recluster_and_explore()
            return

        self.clusters = clusters
        min_distances = self._calculate_min_distances(clusters)
        max_length    = self._calculate_max_lengths(clusters)
        target_label, target_points = self._score_and_select(
            min_distances, max_length, clusters)

        if target_points:
            best_vp = self._maximum_information_gain(target_points)
            if best_vp is not None:
                self.get_logger().info(f'[EXPLORE] best viewpoint grid={best_vp}')
                self._navigate_to_viewpoint(best_vp, target_label)
            else:
                self.get_logger().warn('[EXPLORE] no valid viewpoint in cluster')
        else:
            self.get_logger().warn('[EXPLORE] target cluster empty')
            self._recluster_and_explore()

    def _precompute_next_goal(self, my_gen: int):
        """
        Run in a background thread while robot is still approaching current goal.
        Computes the next frontier centroid and stores it in self._next_goal.
        Does NOT publish — _explore_tick() handles publishing at the right time.
        my_gen: generation number at thread launch; result is discarded if it no
                longer matches self._precompute_gen (normal mode published first).
        """
        self._precomputing = True
        self.get_logger().info('[EXPLORE] pre-computing next frontier...')

        if self.map is None:
            self._precomputing = False
            return

        frontiers = self._get_frontiers(self.map)
        if not frontiers:
            self.get_logger().info('[EXPLORE] pre-compute: no frontiers remain')
            self._next_goal       = None
            self._precompute_done = True
            self._precomputing    = False
            return

        clusters = self._cluster_frontiers(frontiers)
        if not clusters:
            self.get_logger().info('[EXPLORE] pre-compute: no clusters formed')
            self._next_goal       = None
            self._precompute_done = True
            self._precomputing    = False
            return

        min_distances = self._calculate_min_distances(clusters)
        max_length    = self._calculate_max_lengths(clusters)
        target_label, target_points = self._score_and_select(
            min_distances, max_length, clusters)

        if not target_points:
            self._next_goal       = None
            self._precompute_done = True
            self._precomputing    = False
            return

        best_vp = self._maximum_information_gain(target_points)
        if best_vp is None:
            self._next_goal       = None
            self._precompute_done = True
            self._precomputing    = False
            return

        world_pt = self.svc.map_to_position(best_vp)
        if self.svc.is_valid_frontier(world_pt):
            next_goal_candidate = (float(world_pt[0]), float(world_pt[1]))
            self.get_logger().info(
                f'[EXPLORE] pre-compute done: next goal = '
                f'({next_goal_candidate[0]:.2f}, {next_goal_candidate[1]:.2f})')
        else:
            next_goal_candidate = None
            self.get_logger().warn('[EXPLORE] pre-compute: best viewpoint invalid')

        # Discard result if normal mode already published a fresh goal
        if self._precompute_gen != my_gen:
            self.get_logger().info(
                '[EXPLORE] pre-compute result discarded (generation changed — '
                'normal mode already published a fresh goal)')
            self._precomputing = False
            return

        self._next_goal       = next_goal_candidate
        self._precompute_done = True
        self._precomputing    = False

    def _recluster_and_explore(self):
        frontiers = self._get_frontiers(self.map)
        if not frontiers:
            self.get_logger().info('[EXPLORE] no new frontiers — complete')
            self._pub_status('complete')
            self.exploring = False
            return
        clusters = self._cluster_frontiers(frontiers)
        if clusters:
            self._publish_clusters(clusters)
            self._explore(clusters)
        else:
            self.get_logger().info('[EXPLORE] clustering failed')

    # scoring (from original repo) 

    def _bresenham_ray_entropy(self, grid: np.ndarray, cx: int, cy: int,
                            num_rays: int = 12, max_cells: int = 50) -> float:
        """
        Cast num_rays evenly around (cx, cy), stopping each ray at the first
        occupied cell. Count unknown cells that are visible (not behind walls).
        Returns a score in [0, 1]: fraction of visible cells that are unknown.

        Professor's spec: 10-16 rays, 3-5 m range. With default resolution 0.05 m/cell:
          3 m / 0.05 = 60 cells, 5 m / 0.05 = 100 cells → use max_cells = 60.
        """
        W, H = grid.shape
        unknown_count = 0
        total_count   = 0

        for i in range(num_rays):
            angle = 2.0 * math.pi * i / num_rays
            dx = math.cos(angle)
            dy = math.sin(angle)

            for step in range(1, max_cells + 1):
                nx = int(round(cx + dx * step))
                ny = int(round(cy + dy * step))

                if nx < 0 or nx >= W or ny < 0 or ny >= H:
                    break                       # out of map bounds

                v = grid[nx, ny]
                if v > 0:                       # occupied — wall found, stop this ray
                    break
                elif v == -1:                   # unknown — visible and unknown
                    unknown_count += 1
                    total_count   += 1
                else:                           # v == 0 — free
                    total_count += 1

        if total_count == 0:
            return 0.0
        return unknown_count / total_count

    def _calculate_max_lengths(self, clusters: dict) -> dict:
        return {label: len(pts) for label, pts in clusters.items()}

    def _calculate_travel_cost(self, pos_a, pos_b) -> float:
        return float(np.linalg.norm(np.array(pos_a) - np.array(pos_b)))

    def _calculate_min_distances(self, clusters: dict) -> dict:
        if self.current_pose is None:
            return {}
        robot_xy  = self.current_pose[:2]
        min_dists = {}
        for label, pts in clusters.items():
            min_d = np.inf
            for pt in pts:
                world = self.svc.map_to_position(pt)
                d     = self._calculate_travel_cost(robot_xy, world)
                if d < min_d:
                    min_d = d
            min_dists[label] = min_d
        return min_dists

    def _score_and_select(self, min_distances, max_length, clusters):
        """
        Normalised score formula (large-map fix):
          norm_size     = cluster_size / max_cluster_size       [0,1]
          inv_norm_dist = 1.0 - (distance / max_distance)      [0,1] nearer=better
          score         = norm_size × length_weight + inv_norm_dist × distance_weight

        Both terms are bounded to [0,1], so distance can no longer dominate on
        large maps where the farthest cluster might be 10+ metres away.
        """
        if not clusters or not min_distances or not max_length:
            return None, []

        valid_labels = [l for l in clusters
                        if clusters[l] and l in min_distances and l in max_length]
        if not valid_labels:
            return None, []

        max_size = max(max_length[l]    for l in valid_labels) or 1.0
        max_dist = max(min_distances[l] for l in valid_labels) or 1.0

        scores = {}
        for label in valid_labels:
            norm_size     = max_length[label]    / max_size
            inv_norm_dist = 1.0 - (min_distances[label] / max_dist)
            scores[label] = norm_size * self.length_weight + inv_norm_dist * self.distance_weight

        self.sorted_clusters = sorted(scores.items(),
                                      key=lambda x: x[1], reverse=True)
        best_label  = max(scores, key=scores.get)
        best_points = self.clusters.get(best_label, [])
        self.get_logger().info(
            f'[EXPLORE] {len(scores)} clusters scored (normalised) — '
            f'best label={best_label} size={len(best_points)} '
            f'score={scores[best_label]:.2f}')
        return best_label, best_points

    # maximum information gain (from original repo) 

    def _calculate_entropy(self, grid: np.ndarray, x: int, y: int,
                           radius: int) -> float:
        """
        Information gain via Bresenham ray casting — professor Week 3 spec.
        radius arg kept for API compatibility; ray depth set by entropy_max_cells.
        """
        return self._bresenham_ray_entropy(
            grid, x, y,
            num_rays=self.entropy_rays,
            max_cells=self.entropy_max_cells,
        )

    def _maximum_information_gain(self, cluster_points: list):
        """
        Return the grid point (col, row) with the highest local entropy.
        Identical to original repo's maximum_information_gain().
        """
        if self.map is None or self.resolution is None:
            return None

        r          = self.entropy_radius
        max_ent    = -np.inf
        best_point = None

        for pt in cluster_points:
            x, y    = int(pt[0]), int(pt[1])
            entropy = self._calculate_entropy(self.map, x, y, r)
            if entropy > max_ent:
                max_ent    = entropy
                best_point = pt

        if best_point is None:
            self.get_logger().warn('[EXPLORE] no valid frontier — exploration complete')
        return best_point

    # navigation (from original repo) 

    def _navigate_to_viewpoint(self, grid_point, cluster_id: int,
                               attempt: int = 0):
        """
        Convert grid_point → world, validate, publish goal.
        Falls back to next point in same cluster, then next cluster.
        """
        world_pt = self.svc.map_to_position(grid_point)

        if not self.svc.is_valid_frontier(world_pt):
            self.get_logger().warn(
                f'[EXPLORE] attempt {attempt+1}: '
                f'({world_pt[0]:.2f},{world_pt[1]:.2f}) invalid — trying next')

            if attempt < self.max_nav_attempts:
                nxt = self._select_next_target_point(cluster_id, grid_point)
                if nxt is not None:
                    self._navigate_to_viewpoint(nxt, cluster_id, attempt + 1)
                    return
                self.get_logger().warn('[EXPLORE] cluster exhausted')
            else:
                self.get_logger().warn('[EXPLORE] max attempts — next cluster')

            # move to next cluster
            nxt_pt, nxt_label = self._select_next_cluster_viewpoint(cluster_id)
            if nxt_pt is not None:
                self._navigate_to_viewpoint(nxt_pt, nxt_label)
            else:
                self.get_logger().error('[EXPLORE] no more clusters')
            return

        # valid point — publish goal
        self.get_logger().info(
            f'[EXPLORE] ➡  goal ({world_pt[0]:.2f},{world_pt[1]:.2f})')
        self._publish_goal(world_pt)
        self._publish_best_viewpoint_marker(world_pt)

        # remove used cluster so it isn't re-chosen
        if cluster_id in self.clusters:
            del self.clusters[cluster_id]

    def _select_next_target_point(self, cluster_id: int, bad_point):
        """Remove bad_point from cluster, re-run info gain on remainder."""
        pts = list(self.clusters.get(cluster_id, []))
        pts = [p for p in pts if not np.array_equal(p, bad_point)]
        self.clusters[cluster_id] = pts
        if not pts:
            return None
        return self._maximum_information_gain(pts)

    def _select_next_cluster_viewpoint(self, current_label: int):
        """Remove current cluster from sorted list, pick next best."""
        if self.sorted_clusters is None:
            return None, None
        self.sorted_clusters = [c for c in self.sorted_clusters
                                 if c[0] != current_label]
        if not self.sorted_clusters:
            self.get_logger().warn('[EXPLORE] all clusters visited')
            return None, None
        nxt_label = self.sorted_clusters[0][0]
        nxt_pts   = self.clusters.get(nxt_label, [])
        if not nxt_pts:
            return None, None
        return self._maximum_information_gain(nxt_pts), nxt_label

    # publish helpers 

    def _publish_goal(self, world_xy):
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.goal_frame
        msg.pose.position.x    = float(world_xy[0])
        msg.pose.position.y    = float(world_xy[1])
        msg.pose.orientation.w = 1.0
        self.goal          = (float(world_xy[0]), float(world_xy[1]))
        self.goal_reached  = False
        self.goal_pub.publish(msg)

    def _publish_goal_xy(self, gx: float, gy: float):
        """Publish a goal by raw world coordinates. Used by pre-emptive planning."""
        world_xy = np.array([gx, gy])
        self._publish_best_viewpoint_marker(world_xy)
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.goal_frame
        msg.pose.position.x = gx
        msg.pose.position.y = gy
        msg.pose.orientation.w = 1.0
        self.goal = (gx, gy)
        self.goal_pub.publish(msg)

    def _frontier_publish(self, frontier_points: list):
        """Publish all frontier cells as PoseArray."""
        pa = PoseArray()
        pa.header.stamp    = self.get_clock().now().to_msg()
        pa.header.frame_id = 'world_enu' if self.use_sim else 'odom'
        for pt in frontier_points:
            world = self.svc.map_to_position(pt)
            p = Pose()
            p.position.x = float(world[0])
            p.position.y = float(world[1])
            pa.poses.append(p)
        self.frontier_pub.publish(pa)

    def _publish_clusters(self, clusters: dict):
        """
        Publish DBSCAN clusters as coloured MarkerArray.
        Each cluster gets a random colour + text label — identical to
        the original repo's publish_clusters().
        """
        ma  = MarkerArray()
        now = self.get_clock().now().to_msg()

        # delete old
        del_m = Marker()
        del_m.header.frame_id = 'world_enu' if self.use_sim else 'odom'
        del_m.header.stamp    = now
        del_m.action          = Marker.DELETEALL
        del_m.id              = 0
        ma.markers.append(del_m)

        rng = np.random.default_rng(seed=42)   # fixed seed → stable colours

        for idx, (label, pts) in enumerate(clusters.items()):
            r_, g_, b_ = rng.random(3).tolist()
            frame      = 'world_enu' if self.use_sim else 'odom'

            # coloured POINTS marker
            pm = Marker()
            pm.header.frame_id = frame
            pm.header.stamp    = now
            pm.ns              = 'frontier_clusters'
            pm.id              = label
            pm.type            = Marker.POINTS
            pm.action          = Marker.ADD
            pm.scale.x         = 0.08
            pm.scale.y         = 0.08
            pm.color.r         = r_
            pm.color.g         = g_
            pm.color.b         = b_
            pm.color.a         = 1.0
            pm.pose.orientation.w = 1.0
            # lifetime = 0 → permanent in RViz

            world_pts = [self.svc.map_to_position(p) for p in pts]
            for wp in world_pts:
                pt = Point()
                pt.x = float(wp[0]); pt.y = float(wp[1]); pt.z = 0.0
                pm.points.append(pt)

            avg_x = float(np.mean([w[0] for w in world_pts]))
            avg_y = float(np.mean([w[1] for w in world_pts]))

            # text label
            tm = Marker()
            tm.header.frame_id = frame
            tm.header.stamp    = now
            tm.ns              = 'frontier_cluster_labels'
            tm.id              = label
            tm.type            = Marker.TEXT_VIEW_FACING
            tm.action          = Marker.ADD
            tm.pose.position.x = avg_x
            tm.pose.position.y = avg_y + 0.3
            tm.pose.position.z = 0.3
            tm.pose.orientation.w = 1.0
            tm.scale.z         = 0.25
            tm.color.r         = 0.0
            tm.color.g         = 0.0
            tm.color.b         = 1.0
            tm.color.a         = 1.0
            tm.text            = f'C{label}({len(pts)})'
            # lifetime = 0 → permanent in RViz

            ma.markers.append(pm)
            ma.markers.append(tm)

        self.cluster_pub.publish(ma)

    def _publish_best_viewpoint_marker(self, world_xy):
        """Green sphere + 'Best Viewpoint' text — from original repo."""
        frame = 'world_enu' if self.use_sim else 'odom'
        now   = self.get_clock().now().to_msg()

        sphere = Marker()
        sphere.header.frame_id = frame
        sphere.header.stamp    = now
        sphere.ns              = 'viewpoint_markers'
        sphere.id              = 0
        sphere.type            = Marker.SPHERE
        sphere.action          = Marker.ADD
        sphere.pose.position.x = float(world_xy[0])
        sphere.pose.position.y = float(world_xy[1])
        sphere.pose.position.z = 0.0
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = 0.2; sphere.scale.y = 0.2; sphere.scale.z = 0.2
        sphere.color.r = 0.0; sphere.color.g = 1.0; sphere.color.b = 0.0
        sphere.color.a = 1.0
        # lifetime = 0 → permanent in RViz
        self.marker_pub.publish(sphere)

        txt = Marker()
        txt.header.frame_id = frame
        txt.header.stamp    = now
        txt.ns              = 'viewpoint_markers'
        txt.id              = 1
        txt.type            = Marker.TEXT_VIEW_FACING
        txt.action          = Marker.ADD
        txt.pose.position.x = float(world_xy[0])
        txt.pose.position.y = float(world_xy[1]) + 0.4
        txt.pose.position.z = 0.5
        txt.pose.orientation.w = 1.0
        txt.scale.z = 0.3
        txt.color.r = 1.0; txt.color.g = 1.0; txt.color.b = 0.0
        txt.color.a = 1.0
        txt.text    = 'Best Viewpoint'
        txt.lifetime.nanosec = int(5e9)
        self.marker_pub.publish(txt)

    def _clear_clusters_and_frontiers(self):
        self._clear_clusters()
        self._clear_frontiers()

    def _clear_clusters(self):
        ma = MarkerArray()
        m  = Marker()
        m.action = Marker.DELETEALL
        m.id     = 0
        ma.markers.append(m)
        self.cluster_pub.publish(ma)
        self.clusters.clear()

    def _clear_frontiers(self):
        pa = PoseArray()
        pa.header.stamp    = self.get_clock().now().to_msg()
        pa.header.frame_id = 'world_enu' if self.use_sim else 'odom'
        self.frontier_pub.publish(pa)

    def _pub_status(self, text: str):
        m = String(); m.data = text
        self.status_pub.publish(m)


# 
def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
