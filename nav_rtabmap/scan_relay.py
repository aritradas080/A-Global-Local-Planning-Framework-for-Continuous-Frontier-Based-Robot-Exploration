#!/usr/bin/env python3
"""
scan_relay.py — Fixes Stonefish LiDAR convention for rtabmap.

Stonefish publishes the RPLidar scan with roll=-180° on the sensor frame.
This makes the scan angles run CLOCKWISE instead of the standard ROS
counterclockwise convention.  rtabmap reads the angles as-is and builds
a mirror-inverted map.

The lidar also has yaw=180° (faces backward), so beam index 0 in the raw
scan points at the robot's BACK, not its front.

This node republishes the scan with:
  - angles negated  (clockwise → counterclockwise)
  - ranges reindexed with (half - j) % n — reverses direction AND rotates
    180° so beam 0 aligns with the robot's front
  - frame_id changed to turtlebot/base_link  (clean 2D frame, no roll)

rtabmap is configured to subscribe to /turtlebot/scan_rtabmap.

NOTE: Do NOT add /turtlebot/scan_rtabmap as a LaserScan display in RViz —
the angles are manually corrected so they won't align visually.
Instead visualise the raw /turtlebot/scan topic (frame turtlebot/rplidar_link).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanRelay(Node):
    def __init__(self):
        super().__init__('scan_relay')
        self.sub = self.create_subscription(
            LaserScan, '/turtlebot/scan', self._cb, 10)
        self.pub = self.create_publisher(
            LaserScan, '/turtlebot/scan_rtabmap', 10)
        self.get_logger().info('scan_relay: inverting scan angles for rtabmap')

    def _cb(self, msg: LaserScan):
        out = LaserScan()
        out.header.stamp    = msg.header.stamp
        out.header.frame_id = 'turtlebot/base_link'   # clean 2D frame

        n = len(msg.ranges)

        # Negate angle boundaries to switch from Clockwise to Counter-Clockwise
        out.angle_min       = -msg.angle_max
        out.angle_max       = -msg.angle_min
        out.angle_increment =  msg.angle_increment     # stays positive
        out.time_increment  =  msg.time_increment
        out.scan_time       =  msg.scan_time
        out.range_min       =  msg.range_min
        out.range_max       =  msg.range_max

        # Fixes BOTH the clockwise direction AND the 180° yaw offset:
        #   (half - j) % n  reverses beam order (clockwise → CCW)
        #                   AND rotates by n/2 (180°) so beam 0 → robot front
        half = n // 2
        out.ranges = [msg.ranges[(half - j) % n] for j in range(n)]

        if msg.intensities and len(msg.intensities) == n:
            out.intensities = [msg.intensities[(half - j) % n] for j in range(n)]

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ScanRelay()
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
