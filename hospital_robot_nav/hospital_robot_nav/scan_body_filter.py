#!/usr/bin/env python3
"""
scan_body_filter.py  --  ROS 2 Jazzy node "scan_body_filter".

    /scan (sensor_msgs/LaserScan, frame lidar_link)
        -> [scan_body_filter] ->
    /scan_filtered (same, but every beam that hits the ROBOT ITSELF is removed)

Why: the LD06 sits right next to the head column/mast (URDF: lidar at
x 0.222, column at x 0.175 in base_link). Those returns look like an obstacle
glued to the robot. Nav2 would then think the robot is permanently inside an
obstacle ("start occupied", or the collision monitor stops it forever).
Phase 6C step B5-4 measured this "blind sector".

How: each beam's end point is moved into the base_footprint frame (using the
lidar_link -> base_footprint transform from your URDF, read once from TF).
If the point lies inside the robot's own box (the Nav2 footprint plus a small
margin), the range is replaced by +inf ("no return", REP 117).
Everything outside the box is passed through unchanged. This only REMOVES
points inside the robot's own outline; it never removes a real obstacle that
is outside the robot.

SLAM Toolbox, AMCL, the costmaps and the collision monitor all use
/scan_filtered (see config/). lidar_gaze and anything else keep using /scan.
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from hospital_robot_nav.scan_geometry import filter_ranges


class ScanBodyFilter(Node):

    def __init__(self):
        super().__init__('scan_body_filter')
        dp = lambda n, d: self.declare_parameter(n, d).value  # noqa: E731
        self.base_frame = dp('base_frame', 'base_footprint')
        # Robot outline in base_footprint (metres). Default = the Nav2 footprint
        # in config/nav2_params_real.yaml (+/-0.20 x, +/-0.23 y) plus 3 cm.
        self.box = (float(dp('box_x_min', -0.23)), float(dp('box_x_max', 0.23)),
                    float(dp('box_y_min', -0.26)), float(dp('box_y_max', 0.26)))
        self.pose = None                      # (x, y, yaw) of the lidar in base frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(LaserScan, dp('output_topic', 'scan_filtered'), 10)
        self.create_subscription(LaserScan, dp('input_topic', 'scan'), self.on_scan,
                                 qos_profile_sensor_data)
        self.n_in = self.n_removed = 0
        self.create_timer(30.0, self.report)
        self.get_logger().info(
            f'scan_body_filter: removing points inside x[{self.box[0]:+.2f},{self.box[1]:+.2f}] '
            f'y[{self.box[2]:+.2f},{self.box[3]:+.2f}] m of {self.base_frame}.')

    def lookup(self, frame):
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, frame, Time())
        except Exception as err:                       # noqa: BLE001  (tf2 raises several types)
            self.get_logger().warn(f'waiting for TF {self.base_frame} <- {frame}: {err}',
                                   throttle_duration_sec=5.0)
            return None
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        p = (tf.transform.translation.x, tf.transform.translation.y, yaw)
        self.get_logger().info(f'lidar pose in {self.base_frame}: x={p[0]:.3f} y={p[1]:.3f} '
                               f'yaw={math.degrees(p[2]):.1f} deg (from the URDF)')
        return p

    def on_scan(self, msg: LaserScan):
        if self.pose is None:
            self.pose = self.lookup(msg.header.frame_id)
            if self.pose is None:
                return                          # publish nothing until we know the geometry
        ranges, removed = filter_ranges(msg.ranges, msg.angle_min, msg.angle_increment,
                                        *self.pose, self.box)
        out = LaserScan()
        out.header = msg.header
        out.angle_min, out.angle_max = msg.angle_min, msg.angle_max
        out.angle_increment, out.time_increment = msg.angle_increment, msg.time_increment
        out.scan_time, out.range_min, out.range_max = msg.scan_time, msg.range_min, msg.range_max
        out.ranges = ranges
        out.intensities = msg.intensities
        self.pub.publish(out)
        self.n_in += len(msg.ranges)
        self.n_removed += removed

    def report(self):
        if self.n_in:
            self.get_logger().info(f'last 30 s: removed {self.n_removed} of {self.n_in} beams '
                                   f'({100.0 * self.n_removed / self.n_in:.1f} %) as robot body')
        self.n_in = self.n_removed = 0


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ScanBodyFilter()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
