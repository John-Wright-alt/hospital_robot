#!/usr/bin/env python3
"""
lidar_gaze_node.py  --  ROS 2 Jazzy node: "where should the eyes look?" from the LD06.

    /scan (sensor_msgs/LaserScan)  ──►  [lidar_gaze]  ──►  /hri/gaze/status   (std_msgs/String, JSON)
    /odom (nav_msgs/Odometry, optional) ─┘                 /hri/gaze/target   (geometry_msgs/PointStamped)
                                                           /hri/gaze/markers  (visualization_msgs/MarkerArray, RViz)

All the logic is in gaze_core.py (unit tested). This file only moves data
between ROS and GazeEngine.

/hri/gaze/status is published on EVERY scan, so the UI can tell
"nothing there" apart from "this node is not running". Example:
  {"source": "lidar", "simulated": false, "state": "tracking",
   "bearing_deg": 34.2, "range_m": 1.12, "speed_mps": 0.41,
   "track_id": 7, "n_objects": 1, "frame": "base_footprint"}

state: learning | none | tracking | lost | robot_moving | waiting_for_tf
bearing_deg: + = the ROBOT'S LEFT (ROS convention).

FOR THE EYES ONLY. Never use these topics for obstacle avoidance or safety.
"""

import json
import math

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException

from hospital_robot_base.gaze_core import GazeEngine, GazeParams


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class LidarGazeNode(Node):

    def __init__(self):
        super().__init__('lidar_gaze')
        dp = self.declare_parameter
        self.scan_topic = dp('scan_topic', 'scan').value
        self.odom_topic = dp('odom_topic', 'odom').value
        self.target_frame = dp('target_frame', 'base_footprint').value
        self.use_tf = dp('use_tf', True).value
        self.fallback_yaw_deg = dp('fallback_yaw_deg', 0.0).value
        self.moving_linear = dp('moving_linear_mps', 0.03).value
        self.moving_angular = dp('moving_angular_rps', 0.08).value
        self.settle_s = dp('settle_after_motion_s', 1.0).value
        self.publish_markers = dp('publish_markers', True).value

        # Every GazeParams field can be set from YAML with the same name.
        p = GazeParams()
        for name, default in vars(GazeParams()).items():
            if name.startswith('frame_'):
                continue                       # comes from TF, not YAML
            setattr(p, name, type(default)(dp(name, default).value))
        self.params = p
        self.engine = None                     # created once the frame is known

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_wait_start = None
        self.out_frame = self.target_frame

        self.moving = False
        self.still_since = None
        self.odom_seen = False

        self.status_pub = self.create_publisher(String, 'hri/gaze/status', 10)
        self.target_pub = self.create_publisher(PointStamped, 'hri/gaze/target', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'hri/gaze/markers', 10)
        self.create_subscription(LaserScan, self.scan_topic, self.on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)

        self.last_state = None
        self.get_logger().info(
            f'lidar_gaze started: {self.scan_topic} -> /hri/gaze/status. '
            f'Bearings in "{self.target_frame}". Keep the area clear for '
            f'~{p.learn_scans / 10:.0f} s while the background is learned.')

    # ------------------------------------------------------------------ setup
    def make_engine(self, scan_frame):
        """Find where the LiDAR sits in target_frame (from your URDF via TF)."""
        p = self.params
        if not self.use_tf:
            p.frame_x = p.frame_y = 0.0
            p.frame_yaw = math.radians(self.fallback_yaw_deg)
            self.out_frame = scan_frame
            self.get_logger().warn(
                f'use_tf is false: bearings are relative to the LiDAR itself, '
                f'rotated by fallback_yaw_deg={self.fallback_yaw_deg}.')
            self.engine = GazeEngine(p)
            return True
        try:
            tf = self.tf_buffer.lookup_transform(self.target_frame, scan_frame, Time())
        except TransformException as err:
            now = self.get_clock().now()
            if self.tf_wait_start is None:
                self.tf_wait_start = now
            if (now - self.tf_wait_start) > Duration(seconds=5.0):
                self.get_logger().warn(
                    f'No TF {self.target_frame} <- {scan_frame} yet ({err}). Is '
                    'robot_state_publisher running (base.launch.py)? Or set '
                    'use_tf:=false for a LiDAR-only test.', throttle_duration_sec=10.0)
            return False
        t = tf.transform
        p.frame_x, p.frame_y = t.translation.x, t.translation.y
        p.frame_yaw = yaw_from_quat(t.rotation)
        self.out_frame = self.target_frame
        self.get_logger().info(
            f'LiDAR "{scan_frame}" in "{self.target_frame}": x={p.frame_x:.3f} '
            f'y={p.frame_y:.3f} m, yaw={math.degrees(p.frame_yaw):.1f} deg (from TF/URDF)')
        self.engine = GazeEngine(p)
        return True

    # ------------------------------------------------------------------- odom
    def on_odom(self, msg: Odometry):
        """Background subtraction only works while the robot stands still."""
        self.odom_seen = True
        tw = msg.twist.twist
        moving = (abs(tw.linear.x) > self.moving_linear
                  or abs(tw.angular.z) > self.moving_angular)
        now = self.get_clock().now()
        if moving and not self.moving:
            self.get_logger().info('Robot moving: curiosity gaze paused.')
            if self.engine is not None:
                self.engine.reset()
        if not moving and self.moving:
            self.still_since = now
        self.moving = moving

    def settling(self):
        if self.moving:
            return True
        if self.still_since is None:
            return False
        if (self.get_clock().now() - self.still_since) < Duration(seconds=self.settle_s):
            return True
        self.still_since = None
        if self.engine is not None:
            self.engine.reset()                # re-learn from where it stopped
        return False

    # ------------------------------------------------------------------- scan
    def on_scan(self, scan: LaserScan):
        stamp = scan.header.stamp
        t = stamp.sec + stamp.nanosec * 1e-9
        if t == 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9

        if self.engine is None and not self.make_engine(scan.header.frame_id):
            self.publish_status({'state': 'waiting_for_tf'})
            return
        if self.settling():
            self.publish_status({'state': 'robot_moving'})
            return

        g = self.engine.process(scan.ranges, scan.angle_min, scan.angle_increment,
                                scan.range_min, scan.range_max, t)
        status = {'state': g.state, 'n_objects': g.n_objects}
        if g.state in ('tracking', 'lost'):
            status.update(bearing_deg=round(math.degrees(g.bearing_rad), 1),
                          range_m=round(g.range_m, 2),
                          speed_mps=round(g.speed_mps, 2), track_id=g.track_id)
            pt = PointStamped()
            pt.header.stamp = stamp
            pt.header.frame_id = self.out_frame
            pt.point.x, pt.point.y = float(g.x), float(g.y)
            self.target_pub.publish(pt)
        self.publish_status(status)
        if self.publish_markers:
            self.publish_debug(g, stamp)

        if g.state != self.last_state:
            extra = (f' bearing={status["bearing_deg"]:+.0f} deg range={status["range_m"]} m'
                     if 'bearing_deg' in status else '')
            self.get_logger().info(f'gaze: {g.state}{extra}')
            self.last_state = g.state

    def publish_status(self, fields):
        msg = {'source': 'lidar', 'simulated': False, 'frame': self.out_frame,
               'odom': self.odom_seen}
        msg.update(fields)
        self.status_pub.publish(String(data=json.dumps(msg)))

    def publish_debug(self, g, stamp):
        """Grey spheres = every detected object; green = the one being looked at."""
        arr = MarkerArray()
        blobs = Marker()
        blobs.header.stamp, blobs.header.frame_id = stamp, self.out_frame
        blobs.ns, blobs.id, blobs.type = 'gaze_objects', 0, Marker.SPHERE_LIST
        blobs.scale.x = blobs.scale.y = blobs.scale.z = 0.12
        blobs.color.r = blobs.color.g = blobs.color.b = 0.7
        blobs.color.a = 1.0
        blobs.pose.orientation.w = 1.0
        blobs.lifetime = Duration(seconds=0.3).to_msg()
        for d in g.detections:
            pt = Point(x=d.x, y=d.y, z=0.3)
            blobs.points.append(pt)
        arr.markers.append(blobs)

        focus = Marker()
        focus.header.stamp, focus.header.frame_id = stamp, self.out_frame
        focus.ns, focus.id = 'gaze_focus', 1
        focus.lifetime = Duration(seconds=0.3).to_msg()
        if g.state in ('tracking', 'lost'):
            focus.type, focus.action = Marker.ARROW, Marker.ADD
            start = Point(x=0.0, y=0.0, z=0.3)
            end = Point(x=float(g.x), y=float(g.y), z=0.3)
            focus.points = [start, end]
            focus.scale.x, focus.scale.y, focus.scale.z = 0.03, 0.08, 0.1
            focus.color.g, focus.color.a = 1.0, 1.0
            if g.state == 'lost':
                focus.color.r = 1.0            # yellow while coasting
        else:
            focus.action = Marker.DELETE
        arr.markers.append(focus)
        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = LidarGazeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
