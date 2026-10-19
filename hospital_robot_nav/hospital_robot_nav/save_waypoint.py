#!/usr/bin/env python3
"""
save_waypoint.py  --  store the robot's CURRENT position on the map under a name.

    ros2 run hospital_robot_nav save_waypoint home --site apartment --label "Front door"
    ros2 run hospital_robot_nav save_waypoint pharmacy --site apartment --label "Kitchen"
    ros2 run hospital_robot_nav save_waypoint --site apartment --list

How to use: with navigation.launch.py running and the robot localised (scan
lies on the map walls in RViz), drive it by teleop to the exact spot and
heading where it should stop, then run this. It reads TF map -> base_footprint
(that is AMCL's current estimate) and writes ~/robot_ws/maps/<site>/waypoints.yaml.
The heading matters: at a destination, face the robot so the patient can read
the screen and the door is in view.
"""
import argparse
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from hospital_robot_nav import waypoints as wp


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('name', nargs='?')
    ap.add_argument('--label', default=None)
    ap.add_argument('--site', default='apartment',
                    help='map folder in ~/robot_ws/maps (same as navigation.launch.py site:=)')
    ap.add_argument('--file', default=None,
                    help='override: full path of the waypoints file')
    ap.add_argument('--list', action='store_true')
    a = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    if a.file is None:
        a.file = os.path.join('~/robot_ws/maps', a.site, 'waypoints.yaml')

    if a.list:
        try:
            for n, (x, y, yaw) in sorted(wp.load(a.file).items()):
                print(f'{n:10} x={x:7.2f}  y={y:7.2f}  yaw={math.degrees(yaw):6.1f} deg')
        except wp.WaypointError as err:
            print(err)
            return 1
        return 0
    if not a.name:
        ap.error('give a name (home, doctor, xray, pharmacy) or --list')

    rclpy.init()
    node = Node('save_waypoint')
    buf = Buffer()
    TransformListener(buf, node)
    t0 = time.monotonic()
    tf = None
    while time.monotonic() - t0 < 5.0 and tf is None:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            tf = buf.lookup_transform('map', 'base_footprint', Time())
        except Exception:                                     # noqa: BLE001
            tf = None
    if tf is None:
        print('No TF map -> base_footprint. Is navigation.launch.py running, and is the robot '
              'localised (2D Pose Estimate)?')
        return 1
    x, y = tf.transform.translation.x, tf.transform.translation.y
    q = tf.transform.rotation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    saved = wp.save(a.file, a.name, x, y, yaw, a.label)
    print(f'saved "{a.name}": x={saved["x"]} y={saved["y"]} yaw={saved["yaw_deg"]} deg '
          f'-> {a.file}')
    if a.name.lower() == 'home':
        print('home also becomes AMCL\'s start pose the next time navigation.launch.py starts.')
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
