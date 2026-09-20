#!/usr/bin/env python3
"""
odom_calibration.py  --  drives the robot a precise amount BY ODOMETRY, then
tells you how to correct wheel_radius / wheel_separation from what you measure.

    ros2 run hospital_robot_base odom_calibration straight 2.0
    ros2 run hospital_robot_base odom_calibration rotate 10
    ros2 run hospital_robot_base odom_calibration rotate -10     (clockwise)

Needs the base running (pico_bridge publishing /odom) and the motors ARMED.
Press Ctrl+C at any time: it sends zero speed on exit.
Only use this with the robot ON THE FLOOR, in a clear space, E-stop in hand.
"""

import argparse
import math
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class OdomCalibration(Node):

    def __init__(self, mode, amount, speed, stamped):
        super().__init__('odom_calibration')
        self.mode = mode
        self.direction = 1.0 if amount >= 0 else -1.0   # negative = backwards / right
        self.target = abs(amount) if mode == 'straight' else abs(amount) * 2.0 * math.pi
        self.speed = speed
        self.stamped = stamped
        self.pub = self.create_publisher(TwistStamped if stamped else Twist,
                                         'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 20)
        self.start = None
        self.progress = 0.0
        self.last_yaw = None
        self.done = False
        self.create_timer(0.05, self.tick)
        if mode == 'straight':
            what = f'{abs(amount)} m {"forwards" if amount >= 0 else "BACKWARDS"}'
        else:
            what = f'{abs(amount)} turns {"LEFT (anticlockwise)" if amount >= 0 else "RIGHT (clockwise)"}'
        self.get_logger().info(f'Will drive {what} by odometry at {speed}. '
                               'Waiting for /odom...')

    def on_odom(self, msg):
        p = msg.pose.pose.position
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        if self.start is None:
            self.start = (p.x, p.y)
            self.last_yaw = yaw
            self.get_logger().info('Got /odom - starting.')
            return
        if self.mode == 'straight':
            self.progress = math.hypot(p.x - self.start[0], p.y - self.start[1])
        else:
            d = math.atan2(math.sin(yaw - self.last_yaw), math.cos(yaw - self.last_yaw))
            self.progress += d            # unwrap: add up small heading changes
            self.last_yaw = yaw

    def send(self, vx, wz):
        if self.stamped:
            m = TwistStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = 'base_footprint'
            m.twist.linear.x, m.twist.angular.z = vx, wz
        else:
            m = Twist()
            m.linear.x, m.angular.z = vx, wz
        self.pub.publish(m)

    def tick(self):
        if self.start is None or self.done:
            return
        remaining = self.target - abs(self.progress)
        if remaining <= 0.0:
            self.send(0.0, 0.0)
            self.done = True
            self.report()
            return
        # Slow down over the last part so it doesn't overshoot
        slow_zone = 0.20 if self.mode == 'straight' else 0.5
        factor = max(0.25, min(1.0, remaining / slow_zone))
        if self.mode == 'straight':
            self.send(self.direction * self.speed * factor, 0.0)
        else:
            self.send(0.0, self.direction * self.speed * factor)

    def report(self):
        if self.mode == 'straight':
            self.get_logger().info(
                f'\nODOMETRY SAYS: {abs(self.progress):.3f} m.\n'
                'Now MEASURE the real distance with a tape measure.\n'
                '  new wheel_radius = old wheel_radius x (real distance / odometry distance)\n'
                'Example: odometry 2.000 m, tape says 1.940 m -> multiply radius by 0.970')
        else:
            turns = abs(self.progress) / (2 * math.pi)
            self.get_logger().info(
                f'\nODOMETRY SAYS: {turns:.3f} turns ({math.degrees(abs(self.progress)):.1f} deg).\n'
                'Now look at where the robot points compared with its start mark.\n'
                'Work out the REAL total angle it turned, in degrees.\n'
                '  new wheel_separation = old wheel_separation x (odometry angle / real angle)\n'
                'Example: odometry 3600 deg, it overshot by 30 deg (real 3630)\n'
                '         -> multiply separation by 3600/3630 = 0.9917')


def main():
    parser = argparse.ArgumentParser(description='Odometry calibration drive')
    parser.add_argument('mode', choices=['straight', 'rotate'])
    parser.add_argument('amount', type=float, help='metres, or number of turns (negative = backwards / clockwise)')
    parser.add_argument('--speed', type=float, default=None,
                        help='m/s for straight (default 0.15), rad/s for rotate (default 0.6)')
    parser.add_argument('--unstamped', action='store_true',
                        help='publish Twist instead of TwistStamped')
    args, ros_args = parser.parse_known_args()
    speed = args.speed or (0.15 if args.mode == 'straight' else 0.6)

    rclpy.init(args=[sys.argv[0]] + ros_args)
    node = OdomCalibration(args.mode, args.amount, speed, not args.unstamped)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            for _ in range(5):
                node.send(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
