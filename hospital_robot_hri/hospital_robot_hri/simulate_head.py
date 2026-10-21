#!/usr/bin/env python3
"""
simulate_head.py  --  SIMULATED head column for testing Anna's wake-up flow
WITHOUT moving the real lift. It pretends to be the Phase 8 head_column node and
the Pico's lift, but ONLY on its own test topics:

    listens   /hri/sim/head_command   (search | hold | park | stop | home)
    publishes /hri/sim/head_state     (SEARCHING, TRACKING, WAITING, PARKED, ...)
              /hri/sim/lift_state     ('l <state> <homed> <pos> <target> <switch> <fault>')

It never publishes on /head_column/* or /lift/*, so it cannot move anything even
when the real robot is running. hri_interaction uses these topics only when
started with  sim_head:=true  (hri.launch.py does this for you).

    ros2 run hospital_robot_hri simulate_head normal     # finds a face 4 s after 'search'
    ros2 run hospital_robot_hri simulate_head noface     # never finds a face
    ros2 run hospital_robot_hri simulate_head notarmed   # stays WAIT_ARM (robot not armed)

All values are made-up TEST data, not sensor data.
"""

import argparse
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

SPEED_MM_S = 20.0


class SimulateHead(Node):

    def __init__(self, scenario):
        super().__init__('simulate_head')
        self.scenario = scenario
        self.state = 'WAIT_ARM' if scenario == 'notarmed' else 'PARKED'
        self.pos = 0.0
        self.target = 0.0
        self.face_mm = 160.0          # pretend the face needs the column at 160 mm
        self.t_state = 0.0
        self.state_pub = self.create_publisher(String, 'hri/sim/head_state', 10)
        self.lift_pub = self.create_publisher(String, 'hri/sim/lift_state', 10)
        self.create_subscription(String, 'hri/sim/head_command', self.on_cmd, 10)
        self.create_timer(0.1, self.tick)
        self.k = 0
        self.get_logger().warn(f'SIMULATED head column, scenario "{scenario}". Test data only; '
                               'publishes on /hri/sim/* and never moves the real lift.')

    def set_state(self, s):
        if s != self.state:
            self.get_logger().info(f'{self.state} -> {s}')
            self.state, self.t_state = s, 0.0
            self.state_pub.publish(String(data=s))

    def on_cmd(self, msg):
        cmd = msg.data.strip().lower()
        self.get_logger().info(f'command: {cmd}')
        if self.state == 'WAIT_ARM':
            return
        if cmd == 'search':
            self.set_state('SEARCHING')
            self.target = 400.0
        elif cmd == 'hold':
            self.target = self.pos
            self.set_state('WAITING')
        elif cmd == 'park':
            self.target = 0.0
            self.set_state('PARKING')
        elif cmd == 'stop':
            self.target = self.pos
            self.set_state('STOPPED')

    def tick(self):
        self.k += 1
        self.t_state += 0.1
        step = SPEED_MM_S * 0.1
        if abs(self.target - self.pos) <= step:
            self.pos = self.target
        else:
            self.pos += step if self.target > self.pos else -step
        moving = self.pos != self.target

        if self.state == 'SEARCHING' and self.scenario == 'normal' and self.pos >= self.face_mm - 40:
            self.set_state('TRACKING')                     # "sees" the face ...
            self.target = self.face_mm                     # ... one short correction, then still
        if self.state == 'PARKING' and not moving:
            self.set_state('PARKED')

        if self.k % 5 == 0:
            self.state_pub.publish(String(data=self.state))
        lift = 1 if moving else 0
        self.lift_pub.publish(String(
            data=f'l {lift} 1 {self.pos:.1f} {self.target:.1f} {int(self.pos == 0.0)} 0'))


def main():
    ap = argparse.ArgumentParser(description='SIMULATED head column')
    ap.add_argument('scenario', nargs='?', default='normal', choices=['normal', 'noface', 'notarmed'])
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=[sys.argv[0]] + ros_args)
    node = SimulateHead(args.scenario)
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
