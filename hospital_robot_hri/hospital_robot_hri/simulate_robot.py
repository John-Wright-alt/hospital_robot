#!/usr/bin/env python3
"""
simulate_robot.py  --  SIMULATED robot inputs for testing the SPEAKER without
moving anything. Publishes ONLY on its own test topics:

    /hri/sim/odom          (nav_msgs/Odometry)   instead of /odom
    /hri/sim/robot_state   (std_msgs/String)     instead of /robot_state
    /hri/sim/pico_status   (std_msgs/String)     instead of /pico/status

It never publishes /cmd_vel, /odom or anything pico_bridge uses, so it cannot
move the robot even if the real base is running. hri_speech listens to these
topics only when started with:  ros2 launch hospital_robot_hri speech.launch.py simulate:=true
(the launch file starts this node for you; pick the scenario with  scenario:=...)

Scenarios (all values are made-up TEST inputs, not sensor data):
    drive      arm, drive forward, stop, resume, turn, reverse, disarm   (~45 s)
    long       drive forward for 100 s (hear the reminders and ticks)
    estop      arm, drive, E-stop pressed, released
    fault      arm, then a latched FAULT that lasts 70 s (hear it repeat)
    lost       state messages stop after 5 s (pico_bridge / USB cable gone)
    battery    battery warning flag appears after 5 s, clears after 30 s
    noise      parked, with odometry noise only (must say NOTHING about motion)
"""

import argparse
import sys

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
import random

# (duration_s, robot_state, v m/s, w rad/s, flags text)   robot_state None = publish nothing
S = 'DISARMED'
SCENARIOS = {
    'drive': [(4, S, 0, 0, 'none'), (3, 'ARMED', 0, 0, 'none'),
              (8, 'ARMED', 0.15, 0, 'none'), (4, 'ARMED', 0, 0, 'none'),
              (5, 'ARMED', 0.15, 0, 'none'), (4, 'ARMED', 0, 0.5, 'none'),
              (3, 'ARMED', 0, 0, 'none'), (4, 'ARMED', -0.1, 0, 'none'),
              (4, 'ARMED', 0, 0, 'none'), (5, S, 0, 0, 'none')],
    'long': [(4, S, 0, 0, 'none'), (3, 'ARMED', 0, 0, 'none'),
             (100, 'ARMED', 0.15, 0, 'none'), (5, S, 0, 0, 'none')],
    'estop': [(3, S, 0, 0, 'none'), (3, 'ARMED', 0, 0, 'none'), (4, 'ARMED', 0.15, 0, 'none'),
              (6, 'ESTOP', 0, 0, 'ESTOP NO_VMOT'), (5, S, 0, 0, 'NO_VMOT')],
    'fault': [(3, S, 0, 0, 'none'), (3, 'ARMED', 0, 0, 'none'),
              (70, 'FAULT', 0, 0, 'STALL_L*'), (5, S, 0, 0, 'none')],
    'lost': [(5, S, 0, 0, 'none'), (30, None, 0, 0, 'none'), (5, S, 0, 0, 'none')],
    'battery': [(5, 'ARMED', 0, 0, 'none'), (25, 'ARMED', 0, 0, 'BATT_WARN'),
                (5, 'ARMED', 0, 0, 'none')],
    'noise': [(60, 'ARMED', 0, 0, 'none')],
}


class SimulateRobot(Node):

    def __init__(self, scenario):
        super().__init__('simulate_robot')
        self.steps = SCENARIOS[scenario]
        self.scenario = scenario
        self.odom_pub = self.create_publisher(Odometry, 'hri/sim/odom', 10)
        self.state_pub = self.create_publisher(String, 'hri/sim/robot_state', 10)
        self.status_pub = self.create_publisher(String, 'hri/sim/pico_status', 10)
        self.t = 0.0
        self.i = 0
        self.step_t = 0.0
        self.k = 0
        self.rng = random.Random(0)
        self.create_timer(0.02, self.tick)                 # 50 Hz like pico_bridge
        self.get_logger().warn(f'SIMULATED robot, scenario "{scenario}". Test data only; '
                               'publishes on /hri/sim/* and never on real robot topics.')
        self._announce()

    def _announce(self):
        d, state, v, w, flags = self.steps[self.i]
        self.get_logger().info(f'step {self.i + 1}/{len(self.steps)}: {d} s  state={state} '
                               f'v={v} m/s w={w} rad/s flags={flags}')

    def tick(self):
        self.t += 0.02
        self.step_t += 0.02
        self.k += 1
        d, state, v, w, flags = self.steps[self.i]
        if self.step_t >= d and self.i < len(self.steps) - 1:
            self.i += 1
            self.step_t = 0.0
            self._announce()
            d, state, v, w, flags = self.steps[self.i]

        if self.scenario == 'noise':                        # encoder-like jitter only
            v = self.rng.uniform(-0.008, 0.008)
            w = self.rng.uniform(-0.04, 0.04)
        o = Odometry()
        o.header.stamp = self.get_clock().now().to_msg()
        o.header.frame_id = 'odom'
        o.child_frame_id = 'base_footprint'
        o.twist.twist.linear.x = float(v)
        o.twist.twist.angular.z = float(w)
        self.odom_pub.publish(o)

        if self.k % 25 == 0 and state is not None:          # 2 Hz like the Pico 's' lines
            self.state_pub.publish(String(data=state))
            self.status_pub.publish(String(
                data=f'{state}  flags=0000 ({flags})  vbat=11.80 V  vmot=11.70 V'))


def main():
    ap = argparse.ArgumentParser(description='SIMULATED robot inputs for the speaker')
    ap.add_argument('scenario', nargs='?', default='drive', choices=sorted(SCENARIOS))
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=[sys.argv[0]] + ros_args)
    node = SimulateRobot(args.scenario)
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
