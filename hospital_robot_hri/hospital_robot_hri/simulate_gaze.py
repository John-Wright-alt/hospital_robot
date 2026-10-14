#!/usr/bin/env python3
"""
simulate_gaze.py  --  SIMULATED gaze input for testing the eyes WITHOUT the LiDAR.

Publishes on /hri/gaze/status exactly like lidar_gaze does, but every message
says "simulated": true and "source": "test", and the face page then shows a
"Simulated input" badge. These are test inputs, NOT sensor observations.

    ros2 run hospital_robot_hri simulate_gaze sweep      # slowly left <-> right
    ros2 run hospital_robot_hri simulate_gaze left       # robot's LEFT  (+60 deg)
    ros2 run hospital_robot_hri simulate_gaze right      # robot's RIGHT (-60 deg)
    ros2 run hospital_robot_hri simulate_gaze centre
    ros2 run hospital_robot_hri simulate_gaze visits     # "someone" appears every 6 s
                                                         # at a new place (tests 'curious')

Do NOT run this at the same time as lidar_gaze: both publish the same topic.
"""

import argparse
import json
import math
import random
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

FIXED = {'left': 60.0, 'right': -60.0, 'centre': 0.0, 'center': 0.0}


class SimulateGaze(Node):

    def __init__(self, mode, rate):
        super().__init__('simulate_gaze')
        self.mode = mode
        self.pub = self.create_publisher(String, 'hri/gaze/status', 10)
        self.t = 0.0
        self.dt = 1.0 / rate
        self.track_id = 1
        self.visit_bearing = 0.0
        self.create_timer(self.dt, self.tick)
        self.get_logger().warn(f'SIMULATED gaze input, mode "{mode}". '
                               'This is test data, not a sensor.')

    def tick(self):
        self.t += self.dt
        msg = {'source': 'test', 'simulated': True, 'frame': 'base_footprint',
               'odom': False, 'n_objects': 1, 'range_m': 1.0, 'speed_mps': 0.0}
        if self.mode in FIXED:
            msg.update(state='tracking', bearing_deg=FIXED[self.mode], track_id=1)
        elif self.mode == 'sweep':
            msg.update(state='tracking', track_id=1,
                       bearing_deg=round(70.0 * math.sin(2 * math.pi * self.t / 8.0), 1),
                       speed_mps=0.4)
        else:  # visits: 3 s present, 3 s absent, new ID and place each time
            phase = self.t % 6.0
            if phase < self.dt:
                self.track_id += 1
                self.visit_bearing = random.choice([-70, -40, -15, 15, 40, 70])
            if phase < 3.0:
                msg.update(state='tracking', track_id=self.track_id,
                           bearing_deg=float(self.visit_bearing))
            else:
                msg.update(state='none', n_objects=0)
        self.pub.publish(String(data=json.dumps(msg)))


def main():
    ap = argparse.ArgumentParser(description='SIMULATED gaze input for the eyes')
    ap.add_argument('mode', nargs='?', default='sweep',
                    choices=['sweep', 'left', 'right', 'centre', 'center', 'visits'])
    ap.add_argument('--rate', type=float, default=10.0)
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=[sys.argv[0]] + ros_args)
    node = SimulateGaze(args.mode, args.rate)
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
