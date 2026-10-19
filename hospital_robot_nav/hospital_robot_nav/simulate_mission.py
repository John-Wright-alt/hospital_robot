#!/usr/bin/env python3
"""
simulate_mission.py  --  a PRETEND mission node for testing Anna's whole trip
flow on the PC or the Pi WITHOUT Nav2, a map or a charged battery.
Nothing moves: it only answers /mission/command on /mission/status.

    ros2 run hospital_robot_nav simulate_mission                 # every trip succeeds
    ros2 run hospital_robot_nav simulate_mission --scenario fail # every trip fails after 5 s
    ros2 run hospital_robot_nav simulate_mission --scenario notarmed
    ros2 run hospital_robot_nav simulate_mission --trip-s 8      # each trip takes 8 s

Use it with the simulated head column:
    ros2 launch hospital_robot_hri hri.launch.py sim_head:=true nav:=true remote_touch:=true
Do NOT run it together with the real mission node (navigation.launch.py):
both would answer.
"""
import argparse
import json
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class SimulateMission(Node):

    def __init__(self, scenario, trip_s):
        super().__init__('simulate_mission')
        self.scenario, self.trip_s = scenario, trip_s
        self.pub = self.create_publisher(String, 'mission/status', 10)
        self.create_subscription(String, 'mission/command', self.on_command, 10)
        self.create_timer(0.5, self.tick)
        self.trip = None            # dict: id, target, mode, t0, state, reason
        self.get_logger().warn(f'SIMULATED mission node (scenario "{scenario}", {trip_s:.0f} s '
                               'per trip). Nothing moves. Not for use with navigation.launch.py.')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_command(self, msg):
        try:
            cmd = json.loads(msg.data)
        except (ValueError, TypeError):
            w = str(msg.data).split()
            cmd = {'cmd': w[0] if w else '', 'target': w[1] if len(w) > 1 else ''}
        kind = str(cmd.get('cmd', '')).lower()
        if kind == 'go':
            self.trip = {'id': cmd.get('id'), 'target': str(cmd.get('target', '')),
                         'mode': str(cmd.get('mode', 'guide')), 't0': self.now(),
                         'state': 'STARTING', 'reason': ''}
            if self.scenario == 'notarmed':
                self.trip.update(state='FAILED', reason='not_armed: (simulated) /robot_state is DISARMED')
            self.get_logger().info(f'go {self.trip["target"]} ({self.trip["mode"]}) id={self.trip["id"]}')
            self.publish()
        elif kind == 'cancel' and self.trip and self.trip['state'] in ('STARTING', 'ACTIVE'):
            self.trip.update(state='CANCELED', reason='cancelled by request')
            self.get_logger().info('cancel')
            self.publish()

    def tick(self):
        if not self.trip:
            return
        tr, age = self.trip, self.now() - self.trip['t0']
        if tr['state'] == 'STARTING' and age > 1.0:
            tr['state'] = 'ACTIVE'
        elif tr['state'] == 'ACTIVE':
            if self.scenario == 'fail' and age > 5.0:
                tr.update(state='FAILED', reason='nav_failed: (simulated) Nav2 status 6')
            elif self.scenario != 'fail' and age > self.trip_s:
                tr['state'] = 'SUCCEEDED'
        self.publish()

    def publish(self):
        tr = self.trip
        rem = None
        if tr['state'] == 'ACTIVE':
            rem = round(max(0.0, 0.25 * (self.trip_s - (self.now() - tr['t0']))), 2)
        st = {'id': tr['id'], 'state': tr['state'], 'target': tr['target'], 'mode': tr['mode'],
              'reason': tr['reason'], 'remaining_m': rem, 'simulated': True}
        self.pub.publish(String(data=json.dumps(st)))


def main(args=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--scenario', choices=('ok', 'fail', 'notarmed'), default='ok')
    ap.add_argument('--trip-s', type=float, default=12.0)
    a = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    rclpy.init(args=args)
    node = None
    try:
        node = SimulateMission(a.scenario, a.trip_s)
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
