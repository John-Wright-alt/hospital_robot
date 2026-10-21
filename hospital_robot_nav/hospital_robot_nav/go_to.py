#!/usr/bin/env python3
"""
go_to.py  --  send the robot to a named waypoint through the mission node,
and print its progress until it finishes. A TEST TOOL for Phase 7 step N.

    ros2 run hospital_robot_nav go_to pharmacy              # guide speed
    ros2 run hospital_robot_nav go_to home --mode alone     # alone speed
    ros2 run hospital_robot_nav go_to --cancel              # stop the current goal

Ctrl+C while it runs CANCELS the goal (the robot stops).
The robot must already be ARMED by a person and the column PARKED:
this tool, like the mission node, never arms anything.
"""
import argparse
import json
import random
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

TERMINAL = ('SUCCEEDED', 'FAILED', 'CANCELED')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('target', nargs='?', default='')
    ap.add_argument('--mode', choices=('guide', 'alone'), default='guide')
    ap.add_argument('--cancel', action='store_true')
    a = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    if not a.cancel and not a.target:
        ap.error('give a waypoint name, or --cancel')

    rclpy.init()
    node = Node('go_to')
    pub = node.create_publisher(String, 'mission/command', 10)
    gid = random.randint(1000, 9999)
    last = {}

    def on_status(msg):
        try:
            st = json.loads(msg.data)
        except ValueError:
            return
        if st.get('id') != gid:
            return
        key = (st.get('state'), st.get('reason'))
        if key != last.get('key') or st.get('state') == 'ACTIVE':
            rem = st.get('remaining_m')
            print(f'[{time.strftime("%H:%M:%S")}] {st.get("state"):10} {st.get("target")}'
                  + (f'  {rem:.2f} m to go' if isinstance(rem, (int, float)) else '')
                  + (f'  - {st["reason"]}' if st.get('reason') else ''), flush=True)
            last['key'] = key
        last['state'] = st.get('state')

    node.create_subscription(String, 'mission/status', on_status, 10)
    t0 = time.monotonic()
    while pub.get_subscription_count() == 0 and time.monotonic() - t0 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() == 0:
        print('No mission node is listening on /mission/command. Start navigation.launch.py.')
        return 1

    if a.cancel:
        pub.publish(String(data=json.dumps({'cmd': 'cancel'})))
        rclpy.spin_once(node, timeout_sec=0.5)
        print('cancel sent')
        return 0

    pub.publish(String(data=json.dumps({'cmd': 'go', 'target': a.target, 'mode': a.mode,
                                        'id': gid})))
    print(f'go {a.target} ({a.mode}), id {gid}. Ctrl+C cancels.')
    try:
        while rclpy.ok() and last.get('state') not in TERMINAL:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pub.publish(String(data=json.dumps({'cmd': 'cancel', 'id': gid})))
        t1 = time.monotonic()
        while time.monotonic() - t1 < 1.0:
            rclpy.spin_once(node, timeout_sec=0.1)
        print('\ncancel sent')
    node.destroy_node()
    rclpy.try_shutdown()
    return 0 if last.get('state') == 'SUCCEEDED' else 1


if __name__ == '__main__':
    sys.exit(main())
