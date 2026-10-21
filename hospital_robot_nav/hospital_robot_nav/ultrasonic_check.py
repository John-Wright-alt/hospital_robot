#!/usr/bin/env python3
"""
ultrasonic_check.py  --  live table of the 3 ultrasonic sensors (Phase 7 step U).
Read-only: it only listens.

    ros2 run hospital_robot_nav ultrasonic_check

Shows, once a second, for /ultrasonic/left, /front, /right: rate (Hz), the last
range, min/max over the last second, and how many readings were "no echo"
(= exactly max_range). Ctrl+C to stop.
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range

TOPICS = ('ultrasonic/left', 'ultrasonic/front', 'ultrasonic/right')


def main():
    rclpy.init()
    node = Node('ultrasonic_check')
    data = {t: [] for t in TOPICS}
    frames = {}
    for t in TOPICS:
        def cb(msg, t=t):
            data[t].append((msg.range, msg.max_range))
            frames[t] = msg.header.frame_id
        node.create_subscription(Range, t, cb, qos_profile_sensor_data)
    print('listening... (needs pico_bridge with the Phase 7 patch and firmware 1.2.0)')
    last = time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.monotonic() - last < 1.0:
                continue
            dt, last = time.monotonic() - last, time.monotonic()
            row = []
            for t in TOPICS:
                d, data[t] = data[t], []
                name = t.split('/')[-1]
                if not d:
                    row.append(f'{name:5}: --- no data ---          ')
                    continue
                rs = [r for r, _ in d]
                none = sum(1 for r, mx in d if math.isclose(r, mx))
                row.append(f'{name:5}: {len(d) / dt:4.1f} Hz last {rs[-1]:5.2f} m '
                           f'[{min(rs):4.2f}-{max(rs):4.2f}] no-echo {none}/{len(d)}')
            print(' | '.join(row), flush=True)
    except KeyboardInterrupt:
        pass
    print('frames:', frames)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
