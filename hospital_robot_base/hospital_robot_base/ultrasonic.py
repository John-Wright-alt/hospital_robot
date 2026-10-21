"""
ultrasonic.py  --  turns the Pico's 'u <n> <mm>' lines into sensor_msgs/Range.
(Phase 7, used by pico_bridge; firmware 1.2.0 sends the lines.)

    Pico:  u 1 523     ->   /ultrasonic/left   range 0.523 m   frame us_left_link
           u 2 -1      ->   /ultrasonic/front  range = max_range (nothing in range)
           u 3 -2      ->   (not published: sensor stuck - logged as a warning)

Why pico_bridge and not a separate node? Only one program can own /dev/pico,
and pico_bridge already does. This file keeps the new code separate so the
bridge only needs three short lines (see apply_bridge_patch.py).

The parsing/conversion functions have NO ROS code, so they are unit tested in
test/test_ultrasonic.py on any computer.

"No echo" is published as exactly max_range. That is what Nav2 expects for
"nothing seen": the costmap RangeSensorLayer clears the cone on a max reading
(clear_on_max_reading: true) and the collision monitor ignores it unless a
polygon reaches that far. A sensor that is unplugged ALSO reads "no echo" -
the hardware cannot tell the difference - so the guide checks each sensor by
hand before it is trusted.
"""

import math
from typing import Optional, Tuple

NO_ECHO = -1
STUCK = -2
NAMES = ('left', 'front', 'right')          # firmware order: US1, US2, US3


def parse_ultrasonic_line(line: str) -> Optional[Tuple[int, int]]:
    """'u 2 523' -> (2, 523). Anything else (or a corrupted line) -> None."""
    parts = line.strip().split()
    if len(parts) != 3 or parts[0] != 'u':
        return None
    try:
        n, mm = int(parts[1]), int(parts[2])
    except ValueError:
        return None
    if n not in (1, 2, 3) or mm < STUCK:
        return None
    return n, mm


def mm_to_range(mm: int, min_range: float, max_range: float) -> Optional[float]:
    """Firmware millimetres -> metres for sensor_msgs/Range.
    No echo -> max_range (clear). Stuck -> None (do not publish)."""
    if mm == STUCK:
        return None
    if mm == NO_ECHO:
        return max_range
    r = mm / 1000.0
    return min(max(r, min_range), max_range)


class UltrasonicPublisher:
    """Created by pico_bridge. Declares its own parameters on that node:

    ultrasonic_enabled  (bool)       publish at all
    ultrasonic_topics   (string[3])  left, front, right
    ultrasonic_frames   (string[3])  must exist in the URDF (ultrasonic.xacro)
    ultrasonic_fov      (double)     full cone angle in radians
    ultrasonic_min_range / ultrasonic_max_range (m)
    """

    def __init__(self, node):
        from sensor_msgs.msg import Range
        self._Range = Range
        self.node = node
        p = node.declare_parameter
        self.enabled = bool(p('ultrasonic_enabled', True).value)
        topics = list(p('ultrasonic_topics', ['ultrasonic/left', 'ultrasonic/front',
                                               'ultrasonic/right']).value)
        self.frames = list(p('ultrasonic_frames', ['us_left_link', 'us_front_link',
                                                   'us_right_link']).value)
        self.fov = float(p('ultrasonic_fov', 0.35).value)
        self.min_range = float(p('ultrasonic_min_range', 0.03).value)
        self.max_range = float(p('ultrasonic_max_range', 2.0).value)
        if len(topics) != 3 or len(self.frames) != 3:
            raise ValueError('ultrasonic_topics and ultrasonic_frames need 3 entries each')
        # RELIABLE publisher (depth 10): works with BOTH reliable subscribers
        # (RViz by default) and best-effort ones (Nav2 range layer, collision
        # monitor). A best-effort publisher was invisible to RViz (19 Oct 2026).
        self.pubs = [node.create_publisher(Range, t, 10) for t in topics]
        self.count = [0, 0, 0]
        self.stuck = [0, 0, 0]
        self.seen_any = False
        if self.enabled:
            node.get_logger().info(
                f'Ultrasonic: publishing {", ".join(topics)} (frames {", ".join(self.frames)}, '
                f'{self.min_range:.2f}-{self.max_range:.2f} m). Needs Pico firmware 1.2.0.')

    def handle_line(self, line: str) -> bool:
        """True if the line was an ultrasonic line (published or deliberately
        dropped); False if it is something else for the bridge to handle."""
        parsed = parse_ultrasonic_line(line)
        if parsed is None:
            return False
        if not self.enabled:
            return True
        n, mm = parsed
        i = n - 1
        if not self.seen_any:
            self.seen_any = True
            self.node.get_logger().info('Ultrasonic: first reading received from the Pico.')
        r = mm_to_range(mm, self.min_range, self.max_range)
        if r is None:
            self.stuck[i] += 1
            self.node.get_logger().warn(
                f'Ultrasonic {NAMES[i]} (US{n}): ECHO stuck HIGH - check its wiring '
                f'({self.stuck[i]} times so far). Not published.', throttle_duration_sec=10.0)
            return True
        msg = self._Range()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self.frames[i]
        msg.radiation_type = self._Range.ULTRASOUND
        msg.field_of_view = self.fov
        msg.min_range = self.min_range
        msg.max_range = self.max_range
        msg.range = float(r) if math.isfinite(r) else self.max_range
        self.pubs[i].publish(msg)
        self.count[i] += 1
        return True
