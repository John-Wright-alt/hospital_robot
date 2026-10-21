"""scan_geometry.py -- pure maths for scan_body_filter (NO ROS code, unit tested)."""

import math


def filter_ranges(ranges, angle_min, angle_increment, lidar_x, lidar_y, lidar_yaw, box):
    """Pure function (unit tested). box = (x_min, x_max, y_min, y_max) in the
    base frame. Returns (new_ranges, number_removed)."""
    x_min, x_max, y_min, y_max = box
    out = list(ranges)
    removed = 0
    c0, s0 = math.cos(lidar_yaw), math.sin(lidar_yaw)
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r <= 0.0:
            continue
        a = angle_min + i * angle_increment
        lx, ly = r * math.cos(a), r * math.sin(a)          # point in lidar frame
        bx = lidar_x + c0 * lx - s0 * ly                     # point in base frame
        by = lidar_y + s0 * lx + c0 * ly
        if x_min <= bx <= x_max and y_min <= by <= y_max:
            out[i] = math.inf
            removed += 1
    return out, removed
