import math

from hospital_robot_nav.scan_geometry import filter_ranges

BOX = (-0.23, 0.23, -0.26, 0.26)


def test_points_inside_robot_removed_outside_kept():
    # lidar at the origin, yaw 0: beam 0 points +x, beam 1 points +y
    ranges = [0.10, 1.00, float('inf'), 0.0]
    out, removed = filter_ranges(ranges, 0.0, math.pi / 2, 0.0, 0.0, 0.0, BOX)
    assert out[0] == math.inf and out[1] == 1.0 and removed == 1
    assert math.isinf(out[2]) and out[3] == 0.0           # invalid beams untouched


def test_lidar_offset_and_yaw():
    # URDF-like: lidar 0.2 m ahead, yawed -90 deg. Its +x beam points to base -y.
    # 0.15 m along it -> base (0.2, -0.15): inside. 1 m -> (0.2, -1.0): outside.
    out, removed = filter_ranges([0.15, 1.0], 0.0, 0.0, 0.2, 0.0, -math.pi / 2, BOX)
    assert out == [math.inf, 1.0] and removed == 1


def test_original_list_not_modified():
    r = [0.1]
    filter_ranges(r, 0.0, 0.1, 0.0, 0.0, 0.0, BOX)
    assert r == [0.1]
