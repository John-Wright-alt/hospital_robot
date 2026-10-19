"""Unit tests for ultrasonic.py (no ROS needed for these functions).

    cd ~/robot_ws/src/hospital_robot_base && python3 -m pytest test/test_ultrasonic.py -v
"""
from hospital_robot_base.ultrasonic import mm_to_range, parse_ultrasonic_line


def test_parse_good_lines():
    assert parse_ultrasonic_line('u 1 523') == (1, 523)
    assert parse_ultrasonic_line('u 3 -1\r') == (3, -1)
    assert parse_ultrasonic_line('  u 2 -2 ') == (2, -2)


def test_parse_rejects_other_and_corrupt_lines():
    for line in ('e 1 2 0.1 0.2 3', 's 0000 0 12.1 0.0', '# hello', 'u', 'u 4 100',
                 'u 0 100', 'u 1 abc', 'u 1 100 7', 'u 1 -5', '', 'l 0 1 0.0 0.0 1 0'):
        assert parse_ultrasonic_line(line) is None, line


def test_mm_to_range():
    assert mm_to_range(523, 0.03, 2.0) == 0.523
    assert mm_to_range(10, 0.03, 2.0) == 0.03          # clamped to min
    assert mm_to_range(3500, 0.03, 2.0) == 2.0         # beyond Nav2's use: max
    assert mm_to_range(-1, 0.03, 2.0) == 2.0           # no echo = clear
    assert mm_to_range(-2, 0.03, 2.0) is None          # stuck = do not publish
