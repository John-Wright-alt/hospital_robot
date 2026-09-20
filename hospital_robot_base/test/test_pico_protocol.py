"""Unit tests for pico_protocol.py -- run with:  colcon test --packages-select hospital_robot_base
or directly:  python3 -m pytest test/ -v   (from inside the package folder)"""

import math
import pytest

from hospital_robot_base.pico_protocol import (
    EncoderMsg, StatusMsg, parse_line, status_summary, status_text, decode_flags, body_to_wheels,
    wheels_to_body, DiffDriveOdometry, micros_delta, TickScaleChecker)

R, L, TPR = 0.072, 0.26, 15360.0


def test_parse_full_encoder_line():
    m = parse_line('e 1200 -340 2.01 -1.98 123456789\r\n')
    assert isinstance(m, EncoderMsg)
    assert (m.ticks_l, m.ticks_r, m.vel_l, m.vel_r, m.micros) == \
        (1200, -340, 2.01, -1.98, 123456789)


def test_parse_short_encoder_line():
    m = parse_line('e 10 20 5000')
    assert m.ticks_l == 10 and m.micros == 5000 and m.vel_l is None


def test_garbage_is_ignored():
    for junk in ['', 'e', 'e abc def', 'xyz 1 2', 'e 1', '\x00\xff']:
        assert parse_line(junk) is None


def test_status_line_from_real_firmware():
    # exactly the format printed by: say("s %04X %u %.2f %.2f", ...)
    s = parse_line('s 0006 0 12.21 0.00')
    assert isinstance(s, StatusMsg)
    assert s.flags == 0x0006 and s.arm_state == 0
    assert s.vbat == pytest.approx(12.21) and s.vmot == pytest.approx(0.0)
    assert decode_flags(s.flags) == 'NO_VMOT CMD_TIMEOUT'
    assert status_summary(s) == 'DISARMED'
    assert status_summary(parse_line('s 0004 2 12.2 12.1')) == 'ARMED'
    assert status_summary(parse_line('s 0003 3 12.2 9.0')) == 'ESTOP'
    assert status_summary(parse_line('s 0201 0 12.2 0.0')) == 'FAULT'   # latched beats estop
    assert status_summary(parse_line('s 0000 1 12.2 0.0')) == 'ARMING'
    assert 'ARMED' in status_text(parse_line('s 0004 2 12.2 12.1'))
    assert parse_line('s 00ZZ 0 1 2') is None                               # corrupted
    assert parse_line('# hello there') == ('text', 'hello there')


def test_straight_line_kinematics():
    left, right = body_to_wheels(0.3, 0.0, R, L, 100)
    assert left == pytest.approx(0.3 / R) and right == pytest.approx(0.3 / R)
    assert wheels_to_body(left, right, R, L) == pytest.approx((0.3, 0.0))


def test_turning_kinematics_round_trip():
    left, right = body_to_wheels(0.2, 0.8, R, L, 100)
    assert right > left                         # turning left: right wheel faster
    assert wheels_to_body(left, right, R, L) == pytest.approx((0.2, 0.8))


def test_speed_clamp_keeps_curvature():
    left, right = body_to_wheels(1.0, 2.0, R, L, 9.0)
    assert max(abs(left), abs(right)) == pytest.approx(9.0)
    v, w = wheels_to_body(left, right, R, L)
    assert w / v == pytest.approx(2.0)          # same curve, just slower


def test_odometry_one_metre_straight():
    odo = DiffDriveOdometry(R, L, TPR)
    odo.update(0, 0, 10**9)
    ticks_per_metre = TPR / (2 * math.pi * R)
    total = 0
    for _ in range(100):                        # 100 small steps
        total += ticks_per_metre / 100
        odo.update(round(total), round(total), 10**9)
    assert odo.x == pytest.approx(1.0, abs=1e-3)
    assert odo.y == pytest.approx(0.0, abs=1e-6)


def test_odometry_spin_on_the_spot_full_turn():
    odo = DiffDriveOdometry(R, L, TPR)
    odo.update(0, 0, 10**9)
    # one full robot turn: each wheel travels pi*L metres (in opposite directions)
    wheel_ticks = (math.pi * L) / (2 * math.pi * R) * TPR
    steps = 400
    for i in range(1, steps + 1):
        t = round(wheel_ticks * i / steps)
        odo.update(-t, t, 10**9)
        if i == steps // 4:
            assert odo.theta == pytest.approx(math.pi / 2, abs=0.01)
    assert abs(math.sin(odo.theta)) < 0.01      # back to facing forward
    assert math.hypot(odo.x, odo.y) < 1e-3      # didn't move its centre


def test_reboot_jump_is_rejected():
    odo = DiffDriveOdometry(R, L, TPR)
    odo.update(500000, 500000, 30000)
    assert odo.update(0, 0, 30000) is None      # counters reset -> ignored
    assert odo.x == 0.0
    assert odo.update(100, 100, 30000) is not None


def test_micros_wraparound():
    assert micros_delta(1000, 0xFFFFFFFF - 999) == 2000
    assert micros_delta(25000, 5000) == 20000


def test_tick_scale_checker_detects_wrong_scale():
    chk = TickScaleChecker(min_angle_rad=5.0)
    # The Pico says 2 rad/s for 5 s, but ticks claim twice as much rotation
    for _ in range(250):
        chk.add(0.08, 0.08, 2.0, 2.0, 0.02)
    assert chk.result() == pytest.approx(2.0)
