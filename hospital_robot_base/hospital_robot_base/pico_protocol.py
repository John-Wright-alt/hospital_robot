"""
pico_protocol.py  --  the maths and text-parsing for the Pico bridge.

There is NO ROS code in this file on purpose. That means:
  * you can unit-test it on any computer (see test/test_pico_protocol.py)
  * if the robot drives strangely, you can test the maths on its own

Protocol (from pico_motor_firmware 1.0.x):
  PC -> Pico :  v <left_rad_s> <right_rad_s>     wheel rad/s, + = forward
                a 1 / a 0                        arm / disarm
                c                                clear latched faults
                i                                print settings
  Pico -> PC :  e <ticksL> <ticksR> <velL> <velR> <micros>     50 Hz
                   ticks = MOTOR encoder counts, + = forward
                           (4096 x GEAR_RATIO per wheel turn = 15360)
                   vel   = WHEEL rad/s, + = forward
                s <flags_hex> <arm_state> <vbat_V> <vmot_V>   on change + every 500 ms
                   arm_state: 0 DISARMED, 1 ARMING, 2 ARMED, 3 DISARMING
                # <text>                                     human message
"""

import math
from dataclasses import dataclass
from typing import Optional


# --------------------------------------------------------------------------
# 1. Parsing lines that come FROM the Pico
# --------------------------------------------------------------------------

@dataclass
class EncoderMsg:
    ticks_l: int
    ticks_r: int
    vel_l: Optional[float]      # wheel rad/s as measured by the Pico (None if not sent)
    vel_r: Optional[float]
    micros: Optional[int]       # Pico clock in microseconds (None if not sent)


@dataclass
class StatusMsg:
    raw: str                    # the whole line, for logging
    flags: int                  # bit field, see FLAG_NAMES
    arm_state: int              # 0..3, see ARM_NAMES
    vbat: float
    vmot: float


def parse_line(line: str):
    """Turn one text line from the Pico into a Python object.

    Returns EncoderMsg, StatusMsg, ('text', str) or None (ignore the line).
    Never raises: a corrupted line just returns None.
    """
    line = line.strip()
    if not line:
        return None

    if line.startswith('#'):
        return ('text', line[1:].strip())

    parts = line.split()
    kind = parts[0]

    try:
        if kind == 'e':
            # Full format: e ticksL ticksR velL velR micros   (6 parts)
            # Roadmap format: e ticksL ticksR micros          (4 parts)
            if len(parts) >= 6:
                return EncoderMsg(int(float(parts[1])), int(float(parts[2])),
                                  float(parts[3]), float(parts[4]),
                                  int(float(parts[5])))
            if len(parts) == 5:
                return EncoderMsg(int(float(parts[1])), int(float(parts[2])),
                                  float(parts[3]), float(parts[4]), None)
            if len(parts) == 4:
                return EncoderMsg(int(float(parts[1])), int(float(parts[2])),
                                  None, None, int(float(parts[3])))
            if len(parts) == 3:
                return EncoderMsg(int(float(parts[1])), int(float(parts[2])),
                                  None, None, None)
            return None

        if kind == 's' and len(parts) >= 5:
            return StatusMsg(line, int(parts[1], 16), int(parts[2]),
                             float(parts[3]), float(parts[4]))

    except (ValueError, IndexError):
        return None

    return None


# Copied from pico_motor_firmware.ino (enum near the top). Keep them in step!
FLAG_NAMES = {
    0x0001: 'ESTOP', 0x0002: 'NO_VMOT', 0x0004: 'CMD_TIMEOUT', 0x0008: 'BATT_WARN',
    0x0010: 'NO_BATT', 0x0020: 'LOOP_SLOW',
    0x0100: 'BATT_CUTOFF*', 0x0200: 'STALL_L*', 0x0400: 'STALL_R*',
    0x0800: 'ENCODER_L*', 0x1000: 'ENCODER_R*', 0x2000: 'RELAY*',
}
LATCHED_MASK = 0xFF00           # upper byte = latched faults, need 'c' to clear
FLAG_ESTOP = 0x0001
ARM_NAMES = {0: 'DISARMED', 1: 'ARMING', 2: 'ARMED', 3: 'DISARMING'}


def decode_flags(flags: int) -> str:
    """0x0006 -> 'NO_VMOT CMD_TIMEOUT'.  (* = latched fault)"""
    names = [n for bit, n in FLAG_NAMES.items() if flags & bit]
    return ' '.join(names) if names else 'none'


def status_summary(status: StatusMsg) -> str:
    """One word for /robot_state: FAULT, ESTOP, or the arm state name.
    Priority: a latched fault beats the E-stop, which beats everything else."""
    if status.flags & LATCHED_MASK:
        return 'FAULT'
    if status.flags & FLAG_ESTOP:
        return 'ESTOP'
    return ARM_NAMES.get(status.arm_state, 'UNKNOWN')


def status_text(status: StatusMsg) -> str:
    """Human-readable status line, same style as pico_console.py."""
    return (f'{ARM_NAMES.get(status.arm_state, status.arm_state)}  '
            f'flags={status.flags:04X} ({decode_flags(status.flags)})  '
            f'vbat={status.vbat:.2f} V  vmot={status.vmot:.2f} V')


# --------------------------------------------------------------------------
# 2. Kinematics: robot velocity  <->  wheel velocities
# --------------------------------------------------------------------------

def body_to_wheels(v_x: float, w_z: float, wheel_radius: float,
                   wheel_separation: float, max_wheel_speed: float):
    """cmd_vel (m/s, rad/s) -> (left, right) wheel speed in rad/s.

    If either wheel would exceed max_wheel_speed, BOTH are scaled by the same
    factor. That keeps the ratio between them, so the robot still drives the
    same curve, just slower (the same trick your firmware uses).
    """
    left = (v_x - w_z * wheel_separation / 2.0) / wheel_radius
    right = (v_x + w_z * wheel_separation / 2.0) / wheel_radius

    biggest = max(abs(left), abs(right))
    if max_wheel_speed > 0 and biggest > max_wheel_speed:
        scale = max_wheel_speed / biggest
        left *= scale
        right *= scale
    return left, right


def wheels_to_body(w_left: float, w_right: float, wheel_radius: float,
                   wheel_separation: float):
    """(left, right) wheel rad/s -> (v_x m/s, w_z rad/s)."""
    v_x = wheel_radius * (w_right + w_left) / 2.0
    w_z = wheel_radius * (w_right - w_left) / wheel_separation
    return v_x, w_z


# --------------------------------------------------------------------------
# 3. Odometry: add up tick changes into a position (x, y, heading)
# --------------------------------------------------------------------------

class DiffDriveOdometry:
    """Integrates encoder ticks into a 2-D pose.

    ticks_per_wheel_rev = encoder counts for ONE full turn of the WHEEL.
    For your robot: 4096 counts per motor turn x 3.75 gear ratio = 15360.
    """

    def __init__(self, wheel_radius, wheel_separation, ticks_per_wheel_rev):
        self.r = wheel_radius
        self.L = wheel_separation
        self.tpr = ticks_per_wheel_rev
        self.reset()

    def reset(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_l = None
        self.last_r = None
        self.last_micros = None

    def ticks_to_rad(self, ticks):
        return ticks * 2.0 * math.pi / self.tpr

    def update(self, ticks_l: int, ticks_r: int, max_jump_ticks: int):
        """Feed in the latest tick counts.

        Returns (d_left_rad, d_right_rad) wheel rotation since the last call,
        or None if this is the first reading or the jump was impossible
        (which means the Pico restarted and its counters went back to 0).
        """
        if self.last_l is None:
            self.last_l, self.last_r = ticks_l, ticks_r
            return None

        dl_ticks = ticks_l - self.last_l
        dr_ticks = ticks_r - self.last_r
        self.last_l, self.last_r = ticks_l, ticks_r

        if abs(dl_ticks) > max_jump_ticks or abs(dr_ticks) > max_jump_ticks:
            return None     # Pico rebooted or a corrupted line: skip this step

        d_left = self.ticks_to_rad(dl_ticks)       # wheel radians
        d_right = self.ticks_to_rad(dr_ticks)

        ds_left = d_left * self.r                  # metres each wheel rolled
        ds_right = d_right * self.r
        ds = (ds_right + ds_left) / 2.0            # metres the centre moved
        dtheta = (ds_right - ds_left) / self.L     # radians the robot turned

        # Use the heading halfway through the step: more accurate on curves
        mid = self.theta + dtheta / 2.0
        self.x += ds * math.cos(mid)
        self.y += ds * math.sin(mid)
        self.theta = math.atan2(math.sin(self.theta + dtheta),
                                math.cos(self.theta + dtheta))
        return d_left, d_right


def micros_delta(now: int, before: int) -> int:
    """Difference between two Pico micros() values, safe across the
    32-bit wrap-around that happens every ~71.6 minutes."""
    return (now - before) & 0xFFFFFFFF


def yaw_to_quaternion(yaw: float):
    """Heading angle -> quaternion (x, y, z, w) for a robot on flat ground."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


# --------------------------------------------------------------------------
# 4. Self-check: are ticks_per_wheel_rev and the Pico's speeds consistent?
# --------------------------------------------------------------------------

class TickScaleChecker:
    """Compares 'wheel angle from ticks' with 'wheel angle from the Pico's
    own speed readings'. If ticks_per_wheel_rev is wrong, the two disagree
    by a constant factor, and this class tells you the corrected value."""

    def __init__(self, min_angle_rad=20.0):
        self.min_angle = min_angle_rad
        self.clear()

    def clear(self):
        self.from_ticks = 0.0
        self.from_vel = 0.0

    def add(self, d_left, d_right, vel_l, vel_r, dt):
        if vel_l is None or vel_r is None or dt <= 0:
            return
        # Only count while actually moving, to ignore noise when stopped
        if abs(vel_l) < 0.5 and abs(vel_r) < 0.5:
            return
        self.from_ticks += abs(d_left) + abs(d_right)
        self.from_vel += (abs(vel_l) + abs(vel_r)) * dt

    def result(self):
        """Returns ratio (ticks-angle / velocity-angle) once enough data is in,
        else None. 1.00 means consistent."""
        if self.from_vel < self.min_angle:
            return None
        ratio = self.from_ticks / self.from_vel
        self.clear()
        return ratio
