"""
face_logic.py  --  decides what the eyes do. NO ROS code (unit tested in
test/test_face_logic.py), same pattern as pico_protocol.py and gaze_core.py.

Inputs
  * the "base" expression requested by the robot software (later: the Phase 6
    interaction manager publishes it on /hri/expression). Default: idle.
  * the latest /hri/gaze/status message from lidar_gaze (or simulate_gaze).

Output: one small dict that the face web page draws, e.g.
  {"expression": "curious", "gaze": {"x": 0.62, "y": 0.0, "source": "lidar"},
   "gaze_data": "tracking", "simulated": false, ...}

Gaze x/y are in SCREEN terms, -1..+1, as seen by the person LOOKING AT the
screen: x = +1 is the viewer's right, y = +1 is down.

The left/right trap: the screen faces the patient, so the ROBOT's left is the
VIEWER's right. LiDAR bearings are + towards the robot's left, so with
mirror = True a + bearing becomes a + (viewer's right) screen x.
"""

import math
from collections import deque
from dataclasses import dataclass

EXPRESSIONS = ('idle', 'curious', 'listening', 'speaking', 'thinking', 'happy',
               'waiting', 'confused', 'assistance', 'sleeping')

# Expressions in which the eyes may follow sensor data. In every other
# expression the expression owns the eyes (e.g. never glance at a passer-by
# while asking for assistance).
GAZE_ALLOWED = frozenset(('idle', 'waiting', 'curious'))

TRACK_STATES = ('tracking', 'lost')


@dataclass
class FaceParams:
    gaze_timeout_s: float = 1.0     # no gaze status for this long -> "no data", eyes go neutral
    max_bearing_deg: float = 90.0   # bearing that puts the pupils fully to one side
    mirror: bool = True             # robot-left -> viewer-right (see module docstring)
    curious_s: float = 1.5          # how long the "oh, what's that?" widening lasts
    curious_cooldown_s: float = 4.0 # at most one curious reaction per this many seconds


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class FaceLogic:

    def __init__(self, params: FaceParams = None, default_expression='idle'):
        self.p = params or FaceParams()
        self.base = default_expression if default_expression in EXPRESSIONS else 'idle'
        self.status = None
        self.status_t = None
        self.curious_until = -math.inf
        self.last_curious = -math.inf
        self.seen_ids = deque(maxlen=50)     # track IDs that already got a reaction

    # ------------------------------------------------------------ inputs
    def set_expression(self, name: str) -> bool:
        name = (name or '').strip().lower()
        if name not in EXPRESSIONS:
            return False
        self.base = name
        if name not in GAZE_ALLOWED:
            self.curious_until = -math.inf   # a curious glance must not outlive e.g. 'assistance'
        return True

    def on_gaze_status(self, status: dict, t: float):
        """status = the decoded JSON from /hri/gaze/status."""
        if not isinstance(status, dict):
            return
        self.status = status
        self.status_t = t
        if status.get('state') == 'learning':
            self.seen_ids.clear()            # the gaze node restarted: IDs start again at 1

    # ------------------------------------------------------------ output
    def update(self, t: float) -> dict:
        p = self.p
        fresh = self.status_t is not None and (t - self.status_t) <= p.gaze_timeout_s
        s = self.status if fresh else {}
        gaze_data = s.get('state', 'no_data') if fresh else 'no_data'

        out = {
            'expression': self.base,
            'base_expression': self.base,
            'gaze': {'x': 0.0, 'y': 0.0, 'source': 'none'},
            'gaze_data': gaze_data,
            'simulated': bool(s.get('simulated', False)),
            'mirror': p.mirror,
        }

        allowed = self.base in GAZE_ALLOWED
        if allowed and gaze_data in TRACK_STATES and 'bearing_deg' in s:
            try:
                bearing = float(s['bearing_deg'])
            except (TypeError, ValueError):
                bearing = None
            if bearing is not None and math.isfinite(bearing):
                sign = 1.0 if p.mirror else -1.0
                out['gaze'] = {'x': clamp(sign * bearing / p.max_bearing_deg, -1.0, 1.0),
                               'y': 0.0, 'source': str(s.get('source', 'lidar'))}
                out['bearing_deg'] = bearing
                out['range_m'] = s.get('range_m')
                out['track_id'] = s.get('track_id')

                tid = s.get('track_id')
                if gaze_data == 'tracking' and tid is not None and tid not in self.seen_ids:
                    self.seen_ids.append(tid)
                    if t - self.last_curious >= p.curious_cooldown_s:
                        self.curious_until = t + p.curious_s
                        self.last_curious = t

        if allowed and t < self.curious_until:
            out['expression'] = 'curious'
        return out
