"""
face_logic.py  --  decides what the eyes do. NO ROS code (unit tested in
test/test_face_logic.py), same pattern as pico_protocol.py and gaze_core.py.

Inputs
  * the "base" expression requested by the robot software (later: the Phase 6
    interaction manager publishes it on /hri/expression). Default: idle.
  * the latest /hri/gaze/status message from lidar_gaze (or simulate_gaze).
  * the latest /hri/speech/status message from hri_speech (Phase 9 speech):
    what she is saying (shown as a caption) and whether the robot is moving
    (shown as a banner; while driving the eyes look where the robot turns).

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
               'waiting', 'confused', 'assistance', 'sleeping',
               'searching')     # Anna looking for the person's face while the column rises

UI_MODES = ('sleep', 'adjusting', 'menu', 'eyes')    # from hri_interaction on /hri/ui

# Expressions in which the eyes may follow sensor data. In every other
# expression the expression owns the eyes (e.g. never glance at a passer-by
# while asking for assistance).
GAZE_ALLOWED = frozenset(('idle', 'waiting', 'curious'))

TRACK_STATES = ('tracking', 'lost')

MOVING = ('forward', 'reverse', 'turning')      # /hri/speech/status "motion" values


@dataclass
class FaceParams:
    gaze_timeout_s: float = 1.0     # no gaze status for this long -> "no data", eyes go neutral
    max_bearing_deg: float = 90.0   # bearing that puts the pupils fully to one side
    mirror: bool = True             # robot-left -> viewer-right (see module docstring)
    curious_s: float = 1.5          # how long the "oh, what's that?" widening lasts
    curious_cooldown_s: float = 4.0 # at most one curious reaction per this many seconds
    speech_timeout_s: float = 2.0   # no /hri/speech/status this long -> no caption, no banner
    motion_gaze: bool = True        # while driving, look where the robot is turning
    ui_timeout_s: float = 3.0       # no /hri/ui this long -> plain eyes (hri_interaction gone)


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
        self.speech = None
        self.speech_t = None
        self.ui = None
        self.ui_t = None

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

    def on_ui(self, ui: dict, t: float):
        """ui = the decoded JSON from /hri/ui (hri_interaction): which screen to show."""
        if isinstance(ui, dict) and ui.get('mode') in UI_MODES:
            opts = ui.get('options') if isinstance(ui.get('options'), list) else []
            self.ui = {'mode': ui['mode'],
                       'options': [{'id': str(o.get('id', ''))[:24],
                                    'label': str(o.get('label', ''))[:40]}
                                   for o in opts[:6] if isinstance(o, dict)],
                       'destination': str(ui.get('destination', ''))[:40]}
            self.ui_t = t

    def on_speech_status(self, status: dict, t: float):
        """status = the decoded JSON from /hri/speech/status (hri_speech)."""
        if isinstance(status, dict):
            self.speech = status
            self.speech_t = t

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

        # ---- speech and motion (from hri_speech) ----
        sp_fresh = self.speech_t is not None and (t - self.speech_t) <= p.speech_timeout_s
        sp = self.speech if sp_fresh else {}
        speaking = bool(sp.get('speaking')) and bool(sp.get('text'))
        motion = str(sp.get('motion', 'unknown')) if sp_fresh else 'unknown'
        out['speaking'] = speaking
        out['caption'] = str(sp.get('text', ''))[:220] if speaking else ''
        out['caption_priority'] = str(sp.get('priority', '')) if speaking else ''
        out['motion'] = motion
        out['audio_ok'] = bool(sp.get('audio_ok', True)) if sp_fresh else True
        out['speech_simulated'] = bool(sp.get('simulated', False))

        moving = motion in MOVING
        if moving and allowed and p.motion_gaze:
            # lidar_gaze assumes a parked robot, so while driving the eyes show
            # where the robot is going instead: + turn = robot's left.
            try:
                turn = clamp(float(sp.get('turn', 0.0)), -1.0, 1.0)
            except (TypeError, ValueError):
                turn = 0.0
            sign = 1.0 if p.mirror else -1.0
            out['gaze'] = {'x': sign * 0.8 * turn, 'y': 0.0, 'source': 'motion'}
            for k in ('bearing_deg', 'range_m', 'track_id'):
                out.pop(k, None)

        if allowed and t < self.curious_until and not moving:
            out['expression'] = 'curious'
        if speaking and allowed:
            out['expression'] = 'speaking'

        # ---- which screen (from hri_interaction) ----
        ui_fresh = self.ui_t is not None and (t - self.ui_t) <= p.ui_timeout_s
        out['ui'] = self.ui if ui_fresh else {'mode': 'eyes', 'options': [], 'destination': ''}
        return out
