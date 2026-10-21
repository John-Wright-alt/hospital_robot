"""
mission_logic.py  --  the "drive to a named place" task layer. NO ROS code
(unit tested in test/test_mission_logic.py), same pattern as your
pico_protocol.py and interaction_logic.py.

    /mission/command  {"cmd": "go", "target": "pharmacy", "mode": "guide", "id": 7}
                      {"cmd": "cancel", "id": 7}
    /mission/status   {"id": 7, "state": "ACTIVE", "target": "pharmacy", "mode": "guide",
                       "reason": "", "remaining_m": 4.2}

    state:  IDLE -> PARKING -> STARTING -> ACTIVE -> SUCCEEDED | FAILED | CANCELED
                    (only if the head column is not yet PARKED)

mode "guide" = a person is following: guide_speed (slow).
mode "alone" = returning to base without a person: alone_speed.

SAFETY RULES (checked here, on top of the Pico's own limits):
  * never sends a goal unless /robot_state is ARMED  (this code never arms)
  * never sends a goal unless the head column reports PARKED
    (Phase 8 rule: never drive with the screen raised)
  * cancels the goal at once if the robot stops being ARMED, or the column
    leaves PARKED, or the goal takes longer than max_goal_s
The logic only RETURNS actions; mission_node.py carries them out:
    ('goal', {'x','y','yaw','speed'})   send NavigateToPose
    ('cancel', None)                    cancel the active goal
    ('head', 'park')                    -> /head_column/command
    ('speed', m_s)                      -> /speed_limit
    ('status', dict)                    -> /mission/status
    ('log', text)
"""

import json
import math
import re
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

# head_column_node.py states (Pi copy, 18 Oct 2026). STARTING = node just started.
HEAD_STATES = ('STARTING', 'WAIT_ARM', 'HOMING', 'SEARCHING', 'TRACKING', 'WAITING',
               'PARKING', 'PARKED', 'STOPPED', 'FAULT')
_HEAD_RE = re.compile(r'\b(' + '|'.join(HEAD_STATES) + r')\b')

IDLE, PARKING, STARTING, ACTIVE, CANCELING = 'IDLE', 'PARKING', 'STARTING', 'ACTIVE', 'CANCELING'
SUCCEEDED, FAILED, CANCELED = 'SUCCEEDED', 'FAILED', 'CANCELED'
BUSY = (PARKING, STARTING, ACTIVE, CANCELING)

# action_msgs/GoalStatus codes
GOAL_SUCCEEDED, GOAL_CANCELED, GOAL_ABORTED = 4, 5, 6


def parse_head_state(text) -> Optional[str]:
    """Same rules as hospital_robot_hri.interaction_logic.parse_head_state."""
    if not isinstance(text, str):
        return None
    t = text.strip()
    if t.startswith('{'):
        try:
            t = str(json.loads(t).get('state', ''))
        except (ValueError, TypeError, AttributeError):
            pass
    m = _HEAD_RE.search(t.upper())
    return m.group(1) if m else None


@dataclass
class MissionParams:
    require_armed: bool = True
    require_parked: bool = True     # head column must report PARKED before driving
    head_optional: bool = False     # True = allow driving if NO head_column state was ever
                                    # received (testing without the column only!)
    park_wait_s: float = 45.0       # wait this long for PARKED after asking to park
    start_timeout_s: float = 10.0   # Nav2 must accept the goal within this time
    max_goal_s: float = 300.0       # give up on one goal after this long
    cancel_timeout_s: float = 5.0
    guide_speed: float = 0.25       # m/s, a person is following
    alone_speed: float = 0.35       # m/s, returning on its own
    state_timeout_s: float = 3.0    # /robot_state older than this = unknown = not armed


class MissionLogic:

    def __init__(self, params: MissionParams = None):
        self.p = params or MissionParams()
        self.state = IDLE
        self.entered = 0.0
        self.robot_state = None
        self.robot_state_t = None
        self.head = None
        self.goal = None            # dict: id, target, mode, pose
        self.remaining = None
        self.last_status = {'id': None, 'state': IDLE, 'target': '', 'mode': '', 'reason': ''}

    # ------------------------------------------------------------ helpers
    def armed(self, t) -> bool:
        if not self.p.require_armed:
            return True
        fresh = self.robot_state_t is not None and t - self.robot_state_t <= self.p.state_timeout_s
        return fresh and self.robot_state == 'ARMED'

    def head_ok(self) -> Tuple[bool, str]:
        if not self.p.require_parked:
            return True, ''
        if self.head is None:
            return (True, '') if self.p.head_optional else (False, 'no head_column state')
        return (self.head == 'PARKED'), f'head column is {self.head}'

    def _status(self, state, reason=''):
        g = self.goal or {}
        self.last_status = {'id': g.get('id'), 'state': state, 'target': g.get('target', ''),
                            'mode': g.get('mode', ''), 'reason': reason,
                            'remaining_m': None if self.remaining is None
                            else round(self.remaining, 2)}
        return ('status', dict(self.last_status))

    def _enter(self, t, state, reason=''):
        self.state, self.entered = state, t
        return [('log', f'-> {state}' + (f' ({reason})' if reason else '')),
                self._status(state, reason)]

    def _finish(self, t, state, reason=''):
        acts = self._enter(t, state, reason)
        self.state = IDLE                  # ready for the next command
        return acts

    def _send(self, t):
        g = self.goal
        speed = self.p.guide_speed if g['mode'] == 'guide' else self.p.alone_speed
        x, y, yaw = g['pose']
        return [('speed', speed), ('goal', {'x': x, 'y': y, 'yaw': yaw, 'speed': speed})] + \
            self._enter(t, STARTING)

    # ------------------------------------------------------------ inputs
    def on_robot_state(self, t, text):
        self.robot_state = str(text or '').strip().upper()
        self.robot_state_t = t

    def on_head_state(self, t, text):
        s = parse_head_state(text)
        if s is not None:
            self.head = s

    def on_command(self, t, cmd: dict, waypoints: Dict[str, Tuple[float, float, float]]
                   ) -> List[tuple]:
        if not isinstance(cmd, dict):
            return [('log', 'ignored: command is not a JSON object')]
        kind = str(cmd.get('cmd', '')).lower()
        if kind == 'cancel':
            if self.state in BUSY and (cmd.get('id') in (None, (self.goal or {}).get('id'))):
                return self._cancel(t, 'cancelled by request')
            return [('log', f'cancel ignored in {self.state}')]
        if kind != 'go':
            return [('log', f'unknown command "{kind}"')]

        target = str(cmd.get('target', '')).strip().lower()
        mode = 'alone' if str(cmd.get('mode', 'guide')).lower() == 'alone' else 'guide'
        acts = []
        if self.state in BUSY:                        # a new goal replaces the old one
            acts += [('log', f'new goal "{target}" replaces "{self.goal["target"]}"'),
                     ('cancel', None)]
        self.goal = {'id': cmd.get('id'), 'target': target, 'mode': mode, 'pose': None}
        self.remaining = None
        if target not in waypoints:
            return acts + self._finish(t, FAILED, f'unknown_target: "{target}" is not in '
                                                  f'waypoints ({", ".join(sorted(waypoints))})')
        self.goal['pose'] = tuple(waypoints[target])
        if not self.armed(t):
            return acts + self._finish(t, FAILED, f'not_armed: /robot_state is '
                                                  f'{self.robot_state or "not received"}')
        ok, why = self.head_ok()
        if ok:
            return acts + self._send(t)
        if self.head in ('STARTING', 'WAIT_ARM', 'FAULT') or self.head is None:
            return acts + self._finish(t, FAILED, f'head_not_parked: {why}')
        return acts + [('head', 'park')] + self._enter(t, PARKING, why)

    def on_goal_response(self, t, accepted: bool) -> List[tuple]:
        if self.state != STARTING:
            return []
        if accepted:
            return self._enter(t, ACTIVE)
        return self._finish(t, FAILED, 'rejected: Nav2 refused the goal (is it active?)')

    def on_feedback(self, t, remaining_m: float):
        if self.state == ACTIVE and remaining_m is not None and math.isfinite(remaining_m):
            self.remaining = remaining_m

    def on_result(self, t, status_code: int) -> List[tuple]:
        if self.state not in (STARTING, ACTIVE, CANCELING):
            return []
        if self.state == CANCELING:
            return self._finish(t, self._cancel_outcome, self._cancel_reason)
        if status_code == GOAL_SUCCEEDED:
            self.remaining = 0.0
            return self._finish(t, SUCCEEDED)
        if status_code == GOAL_CANCELED:
            return self._finish(t, CANCELED, 'cancelled outside the mission node')
        return self._finish(t, FAILED, f'nav_failed: Nav2 status {status_code}')

    def _cancel(self, t, reason, outcome=CANCELED):
        if self.state == PARKING:                    # no Nav2 goal yet
            return self._finish(t, outcome, reason)
        self._cancel_reason, self._cancel_outcome = reason, outcome
        return [('cancel', None)] + self._enter(t, CANCELING, reason)

    # ------------------------------------------------------------ time
    def tick(self, t) -> List[tuple]:
        st, age = self.state, t - self.entered
        if st == PARKING:
            if not self.armed(t):
                return self._finish(t, FAILED, 'not_armed while waiting for the column')
            ok, why = self.head_ok()
            if ok:
                return self._send(t)
            if age > self.p.park_wait_s:
                return self._finish(t, FAILED, f'head_not_parked after {age:.0f} s: {why}')
        elif st in (STARTING, ACTIVE):
            if not self.armed(t):
                return self._cancel(t, f'motors_off: /robot_state is '
                                       f'{self.robot_state or "silent"}', FAILED)
            ok, why = self.head_ok()
            if not ok:
                return self._cancel(t, f'head_not_parked: {why}', FAILED)
            if st == STARTING and age > self.p.start_timeout_s:
                return self._cancel(t, 'start_timeout: Nav2 did not answer', FAILED)
            if st == ACTIVE and age > self.p.max_goal_s:
                return self._cancel(t, f'timeout after {age:.0f} s', FAILED)
        elif st == CANCELING and age > self.p.cancel_timeout_s:
            return self._finish(t, self._cancel_outcome,
                                self._cancel_reason + ' (Nav2 did not confirm the cancel)')
        return []
