"""
nav_flow.py  --  Phase 7: Anna's flow WITH navigation. NO ROS code (unit tested
in test/test_nav_flow.py).

NavInteractionLogic is your Phase 9 InteractionLogic (interaction_logic.py,
NOT changed) plus the trip. While nav_available is false it behaves exactly
like InteractionLogic, so every Phase 9 test and procedure stays valid.

With nav_available: true, the GUIDING placeholder is replaced by:

  CHOSEN ─parked─► GUIDING ──SUCCEEDED──► ARRIVED ──"Wait for me"──► WAITING
                   (person follows,        "Wait / Go back"            "Take me back / Go back
                    guide speed)               │  timeout                without me"   │ timeout
                     │ Stop                    ▼  or "Go back"            │             │
                     ▼                    RETURNING (alone) ◄─────────────┴─"Go back"───┘
                   PAUSED                      │ SUCCEEDED                │"Take me back"
                   "Continue / Take me back /  ▼                          ▼
                    Go back / Stay here"     SLEEPING (at home)    RETURNING (with person)
                                                                          │ SUCCEEDED
                   any FAILED ──► NAV_FAILED "Try again / Go back /       ▼
                                  Stay here"                  "We're back" ► SLEEPING

The trip itself is done by hospital_robot_nav's mission node:
    ('mission', {"cmd": "go", "target": "pharmacy", "mode": "guide", "id": 7})
    ('mission', {"cmd": "cancel", "id": 7})
and its answers arrive through on_mission_status() (/mission/status).

SAFETY: this file never arms anything and never drives by itself. The mission
node refuses to drive unless the robot is ARMED and the column PARKED, Nav2's
collision monitor and the Pico's limits still apply, and the E-stop is always
the real stop. The on-screen "Stop" button is a convenience, NOT a safety stop.
Automatic driving without a fresh tap happens in exactly two cases, both
returning to 'home' alone: nobody answers at the destination (arrive_timeout_s)
or the person never comes back (wait_timeout_s). Set them to 0 to disable.
"""

from dataclasses import dataclass
from typing import List

from hospital_robot_hri.interaction_logic import (DEST_LABEL, GOODBYE, GUIDING, HEAD_CANNOT_MOVE,
                                                  MENU, SLEEPING, InteractionLogic,
                                                  InteractionParams)

PAUSED, ARRIVED, WAITING, RETURNING, NAV_FAILED = (
    'PAUSED', 'ARRIVED', 'WAITING', 'RETURNING', 'NAV_FAILED')
DRIVING = (GUIDING, RETURNING)
TRIP_MENUS = (PAUSED, ARRIVED, WAITING, NAV_FAILED)

# extra screen buttons (ids must match [a-z_]{1,24}, face_http.parse_touch)
CHOICE_LABELS = {
    'wait': 'Wait for me here',
    'go_back': 'Go back without me',
    'return_with_me': 'Take me back to reception',
    'continue': 'Continue',
    'retry': 'Try again',
}


@dataclass
class NavParams:
    home_name: str = 'home'          # waypoint name of reception / the start position
    home_label: str = 'reception'
    arrive_timeout_s: float = 45.0   # nobody answers "wait / go back": go back alone (0 = never)
    wait_timeout_s: float = 600.0    # waited this long for the person: go back alone (0 = never)
    pause_timeout_s: float = 90.0    # stopped by a person, nobody chooses: stay here, sleep
    failed_timeout_s: float = 120.0  # after a failure nobody chooses: stay here, sleep
    mission_answer_s: float = 10.0   # no /mission/status for our trip by then = mission node down


class NavInteractionLogic(InteractionLogic):

    def __init__(self, params: InteractionParams = None, nav: NavParams = None):
        super().__init__(params)
        self.n = nav or NavParams()
        self.trip = None              # dict: id, target, mode, sent_t, answered, state
        self.mission_seq = 0
        self.remaining = None

    # ============================================================ helpers
    @property
    def nav(self) -> bool:
        return bool(self.p.nav_available)

    def _label(self, target):
        if target == self.n.home_name:
            return self.n.home_label
        return DEST_LABEL.get(target, target)

    def _go(self, t, target, mode) -> List[tuple]:
        self.mission_seq += 1
        self.trip = {'id': self.mission_seq, 'target': target, 'mode': mode,
                     'sent_t': t, 'answered': False, 'state': 'SENT'}
        self.remaining = None
        return [('mission', {'cmd': 'go', 'target': target, 'mode': mode,
                             'id': self.mission_seq})]

    def _cancel_trip(self) -> List[tuple]:
        if self.trip is None:
            return []
        return [('mission', {'cmd': 'cancel', 'id': self.trip['id']})]

    def _with_person(self) -> bool:
        return bool(self.trip) and self.trip['mode'] == 'guide'

    def _ui_for(self, state):
        if not self.nav:
            return super()._ui_for(state)
        trip = self.trip or {}
        target = trip.get('target', self.destination or '')
        label = self._label(target)
        if state in DRIVING:
            if target == self.n.home_name:
                title = (f'Taking you back to {label}' if self._with_person()
                         else f'Going back to {label}')
            else:
                title = f'Taking you to the {label}'
            return ('ui', {'mode': 'eyes', 'state': state, 'destination': label,
                           'title': title, 'stop': True})
        if state in TRIP_MENUS:
            if state == ARRIVED:
                title, ids, cancel = f'We have arrived: {label}', ['wait', 'go_back'], ''
            elif state == WAITING:
                title, ids, cancel = 'I am waiting for you', ['return_with_me', 'go_back'], ''
            elif state == PAUSED:
                title, cancel = 'I have stopped', 'Stay here'
                ids = ['continue'] + (['return_with_me', 'go_back']
                                      if target != self.n.home_name else [])
            else:  # NAV_FAILED
                title, cancel = 'I could not get there', 'Stay here'
                ids = ['retry'] + (['go_back'] if target != self.n.home_name else [])
            opts = []
            for i in ids:
                lab = CHOICE_LABELS[i]
                if i == 'continue':
                    lab = f'Continue to the {label}' if target != self.n.home_name \
                        else f'Continue to {label}'
                opts.append({'id': i, 'label': lab})
            return ('ui', {'mode': 'menu', 'state': state, 'title': title,
                           'options': opts, 'cancel': cancel, 'destination': label})
        return super()._ui_for(state)

    def _drive(self, t, target, mode, say_key) -> List[tuple]:
        """Start (or restart) a trip and enter GUIDING or RETURNING."""
        acts = self._go(t, target, mode)
        state = RETURNING if target == self.n.home_name else GUIDING
        acts = self._enter(t, state, ('expression', 'idle'), self._ui_for(state)) + acts
        if self.head is not None and self.head not in HEAD_CANNOT_MOVE and self.head != 'PARKED':
            acts.append(('log', 'column not PARKED: the mission node parks it first '
                                'or refuses to drive'))
        if say_key:
            acts += self._say(t, say_key)
        return acts

    def _go_home_alone(self, t, why) -> List[tuple]:
        return [('log', why)] + self._drive(t, self.n.home_name, 'alone', 'returning_to_base')

    def _menu_state(self, t, state, say_key, why='') -> List[tuple]:
        acts = self._enter(t, state, ('expression', 'listening'), self._ui_for(state))
        if why:
            acts.insert(1, ('log', why))
        if say_key:
            acts += self._say(t, say_key)
        return acts

    def _stay(self, t, why) -> List[tuple]:
        """End here, without driving: goodbye, then sleep where we are."""
        self.trip = None
        return self._goodbye(t, why)

    # ============================================================ overrides
    def _guide(self, t):
        if not self.nav:
            return super()._guide(t)
        if self.destination not in DEST_LABEL:
            return self._stay(t, f'no valid destination ({self.destination})')
        acts = self._drive(t, self.destination, 'guide', 'journey_start')
        if not self.parked:
            acts.insert(1, ('log', 'WARNING: column not confirmed PARKED. The mission node '
                                   'will not drive until head_column reports PARKED.'))
        return acts

    def on_choice(self, t, value) -> List[tuple]:
        if not self.nav or self.state == MENU:
            return super().on_choice(t, value)
        v = str(value or '').strip().lower()
        st = self.state
        trip = self.trip or {}
        target, mode = trip.get('target'), trip.get('mode', 'guide')
        if st in DRIVING and v == 'stop':
            return self._cancel_trip() + self._menu_state(t, PAUSED, 'guide_paused',
                                                          'Stop pressed on the screen')
        if st == ARRIVED:
            if v == 'wait':
                return self._menu_state(t, WAITING, 'waiting_here', 'person: wait for me')
            if v == 'go_back':
                return self._go_home_alone(t, 'person: go back without me')
        if st == WAITING:
            if v == 'return_with_me':
                return [('log', 'person: take me back')] + \
                    self._drive(t, self.n.home_name, 'guide', 'returning_with_you')
            if v == 'go_back':
                return self._go_home_alone(t, 'person: go back without me')
        if st == PAUSED:
            if v == 'continue' and target:
                return [('log', f'person: continue to {target}')] + \
                    self._drive(t, target, mode, 'journey_start' if mode == 'guide' else '')
            if v == 'return_with_me':
                return [('log', 'person: take me back')] + \
                    self._drive(t, self.n.home_name, 'guide', 'returning_with_you')
            if v == 'go_back':
                return self._go_home_alone(t, 'person: go back without me')
            if v == 'cancel':
                return self._stay(t, 'person: stay here')
        if st == NAV_FAILED:
            if v == 'retry' and target:
                return [('log', f'person: try again to {target}')] + \
                    self._drive(t, target, mode, 'journey_start' if mode == 'guide' else '')
            if v == 'go_back':
                return self._go_home_alone(t, 'person: go back without me')
            if v == 'cancel':
                return self._stay(t, 'person: stay here after the failure')
        return [('log', f'choice "{v}" ignored in {st}')]

    def on_command(self, t, cmd) -> List[tuple]:
        """Adds to wake | sleep | done:  stop (= the screen's Stop button) and
        home (staff: send the robot back to reception alone)."""
        if not self.nav:
            return super().on_command(t, cmd)
        c = str(cmd or '').strip().lower()
        if c == 'stop' and self.state in DRIVING:
            return self.on_choice(t, 'stop')
        if c == 'home' and self.state in (SLEEPING,) + TRIP_MENUS:
            return self._go_home_alone(t, 'command: home')
        if c in ('sleep', 'done') and self.trip is not None and self.state in DRIVING:
            acts = self._cancel_trip()
            self.trip = None
            return acts + super().on_command(t, 'sleep')
        if c == 'sleep':
            self.trip = None
        return super().on_command(t, c)

    def on_mission_status(self, t, status) -> List[tuple]:
        """/mission/status JSON from hospital_robot_nav's mission node."""
        if not self.nav or not isinstance(status, dict) or self.trip is None:
            return []
        if status.get('id') != self.trip['id']:
            return []                                 # an older trip, or someone else's
        self.trip['answered'] = True
        st = str(status.get('state', ''))
        self.trip['state'] = st
        if status.get('remaining_m') is not None:
            try:
                self.remaining = float(status['remaining_m'])
            except (TypeError, ValueError):
                pass
        if self.state not in DRIVING:
            return []                  # e.g. the CANCELED that follows our own Stop
        if st == 'SUCCEEDED':
            if self.trip['target'] == self.n.home_name:
                if self._with_person():
                    self.trip = None
                    return self._goodbye_with(t, 'back_at_base', 'back at reception with the person')
                self.trip = None
                return [('log', 'back at reception (alone): sleeping')] + self._sleep(t, park=True)
            return self._menu_state(t, ARRIVED, 'arrived', f'arrived at {self.trip["target"]}')
        if st in ('FAILED', 'CANCELED'):
            reason = str(status.get('reason', ''))
            key = 'nav_not_armed' if reason.startswith(('not_armed', 'motors_off')) \
                else 'nav_failed'
            return self._menu_state(t, NAV_FAILED, key, f'trip {st}: {reason}')
        return []

    def _goodbye_with(self, t, key, why):
        acts = self._goodbye(t, why)
        # replace the 'goodbye' line by the given one (same GOODBYE state and timing)
        out = []
        for a in acts:
            if a == ('say', 'goodbye'):
                out += self._say(t, key)
            else:
                out.append(a)
        return out

    def tick(self, t) -> List[tuple]:
        acts = super().tick(t)
        if acts or not self.nav:
            return acts
        st, age = self.state, t - self.entered
        trip = self.trip
        if st in DRIVING and trip and not trip['answered'] \
                and t - trip['sent_t'] > self.n.mission_answer_s:
            return self._menu_state(t, NAV_FAILED, 'nav_failed',
                                    'mission node did not answer: is navigation.launch.py running?')
        if st == ARRIVED and self.n.arrive_timeout_s > 0 and age >= self.n.arrive_timeout_s:
            return self._go_home_alone(t, 'nobody answered at the destination')
        if st == WAITING and self.n.wait_timeout_s > 0 and age >= self.n.wait_timeout_s:
            return self._go_home_alone(t, f'waited {age:.0f} s: going back alone')
        if st == PAUSED and age >= self.n.pause_timeout_s:
            return self._stay(t, 'paused and nobody chose: staying here')
        if st == NAV_FAILED and age >= self.n.failed_timeout_s:
            return self._stay(t, 'failure and nobody chose: staying here')
        return []

    def status(self) -> dict:
        s = super().status()
        if self.nav:
            s['trip'] = dict(self.trip) if self.trip else None
            s['remaining_m'] = self.remaining
        return s


__all__ = ['NavInteractionLogic', 'NavParams', 'PAUSED', 'ARRIVED', 'WAITING', 'RETURNING',
           'NAV_FAILED', 'GOODBYE']
