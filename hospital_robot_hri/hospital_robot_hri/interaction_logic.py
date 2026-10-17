"""
interaction_logic.py  --  Anna's wake-up and conversation flow. NO ROS code
(unit tested in test/test_interaction_logic.py), same pattern as face_logic.py.

    SLEEPING ──tap──► WAKING ──intro said──► ADJUSTING ──face found, lift still──► CONFIRMING
       ▲              "Hello, I am Anna..."   head_column 'search'   'hold'          "The touchscreen is
       │                                      eyes: searching                         now positioned..."
       │                                                                                   │
       │   GOODBYE ◄──timeout / "No thanks"── MENU ◄───────────────────────────────────────┘
       │   "Thank you..."                      3 big buttons
       │   head 'park'                           │ choice
       │      ▲                                  ▼
       └──────┴────────── GUIDING ◄──said + parked── CHOSEN
                          eyes + LiDAR curiosity       "The pharmacy. Thank you. I am lowering..."
                          (Phase 7: Nav2 drives here)  head 'park', /hri/destination

The logic only RETURNS actions; the ROS node carries them out:
    ('expression', name)   -> /hri/expression
    ('say', event_key)     -> /hri/speech/event
    ('head', command)      -> /head_column/command   (search | hold | park | stop)
    ('ui', dict)           -> /hri/ui  (what the touchscreen shows)
    ('destination', key)   -> /hri/destination  (for the Phase 7 navigation layer)
    ('log', text)          -> the node's log

SAFETY: the column only moves after (1) a human armed the robot (the lift has
no power otherwise - Phase 8 design) and (2) a person tapped the screen and
heard/read the warning "Please wait so I can adjust". Nothing moves at start-up.
The head_column node and the Pico keep all their own limits; this file only
sends the same commands you can type with `ros2 topic pub`.
"""

import json
import math
import re
from dataclasses import dataclass
from typing import List, Optional

# Phase 8 head_column states (PHASE8_GUIDE.md section 9.6)
HEAD_STATES = ('WAIT_ARM', 'HOMING', 'SEARCHING', 'TRACKING', 'WAITING',
               'PARKING', 'PARKED', 'STOPPED', 'FAULT')
HEAD_CANNOT_MOVE = ('WAIT_ARM', 'FAULT')
_HEAD_RE = re.compile(r'\b(' + '|'.join(HEAD_STATES) + r')\b')

# Pico lift states in the 'l <state> <homed> <pos> <target> <switch> <fault>' line
LIFT_STATES = {0: 'IDLE', 1: 'MOVING', 2: 'HOMING', 3: 'PARKING', 4: 'FAULT'}
_LIFT_RE = re.compile(r'\b(IDLE|MOVING|HOMING|PARKING|FAULT)\b')

# The three menu choices (Phase 7 will give each one a map pose)
DESTINATIONS = (
    ('doctor', "Doctor's office"),
    ('xray', 'X-ray room'),
    ('pharmacy', 'Pharmacy'),
)
DEST_LABEL = dict(DESTINATIONS)

SLEEPING, WAKING, ADJUSTING, CONFIRMING, MENU, CHOSEN, GUIDING, GOODBYE = (
    'SLEEPING', 'WAKING', 'ADJUSTING', 'CONFIRMING', 'MENU', 'CHOSEN', 'GUIDING', 'GOODBYE')


def parse_head_state(text) -> Optional[str]:
    """'TRACKING', 'state: TRACKING (face 12 mm)', or JSON {"state": "TRACKING"}
    -> 'TRACKING'. None if no known state word is found."""
    if not isinstance(text, str):
        return None
    t = text.strip()
    if t.startswith('{'):
        try:
            d = json.loads(t)
            t = str(d.get('state', ''))
        except (ValueError, TypeError, AttributeError):
            pass
    m = _HEAD_RE.search(t.upper())
    return m.group(1) if m else None


def parse_lift_state(text) -> Optional[str]:
    """Pico line 'l 0 1 42.5 80.0 0 0' -> 'IDLE'. Also accepts text such as
    'Lift: IDLE homed=1 ...'. None if not understood."""
    if not isinstance(text, str):
        return None
    parts = text.split()
    if len(parts) >= 2 and parts[0] == 'l':
        try:
            return LIFT_STATES.get(int(float(parts[1])))
        except ValueError:
            return None
    m = _LIFT_RE.search(text.upper())
    return m.group(1) if m else None


@dataclass
class InteractionParams:
    adjust_enabled: bool = True     # false = never move the column (UI and speech only)
    settle_s: float = 1.5           # face tracked AND lift still this long = "found the height"
    max_track_s: float = 10.0       # tracking this long without the lift settling: hold anyway
    adjust_timeout_s: float = 30.0  # no face found in this time: hold where it is
    menu_timeout_s: float = 60.0    # nobody chooses: say goodbye, go back to sleep
    speech_start_timeout_s: float = 4.0   # hri_speech silent this long = not running: carry on
    speech_max_s: float = 25.0      # never wait longer than this for one sentence
    park_timeout_s: float = 40.0    # column not parked after this: carry on, but log it
    nav_available: bool = False     # Phase 7: true when the Nav2 task layer takes over
    guiding_s: float = 20.0         # Phase 9 placeholder: eyes + curiosity this long, then sleep
    tap_cooldown_s: float = 1.0     # ignore taps this soon after going to sleep


class InteractionLogic:

    def __init__(self, params: InteractionParams = None):
        self.p = params or InteractionParams()
        self.state = SLEEPING
        self.entered = 0.0
        self.head = None            # last head_column state word (None = never heard)
        self.head_t = None
        self.lift = None            # last lift state word (None = never heard)
        self.lift_idle_since = None
        self.track_since = None
        self.destination = None
        self.next_say = None        # line to say in CONFIRMING
        # speech tracking
        self.say_key = None
        self.say_sent = None
        self.say_started = False
        self.speech_seen = None     # time of the last /hri/speech/status
        self.speaking_key = ''
        self.speaking = False
        self.done_count = 0         # hri_speech's count of finished messages
        self.say_base = 0
        self.say_finished = False
        self.parked = False
        self.slept_at = -math.inf

    # ================================================================ helpers
    def _say(self, t, key):
        self.say_key, self.say_sent, self.say_started = key, t, False
        self.say_base, self.say_finished = self.done_count, False
        return [('say', key)]

    def speech_done(self, t) -> bool:
        """True once the last requested line has finished (or clearly never
        will: hri_speech not running, or far too long)."""
        if self.say_key is None or self.say_finished:
            return True
        if t - self.say_sent > self.p.speech_max_s:
            return True
        speech_alive = self.speech_seen is not None and t - self.speech_seen < 1.5
        if not self.say_started:
            if not speech_alive and t - self.say_sent > self.p.speech_start_timeout_s:
                return True                         # no speech node: don't hang
            return False
        return not (self.speaking and self.speaking_key == self.say_key)

    def _enter(self, t, state, *actions):
        self.state, self.entered = state, t
        return [('log', f'-> {state}')] + list(actions)

    # ================================================================ states
    def start(self, t) -> List[tuple]:
        """Called once when the node starts. Never moves anything."""
        return self._sleep(t, park=False)

    def _sleep(self, t, park=True):
        self.slept_at = t
        self.destination = None
        self.say_key = None
        acts = self._enter(t, SLEEPING, ('expression', 'sleeping'), self._ui_for(SLEEPING))
        if park and self.head is not None and self.head not in HEAD_CANNOT_MOVE:
            acts.append(('head', 'park'))
        return acts

    def _ui_for(self, state):
        if state == SLEEPING:
            return ('ui', {'mode': 'sleep', 'state': state})
        if state == ADJUSTING:
            return ('ui', {'mode': 'adjusting', 'state': state})
        if state == MENU:
            return ('ui', {'mode': 'menu', 'state': state,
                           'options': [{'id': k, 'label': v} for k, v in DESTINATIONS]})
        return ('ui', {'mode': 'eyes', 'state': state,
                       'destination': DEST_LABEL.get(self.destination, '')})

    def _wake(self, t):
        return self._enter(t, WAKING, ('expression', 'happy'), self._ui_for(WAKING),
                           *self._say(t, 'wake_intro'))

    def _adjust(self, t):
        if not self.p.adjust_enabled:
            return self._confirm(t, 'adjust_unavailable', 'column adjustment switched off')
        if self.head is None:
            return self._confirm(t, 'adjust_unavailable',
                                 'no /head_column/state received - is head_column running?')
        if self.head in HEAD_CANNOT_MOVE:
            return self._confirm(t, 'adjust_unavailable',
                                 f'head_column is {self.head} (robot not armed or lift fault)')
        self.track_since = None
        return self._enter(t, ADJUSTING, ('expression', 'searching'), self._ui_for(ADJUSTING),
                           ('head', 'search'))

    def _confirm(self, t, key, why=''):
        acts = self._enter(t, CONFIRMING, ('expression', 'happy'), self._ui_for(CONFIRMING),
                           *self._say(t, key))
        if why:
            acts.insert(1, ('log', why))
        return acts

    def _menu(self, t):
        return self._enter(t, MENU, ('expression', 'listening'), self._ui_for(MENU))

    def _choose(self, t, key):
        self.destination = key
        self.parked = self.head == 'PARKED'
        acts = self._enter(t, CHOSEN, ('destination', key), ('expression', 'happy'),
                           self._ui_for(CHOSEN), *self._say(t, f'chose_{key}'))
        if self.head is not None and self.head not in HEAD_CANNOT_MOVE:
            acts.append(('head', 'park'))           # Phase 8 rule: lower before driving
        return acts

    def _guide(self, t):
        acts = self._enter(t, GUIDING, ('expression', 'idle'), self._ui_for(GUIDING))
        if not self.parked:
            acts.insert(1, ('log', 'WARNING: column not confirmed PARKED. '
                                   'Phase 7 must NOT drive in this case.'))
        if not self.p.nav_available:
            acts += self._say(t, 'nav_not_ready')
        return acts

    def _goodbye(self, t, why=''):
        acts = self._enter(t, GOODBYE, ('expression', 'happy'), self._ui_for(GOODBYE),
                           *self._say(t, 'goodbye'))
        if why:
            acts.insert(1, ('log', why))
        if self.head is not None and self.head not in HEAD_CANNOT_MOVE and self.head != 'PARKED':
            acts.append(('head', 'park'))
        return acts

    # ================================================================ inputs
    def on_tap(self, t) -> List[tuple]:
        if self.state == SLEEPING and t - self.slept_at >= self.p.tap_cooldown_s:
            return [('log', 'screen tapped: waking up')] + self._wake(t)
        return []

    def on_choice(self, t, value) -> List[tuple]:
        if self.state != MENU:
            return []
        value = str(value or '').strip().lower()
        if value in DEST_LABEL:
            return self._choose(t, value)
        if value == 'cancel':
            return self._goodbye(t, 'menu: no thanks')
        return [('log', f'unknown menu choice "{value}" ignored')]

    def on_command(self, t, cmd) -> List[tuple]:
        """/hri/interaction/command: wake | sleep | done (done = Phase 7 arrived/finished)."""
        cmd = str(cmd or '').strip().lower()
        if cmd == 'wake':
            return self.on_tap(t + self.p.tap_cooldown_s)
        if cmd == 'sleep':
            return [('log', 'command: sleep')] + self._sleep(t, park=True)
        if cmd == 'done' and self.state == GUIDING:
            return self._goodbye(t, 'command: done')
        return [('log', f'command "{cmd}" ignored in {self.state}')]

    def on_speech_status(self, t, status: dict):
        if not isinstance(status, dict):
            return
        self.speech_seen = t
        self.speaking = bool(status.get('speaking'))
        self.speaking_key = str(status.get('key', ''))
        if self.say_key and self.speaking and self.speaking_key == self.say_key:
            self.say_started = True
        try:
            self.done_count = int(status.get('done_count', self.done_count))
        except (TypeError, ValueError):
            pass
        if (self.say_key and status.get('last_key') == self.say_key
                and self.done_count > self.say_base):
            self.say_started = self.say_finished = True      # even if it was very short

    def on_head_state(self, t, text) -> List[tuple]:
        s = parse_head_state(text)
        if s is None:
            return []
        old, self.head, self.head_t = self.head, s, t
        if s == 'PARKED':
            self.parked = True
        elif s in ('SEARCHING', 'TRACKING', 'HOMING', 'WAITING', 'PARKING'):
            self.parked = False
        if self.state == ADJUSTING:
            if s == 'TRACKING' and self.track_since is None:
                self.track_since = t
            elif s != 'TRACKING':
                self.track_since = None
        return [('log', f'head_column: {old} -> {s}')] if s != old else []

    def on_lift_state(self, t, text):
        s = parse_lift_state(text)
        if s is None:
            return
        if s == 'IDLE':
            if self.lift != 'IDLE' or self.lift_idle_since is None:
                self.lift_idle_since = t
        else:
            self.lift_idle_since = None
        self.lift = s

    # ================================================================ time
    def tick(self, t) -> List[tuple]:
        st, age = self.state, t - self.entered
        if st == WAKING and self.speech_done(t):
            return self._adjust(t)

        if st == ADJUSTING:
            if self.head in HEAD_CANNOT_MOVE:
                return self._confirm(t, 'adjust_unavailable',
                                     f'head_column went {self.head} while adjusting')
            if self.track_since is not None:
                tracked = t - self.track_since
                lift_still = (self.lift_idle_since is not None
                              and t - max(self.lift_idle_since, self.track_since) >= self.p.settle_s)
                no_lift_info = self.lift is None and tracked >= self.p.settle_s + 1.0
                if lift_still or no_lift_info or tracked >= self.p.max_track_s:
                    return [('head', 'hold')] + self._confirm(
                        t, 'adjust_done', f'face found, holding (tracked {tracked:.1f} s)')
            if age >= self.p.adjust_timeout_s:
                return [('head', 'hold')] + self._confirm(t, 'adjust_no_face',
                                                          'no face found in time: holding')

        if st == CONFIRMING and self.speech_done(t):
            return self._menu(t)

        if st == MENU and age >= self.p.menu_timeout_s:
            return self._goodbye(t, 'menu timed out')

        if st == CHOSEN and self.speech_done(t):
            head_gone = self.head is None or self.head in HEAD_CANNOT_MOVE
            if self.parked or head_gone:
                return self._guide(t)
            if age >= self.p.park_timeout_s:
                return [('log', f'column not PARKED after {self.p.park_timeout_s:.0f} s '
                                f'(head_column: {self.head})')] + self._guide(t)

        if st == GUIDING and not self.p.nav_available and age >= self.p.guiding_s \
                and self.speech_done(t):
            return self._goodbye(t, 'Phase 9 placeholder finished (no navigation yet)')

        if st == GOODBYE and self.speech_done(t):
            return self._sleep(t, park=False)       # park was already sent in GOODBYE
        return []

    def status(self) -> dict:
        return {'state': self.state, 'head': self.head, 'lift': self.lift,
                'destination': self.destination, 'parked': self.parked,
                'waiting_for_speech': self.say_key if self.say_key and not self.say_started
                else ''}
