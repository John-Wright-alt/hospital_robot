"""
speech_logic.py  --  decides WHAT the robot says and WHEN. NO ROS code and NO
audio code, so every rule here is unit tested on any computer
(test/test_speech_logic.py). Same pattern as face_logic.py and pico_protocol.py.

The pieces:
  load_library()       reads config/phrases.yaml (the message library) and checks it
  SpeechPlanner        event key -> one Utterance (picks the phrase, duplicate cooldown,
                       occasional humour)
  SpeechQueue          priority queue: no overlapping speech, safety messages first,
                       stale messages dropped
  MotionDetector       /odom speeds -> stopped / forward / reverse / turning
  MotionAnnouncer      motion changes -> "I'm moving", reversing warning, periodic cue
  RobotStateWatcher    /robot_state words -> armed / E-stop / fault / controller lost
  FlagWatcher          rising edge of a flag in /pico/status (e.g. BATT_WARN)
  GreetingDetector     someone stands close to the parked robot -> one greeting
  Repeater             "say this again every N seconds while the condition lasts"

IMPORTANT: none of this decides whether the robot is safe to move. The speech
node only listens and talks. Stopping is done by the Pico firmware, the E-stop
and (later) Nav2, exactly as before.
"""

import math
import random
from dataclasses import dataclass, field
from typing import Dict, List, Optional

# ---------------------------------------------------------------------------
# Priorities
# ---------------------------------------------------------------------------
#   cue       short sound, no words (the "I'm still moving" tick). Dropped if
#             anything else is waiting.
#   chatter   greetings, jokes. Never interrupts anything; anything else
#             interrupts it.
#   info      useful status ("Robot passing"). Waits for info and above.
#   warning   safety-related ("I'm reversing"). Interrupts cue/chatter/info.
#   critical  faults and E-stop. Interrupts everything below it and waits up
#             to 2 minutes in the queue (it is never dropped for chatter).
PRIORITIES = {'cue': 0, 'chatter': 1, 'info': 2, 'warning': 3, 'critical': 4}
PRIORITY_NAMES = {v: k for k, v in PRIORITIES.items()}
INTERRUPT_MIN = PRIORITIES['warning']      # warning/critical cut off info too

# How long a message may wait in the queue before it is pointless (seconds).
DEFAULT_MAX_AGE = {'cue': 1.0, 'chatter': 6.0, 'info': 10.0,
                   'warning': 30.0, 'critical': 120.0}

# Humour is only ever used for these. Safety messages must be plain.
PLAYFUL_ALLOWED = ('chatter', 'info')

MAX_TEXT_CHARS = 220                       # keeps every announcement short


@dataclass
class EventDef:
    key: str
    priority: str
    phrases: List[str]
    playful: List[str] = field(default_factory=list)
    sound: Optional[str] = None
    cooldown_s: float = 5.0
    max_age_s: float = 10.0
    rotate: bool = False                   # cycle through `phrases` instead of always the first
    length_scale: Optional[float] = None   # voice speed for this event (None = speech.yaml)
    sentence_silence: Optional[float] = None   # pause between sentences, s (None = speech.yaml)


def load_library(data: dict, known_sounds=None) -> Dict[str, EventDef]:
    """Turn the decoded phrases.yaml into EventDef objects, with clear errors.

    known_sounds: names of the built-in sounds (to catch typos). None = don't check.
    """
    if not isinstance(data, dict) or not isinstance(data.get('events'), dict):
        raise ValueError('phrases.yaml must contain a top-level "events:" mapping')
    lib = {}
    for key, e in data['events'].items():
        if not isinstance(e, dict):
            raise ValueError(f'event "{key}": must be a mapping')
        prio = e.get('priority')
        if prio not in PRIORITIES:
            raise ValueError(f'event "{key}": priority must be one of {list(PRIORITIES)}')
        phrases = [str(p).strip() for p in (e.get('phrases') or []) if str(p).strip()]
        playful = [str(p).strip() for p in (e.get('playful') or []) if str(p).strip()]
        sound = e.get('sound')
        if not phrases and not sound:
            raise ValueError(f'event "{key}": needs at least one phrase or a sound')
        if playful and prio not in PLAYFUL_ALLOWED:
            raise ValueError(f'event "{key}": "playful" phrases are not allowed for '
                             f'{prio} messages (safety messages must be plain)')
        for p in phrases + playful:
            if len(p) > MAX_TEXT_CHARS:
                raise ValueError(f'event "{key}": phrase longer than {MAX_TEXT_CHARS} '
                                 f'characters: "{p[:40]}..."')
        if known_sounds is not None and sound and sound not in known_sounds:
            raise ValueError(f'event "{key}": unknown sound "{sound}". '
                             f'Known: {sorted(known_sounds)}')
        ls, ss = e.get('length_scale'), e.get('sentence_silence')
        if ls is not None and not 0.5 <= float(ls) <= 2.0:
            raise ValueError(f'event "{key}": length_scale must be 0.5 .. 2.0')
        if ss is not None and not 0.0 <= float(ss) <= 2.0:
            raise ValueError(f'event "{key}": sentence_silence must be 0.0 .. 2.0 s')
        lib[key] = EventDef(
            key=key, priority=prio, phrases=phrases, playful=playful, sound=sound,
            cooldown_s=float(e.get('cooldown_s', 5.0)),
            max_age_s=float(e.get('max_age_s', DEFAULT_MAX_AGE[prio])),
            rotate=bool(e.get('rotate', False)),
            length_scale=None if ls is None else float(ls),
            sentence_silence=None if ss is None else float(ss))
    return lib


def all_texts(lib: Dict[str, EventDef]) -> List[str]:
    """Every sentence in the library, without duplicates."""
    seen, out = set(), []
    for e in lib.values():
        for p in e.phrases + e.playful:
            if p not in seen:
                seen.add(p)
                out.append(p)
    return out


def all_jobs(lib: Dict[str, EventDef]) -> List[tuple]:
    """Every (sentence, length_scale, sentence_silence) to pre-render. The same
    sentence spoken in two styles is two jobs. None = the speech.yaml default."""
    seen, out = set(), []
    for e in lib.values():
        for p in e.phrases + e.playful:
            job = (p, e.length_scale, e.sentence_silence)
            if job not in seen:
                seen.add(job)
                out.append(job)
    return out


# ---------------------------------------------------------------------------
# Utterances and the planner
# ---------------------------------------------------------------------------
@dataclass
class Utterance:
    key: str                 # event key, or 'say' for free text
    priority: int
    text: str                # '' = sound only
    sound: Optional[str]
    created: float           # time it was requested (for expiry and latency)
    max_age: float
    seq: int = 0
    length_scale: Optional[float] = None       # None = speech.yaml default
    sentence_silence: Optional[float] = None

    @property
    def priority_name(self):
        return PRIORITY_NAMES[self.priority]


class SpeechPlanner:
    """Event key -> Utterance. Handles duplicate cooldown and phrase choice."""

    def __init__(self, library: Dict[str, EventDef], humour_chance: float = 0.25,
                 rng: Optional[random.Random] = None):
        self.lib = library
        self.humour_chance = max(0.0, min(1.0, humour_chance))
        self.rng = rng or random.Random()
        self.last_made: Dict[str, float] = {}
        self.rotation: Dict[str, int] = {}
        self._seq = 0

    def _next_seq(self):
        self._seq += 1
        return self._seq

    def make(self, key: str, now: float, force: bool = False,
             allow_humour: bool = True) -> Optional[Utterance]:
        """None if the key is unknown or it was said less than cooldown_s ago."""
        e = self.lib.get(key)
        if e is None:
            return None
        last = self.last_made.get(key)
        if not force and last is not None and now - last < e.cooldown_s:
            return None
        self.last_made[key] = now

        text = ''
        if e.phrases:
            if e.rotate:
                i = self.rotation.get(key, 0)
                text = e.phrases[i % len(e.phrases)]
                self.rotation[key] = i + 1
            else:
                text = e.phrases[0]                       # the primary phrase
        if (allow_humour and e.playful and e.priority in PLAYFUL_ALLOWED
                and self.rng.random() < self.humour_chance):
            text = self.rng.choice(e.playful)
        return Utterance(key=key, priority=PRIORITIES[e.priority], text=text,
                         sound=e.sound, created=now, max_age=e.max_age_s,
                         seq=self._next_seq(), length_scale=e.length_scale,
                         sentence_silence=e.sentence_silence)

    def make_text(self, text: str, now: float, priority: str = 'info') -> Optional[Utterance]:
        """Free text from /hri/speech/say. Never critical: real alarms use events."""
        text = ' '.join(str(text).split())[:MAX_TEXT_CHARS]
        if not text:
            return None
        prio = priority if priority in ('chatter', 'info', 'warning') else 'info'
        return Utterance(key='say', priority=PRIORITIES[prio], text=text, sound=None,
                         created=now, max_age=DEFAULT_MAX_AGE[prio], seq=self._next_seq())


# ---------------------------------------------------------------------------
# The queue: exactly one thing plays at a time
# ---------------------------------------------------------------------------
class SpeechQueue:
    """Ordered by priority (highest first), then by age (oldest first).

    offer() returns:
      'interrupt'  queued at the front AND the caller must stop what is playing
      'queued'     will play after the current message
      'dropped'    not accepted (duplicate, cue while busy, chatter while busy, full)
    """

    def __init__(self, max_len: int = 8):
        self.max_len = max_len
        self.items: List[Utterance] = []
        self.expired: List[Utterance] = []     # filled by pop(); the node logs and clears it
        self.evicted: List[Utterance] = []

    def __len__(self):
        return len(self.items)

    def _sort(self):
        self.items.sort(key=lambda u: (-u.priority, u.created, u.seq))

    def offer(self, u: Utterance, playing: Optional[Utterance] = None) -> str:
        busy = playing is not None or bool(self.items)
        if u.priority == PRIORITIES['cue'] and busy:
            return 'dropped'
        if u.priority == PRIORITIES['chatter'] and busy:
            return 'dropped'
        if any(q.key == u.key and q.text == u.text for q in self.items):
            return 'dropped'                       # identical message already waiting
        self.items.append(u)
        self._sort()
        if len(self.items) > self.max_len:
            victim = max(self.items, key=lambda q: (-q.priority, q.created, q.seq))
            self.items.remove(victim)
            self.evicted.append(victim)
            if victim is u:
                return 'dropped'
        if playing is not None and u.priority > playing.priority and (
                u.priority >= INTERRUPT_MIN or playing.priority <= PRIORITIES['chatter']):
            return 'interrupt'                     # warnings cut off anything lower;
        return 'queued'                            # anything cuts off chatter or a cue

    # Note: a message that is cut off is NOT replayed. It was interrupted by
    # something more important (e.g. the E-stop), so the old message ("I'm
    # reversing") is no longer true. Anything that still matters repeats by
    # itself (fault, assistance, battery: see Repeater).

    def pop(self, now: float) -> Optional[Utterance]:
        keep = []
        for u in self.items:
            (self.expired if now - u.created > u.max_age else keep).append(u)
        self.items = keep
        return self.items.pop(0) if self.items else None


# ---------------------------------------------------------------------------
# Motion
# ---------------------------------------------------------------------------
MOVING = ('forward', 'reverse', 'turning')


@dataclass
class MotionParams:
    # Initial values. Validate with test M1 in the guide (odom noise at rest).
    v_on: float = 0.03       # m/s   |linear| above this (for start_s) = moving
    v_off: float = 0.015     # m/s   both below the *_off values (for stop_s) = stopped
    w_on: float = 0.15       # rad/s
    w_off: float = 0.08      # rad/s
    start_s: float = 0.25    # must be moving this long before we call it moving
    stop_s: float = 1.0      # must be still this long before we call it stopped
    stale_s: float = 0.5     # no /odom for this long = 'unknown' (bridge not running)
    w_full: float = 0.6      # rad/s that turns the eyes fully to one side


class MotionDetector:
    """/odom twist -> 'unknown' | 'stopped' | 'forward' | 'reverse' | 'turning'.
    Hysteresis + time filters, so encoder noise and a single bad sample can
    never trigger an announcement."""

    def __init__(self, params: MotionParams = None):
        self.p = params or MotionParams()
        self.state = 'unknown'
        self.v = 0.0
        self.w = 0.0
        self.last_t = None
        self.active_since = None
        self.quiet_since = None

    def _kind(self, v, w):
        if abs(v) >= self.p.v_on:
            return 'forward' if v > 0 else 'reverse'
        return 'turning'

    def _set(self, new):
        if new != self.state:
            self.state = new
            self.active_since = None
            self.quiet_since = None
            return new
        return None

    def update(self, t: float, v: float, w: float) -> Optional[str]:
        """Feed one odometry sample. Returns the new state if it changed."""
        if not (math.isfinite(v) and math.isfinite(w)):
            return None
        p = self.p
        self.last_t, self.v, self.w = t, v, w
        active = abs(v) >= p.v_on or abs(w) >= p.w_on
        quiet = abs(v) < p.v_off and abs(w) < p.w_off

        if self.state not in MOVING:
            if active:
                if self.active_since is None:
                    self.active_since = t
                if t - self.active_since >= p.start_s:
                    return self._set(self._kind(v, w))
                return None
            self.active_since = None
            return self._set('stopped') if self.state == 'unknown' else None

        # currently moving
        if quiet:
            if self.quiet_since is None:
                self.quiet_since = t
            if t - self.quiet_since >= p.stop_s:
                return self._set('stopped')
            return None
        self.quiet_since = None
        if active:
            kind = self._kind(v, w)
            if kind != self.state:
                self.state = kind                  # forward <-> turning <-> reverse
                return kind
        return None

    def tick(self, t: float) -> Optional[str]:
        """Call regularly: odometry that stops arriving means 'unknown'."""
        if self.last_t is not None and t - self.last_t > self.p.stale_s:
            self.last_t = None
            return self._set('unknown')
        return None

    @property
    def turn(self) -> float:
        """-1..+1, + = turning to the robot's LEFT (for the eyes)."""
        if self.state not in MOVING or self.p.w_full <= 0:
            return 0.0
        return max(-1.0, min(1.0, self.w / self.p.w_full))


class Repeater:
    """Fires every interval_s while active. interval_s <= 0 disables it."""

    def __init__(self, interval_s: float):
        self.interval_s = interval_s
        self.active = False
        self.last = None

    def start(self, t: float, fired_now: bool = True):
        if not self.active:
            self.active = True
            self.last = t if fired_now else t - self.interval_s

    def stop(self):
        self.active = False
        self.last = None

    def due(self, t: float) -> bool:
        if not self.active or self.interval_s <= 0:
            return False
        if self.last is None or t - self.last >= self.interval_s:
            self.last = t
            return True
        return False


@dataclass
class MotionAnnounceParams:
    restart_quiet_s: float = 10.0     # moving again within this time of stopping: chime only
    cue_interval_s: float = 6.0       # soft "still moving" tick while driving (0 = off)
    reminder_interval_s: float = 45.0 # spoken reminder while driving (0 = off)


class MotionAnnouncer:
    """Turns motion changes into event keys:
        moving_start     stopped -> moving (spoken)
        moving_resume    moving again soon after a stop (chime only, not chatty)
        reversing        whenever reversing starts (spoken warning)
        moving_cue       short tick every cue_interval_s while moving
        moving_reminder  spoken reminder every reminder_interval_s while moving
    """

    def __init__(self, params: MotionAnnounceParams = None):
        self.p = params or MotionAnnounceParams()
        self.last_stop = -math.inf
        self.quiet_until = -math.inf
        self.cue = Repeater(self.p.cue_interval_s)
        self.reminder = Repeater(self.p.reminder_interval_s)

    def suppress_start_speech(self, t: float, seconds: float):
        """Something already told people we're about to move (e.g. 'journey_start'):
        don't say it twice, just chime."""
        self.quiet_until = max(self.quiet_until, t + seconds)

    def on_change(self, t: float, old: str, new: str) -> List[str]:
        out = []
        if new == 'reverse' and old != 'reverse':
            out.append('reversing')                # has its own chime: nothing else needed
        elif new in MOVING and old not in MOVING:
            if t - self.last_stop < self.p.restart_quiet_s or t < self.quiet_until:
                out.append('moving_resume')
            else:
                out.append('moving_start')
        if new in MOVING and old not in MOVING:
            self.cue.start(t)
            self.reminder.start(t)
        if new not in MOVING and old in MOVING:
            self.cue.stop()
            self.reminder.stop()
            if new == 'stopped':
                self.last_stop = t
        return out

    def tick(self, t: float) -> List[str]:
        out = []
        if self.reminder.due(t):
            out.append('moving_reminder')
        if self.cue.due(t):
            out.append('moving_cue')
        return out


# ---------------------------------------------------------------------------
# Robot state (/robot_state from pico_bridge) and status flags (/pico/status)
# ---------------------------------------------------------------------------
class RobotStateWatcher:
    """/robot_state words (DISARMED ARMING ARMED DISARMING ESTOP FAULT, from
    pico_protocol.status_summary) -> event keys. If the words stop arriving
    after they have been seen, that is 'controller_lost'. If they were never
    seen (pico_bridge not running, e.g. gaze_test), nothing is said."""

    def __init__(self, timeout_s: float = 3.0):
        self.timeout_s = timeout_s
        self.state = None
        self.last_t = None
        self.lost = False
        self.was_armed = False

    def on_state(self, t: float, s: str) -> List[str]:
        s = (s or '').strip().upper()
        out = []
        self.last_t = t
        if self.lost:
            self.lost = False
            out.append('controller_restored')
        prev, self.state = self.state, s
        if s == prev:
            return out
        if s == 'ARMED':
            out.append('motors_armed')
            self.was_armed = True
        elif s == 'ESTOP':
            out.append('estop')
            self.was_armed = False
        elif s == 'FAULT':
            out.append('fault')
            self.was_armed = False
        elif s in ('DISARMED', 'DISARMING'):
            if prev == 'ESTOP' and s == 'DISARMED':
                out.append('estop_released')
            elif self.was_armed:
                out.append('motors_off')
            self.was_armed = False
        return out

    def tick(self, t: float) -> List[str]:
        if (self.last_t is not None and not self.lost
                and t - self.last_t > self.timeout_s):
            self.lost = True
            return ['controller_lost']
        return []


class FlagWatcher:
    """Rising/falling edge of one flag name inside the /pico/status text,
    e.g. 'ARMED  flags=0008 (BATT_WARN)  vbat=11.05 V  vmot=11.0 V'."""

    def __init__(self, flag: str):
        self.flag = flag
        self.on = False

    def on_text(self, text: str) -> Optional[bool]:
        """True on the rising edge, False on the falling edge, else None."""
        now_on = self.flag in (text or '').split('(', 1)[-1]
        if now_on != self.on:
            self.on = now_on
            return now_on
        return None


# ---------------------------------------------------------------------------
# Greeting
# ---------------------------------------------------------------------------
@dataclass
class GreetParams:
    enabled: bool = True
    range_m: float = 1.2       # someone this close ...
    dwell_s: float = 2.0       # ... for this long (walking past is not enough)
    cooldown_s: float = 90.0   # at most one greeting per this many seconds


class GreetingDetector:
    """Uses /hri/gaze/status (from lidar_gaze). Only while parked and idle,
    because lidar_gaze assumes a stationary robot."""

    def __init__(self, params: GreetParams = None):
        self.p = params or GreetParams()
        self.tid = None
        self.since = None
        self.greeted = set()
        self.last_greet = -math.inf

    def on_gaze(self, t: float, status: dict, allowed: bool) -> bool:
        if not self.p.enabled or not allowed or not isinstance(status, dict):
            self.tid = None
            return False
        if status.get('state') == 'learning':
            self.greeted.clear()                     # lidar_gaze restarted: IDs restart
        try:
            rng = float(status.get('range_m'))
        except (TypeError, ValueError):
            rng = math.inf
        tid = status.get('track_id')
        if status.get('state') != 'tracking' or tid is None or not rng <= self.p.range_m:
            self.tid = None
            return False
        if tid != self.tid:
            self.tid, self.since = tid, t
            return False
        if (t - self.since >= self.p.dwell_s and tid not in self.greeted
                and t - self.last_greet >= self.p.cooldown_s):
            self.greeted.add(tid)
            self.last_greet = t
            return True
        return False


def in_quiet_hours(hour: int, start: int, end: int) -> bool:
    """start/end are whole hours 0..23; start < 0 disables. Wraps midnight
    (22 -> 6 means 22:00 to 05:59)."""
    if start < 0 or end < 0 or start == end:
        return False
    if start < end:
        return start <= hour < end
    return hour >= start or hour < end
