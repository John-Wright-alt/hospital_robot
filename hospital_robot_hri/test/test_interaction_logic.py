"""Unit tests for interaction_logic.py (no ROS, nothing moves).

    python3 -m pytest test/ -v        (from inside the hospital_robot_hri folder)

All head_column / lift / speech inputs here are hand-written test values.
"""

from hospital_robot_hri.interaction_logic import (
    InteractionLogic, InteractionParams, parse_head_state, parse_lift_state)


def kinds(actions, kind):
    return [a[1] for a in actions if a[0] == kind]


class Sim:
    """Drives the logic like the ROS node would, and pretends hri_speech says
    each line for `say_s` seconds."""

    def __init__(self, params=None, say_s=2.0, speech=True):
        self.L = InteractionLogic(params or InteractionParams())
        self.t = 0.0
        self.say_s = say_s
        self.speech = speech
        self.talking = None          # (key, ends_at)
        self.done = 0
        self.log = []
        self.actions = self.L.start(0.0)

    def apply(self, acts):
        self.actions += acts
        for a in acts:
            if a[0] == 'say' and self.speech:
                self.talking = (a[1], self.t + self.say_s)
        return acts

    def step(self, dt=0.1):
        self.t += dt
        if self.speech:
            key, end = self.talking if self.talking else ('', 0)
            if self.talking and self.t >= end:
                self.done += 1
                self.L.on_speech_status(self.t, {'speaking': False, 'key': '',
                                                 'last_key': key, 'done_count': self.done})
                self.talking = None
            else:
                self.L.on_speech_status(self.t, {'speaking': bool(self.talking), 'key': key,
                                                 'done_count': self.done})
        return self.apply(self.L.tick(self.t))

    def run(self, seconds, **feed):
        out = []
        for _ in range(int(round(seconds / 0.1))):
            for name, val in feed.items():
                getattr(self.L, 'on_' + name)(self.t, val)
            out += self.step()
        return out


# ------------------------------------------------------------ parsing
def test_parse_head_state_formats():
    assert parse_head_state('TRACKING') == 'TRACKING'
    assert parse_head_state('state: WAIT_ARM (not armed)') == 'WAIT_ARM'
    assert parse_head_state('{"state": "parked", "height_mm": 0}') == 'PARKED'
    assert parse_head_state('waiting') == 'WAITING'
    assert parse_head_state('hello') is None and parse_head_state(None) is None


def test_parse_lift_state_formats():
    assert parse_lift_state('l 0 1 42.5 80.0 0 0') == 'IDLE'
    assert parse_lift_state('l 1 1 42.5 80.0 0 0') == 'MOVING'
    assert parse_lift_state('Lift: IDLE homed=1 pos=12') == 'IDLE'
    assert parse_lift_state('garbage') is None


# ------------------------------------------------------------ start-up
def test_start_sleeps_and_moves_nothing():
    s = Sim()
    assert s.L.state == 'SLEEPING'
    assert kinds(s.actions, 'expression') == ['sleeping']
    assert kinds(s.actions, 'head') == []                       # never moves at start-up
    assert kinds(s.actions, 'ui')[0]['mode'] == 'sleep'


def test_choice_or_command_ignored_while_sleeping():
    s = Sim()
    assert s.L.on_choice(1.0, 'pharmacy') == []
    assert s.L.state == 'SLEEPING'


# ------------------------------------------------------------ the full happy path
def test_full_flow_with_face_found():
    s = Sim()
    s.L.on_head_state(0.0, 'PARKED')
    s.t = 2.0
    acts = s.apply(s.L.on_tap(s.t))
    assert s.L.state == 'WAKING' and kinds(acts, 'say') == ['wake_intro']
    assert kinds(acts, 'head') == []                            # NOT while she speaks
    s.run(1.0)
    assert s.L.state == 'WAKING'                                # still speaking
    acts = s.run(1.5)
    assert s.L.state == 'ADJUSTING' and kinds(acts, 'head') == ['search']
    assert 'searching' in kinds(acts, 'expression')
    s.L.on_head_state(s.t, 'SEARCHING')
    s.run(2.0, lift_state='l 1 1 100 300 0 0')                  # rising
    s.L.on_head_state(s.t, 'TRACKING')
    s.run(1.0, lift_state='l 1 1 150 160 0 0')                  # still correcting
    assert s.L.state == 'ADJUSTING'
    acts = s.run(2.0, lift_state='l 0 1 160 160 0 0')           # settled on the face
    assert kinds(acts, 'head') == ['hold']
    assert kinds(acts, 'say') == ['adjust_done'] and s.L.state == 'CONFIRMING'
    acts = s.run(2.5)
    assert s.L.state == 'MENU'
    menu = [u for u in kinds(acts, 'ui') if u['mode'] == 'menu'][0]
    assert [o['id'] for o in menu['options']] == ['doctor', 'xray', 'pharmacy']
    acts = s.apply(s.L.on_choice(s.t, 'pharmacy'))
    assert s.L.state == 'CHOSEN'
    assert kinds(acts, 'destination') == ['pharmacy'] and kinds(acts, 'head') == ['park']
    assert kinds(acts, 'say') == ['chose_pharmacy']
    s.run(3.0)
    assert s.L.state == 'CHOSEN'                                # spoken, but not parked yet
    s.L.on_head_state(s.t, 'PARKED')
    acts = s.run(0.5)
    assert s.L.state == 'GUIDING' and 'idle' in kinds(acts, 'expression')
    assert kinds(acts, 'say') == ['nav_not_ready']              # Phase 9: no navigation yet
    acts = s.run(25.0)
    assert 'goodbye' in kinds(acts, 'say')
    assert s.L.state == 'SLEEPING'
    assert kinds(s.actions, 'expression')[-1] == 'sleeping'


def test_lift_starts_only_after_the_intro_finished():
    s = Sim(say_s=6.0)
    s.L.on_head_state(0.0, 'PARKED')
    s.t = 2.0
    s.apply(s.L.on_tap(s.t))
    acts = s.run(5.5)
    assert 'search' not in kinds(acts, 'head')
    acts = s.run(1.0)
    assert kinds(acts, 'head') == ['search']


# ------------------------------------------------------------ things going wrong
def test_not_armed_means_no_lift_command():
    s = Sim()
    s.L.on_head_state(0.0, 'WAIT_ARM')
    s.t = 2.0
    s.apply(s.L.on_tap(s.t))
    acts = s.run(3.0)
    assert 'search' not in kinds(s.actions, 'head')
    assert kinds(acts, 'say') == ['adjust_unavailable']
    s.run(3.0)
    assert s.L.state == 'MENU'


def test_no_head_node_means_no_lift_command():
    s = Sim()
    s.t = 2.0
    s.apply(s.L.on_tap(s.t))
    acts = s.run(3.0)
    assert kinds(s.actions, 'head') == [] and kinds(acts, 'say') == ['adjust_unavailable']


def test_no_face_found_holds_and_carries_on():
    s = Sim(InteractionParams(adjust_timeout_s=8.0))
    s.L.on_head_state(0.0, 'PARKED')
    s.t = 2.0
    s.apply(s.L.on_tap(s.t))
    s.run(2.5)
    s.L.on_head_state(s.t, 'SEARCHING')
    acts = s.run(8.5, lift_state='l 1 1 200 300 0 0')
    assert kinds(acts, 'head') == ['hold'] and kinds(acts, 'say') == ['adjust_no_face']


def test_estop_while_adjusting():
    s = Sim()
    s.L.on_head_state(0.0, 'PARKED')
    s.t = 2.0
    s.apply(s.L.on_tap(s.t))
    s.run(2.5)
    assert s.L.state == 'ADJUSTING'
    s.L.on_head_state(s.t, 'WAIT_ARM')                         # E-stop: lift lost power
    acts = s.run(0.2)
    assert kinds(acts, 'say') == ['adjust_unavailable'] and 'hold' not in kinds(acts, 'head')


def test_face_lost_restarts_the_settle_timer():
    s = Sim()
    s.L.on_head_state(0.0, 'PARKED')
    s.t = 2.0
    s.apply(s.L.on_tap(s.t))
    s.run(2.5)
    s.L.on_head_state(s.t, 'TRACKING')
    s.run(1.0, lift_state='l 0 1 160 160 0 0')
    s.L.on_head_state(s.t, 'SEARCHING')                        # face lost before settling
    acts = s.run(1.0, lift_state='l 0 1 160 160 0 0')
    assert s.L.state == 'ADJUSTING' and 'hold' not in kinds(acts, 'head')


def test_menu_timeout_goes_back_to_sleep():
    s = Sim(InteractionParams(menu_timeout_s=5.0))
    s.L.on_head_state(0.0, 'WAIT_ARM')
    s.t = 2.0
    s.apply(s.L.on_tap(s.t))
    s.run(5.0)
    assert s.L.state == 'MENU'
    acts = s.run(8.0)
    assert 'goodbye' in kinds(acts, 'say') and s.L.state == 'SLEEPING'


def test_no_thanks():
    s = Sim()
    s.L.on_head_state(0.0, 'WAIT_ARM')
    s.t = 2.0
    s.apply(s.L.on_tap(s.t))
    s.run(5.0)
    acts = s.apply(s.L.on_choice(s.t, 'cancel'))
    assert kinds(acts, 'say') == ['goodbye']


def test_flow_continues_without_the_speech_node():
    s = Sim(speech=False)
    s.L.on_head_state(0.0, 'PARKED')
    s.t = 2.0
    s.apply(s.L.on_tap(s.t))
    acts = s.run(4.5)                                           # speech_start_timeout_s = 4
    assert kinds(acts, 'head') == ['search']


def test_unpark_timeout_is_logged():
    s = Sim(InteractionParams(park_timeout_s=5.0))
    s.L.on_head_state(0.0, 'WAIT_ARM')
    s.t = 2.0
    s.apply(s.L.on_tap(s.t))
    s.run(5.0)
    s.L.on_head_state(s.t, 'WAITING')                           # armed meanwhile, lift up
    s.apply(s.L.on_choice(s.t, 'doctor'))
    acts = s.run(6.0)
    assert s.L.state == 'GUIDING'
    assert any('NOT drive' in a[1] for a in acts if a[0] == 'log')


def test_tap_cooldown_after_sleep_and_commands():
    s = Sim()
    s.apply(s.L.on_command(10.0, 'sleep'))
    assert s.L.on_tap(10.2) == []                               # bounce after sleeping
    assert s.L.on_tap(11.5) != []
    s2 = Sim()
    assert s2.L.on_command(5.0, 'wake') != [] and s2.L.state == 'WAKING'
