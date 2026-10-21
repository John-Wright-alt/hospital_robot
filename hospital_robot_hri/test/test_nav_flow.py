"""Unit tests for nav_flow.py (Phase 7 trip flow). No ROS, nothing moves.

    python3 -m pytest test/ -v        (from inside the hospital_robot_hri folder)

All head_column / mission inputs here are hand-written test values.
"""
from hospital_robot_hri.interaction_logic import InteractionParams
from hospital_robot_hri.nav_flow import NavInteractionLogic, NavParams


def kinds(actions, kind):
    return [a[1] for a in actions if a[0] == kind]


class Sim:
    """Like test_interaction_logic.Sim: hri_speech 'says' each line in say_s."""

    def __init__(self, nav=True, say_s=1.0, **navkw):
        self.L = NavInteractionLogic(InteractionParams(nav_available=nav), NavParams(**navkw))
        self.t, self.say_s, self.talking, self.done = 0.0, say_s, None, 0
        self.actions = self.L.start(0.0)

    def apply(self, acts):
        self.actions += acts
        for a in acts:
            if a[0] == 'say':
                self.talking = (a[1], self.t + self.say_s)
        return acts

    def step(self, dt=0.1):
        self.t += dt
        key, end = self.talking if self.talking else ('', 0)
        if self.talking and self.t >= end:
            self.done += 1
            self.L.on_speech_status(self.t, {'speaking': False, 'key': '', 'last_key': key,
                                             'done_count': self.done})
            self.talking = None
        else:
            self.L.on_speech_status(self.t, {'speaking': bool(self.talking), 'key': key,
                                             'done_count': self.done})
        return self.apply(self.L.tick(self.t))

    def run(self, seconds):
        out = []
        for _ in range(int(round(seconds / 0.1))):
            out += self.step()
        return out

    def choose(self, value):
        return self.apply(self.L.on_choice(self.t, value))

    def mission(self, state, reason='', trip_id=None):
        tid = self.L.trip['id'] if trip_id is None else trip_id
        return self.apply(self.L.on_mission_status(self.t, {'id': tid, 'state': state,
                                                            'reason': reason}))

    def to_guiding(self, dest='pharmacy'):
        """Tap -> intro -> (column WAIT_ARM: no adjust) -> menu -> choice -> parked."""
        self.L.on_head_state(0.0, 'PARKED')
        self.t = 2.0
        self.apply(self.L.on_tap(self.t))
        self.run(1.5)                                   # intro said -> ADJUSTING
        self.L.on_head_state(self.t, 'WAIT_ARM')        # pretend not armed: skip adjusting
        self.run(2.5)                                   # adjust_unavailable said -> MENU
        assert self.L.state == 'MENU'
        self.choose(dest)
        self.L.on_head_state(self.t, 'PARKED')
        return self.run(1.5)


def last_ui(acts):
    return [u for u in kinds(acts, 'ui')][-1]


# ------------------------------------------------------------ nav off = Phase 9 unchanged
def test_nav_off_is_phase9_behaviour():
    s = Sim(nav=False)
    acts = s.to_guiding()
    assert s.L.state == 'GUIDING'
    assert kinds(acts, 'say') == ['nav_not_ready'] and kinds(acts, 'mission') == []
    assert s.L.on_mission_status(s.t, {'id': 1, 'state': 'SUCCEEDED'}) == []
    s.run(25.0)
    assert s.L.state == 'SLEEPING'


# ------------------------------------------------------------ the full demo
def test_full_trip_wait_and_return_with_person():
    s = Sim()
    acts = s.to_guiding('pharmacy')
    assert s.L.state == 'GUIDING'
    go = kinds(acts, 'mission')[0]
    assert go['cmd'] == 'go' and go['target'] == 'pharmacy' and go['mode'] == 'guide'
    assert 'journey_start' in kinds(acts, 'say')
    ui = last_ui(acts)
    assert ui['mode'] == 'eyes' and ui['stop'] is True and 'Pharmacy' in ui['title']

    s.mission('ACTIVE')
    assert s.L.state == 'GUIDING'
    acts = s.mission('SUCCEEDED')
    assert s.L.state == 'ARRIVED' and kinds(acts, 'say') == ['arrived']
    assert [o['id'] for o in last_ui(acts)['options']] == ['wait', 'go_back']
    assert last_ui(acts)['cancel'] == ''

    acts = s.choose('wait')
    assert s.L.state == 'WAITING' and kinds(acts, 'say') == ['waiting_here']
    assert [o['id'] for o in last_ui(acts)['options']] == ['return_with_me', 'go_back']

    acts = s.choose('return_with_me')
    assert s.L.state == 'RETURNING'
    back = kinds(acts, 'mission')[0]
    assert back['target'] == 'home' and back['mode'] == 'guide' and back['id'] == go['id'] + 1
    assert 'returning_with_you' in kinds(acts, 'say')
    assert 'back to reception' in last_ui(acts)['title']

    acts = s.mission('SUCCEEDED')
    assert kinds(acts, 'say') == ['back_at_base'] and s.L.state == 'GOODBYE'
    s.run(2.0)
    assert s.L.state == 'SLEEPING'


def test_arrived_go_back_alone_then_sleep_at_home():
    s = Sim()
    s.to_guiding('xray')
    s.mission('SUCCEEDED')
    acts = s.choose('go_back')
    back = kinds(acts, 'mission')[0]
    assert back['target'] == 'home' and back['mode'] == 'alone'
    assert 'returning_to_base' in kinds(acts, 'say')
    assert 'Going back' in last_ui(acts)['title']
    acts = s.mission('SUCCEEDED')
    assert s.L.state == 'SLEEPING' and kinds(acts, 'say') == []


def test_nobody_answers_at_destination_goes_back_alone():
    s = Sim(arrive_timeout_s=10.0)
    s.to_guiding('doctor')
    s.mission('SUCCEEDED')
    acts = s.run(10.5)
    assert s.L.state == 'RETURNING'
    assert kinds(acts, 'mission')[0]['mode'] == 'alone'


def test_arrive_timeout_zero_means_never_drive_alone():
    s = Sim(arrive_timeout_s=0.0, wait_timeout_s=0.0)
    s.to_guiding('doctor')
    s.mission('SUCCEEDED')
    assert kinds(s.run(100.0), 'mission') == [] and s.L.state == 'ARRIVED'


def test_wait_timeout_goes_back_alone():
    s = Sim(wait_timeout_s=20.0)
    s.to_guiding()
    s.mission('SUCCEEDED')
    s.choose('wait')
    assert kinds(s.run(19.0), 'mission') == []
    acts = s.run(1.5)
    assert s.L.state == 'RETURNING' and kinds(acts, 'mission')[0]['mode'] == 'alone'


# ------------------------------------------------------------ stop button
def test_stop_pauses_and_continue_resumes():
    s = Sim()
    acts = s.to_guiding('pharmacy')
    first = kinds(acts, 'mission')[0]['id']
    s.mission('ACTIVE')
    acts = s.choose('stop')
    assert kinds(acts, 'mission') == [{'cmd': 'cancel', 'id': first}]
    assert s.L.state == 'PAUSED' and kinds(acts, 'say') == ['guide_paused']
    ui = last_ui(acts)
    assert [o['id'] for o in ui['options']] == ['continue', 'return_with_me', 'go_back']
    assert ui['cancel'] == 'Stay here'
    assert s.mission('CANCELED', trip_id=first) == [] and s.L.state == 'PAUSED'
    acts = s.choose('continue')
    go = kinds(acts, 'mission')[0]
    assert s.L.state == 'GUIDING' and go['target'] == 'pharmacy' and go['id'] == first + 1


def test_stop_then_stay_here_sleeps_without_driving():
    s = Sim()
    s.to_guiding()
    s.choose('stop')
    acts = s.choose('cancel')
    assert kinds(acts, 'mission') == [] and 'goodbye' in kinds(acts, 'say')
    s.run(2.0)
    assert s.L.state == 'SLEEPING' and s.L.trip is None


def test_pause_timeout_stays_put():
    s = Sim(pause_timeout_s=30.0)
    s.to_guiding()
    s.choose('stop')
    acts = s.run(31.0)
    assert kinds(acts, 'mission') == [] and s.L.state in ('GOODBYE', 'SLEEPING')


def test_stop_ignored_when_not_driving():
    s = Sim()
    s.to_guiding()
    s.mission('SUCCEEDED')
    acts = s.choose('stop')
    assert s.L.state == 'ARRIVED' and kinds(acts, 'mission') == []


# ------------------------------------------------------------ failures
def test_failure_menu_retry_and_go_back():
    s = Sim()
    s.to_guiding('pharmacy')
    acts = s.mission('FAILED', 'nav_failed: Nav2 status 6')
    assert s.L.state == 'NAV_FAILED' and kinds(acts, 'say') == ['nav_failed']
    assert [o['id'] for o in last_ui(acts)['options']] == ['retry', 'go_back']
    acts = s.choose('retry')
    assert s.L.state == 'GUIDING' and kinds(acts, 'mission')[0]['target'] == 'pharmacy'
    s.mission('FAILED', 'nav_failed')
    acts = s.choose('go_back')
    assert kinds(acts, 'mission')[0]['mode'] == 'alone'


def test_not_armed_has_its_own_message():
    s = Sim()
    s.to_guiding()
    acts = s.mission('FAILED', 'not_armed: /robot_state is DISARMED')
    assert kinds(acts, 'say') == ['nav_not_armed']
    s2 = Sim()
    s2.to_guiding()
    assert kinds(s2.mission('FAILED', 'motors_off: /robot_state is ESTOP'), 'say') == \
        ['nav_not_armed']


def test_mission_node_silent_means_failure_not_hang():
    s = Sim(mission_answer_s=5.0)
    s.to_guiding()
    acts = s.run(5.5)
    assert s.L.state == 'NAV_FAILED' and 'nav_failed' in kinds(acts, 'say')


def test_status_of_other_trip_ignored():
    s = Sim()
    s.to_guiding()
    assert s.mission('SUCCEEDED', trip_id=999) == [] and s.L.state == 'GUIDING'


def test_failure_timeout_stays_put():
    s = Sim(failed_timeout_s=20.0)
    s.to_guiding()
    s.mission('FAILED', 'nav_failed')
    acts = s.run(21.0)
    assert kinds(acts, 'mission') == []


def test_failed_return_home_offers_retry_only():
    s = Sim()
    s.to_guiding()
    s.mission('SUCCEEDED')
    s.choose('go_back')
    acts = s.mission('FAILED', 'nav_failed')
    assert [o['id'] for o in last_ui(acts)['options']] == ['retry']


# ------------------------------------------------------------ staff commands
def test_sleep_command_cancels_the_trip():
    s = Sim()
    s.to_guiding()
    tid = s.L.trip['id']
    acts = s.apply(s.L.on_command(s.t, 'sleep'))
    assert {'cmd': 'cancel', 'id': tid} in kinds(acts, 'mission') and s.L.state == 'SLEEPING'


def test_home_command_from_sleep():
    s = Sim()
    acts = s.apply(s.L.on_command(5.0, 'home'))
    assert s.L.state == 'RETURNING' and kinds(acts, 'mission')[0]['mode'] == 'alone'


def test_home_command_ignored_while_talking_to_someone():
    s = Sim()
    s.L.on_head_state(0.0, 'WAIT_ARM')
    s.apply(s.L.on_tap(2.0))
    assert kinds(s.L.on_command(2.5, 'home'), 'mission') == []
