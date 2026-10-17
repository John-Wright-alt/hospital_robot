"""Unit tests for speech_logic.py (no ROS, no audio).

    python3 -m pytest test/ -v        (from inside the hospital_robot_hri folder)

All inputs here are hand-written test values, not sensor data.
"""

import os
import random

import pytest
import yaml

from hospital_robot_hri.audio_player import SOUNDS
from hospital_robot_hri.speech_logic import (
    PRIORITIES, FlagWatcher, GreetingDetector, GreetParams, MotionAnnounceParams,
    MotionAnnouncer, MotionDetector, Repeater, RobotStateWatcher, SpeechPlanner,
    SpeechQueue, Utterance, all_jobs, all_texts, in_quiet_hours, load_library,
    motors_can_drive)

PHRASES = os.path.join(os.path.dirname(__file__), '..', 'config', 'phrases.yaml')


def lib():
    with open(PHRASES) as f:
        return load_library(yaml.safe_load(f), known_sounds=SOUNDS)


def utt(key, prio, t=0.0, text='x', max_age=10.0, seq=0):
    return Utterance(key=key, priority=PRIORITIES[prio], text=text, sound=None,
                     created=t, max_age=max_age, seq=seq)


# ------------------------------------------------------------ the message library
def test_library_loads_and_every_event_the_node_uses_exists():
    L = lib()
    needed = ['startup', 'shutdown', 'greeting', 'thank_you', 'motors_armed', 'motors_off',
              'moving_start', 'moving_resume', 'moving_cue', 'moving_reminder', 'reversing',
              'estop', 'estop_released', 'fault', 'controller_lost', 'controller_restored',
              'battery_low', 'assistance', 'speaker_test']
    for key in needed:
        assert key in L, key


def test_safety_messages_are_plain():
    for e in lib().values():
        if e.priority in ('warning', 'critical'):
            assert not e.playful, e.key


def test_playful_on_a_warning_is_rejected():
    bad = {'events': {'x': {'priority': 'warning', 'phrases': ['a'], 'playful': ['ha']}}}
    with pytest.raises(ValueError, match='not allowed'):
        load_library(bad)


def test_unknown_sound_and_priority_rejected():
    with pytest.raises(ValueError, match='unknown sound'):
        load_library({'events': {'x': {'priority': 'info', 'phrases': ['a'], 'sound': 'kazoo'}}},
                     known_sounds=SOUNDS)
    with pytest.raises(ValueError, match='priority'):
        load_library({'events': {'x': {'priority': 'urgent', 'phrases': ['a']}}})


def test_wake_lines_are_the_requested_words_and_calm():
    L = lib()
    assert L['wake_intro'].phrases[0] == ("Hello, I am Anna. Your personal hospital guidance "
                                          "companion. Please wait so I can adjust for a more "
                                          "comfortable use.")
    assert L['adjust_done'].phrases[0] == ("The touchscreen is now positioned for your ergonomic "
                                           "comfort. How may I assist you today?")
    assert L['wake_intro'].length_scale > 1.1 and L['wake_intro'].sentence_silence >= 0.5
    u = SpeechPlanner(L, humour_chance=0.0).make('wake_intro', 0.0)
    assert u.length_scale == L['wake_intro'].length_scale and u.sound == 'wake'


def test_all_jobs_include_the_style():
    jobs = all_jobs(lib())
    assert any(j[1] is not None for j in jobs) and any(j[1] is None for j in jobs)
    assert len(jobs) == len(set(jobs))


def test_all_texts_has_no_duplicates():
    texts = all_texts(lib())
    assert len(texts) == len(set(texts)) and len(texts) > 20


# ------------------------------------------------------------ planner
def test_primary_phrase_without_humour():
    p = SpeechPlanner(lib(), humour_chance=0.0)
    u = p.make('moving_start', 0.0)
    assert u.text.startswith("Excuse me, I'm starting to move")
    assert u.sound == 'move' and u.priority == PRIORITIES['warning']


def test_cooldown_suppresses_repeats_but_force_does_not():
    p = SpeechPlanner(lib(), humour_chance=0.0)
    assert p.make('reversing', 0.0) is not None
    assert p.make('reversing', 3.0) is None           # cooldown 8 s
    assert p.make('reversing', 3.0, force=True) is not None
    assert p.make('reversing', 20.0) is not None


def test_humour_only_for_chatter_and_info():
    p = SpeechPlanner(lib(), humour_chance=1.0, rng=random.Random(1))
    L = lib()
    g = p.make('greeting', 0.0)
    assert g.text in L['greeting'].playful
    e = p.make('estop', 0.0)
    assert e.text == L['estop'].phrases[0]
    q = p.make('motors_off', 0.0, allow_humour=False)  # e.g. quiet hours
    assert q.text == L['motors_off'].phrases[0]


def test_rotation_cycles_reminders():
    p = SpeechPlanner(lib(), humour_chance=0.0)
    L = lib()['moving_reminder']
    said = [p.make('moving_reminder', 100.0 * i).text for i in range(4)]
    assert said == [L.phrases[0], L.phrases[1], L.phrases[2], L.phrases[0]]


def test_free_text_is_trimmed_and_never_critical():
    p = SpeechPlanner(lib())
    u = p.make_text('  hello   there ', 0.0, priority='critical')
    assert u.text == 'hello there' and u.priority == PRIORITIES['info']
    assert p.make_text('   ', 0.0) is None


# ------------------------------------------------------------ queue
def test_no_overlap_info_waits_for_info():
    q = SpeechQueue()
    playing = utt('a', 'info')
    assert q.offer(utt('b', 'info'), playing) == 'queued'


def test_info_interrupts_chatter_but_not_info():
    q = SpeechQueue()
    assert q.offer(utt('i', 'info'), utt('hello', 'chatter')) == 'interrupt'
    assert q.offer(utt('i2', 'info'), utt('i0', 'info')) == 'queued'


def test_warning_interrupts_info_and_chatter():
    q = SpeechQueue()
    assert q.offer(utt('w', 'warning'), utt('c', 'chatter')) == 'interrupt'
    assert q.offer(utt('w2', 'warning'), utt('i', 'info')) == 'interrupt'


def test_warning_does_not_interrupt_warning_or_critical():
    q = SpeechQueue()
    assert q.offer(utt('w', 'warning'), utt('w0', 'warning')) == 'queued'
    assert q.offer(utt('w3', 'warning'), utt('c', 'critical')) == 'queued'


def test_critical_jumps_the_queue():
    q = SpeechQueue()
    q.offer(utt('chat', 'chatter', t=0), None)
    q.offer(utt('info', 'info', t=1), utt('x', 'info'))
    q.offer(utt('crit', 'critical', t=2), utt('x', 'info'))
    assert q.pop(2.0).key == 'crit'


def test_greeting_never_blocks_a_safety_message():
    """A queued greeting must not delay a fault: the fault plays first."""
    q = SpeechQueue()
    q.offer(utt('greeting', 'chatter', t=0), None)
    assert q.offer(utt('fault', 'critical', t=0.5), None) == 'queued'
    assert q.pop(0.5).key == 'fault'


def test_cue_and_chatter_dropped_when_busy():
    q = SpeechQueue()
    busy = utt('x', 'info')
    assert q.offer(utt('tick', 'cue'), busy) == 'dropped'
    assert q.offer(utt('hi', 'chatter'), busy) == 'dropped'
    assert q.offer(utt('tick', 'cue'), None) == 'queued'


def test_duplicate_waiting_message_dropped():
    q = SpeechQueue()
    q.offer(utt('a', 'info', text='same'), utt('x', 'info'))
    assert q.offer(utt('a', 'info', text='same'), utt('x', 'info')) == 'dropped'


def test_stale_messages_expire():
    q = SpeechQueue()
    q.offer(utt('old', 'info', t=0.0, max_age=5.0), utt('x', 'info'))
    q.offer(utt('new', 'info', t=8.0, max_age=5.0), utt('x', 'info'))
    assert q.pop(9.0).key == 'new'
    assert [u.key for u in q.expired] == ['old']


def test_full_queue_evicts_lowest_priority():
    q = SpeechQueue(max_len=2)
    busy = utt('x', 'critical')
    q.offer(utt('i1', 'info', t=0), busy)
    q.offer(utt('i2', 'info', t=1), busy)
    assert q.offer(utt('c', 'critical', t=2), busy) == 'queued'
    assert sorted(u.key for u in q.items) == ['c', 'i1']
    assert q.offer(utt('i3', 'info', t=3), busy) == 'dropped'


def test_critical_waits_its_turn_behind_critical():
    q = SpeechQueue()
    assert q.offer(utt('fault', 'critical'), utt('estop', 'critical')) == 'queued'


# ------------------------------------------------------------ motion
def feed(det, t0, t1, v, w, dt=0.02):
    changes, t = [], t0
    while t <= t1 + 1e-9:
        c = det.update(t, v, w)
        if c:
            changes.append((round(t, 2), c))
        t += dt
    return changes


def test_noise_at_rest_never_counts_as_moving():
    d = MotionDetector()
    t = 0.0
    rng = random.Random(0)
    for _ in range(500):
        d.update(t, rng.uniform(-0.01, 0.01), rng.uniform(-0.05, 0.05))
        t += 0.02
    assert d.state == 'stopped'


def test_start_needs_start_s_and_stop_needs_stop_s():
    d = MotionDetector()
    feed(d, 0.0, 0.5, 0.0, 0.0)
    ch = feed(d, 0.52, 1.0, 0.15, 0.0)
    assert ch and ch[0][1] == 'forward' and ch[0][0] >= 0.52 + 0.25 - 0.03
    ch = feed(d, 1.02, 1.8, 0.0, 0.0)
    assert ch == []                              # < stop_s still
    ch = feed(d, 1.82, 2.5, 0.0, 0.0)
    assert ch and ch[0][1] == 'stopped'


def test_single_spike_is_ignored():
    d = MotionDetector()
    feed(d, 0.0, 0.2, 0.0, 0.0)
    d.update(0.22, 0.5, 0.0)                     # one bad sample
    feed(d, 0.24, 1.0, 0.0, 0.0)
    assert d.state == 'stopped'


def test_reverse_and_turning_classified():
    d = MotionDetector()
    feed(d, 0.0, 0.1, 0.0, 0.0)
    assert ('reverse' in [c for _, c in feed(d, 0.12, 0.6, -0.1, 0.0)])
    d2 = MotionDetector()
    feed(d2, 0.0, 0.1, 0.0, 0.0)
    feed(d2, 0.12, 0.6, 0.0, 0.5)
    assert d2.state == 'turning' and d2.turn > 0


def test_odom_stops_arriving_means_unknown():
    d = MotionDetector()
    feed(d, 0.0, 0.1, 0.0, 0.0)
    assert d.tick(0.3) is None
    assert d.tick(1.0) == 'unknown'


def test_motion_announcer_sequence():
    a = MotionAnnouncer(MotionAnnounceParams(restart_quiet_s=10, cue_interval_s=6,
                                             reminder_interval_s=45))
    assert a.on_change(0.0, 'stopped', 'forward') == ['moving_start']
    assert a.tick(1.0) == []
    assert a.tick(6.0) == ['moving_cue']
    assert 'moving_reminder' in a.tick(45.0)
    assert a.on_change(50.0, 'forward', 'stopped') == []
    assert a.tick(60.0) == []                    # stopped: no cues
    assert a.on_change(55.0, 'stopped', 'forward') == ['moving_resume']   # soon after stop
    assert a.on_change(56.0, 'forward', 'reverse') == ['reversing']
    a.on_change(57.0, 'reverse', 'stopped')
    assert a.on_change(58.0, 'stopped', 'reverse') == ['reversing']        # not 'resume' too
    a.on_change(59.0, 'reverse', 'stopped')
    assert a.on_change(80.0, 'stopped', 'forward') == ['moving_start']


def test_journey_start_suppresses_double_announcement():
    a = MotionAnnouncer()
    a.suppress_start_speech(0.0, 15.0)
    assert a.on_change(3.0, 'stopped', 'forward') == ['moving_resume']


def test_repeater():
    r = Repeater(30.0)
    assert not r.due(0.0)
    r.start(0.0)
    assert not r.due(10.0)
    assert r.due(30.0)
    r.stop()
    assert not r.due(100.0)
    assert not Repeater(0.0).due(5.0)


# ------------------------------------------------------------ robot state
def test_robot_state_events():
    w = RobotStateWatcher(timeout_s=3.0)
    assert w.on_state(0.0, 'DISARMED') == []
    assert w.on_state(0.5, 'ARMING') == []
    assert w.on_state(1.0, 'ARMED') == ['motors_armed']
    assert w.on_state(1.5, 'ARMED') == []        # repeated status: nothing
    assert w.on_state(2.0, 'ESTOP') == ['estop']
    assert w.on_state(3.0, 'DISARMED') == ['estop_released']
    w.on_state(4.0, 'ARMED')
    assert w.on_state(5.0, 'DISARMING') == ['motors_off']
    assert w.on_state(5.5, 'DISARMED') == []
    assert w.on_state(6.0, 'FAULT') == ['fault']


def test_controller_lost_only_after_it_was_seen():
    w = RobotStateWatcher(timeout_s=3.0)
    assert w.tick(100.0) == []                   # never seen (no pico_bridge): silent
    w.on_state(100.0, 'DISARMED')
    assert w.tick(102.0) == []
    assert w.tick(103.5) == ['controller_lost']
    assert w.tick(110.0) == []                   # said once
    assert w.on_state(111.0, 'DISARMED') == ['controller_restored']


def test_battery_flag_edges():
    f = FlagWatcher('BATT_WARN')
    assert f.on_text('ARMED  flags=0004 (CMD_TIMEOUT)  vbat=12.10 V  vmot=12.0 V') is None
    assert f.on_text('ARMED  flags=0008 (BATT_WARN)  vbat=11.05 V  vmot=11.0 V') is True
    assert f.on_text('ARMED  flags=0008 (BATT_WARN)  vbat=11.04 V  vmot=11.0 V') is None
    assert f.on_text('ARMED  flags=0000 (none)  vbat=11.20 V  vmot=11.1 V') is False


# ------------------------------------------------------------ greeting
def person(tid, rng=1.0, state='tracking'):
    return {'state': state, 'track_id': tid, 'range_m': rng}


def test_greeting_needs_dwell_close_range_and_respects_cooldown():
    g = GreetingDetector(GreetParams(range_m=1.2, dwell_s=2.0, cooldown_s=90.0))
    assert not g.on_gaze(0.0, person(1), True)
    assert not g.on_gaze(1.0, person(1), True)
    assert g.on_gaze(2.1, person(1), True)       # stood 2 s at 1 m
    assert not g.on_gaze(3.0, person(1), True)   # same person: once
    g.on_gaze(10.0, person(2), True)
    assert not g.on_gaze(13.0, person(2), True)  # cooldown
    g.on_gaze(100.0, person(3), True)
    assert g.on_gaze(102.5, person(3), True)


def test_no_greeting_when_far_walking_past_or_not_allowed():
    g = GreetingDetector()
    g.on_gaze(0.0, person(1, rng=3.0), True)
    assert not g.on_gaze(5.0, person(1, rng=3.0), True)
    g.on_gaze(10.0, person(2), False)            # e.g. robot moving
    assert not g.on_gaze(13.0, person(2), False)


def test_quiet_hours():
    assert in_quiet_hours(23, 22, 6) and in_quiet_hours(3, 22, 6)
    assert not in_quiet_hours(12, 22, 6)
    assert in_quiet_hours(13, 12, 14) and not in_quiet_hours(14, 12, 14)
    assert not in_quiet_hours(23, -1, 6)


def test_motors_can_drive_only_false_for_explicit_no_power_states():
    # no motor power: /odom speed cannot be the robot driving itself
    for s in ('DISARMED', 'ESTOP', 'FAULT', ' disarmed '):
        assert not motors_can_drive(s)
    # powered, or unknown: trust /odom (never hide a real warning)
    for s in ('ARMED', 'ARMING', 'DISARMING', None, '', 'SOMETHING_NEW'):
        assert motors_can_drive(s)
