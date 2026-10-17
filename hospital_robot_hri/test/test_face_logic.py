"""Unit tests for face_logic.py and face_http.py (no ROS needed).

    python3 -m pytest test/ -v        (from inside the hospital_robot_hri folder)

All gaze messages here are hand-written test inputs, not sensor data.
"""

import json
import os
import urllib.request

import pytest

from hospital_robot_hri.face_logic import FaceLogic, FaceParams
from hospital_robot_hri.face_http import FaceHttpServer

WEB = os.path.join(os.path.dirname(__file__), '..', 'web')


def lidar(bearing, tid=1, state='tracking'):
    return {'source': 'lidar', 'simulated': False, 'state': state,
            'bearing_deg': bearing, 'range_m': 1.0, 'track_id': tid}


def test_no_data_means_neutral_gaze():
    f = FaceLogic()
    out = f.update(0.0)
    assert out['gaze'] == {'x': 0.0, 'y': 0.0, 'source': 'none'}
    assert out['gaze_data'] == 'no_data'
    assert out['expression'] == 'idle'


def test_robot_left_is_viewer_right():
    f = FaceLogic()
    f.on_gaze_status(lidar(+45.0), 0.0)
    assert f.update(0.1)['gaze']['x'] == pytest.approx(0.5)     # robot's left -> viewer's right
    f.on_gaze_status(lidar(-45.0), 0.2)
    assert f.update(0.3)['gaze']['x'] == pytest.approx(-0.5)


def test_mirror_false_flips_the_sign():
    f = FaceLogic(FaceParams(mirror=False))
    f.on_gaze_status(lidar(+45.0), 0.0)
    assert f.update(0.1)['gaze']['x'] == pytest.approx(-0.5)


def test_bearing_is_clamped():
    f = FaceLogic()
    f.on_gaze_status(lidar(170.0), 0.0)          # behind the robot
    assert f.update(0.1)['gaze']['x'] == 1.0


def test_stale_data_returns_to_neutral():
    f = FaceLogic()
    f.on_gaze_status(lidar(30.0), 0.0)
    assert f.update(0.5)['gaze']['source'] == 'lidar'
    out = f.update(1.6)                          # > gaze_timeout_s after the last message
    assert out['gaze']['source'] == 'none' and out['gaze_data'] == 'no_data'


def test_new_object_makes_eyes_curious_once():
    f = FaceLogic()
    f.on_gaze_status(lidar(20.0, tid=1), 0.0)
    assert f.update(0.0)['expression'] == 'curious'
    f.on_gaze_status(lidar(20.0, tid=1), 1.0)
    assert f.update(1.0)['expression'] == 'curious'   # still within curious_s
    f.on_gaze_status(lidar(20.0, tid=1), 2.0)
    assert f.update(2.0)['expression'] == 'idle'      # same object: no new reaction


def test_curious_cooldown():
    f = FaceLogic()
    f.on_gaze_status(lidar(20.0, tid=1), 0.0)
    f.update(0.0)
    f.on_gaze_status(lidar(-20.0, tid=2), 2.0)        # new object, but only 2 s later
    assert f.update(2.0)['expression'] == 'idle'
    f.on_gaze_status(lidar(-20.0, tid=3), 5.0)        # after the 4 s cooldown
    assert f.update(5.0)['expression'] == 'curious'


def test_gaze_ignored_while_assistance_and_no_curious_leak():
    f = FaceLogic()
    f.on_gaze_status(lidar(20.0, tid=1), 0.0)
    assert f.update(0.0)['expression'] == 'curious'
    assert f.set_expression('assistance')
    f.on_gaze_status(lidar(60.0, tid=2), 0.5)
    out = f.update(0.5)
    assert out['expression'] == 'assistance'           # the expression owns the eyes
    assert out['gaze']['source'] == 'none'
    assert f.set_expression('idle')
    assert f.update(0.6)['expression'] == 'idle'      # the old curious did not come back


def test_unknown_expression_rejected():
    f = FaceLogic()
    assert not f.set_expression('dancing')
    assert f.update(0.0)['expression'] == 'idle'


def test_simulated_flag_is_passed_through():
    f = FaceLogic()
    msg = lidar(10.0)
    msg.update(source='test', simulated=True)
    f.on_gaze_status(msg, 0.0)
    out = f.update(0.1)
    assert out['simulated'] is True and out['gaze']['source'] == 'test'


def test_learning_restart_clears_seen_ids():
    f = FaceLogic()
    f.on_gaze_status(lidar(10.0, tid=1), 0.0)
    f.update(0.0)
    f.on_gaze_status({'source': 'lidar', 'state': 'learning'}, 10.0)   # gaze node reset
    f.on_gaze_status(lidar(10.0, tid=1), 12.0)                          # ID 1 again, new object
    assert f.update(12.0)['expression'] == 'curious'


def test_bad_bearing_is_ignored():
    f = FaceLogic()
    f.on_gaze_status(lidar('nan'), 0.0)
    assert f.update(0.1)['gaze']['source'] == 'none'
    f.on_gaze_status(lidar(None), 0.2)
    assert f.update(0.3)['gaze']['source'] == 'none'


def test_http_server_serves_page_state_and_events():
    srv = FaceHttpServer(WEB, '127.0.0.1', 0)          # port 0 = any free port
    srv.start()
    try:
        base = f'http://127.0.0.1:{srv.port}'
        page = urllib.request.urlopen(base + '/', timeout=2).read()
        assert b'<canvas' in page
        srv.publish({'expression': 'happy'})
        assert json.loads(urllib.request.urlopen(base + '/state', timeout=2).read()) == \
            {'expression': 'happy'}
        with urllib.request.urlopen(base + '/events', timeout=2) as ev:
            line = ev.readline().decode()
            assert line.startswith('data: ') and 'happy' in line
        with pytest.raises(urllib.error.HTTPError):
            urllib.request.urlopen(base + '/../setup.py', timeout=2)
    finally:
        srv.stop()


# ------------------------------------------------------------ Phase 9 speech
def speech(**kw):
    s = {'speaking': False, 'text': '', 'priority': '', 'motion': 'stopped', 'turn': 0.0,
         'audio_ok': True, 'simulated': False}
    s.update(kw)
    return s


def test_without_speech_node_nothing_changes():
    out = FaceLogic().update(0.0)
    assert out['caption'] == '' and out['motion'] == 'unknown' and out['speaking'] is False
    assert out['audio_ok'] is True


def test_caption_and_speaking_expression():
    f = FaceLogic()
    f.on_speech_status(speech(speaking=True, text='Hello!', priority='chatter'), 0.0)
    out = f.update(0.1)
    assert out['caption'] == 'Hello!' and out['expression'] == 'speaking'
    f.on_speech_status(speech(), 0.5)
    assert f.update(0.6)['caption'] == ''


def test_speaking_does_not_override_assistance():
    f = FaceLogic()
    f.set_expression('assistance')
    f.on_speech_status(speech(speaking=True, text='I need help', priority='warning'), 0.0)
    out = f.update(0.1)
    assert out['expression'] == 'assistance' and out['caption'] == 'I need help'


def test_stale_speech_status_clears_caption_and_banner():
    f = FaceLogic()
    f.on_speech_status(speech(speaking=True, text='Hi', motion='forward'), 0.0)
    out = f.update(3.0)                           # > speech_timeout_s (2 s)
    assert out['caption'] == '' and out['motion'] == 'unknown'


def test_while_moving_eyes_look_where_robot_turns_not_at_lidar():
    f = FaceLogic()
    f.on_gaze_status(lidar(-60.0, tid=5), 0.0)
    f.on_speech_status(speech(motion='turning', turn=1.0), 0.0)
    out = f.update(0.1)
    assert out['gaze']['source'] == 'motion'
    assert out['gaze']['x'] == pytest.approx(0.8)  # turning to robot's left -> viewer's right
    assert out['expression'] == 'idle'             # no 'curious' while driving
    assert 'bearing_deg' not in out


def test_motion_gaze_can_be_switched_off():
    f = FaceLogic(FaceParams(motion_gaze=False))
    f.on_gaze_status(lidar(30.0), 0.0)
    f.on_speech_status(speech(motion='forward', turn=0.5), 0.0)
    assert f.update(0.1)['gaze']['source'] == 'lidar'


# ------------------------------------------------------------ Anna's wake-up flow
def test_ui_defaults_to_plain_eyes_and_times_out():
    f = FaceLogic()
    assert f.update(0.0)['ui']['mode'] == 'eyes'
    f.on_ui({'mode': 'sleep'}, 0.0)
    assert f.update(1.0)['ui']['mode'] == 'sleep'
    assert f.update(5.0)['ui']['mode'] == 'eyes'              # hri_interaction gone


def test_ui_menu_options_are_cleaned():
    f = FaceLogic()
    f.on_ui({'mode': 'menu', 'options': [{'id': 'pharmacy', 'label': 'Pharmacy'}, 'junk']}, 0.0)
    assert f.update(0.1)['ui']['options'] == [{'id': 'pharmacy', 'label': 'Pharmacy'}]
    f.on_ui({'mode': 'dance'}, 0.2)                            # unknown mode ignored
    assert f.update(0.3)['ui']['mode'] == 'menu'


def test_searching_expression_exists_and_owns_the_eyes():
    f = FaceLogic()
    assert f.set_expression('searching')
    f.on_gaze_status(lidar(40.0), 0.0)
    out = f.update(0.1)
    assert out['expression'] == 'searching' and out['gaze']['source'] == 'none'


def post(base, body):
    req = urllib.request.Request(base + '/touch', data=body, method='POST',
                                 headers={'Content-Type': 'application/json'})
    return urllib.request.urlopen(req, timeout=2).status


def test_touch_is_forwarded_and_validated():
    got = []
    srv = FaceHttpServer(WEB, '127.0.0.1', 0, on_touch=got.append)
    srv.start()
    try:
        base = f'http://127.0.0.1:{srv.port}'
        assert post(base, b'{"kind": "tap"}') == 204
        assert post(base, b'{"kind": "choice", "value": "pharmacy"}') == 204
        for bad in (b'{"kind": "arm"}', b'{"kind": "choice", "value": "../x"}', b'nonsense'):
            with pytest.raises(urllib.error.HTTPError):
                post(base, bad)
        assert got == [{'kind': 'tap'}, {'kind': 'choice', 'value': 'pharmacy'}]
    finally:
        srv.stop()


def test_touch_disabled_without_callback_and_remote_check():
    srv = FaceHttpServer(WEB, '127.0.0.1', 0)
    srv.start()
    try:
        with pytest.raises(urllib.error.HTTPError):
            post(f'http://127.0.0.1:{srv.port}', b'{"kind": "tap"}')
    finally:
        srv.stop()
    assert '192.168.1.20' not in FaceHttpServer.LOCAL and '127.0.0.1' in FaceHttpServer.LOCAL


def test_touch_from_another_computer_is_refused(monkeypatch):
    got = []
    srv = FaceHttpServer(WEB, '127.0.0.1', 0, on_touch=got.append)
    monkeypatch.setattr(srv, 'LOCAL', ())                      # pretend 127.0.0.1 is remote
    srv.start()
    try:
        with pytest.raises(urllib.error.HTTPError) as e:
            post(f'http://127.0.0.1:{srv.port}', b'{"kind": "tap"}')
        assert e.value.code == 403 and got == []
        srv.allow_remote_touch = True                          # testing switch
        assert post(f'http://127.0.0.1:{srv.port}', b'{"kind": "tap"}') == 204
    finally:
        srv.stop()
