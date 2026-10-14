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
