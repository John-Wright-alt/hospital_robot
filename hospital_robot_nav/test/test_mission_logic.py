"""Unit tests for mission_logic.py -- no ROS needed:  python3 -m pytest test/ -v"""
from hospital_robot_nav.mission_logic import (GOAL_ABORTED, GOAL_CANCELED, GOAL_SUCCEEDED,
                                              MissionLogic, MissionParams, parse_head_state)

WP = {'home': (0.0, 0.0, 0.0), 'pharmacy': (4.0, 1.0, 1.57)}


def kinds(acts):
    return [k for k, _ in acts]


def status(acts):
    return [v for k, v in acts if k == 'status'][-1]


def ready(t=0.0, **kw):
    m = MissionLogic(MissionParams(**kw))
    m.on_robot_state(t, 'ARMED')
    m.on_head_state(t, 'PARKED')
    return m


def test_head_state_parsing():
    assert parse_head_state('PARKED') == 'PARKED'
    assert parse_head_state('{"state": "tracking"}') == 'TRACKING'
    assert parse_head_state('state=PARKING x=1') == 'PARKING'
    assert parse_head_state('STARTING') == 'STARTING'
    assert parse_head_state('hello') is None
    assert parse_head_state(None) is None


def test_go_sends_goal_with_guide_speed():
    m = ready()
    acts = m.on_command(0.1, {'cmd': 'go', 'target': 'pharmacy', 'id': 3}, WP)
    goal = [v for k, v in acts if k == 'goal'][0]
    assert goal['x'] == 4.0 and goal['speed'] == 0.25
    assert ('speed', 0.25) in acts
    assert status(acts)['state'] == 'STARTING' and status(acts)['id'] == 3


def test_alone_mode_uses_alone_speed():
    m = ready()
    acts = m.on_command(0.1, {'cmd': 'go', 'target': 'home', 'mode': 'alone'}, WP)
    assert [v for k, v in acts if k == 'goal'][0]['speed'] == 0.35


def test_success_path():
    m = ready()
    m.on_command(0.1, {'cmd': 'go', 'target': 'pharmacy', 'id': 1}, WP)
    assert status(m.on_goal_response(0.2, True))['state'] == 'ACTIVE'
    m.on_feedback(0.5, 3.2)
    assert m.remaining == 3.2
    acts = m.on_result(5.0, GOAL_SUCCEEDED)
    assert status(acts)['state'] == 'SUCCEEDED'
    assert m.state == 'IDLE'


def test_unknown_target_fails_without_goal():
    m = ready()
    acts = m.on_command(0.1, {'cmd': 'go', 'target': 'canteen'}, WP)
    assert 'goal' not in kinds(acts)
    assert status(acts)['state'] == 'FAILED' and 'unknown_target' in status(acts)['reason']


def test_never_drives_when_not_armed():
    m = MissionLogic()
    m.on_head_state(0, 'PARKED')
    m.on_robot_state(0, 'DISARMED')
    acts = m.on_command(0.1, {'cmd': 'go', 'target': 'pharmacy'}, WP)
    assert 'goal' not in kinds(acts) and 'not_armed' in status(acts)['reason']


def test_stale_robot_state_counts_as_not_armed():
    m = ready(t=0.0)
    acts = m.on_command(10.0, {'cmd': 'go', 'target': 'pharmacy'}, WP)
    assert 'goal' not in kinds(acts)


def test_no_head_state_refuses_unless_optional():
    m = MissionLogic()
    m.on_robot_state(0, 'ARMED')
    acts = m.on_command(0.1, {'cmd': 'go', 'target': 'pharmacy'}, WP)
    assert 'goal' not in kinds(acts) and 'head_not_parked' in status(acts)['reason']
    m2 = MissionLogic(MissionParams(head_optional=True))
    m2.on_robot_state(0, 'ARMED')
    assert 'goal' in kinds(m2.on_command(0.1, {'cmd': 'go', 'target': 'pharmacy'}, WP))


def test_parks_column_first_then_drives():
    m = ready()
    m.on_head_state(0, 'TRACKING')
    acts = m.on_command(0.1, {'cmd': 'go', 'target': 'pharmacy'}, WP)
    assert ('head', 'park') in acts and 'goal' not in kinds(acts)
    assert m.state == 'PARKING'
    m.on_robot_state(1.0, 'ARMED')
    assert m.tick(1.0) == []
    m.on_head_state(2.0, 'PARKED')
    m.on_robot_state(2.0, 'ARMED')
    assert 'goal' in kinds(m.tick(2.0))


def test_park_timeout_fails():
    m = ready()
    m.on_head_state(0, 'TRACKING')
    m.on_command(0.1, {'cmd': 'go', 'target': 'pharmacy'}, WP)
    m.on_robot_state(50.0, 'ARMED')
    acts = m.tick(50.0)
    assert status(acts)['state'] == 'FAILED' and 'goal' not in kinds(acts)


def test_disarm_while_driving_cancels():
    m = ready()
    m.on_command(0.1, {'cmd': 'go', 'target': 'pharmacy'}, WP)
    m.on_goal_response(0.2, True)
    m.on_robot_state(1.0, 'DISARMED')
    acts = m.tick(1.0)
    assert ('cancel', None) in acts and m.state == 'CANCELING'
    acts = m.on_result(1.5, GOAL_CANCELED)
    assert status(acts)['state'] == 'FAILED' and 'motors_off' in status(acts)['reason']


def test_column_leaving_parked_cancels():
    m = ready()
    m.on_command(0.1, {'cmd': 'go', 'target': 'pharmacy'}, WP)
    m.on_goal_response(0.2, True)
    m.on_head_state(1.0, 'HOMING')
    m.on_robot_state(1.0, 'ARMED')
    assert ('cancel', None) in m.tick(1.0)


def test_user_cancel():
    m = ready()
    m.on_command(0.1, {'cmd': 'go', 'target': 'pharmacy', 'id': 5}, WP)
    m.on_goal_response(0.2, True)
    assert m.on_command(0.3, {'cmd': 'cancel', 'id': 99}, WP)[0][0] == 'log'   # other id
    acts = m.on_command(0.3, {'cmd': 'cancel', 'id': 5}, WP)
    assert ('cancel', None) in acts
    assert status(m.on_result(0.5, GOAL_CANCELED))['state'] == 'CANCELED'


def test_cancel_not_confirmed_times_out():
    m = ready()
    m.on_command(0.1, {'cmd': 'go', 'target': 'pharmacy'}, WP)
    m.on_goal_response(0.2, True)
    m.on_command(0.3, {'cmd': 'cancel'}, WP)
    acts = m.tick(6.0)
    assert status(acts)['state'] == 'CANCELED' and m.state == 'IDLE'


def test_rejected_and_aborted():
    m = ready()
    m.on_command(0.1, {'cmd': 'go', 'target': 'pharmacy'}, WP)
    assert status(m.on_goal_response(0.2, False))['state'] == 'FAILED'
    m.on_robot_state(1.0, 'ARMED')
    m.on_command(1.0, {'cmd': 'go', 'target': 'pharmacy'}, WP)
    m.on_goal_response(1.1, True)
    acts = m.on_result(3.0, GOAL_ABORTED)
    assert status(acts)['state'] == 'FAILED' and 'nav_failed' in status(acts)['reason']


def test_start_timeout():
    m = ready()
    m.on_command(0.1, {'cmd': 'go', 'target': 'pharmacy'}, WP)
    m.on_robot_state(11.0, 'ARMED')
    acts = m.tick(11.0)
    assert ('cancel', None) in acts and 'start_timeout' in m._cancel_reason


def test_new_goal_replaces_old():
    m = ready()
    m.on_command(0.1, {'cmd': 'go', 'target': 'pharmacy', 'id': 1}, WP)
    m.on_goal_response(0.2, True)
    acts = m.on_command(0.5, {'cmd': 'go', 'target': 'home', 'mode': 'alone', 'id': 2}, WP)
    assert kinds(acts).index('cancel') < kinds(acts).index('goal')
    assert status(acts)['id'] == 2 and status(acts)['target'] == 'home'


def test_late_messages_ignored_when_idle():
    m = ready()
    assert m.on_goal_response(0.1, True) == []
    assert m.on_result(0.1, GOAL_SUCCEEDED) == []


def test_head_starting_or_wait_arm_refuses_without_park_request():
    for st in ('STARTING', 'WAIT_ARM', 'FAULT'):
        m = ready()
        m.on_head_state(0, st)
        acts = m.on_command(0.1, {'cmd': 'go', 'target': 'pharmacy'}, WP)
        assert 'goal' not in kinds(acts) and ('head', 'park') not in acts, st
        assert 'head_not_parked' in status(acts)['reason']
