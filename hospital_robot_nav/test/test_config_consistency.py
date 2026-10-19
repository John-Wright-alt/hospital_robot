"""Checks on the Phase 7 YAML files that would otherwise only show up as a
silent misbehaviour on the robot (no ROS needed)."""
import os

import yaml

HERE = os.path.dirname(__file__)
P = yaml.safe_load(open(os.path.join(HERE, '..', 'config', 'nav2_params_real.yaml')))


def rp(node, inner=None):
    return (P[node][inner] if inner else P[node])['ros__parameters']


def test_every_cmd_vel_node_is_stamped():
    # pico_bridge subscribes TwistStamped; Jazzy's default is unstamped
    for n in ('controller_server', 'behavior_server', 'velocity_smoother',
              'collision_monitor', 'docking_server'):
        assert rp(n)['enable_stamped_cmd_vel'] is True, n


def test_collision_monitor_chain():
    cm = rp('collision_monitor')
    assert cm['cmd_vel_in_topic'] == 'cmd_vel_smoothed' and cm['cmd_vel_out_topic'] == 'cmd_vel'
    for s in cm['observation_sources']:
        assert s in cm, s


def test_inflation_factor_matches_rpp():
    rpp = rp('controller_server')['FollowPath']
    assert rpp['allow_reversing'] is False
    for inner in ('local_costmap', 'global_costmap'):
        c = rp(inner, inner)
        assert c['inflation_layer']['cost_scaling_factor'] == rpp['inflation_cost_scaling_factor']


def test_all_scan_users_use_filtered_scan():
    assert rp('amcl')['scan_topic'] == 'scan_filtered'
    assert rp('local_costmap', 'local_costmap')['obstacle_layer']['scan']['topic'] == '/scan_filtered'
    assert rp('global_costmap', 'global_costmap')['obstacle_layer']['scan']['topic'] == '/scan_filtered'
    assert rp('collision_monitor')['scan']['topic'] == 'scan_filtered'
    slam = yaml.safe_load(open(os.path.join(HERE, '..', 'config', 'slam_mapping.yaml')))
    assert slam['slam_toolbox']['ros__parameters']['scan_topic'] == '/scan_filtered'


def test_range_layer_topics_match_bridge_defaults():
    lc = rp('local_costmap', 'local_costmap')
    assert 'range_layer' in lc['plugins']
    assert lc['range_layer']['topics'] == ['/ultrasonic/left', '/ultrasonic/front',
                                           '/ultrasonic/right']


def test_speed_ceiling_below_firmware_clamp():
    # Pico: 9 rad/s * 0.072 m = 0.648 m/s
    assert rp('velocity_smoother')['max_velocity'][0] <= 0.65
    mission = yaml.safe_load(open(os.path.join(HERE, '..', 'config', 'mission.yaml')))
    m = mission['mission']['ros__parameters']
    assert m['guide_speed'] <= rp('velocity_smoother')['max_velocity'][0]
    assert m['require_armed'] is True and m['require_parked'] is True


def test_footprint_inside_body_filter_box():
    import ast
    fp = ast.literal_eval(rp('local_costmap', 'local_costmap')['footprint'])
    xs, ys = [p[0] for p in fp], [p[1] for p in fp]
    # scan_body_filter default box: x +/-0.23, y +/-0.26
    assert max(xs) <= 0.23 and min(xs) >= -0.23 and max(ys) <= 0.26 and min(ys) >= -0.26
    # collision polygons start at/after the front edge (robot never "sees itself")
    cm = rp('collision_monitor')
    for poly in ('PolygonStop', 'PolygonSlow'):
        pts = ast.literal_eval(cm[poly]['points'])
        assert min(p[0] for p in pts) >= max(xs)
