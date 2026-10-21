import math

import pytest

from hospital_robot_nav import waypoints as wp


def test_save_and_load_roundtrip(tmp_path):
    f = tmp_path / 'maps' / 'waypoints.yaml'
    wp.save(str(f), 'home', 1.0, 2.0, math.radians(90), label='Reception')
    wp.save(str(f), 'Pharmacy', 3.0, -1.0, 0.0)
    w = wp.load(str(f))
    assert w['home'][0] == 1.0 and abs(w['home'][2] - math.pi / 2) < 1e-3
    assert 'pharmacy' in w
    assert wp.missing(w) == ['doctor', 'xray']
    assert (tmp_path / 'maps' / 'waypoints.yaml.bak').exists()


def test_label_kept_when_resaved(tmp_path):
    f = str(tmp_path / 'w.yaml')
    wp.save(f, 'xray', 0, 0, 0, label='X-ray')
    wp.save(f, 'xray', 1, 1, 0)
    import yaml
    assert yaml.safe_load(open(f))['waypoints']['xray']['label'] == 'X-ray'


def test_bad_inputs(tmp_path):
    with pytest.raises(wp.WaypointError):
        wp.load(str(tmp_path / 'nope.yaml'))
    with pytest.raises(wp.WaypointError):
        wp.save(str(tmp_path / 'w.yaml'), 'bad name!', 0, 0, 0)
    f = tmp_path / 'b.yaml'
    f.write_text('frame_id: map\nwaypoints:\n  home: {x: 1}\n')
    with pytest.raises(wp.WaypointError):
        wp.load(str(f))
    f.write_text('frame_id: odom\nwaypoints: {}\n')
    with pytest.raises(wp.WaypointError):
        wp.load(str(f))


def test_template_loads():
    import os
    here = os.path.dirname(__file__)
    w = wp.load(os.path.join(here, '..', 'config', 'waypoints_template.yaml'))
    assert wp.missing(w) == []
