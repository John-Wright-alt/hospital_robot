"""
waypoints.py  --  named places on the map, stored in a small YAML file that you
edit or fill with `ros2 run hospital_robot_nav save_waypoint <name>`.
NO ROS code (unit tested in test/test_waypoints.py).

File format (~/robot_ws/maps/<site>/waypoints.yaml, e.g. site = apartment):

    frame_id: map
    waypoints:
      home:     {x: 0.00, y: 0.00, yaw_deg: 0.0, label: "Reception"}
      doctor:   {x: 4.20, y: 1.10, yaw_deg: 90.0, label: "Doctor's office"}
      xray:     {...}
      pharmacy: {...}

The names doctor / xray / pharmacy are the ids Anna's menu sends
(hospital_robot_hri interaction_logic.DESTINATIONS). 'home' is where the
robot starts, returns to, and where AMCL's initial pose is taken from.
"""

import math
import os
import time

import yaml

REQUIRED = ('home', 'doctor', 'xray', 'pharmacy')


class WaypointError(Exception):
    pass


def load(path):
    """-> {name: (x, y, yaw_rad)}. Raises WaypointError with a clear message."""
    path = os.path.expanduser(path)
    if not os.path.isfile(path):
        raise WaypointError(f'waypoint file not found: {path}')
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    if data.get('frame_id', 'map') != 'map':
        raise WaypointError(f'{path}: frame_id must be "map"')
    wps = data.get('waypoints') or {}
    out = {}
    for name, w in wps.items():
        try:
            out[str(name).lower()] = (float(w['x']), float(w['y']),
                                      math.radians(float(w.get('yaw_deg', 0.0))))
        except (KeyError, TypeError, ValueError) as err:
            raise WaypointError(f'{path}: waypoint "{name}" needs numbers x, y, yaw_deg ({err})')
        if not all(math.isfinite(v) for v in out[str(name).lower()]):
            raise WaypointError(f'{path}: waypoint "{name}" has a non-finite value')
    return out


def missing(waypoints):
    return [n for n in REQUIRED if n not in waypoints]


def save(path, name, x, y, yaw_rad, label=None):
    """Add or replace one waypoint, keeping the others. Makes a .bak copy first."""
    path = os.path.expanduser(path)
    name = str(name).strip().lower()
    if not name or not name.replace('_', '').isalnum():
        raise WaypointError('waypoint names use letters, digits and _ only')
    data = {'frame_id': 'map', 'waypoints': {}}
    if os.path.isfile(path):
        with open(path) as f:
            data = yaml.safe_load(f) or data
        data.setdefault('waypoints', {})
        with open(path + '.bak', 'w') as f:
            yaml.safe_dump(data, f, sort_keys=False)
    old = (data['waypoints'] or {}).get(name, {}) or {}
    data['waypoints'] = data['waypoints'] or {}
    data['waypoints'][name] = {
        'x': round(float(x), 3), 'y': round(float(y), 3),
        'yaw_deg': round(math.degrees(yaw_rad), 1),
        'label': label if label is not None else old.get('label', name),
        'saved': time.strftime('%Y-%m-%d %H:%M'),
    }
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    with open(path, 'w') as f:
        yaml.safe_dump(data, f, sort_keys=False)
    return data['waypoints'][name]


def yaw_from_quaternion(z, w):
    return 2.0 * math.atan2(z, w)
