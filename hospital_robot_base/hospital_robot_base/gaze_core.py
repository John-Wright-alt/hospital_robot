"""
gaze_core.py  --  LiDAR "curiosity" gaze: the maths, with NO ROS code.

Same idea as pico_protocol.py: everything here can be unit-tested on any
computer (see test/test_gaze_core.py). lidar_gaze_node.py is only a thin ROS
wrapper around GazeEngine.

WHAT THIS DOES (and what it does NOT do)
  1. Detects returns     : bins the LD06 scan into 1-degree sectors.
  2. Finds NEW things    : compares each sector with a learned "background"
                           (walls, furniture, the robot's own column). Only
                           returns clearly CLOSER than the background count.
  3. Estimates direction : groups neighbouring new returns into objects and
                           gives each one a bearing and range.
  4. Tracks over time    : a simple alpha-beta tracker gives each object an
                           ID, a smoothed position and a speed.
  It does NOT recognise people. A person usually shows up as one or two leg
  blobs that move, but so does a trolley. Call them "objects", not "people".

  This is for the EYES ONLY. It is not, and must never become, part of the
  robot's obstacle avoidance or safety system.

ASSUMPTION: the robot is STATIONARY while the background is learned and used.
The ROS node resets everything when /odom says the robot is moving.

Conventions (ROS REP-103): x forward, y left, bearings in radians,
POSITIVE = anticlockwise = towards the ROBOT'S LEFT.
"""

import math
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np


# ---------------------------------------------------------------------------
# Settings. The ROS node fills these from config/gaze.yaml.
# ---------------------------------------------------------------------------
@dataclass
class GazeParams:
    # --- scan pre-processing
    bin_deg: float = 1.0          # sector width. LD06 beams are ~0.8 deg apart
    min_range_m: float = 0.15     # ignore anything closer (self-hits, dust)
    max_range_m: float = 2.5      # "interest zone": further away is ignored

    # --- background model
    learn_scans: int = 20         # ~2 s at 10 Hz: keep the area clear while learning
    fg_margin_m: float = 0.15     # must be at least this much closer than background...
    fg_margin_frac: float = 0.05  # ...or this fraction of the background range
    absorb_s: float = 15.0        # an object that stays still this long becomes background
    raise_scans: int = 5          # scans in a row a sector must read FURTHER before
                                  # the background moves out (e.g. a chair was removed)

    # --- clustering
    cluster_jump_m: float = 0.25  # neighbouring sectors further apart in range = different objects
    min_points: int = 2           # sectors per cluster (rejects single-sector spikes)
    min_width_m: float = 0.02
    max_width_m: float = 1.0      # wider than this is a wall/door change, not an object
    merge_dist_m: float = 0.45    # clusters closer than this are one object (two legs)

    # --- tracking
    gate_m: float = 0.6           # max jump between scans for the same object
    alpha: float = 0.5            # position smoothing (1 = no smoothing)
    beta: float = 0.15            # velocity smoothing
    confirm_hits: int = 3         # seen this many scans before the eyes react
    lost_timeout_s: float = 0.8   # unseen this long -> track deleted

    # --- choosing what to look at
    w_near: float = 1.0           # score for being close
    w_speed: float = 1.0          # score for moving
    speed_ref_mps: float = 0.8    # speed that earns the full speed score
    hold_s: float = 1.5           # minimum time to keep looking at one object
    switch_margin: float = 0.25   # a new object must score this much higher to steal focus

    # --- where the bearings are measured from (scan frame -> output frame)
    frame_x: float = 0.0
    frame_y: float = 0.0
    frame_yaw: float = 0.0


# ---------------------------------------------------------------------------
# 1. Scan -> fixed angular bins
# ---------------------------------------------------------------------------
def bin_scan(ranges, angle_min, angle_increment, range_min, range_max,
             n_bins, min_range_m, max_range_m):
    """Return an array of n_bins: the CLOSEST valid range in each sector, or
    +inf if the sector had no valid return.

    Why bins: the LD06 driver does not send the same number of points every
    revolution, so "beam number 137" is not the same direction in every scan.
    A fixed 1-degree grid is.
    """
    r = np.asarray(ranges, dtype=float)
    out = np.full(n_bins, np.inf)
    if r.size == 0:
        return out
    lo = max(range_min if range_min > 0 else 0.0, min_range_m)
    hi = range_max if range_max > 0 else np.inf
    ok = np.isfinite(r) & (r >= lo) & (r <= hi)
    if not np.any(ok):
        return out
    angles = angle_min + angle_increment * np.arange(r.size)
    idx = (np.floor(np.mod(angles, 2 * math.pi) / (2 * math.pi) * n_bins)
           .astype(int) % n_bins)
    np.minimum.at(out, idx[ok], r[ok])
    return out


# ---------------------------------------------------------------------------
# 2. Background model
# ---------------------------------------------------------------------------
class Background:
    """Per-sector memory of 'what is normally there'."""

    def __init__(self, n_bins, p: GazeParams):
        self.n = n_bins
        self.p = p
        self.reset()

    def reset(self):
        self.bg = np.full(self.n, np.inf)        # learned background range
        self.scans = 0
        self.fg_since = np.full(self.n, np.nan)  # time each sector became foreground
        self.further = np.zeros(self.n, dtype=int)

    @property
    def learning(self):
        return self.scans < self.p.learn_scans

    def update(self, r, t):
        """r = binned ranges for this scan. Returns a boolean foreground mask."""
        p = self.p
        valid = np.isfinite(r)
        self.scans += 1

        if self.scans <= p.learn_scans:
            # Learn the FURTHEST valid reading per sector. Missing readings
            # (dropouts) are ignored instead of being learned as "open space".
            seen = np.where(np.isfinite(self.bg), self.bg, -np.inf)
            seen = np.where(valid, np.maximum(seen, r), seen)
            self.bg = np.where(np.isfinite(seen), seen, np.inf)
            return np.zeros(self.n, dtype=bool)

        margin = np.maximum(p.fg_margin_m, p.fg_margin_frac *
                            np.where(np.isfinite(self.bg), self.bg, 0.0))
        fg = valid & (r <= p.max_range_m) & (r < self.bg - margin)

        # Background moves OUT only after several scans in a row (one long
        # multipath reading must not punch a hole in a wall).
        farther = valid & np.isfinite(self.bg) & (r > self.bg + margin)
        self.further = np.where(farther, self.further + 1, 0)
        raise_now = self.further >= p.raise_scans
        self.bg = np.where(raise_now, r, self.bg)
        self.further[raise_now] = 0

        # Foreground timers: an object that stays put is absorbed ("boredom").
        start = fg & np.isnan(self.fg_since)
        self.fg_since[start] = t
        self.fg_since[~fg] = np.nan
        absorb = fg & ((t - self.fg_since) >= p.absorb_s)
        self.bg = np.where(absorb, r, self.bg)
        self.fg_since[absorb] = np.nan
        return fg & ~absorb


# ---------------------------------------------------------------------------
# 3. Foreground sectors -> objects
# ---------------------------------------------------------------------------
@dataclass
class Detection:
    x: float              # centroid in the OUTPUT frame
    y: float
    width_m: float
    n_points: int


def _bin_angles(n_bins):
    return (np.arange(n_bins) + 0.5) * (2 * math.pi / n_bins)


def cluster(r, fg, p: GazeParams) -> List[Detection]:
    """Group neighbouring foreground sectors (wrapping round 360 deg)."""
    n = r.size
    idx = np.flatnonzero(fg)
    if idx.size == 0:
        return []
    groups = [[int(idx[0])]]
    for i in idx[1:]:
        last = groups[-1][-1]
        if i - last <= 2 and abs(r[i] - r[last]) < p.cluster_jump_m:   # allow 1 missing sector
            groups[-1].append(int(i))
        else:
            groups.append([int(i)])
    # join the group that ends near 359 deg with the one that starts near 0 deg
    if len(groups) > 1:
        first, last = groups[0], groups[-1]
        if (first[0] + n) - last[-1] <= 2 and abs(r[first[0]] - r[last[-1]]) < p.cluster_jump_m:
            groups[0] = last + first
            groups.pop()

    ang = _bin_angles(n)
    c, s = math.cos(p.frame_yaw), math.sin(p.frame_yaw)
    blobs = []
    for g in groups:
        g = np.array(g)
        xs, ys = r[g] * np.cos(ang[g]), r[g] * np.sin(ang[g])
        width = math.hypot(xs[-1] - xs[0], ys[-1] - ys[0])
        if g.size < p.min_points or width < p.min_width_m or width > p.max_width_m:
            continue
        blobs.append([xs, ys])

    # merge blobs whose centroids are close (two legs = one object)
    merged = []
    for xs, ys in blobs:
        cx, cy = xs.mean(), ys.mean()
        for m in merged:
            if math.hypot(cx - m[0].mean(), cy - m[1].mean()) < p.merge_dist_m:
                m[0] = np.concatenate([m[0], xs])
                m[1] = np.concatenate([m[1], ys])
                break
        else:
            merged.append([xs, ys])

    out = []
    for xs, ys in merged:
        sx, sy = xs.mean(), ys.mean()                      # scan frame
        ox = p.frame_x + c * sx - s * sy                   # output frame
        oy = p.frame_y + s * sx + c * sy
        width = float(np.hypot(xs.max() - xs.min(), ys.max() - ys.min()))
        out.append(Detection(float(ox), float(oy), width, int(xs.size)))
    return out


# ---------------------------------------------------------------------------
# 4. Tracking
# ---------------------------------------------------------------------------
@dataclass
class Track:
    id: int
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0
    t_last: float = 0.0
    hits: int = 1
    seen_now: bool = True

    @property
    def speed(self):
        return math.hypot(self.vx, self.vy)


class Tracker:
    def __init__(self, p: GazeParams):
        self.p = p
        self.reset()

    def reset(self):
        self.tracks: List[Track] = []
        self.next_id = 1
        self.t_prev: Optional[float] = None

    def update(self, dets: List[Detection], t: float):
        p = self.p
        dt = 0.1 if self.t_prev is None else max(1e-3, t - self.t_prev)
        self.t_prev = t
        for tr in self.tracks:
            tr.seen_now = False

        # greedy nearest-neighbour association, closest pairs first
        pairs = []
        for ti, tr in enumerate(self.tracks):
            px, py = tr.x + tr.vx * dt, tr.y + tr.vy * dt
            for di, d in enumerate(dets):
                dist = math.hypot(d.x - px, d.y - py)
                if dist < p.gate_m:
                    pairs.append((dist, ti, di))
        pairs.sort()
        used_t, used_d = set(), set()
        for _, ti, di in pairs:
            if ti in used_t or di in used_d:
                continue
            used_t.add(ti)
            used_d.add(di)
            tr, d = self.tracks[ti], dets[di]
            px, py = tr.x + tr.vx * dt, tr.y + tr.vy * dt
            ex, ey = d.x - px, d.y - py                     # alpha-beta update
            tr.x, tr.y = px + p.alpha * ex, py + p.alpha * ey
            tr.vx += p.beta * ex / dt
            tr.vy += p.beta * ey / dt
            tr.t_last, tr.hits, tr.seen_now = t, tr.hits + 1, True

        for di, d in enumerate(dets):
            if di not in used_d:
                self.tracks.append(Track(self.next_id, d.x, d.y, t_last=t))
                self.next_id += 1

        self.tracks = [tr for tr in self.tracks if t - tr.t_last <= p.lost_timeout_s]
        return self.tracks


# ---------------------------------------------------------------------------
# 5. What should the eyes look at?
# ---------------------------------------------------------------------------
@dataclass
class GazeTarget:
    state: str                    # learning | none | tracking | lost
    track_id: int = 0
    x: float = 0.0
    y: float = 0.0
    bearing_rad: float = 0.0      # + = robot's LEFT
    range_m: float = 0.0
    speed_mps: float = 0.0
    n_objects: int = 0
    detections: list = field(default_factory=list)


class GazeEngine:
    """Feed it scans; it tells you where something interesting is."""

    def __init__(self, p: GazeParams = None):
        self.p = p or GazeParams()
        self.n_bins = int(round(360.0 / self.p.bin_deg))
        self.bg = Background(self.n_bins, self.p)
        self.tracker = Tracker(self.p)
        self.focus_id = 0
        self.focus_since = 0.0

    def reset(self):
        """Call when the robot moves: the learned background is no longer valid."""
        self.bg.reset()
        self.tracker.reset()
        self.focus_id = 0

    def _score(self, tr: Track):
        p = self.p
        rng = math.hypot(tr.x, tr.y)
        near = max(0.0, 1.0 - rng / p.max_range_m)
        moving = min(1.0, tr.speed / p.speed_ref_mps)
        return p.w_near * near + p.w_speed * moving

    def process(self, ranges, angle_min, angle_increment, range_min, range_max,
                t) -> GazeTarget:
        p = self.p
        r = bin_scan(ranges, angle_min, angle_increment, range_min, range_max,
                     self.n_bins, p.min_range_m, p.max_range_m)
        fg = self.bg.update(r, t)
        if self.bg.learning:
            return GazeTarget('learning')

        dets = cluster(r, fg, p)
        tracks = self.tracker.update(dets, t)
        confirmed = [tr for tr in tracks if tr.hits >= p.confirm_hits]

        current = next((tr for tr in confirmed if tr.id == self.focus_id), None)
        best = max(confirmed, key=self._score, default=None)
        if current is None:
            if best is not None and best.seen_now:
                self.focus_id, self.focus_since = best.id, t
                current = best
        elif (best is not None and best.id != current.id and best.seen_now
              and t - self.focus_since >= p.hold_s
              and self._score(best) > self._score(current) + p.switch_margin):
            self.focus_id, self.focus_since = best.id, t
            current = best

        if current is None:
            self.focus_id = 0
            return GazeTarget('none', n_objects=len(confirmed), detections=dets)
        return GazeTarget(
            'tracking' if current.seen_now else 'lost',
            track_id=current.id, x=current.x, y=current.y,
            bearing_rad=math.atan2(current.y, current.x),
            range_m=math.hypot(current.x, current.y),
            speed_mps=current.speed, n_objects=len(confirmed), detections=dets)
