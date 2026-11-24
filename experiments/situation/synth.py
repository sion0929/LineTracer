"""DEV ONLY: synthesise node-like line maps, paths and features from a map's ground truth.

The robot is moved along the true route (with a lateral wander, corner cutting, odometry drift
that jumps at sharp corners, and occasional excursions off the line). Every camera frame the
painted lines (route minus gaps, T bars, loops, stubs, cross bars) are drawn into the node's BEV
grid as seen from the TRUE pose, with perception noise (line width, dropouts, false blobs, missed
frames, short view while rocking / on a ramp), and written into a real LineMap at the ODOM pose,
exactly like FollowerNode.on_image does. The node's replan (prune / lock / snap / trace) is
replayed on that map, so path and path end come from the package's own LineMap.trace.

Every ~5 cm of travel, and at every replan whose path ends in a T bar, one sample is taken:
runtime features (package features.extract) + ground-truth label (label.Track) + context.
"""
import math
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src/line_follower"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from line_follower import vision  # noqa: E402
from line_follower.line_map import LineMap  # noqa: E402
from line_follower.situation import INDEX  # noqa: E402
from line_follower.situation import features as FE  # noqa: E402
from label import Track  # noqa: E402

RES = vision.RES
_, VALID = vision.bev_warp(np.zeros((vision.H, vision.W), np.uint8))
VALID_NEAR = VALID.copy()
VALID_NEAR[:int(round((vision.X_FAR - 0.50) / RES)), :] = 0
FRAME_DS = 0.012          # travel per camera frame (~30 fps at 0.35 m/s)
SAMPLE_DS = 0.05          # regular sample spacing


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def path_len(pts):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts[:-1], pts[1:]))


class PathSim:
    """FollowerNode.prune / acquire / replan on a LineMap (no ROS)."""

    def __init__(self, m):
        self.map, self.path, self.end = m, [], "open"

    @staticmethod
    def to_odom(pose, x, y):
        px, py, th = pose
        return px + x * math.cos(th) - y * math.sin(th), py + x * math.sin(th) + y * math.cos(th)

    @staticmethod
    def to_robot(pose, X, Y):
        px, py, th = pose
        dx, dy = X - px, Y - py
        return dx * math.cos(th) + dy * math.sin(th), -dx * math.sin(th) + dy * math.cos(th)

    def prune(self, pose):
        px, py = pose[0], pose[1]
        n = 0
        while n + 1 < len(self.path) and n < 30:
            (ax, ay), (bx, by) = self.path[n], self.path[n + 1]
            ux, uy = bx - ax, by - ay
            if (px - ax) * ux + (py - ay) * uy <= 0.5 * (ux * ux + uy * uy):
                break
            n += 1
        if n:
            del self.path[:n]

    def acquire(self, pose, x0=0.0):
        best = None
        for dx in np.arange(x0, 0.62, 0.02):
            for dy in (0.0, 0.03, -0.03, 0.06, -0.06, 0.10, -0.10, 0.15, -0.15, 0.2, -0.2):
                X, Y = self.to_odom(pose, dx, dy)
                if self.map.hit(X, Y):
                    best = self.map.snap(X, Y, 0.03) or (X, Y)
                    break
            if best:
                break
        if best is None:
            return None, None
        bx, by = self.to_robot(pose, *best)
        n = max(1, int(math.hypot(bx, by) / 0.02))
        return [self.to_odom(pose, bx * i / n, by * i / n) for i in range(1, n)], best

    def replan(self, pose):
        self.prune(pose)
        lock, acc = [], 0.0
        for i, p in enumerate(self.path):
            if i:
                acc += math.hypot(p[0] - self.path[i - 1][0], p[1] - self.path[i - 1][1])
            lock.append(p)
            if acc >= 0.08:
                break
        if len(lock) >= 3 and acc >= 0.04:
            anchor = self.map.snap(*lock[-1], 0.03)
            if anchor is None:
                return
            h = math.atan2(lock[-1][1] - lock[-3][1], lock[-1][0] - lock[-3][0])
        else:
            lock, anchor = self.acquire(pose)
            if anchor is None:
                return
            h = pose[2]
        pts, end = self.map.trace(anchor[0], anchor[1], h)
        old_rest = max(0.0, path_len(self.path) - path_len(lock))
        if end == "open" and path_len(pts) < old_rest - 0.10:
            return
        new = lock[:-1] + pts if lock else pts
        if end == "open" and path_len(pts) < 0.25:
            link, anchor = self.acquire(pose, vision.X_NEAR)
            if anchor is not None and min(math.hypot(anchor[0] - p[0], anchor[1] - p[1]) for p in new) < 0.10:
                pts2, end2 = self.map.trace(anchor[0], anchor[1], pose[2])
                if path_len(pts2) > path_len(pts) + 0.15 or end2 == "t":
                    new, pts, end = link + pts2, pts2, end2
        self.path, self.end = new, end


def compose(drift, pose):
    """odom pose = drift (dx, dy, dth: world -> odom rigid transform) applied to a world pose."""
    dx, dy, dth = drift
    c, s = math.cos(dth), math.sin(dth)
    x, y, th = pose
    return (dx + c * x - s * y, dy + s * x + c * y, wrap(th + dth))


def odom_to_world(p, odom_pose, true_pose):
    """A point in the odom frame -> world, using the robot's odom and true poses (same instant)."""
    rx, ry = PathSim.to_robot(odom_pose, p[0], p[1])
    return PathSim.to_odom(true_pose, rx, ry)


class Renderer:
    """Painted lines of a track drawn into the BEV grid seen from a world pose."""

    def __init__(self, track):
        self.polys = [p for p in track.painted + track.markings if len(p) >= 2]
        self.centres = [p.mean(axis=0) for p in self.polys]
        self.radii = [float(np.max(np.linalg.norm(p - c, axis=1))) for p, c in zip(self.polys, self.centres)]

    def bev(self, pose, rng, noise=True):
        x, y, th = pose
        c, s = math.cos(th), math.sin(th)
        img = np.zeros((vision.BH, vision.BW), np.uint8)
        thick = int(round(0.02 / RES)) + (int(rng.integers(-1, 2)) if noise else 0)
        for p, cen, rad in zip(self.polys, self.centres, self.radii):
            if math.hypot(cen[0] - x, cen[1] - y) > rad + 1.3:
                continue
            dx, dy = p[:, 0] - x, p[:, 1] - y
            rx, ry = dx * c + dy * s, -dx * s + dy * c
            col = (vision.Y_HALF - ry) / RES
            row = (vision.X_FAR - rx) / RES
            pts = np.round(np.column_stack((col, row)) * 16).astype(np.int32)
            cv2.polylines(img, [pts], False, 255, thickness=max(2, thick), lineType=cv2.LINE_8, shift=4)
        if noise:
            if rng.random() < 0.05:                      # dropout patches
                for _ in range(int(rng.integers(1, 4))):
                    r0, c0 = int(rng.integers(0, vision.BH)), int(rng.integers(0, vision.BW))
                    img[r0:r0 + int(rng.integers(4, 20)), c0:c0 + int(rng.integers(4, 20))] = 0
            if rng.random() < 0.03:                      # false dark blob
                cv2.ellipse(img, (int(rng.integers(0, vision.BW)), int(rng.integers(0, vision.BH))),
                            (int(rng.integers(2, 8)), int(rng.integers(2, 8))), 0, 0, 360, 255, -1)
            if rng.random() < 0.2:                       # far rows: thinner / broken
                far = int(rng.integers(5, 30))
                img[:far] = cv2.erode(img[:far], np.ones((2, 2), np.uint8))
        return img


class Drive:
    """A polyline the synthetic robot drives along (same attributes as label.Track's route)."""

    def __init__(self, xy, step=0.02):
        xy = np.asarray(xy, float)
        dense = [xy[0]]
        for a, b in zip(xy[:-1], xy[1:]):
            k = max(1, int(math.ceil(np.linalg.norm(b - a) / step)))
            dense += [a + (b - a) * i / k for i in range(1, k + 1)]
        self.xy = np.array(dense)
        self.arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(self.xy, axis=0), axis=1))]
        self.length = float(self.arc[-1])
        d = np.diff(self.xy, axis=0)
        h = np.unwrap(np.arctan2(d[:, 1], d[:, 0]))
        self.head = np.r_[h, h[-1]]
        self.z = np.zeros(len(self.xy))

    def head_at(self, s):
        return float(np.interp(s, self.arc, self.head))


def route_pose(track, s, lat=0.0):
    s = float(np.clip(s, 0.0, track.length))
    x = float(np.interp(s, track.arc, track.xy[:, 0]))
    y = float(np.interp(s, track.arc, track.xy[:, 1]))
    h = track.head_at(s)
    return x - lat * math.sin(h), y + lat * math.cos(h), h


def smooth_heading(track, s, w=0.05):
    ss = np.linspace(s - w, s + w, 5)
    hh = np.interp(np.clip(ss, 0, track.length), track.arc, track.head)
    return float(hh.mean())


def corners(track):
    """Arc positions of sharp heading changes (>= 50 deg within 0.15 m)."""
    ss = np.arange(0, track.length, 0.02)
    hh = np.interp(ss, track.arc, track.head)
    j = np.minimum(np.arange(len(ss)) + 7, len(ss) - 1)
    sharp = np.abs(hh[j] - hh) >= math.radians(50)
    out, prev = [], -1.0
    for k in np.nonzero(sharp)[0]:
        if ss[k] - prev > 0.3:
            out.append(float(ss[k]) + 0.07)
        prev = ss[k]
    return out


def pitch_at(track, s, reverse=False):
    a, b = max(0.0, s - 0.06), min(track.length, s + 0.06)
    if b - a < 1e-6:
        return 0.0
    za, zb = np.interp([a, b], track.arc, track.z)
    p = math.atan2(zb - za, b - a)
    return -p if reverse else p


def simulate(track, rng, s0=0.0, s1=None, reverse=False, excursions=True, map_name="", run_id=0,
             drive=None, keep=None):
    """Drive [s0, s1] of the route (or of `drive`; reverse: from s1 down to s0, heading flipped).

    Returns the samples; keep(label_index, tcand) may filter them."""
    track_route = track
    drive = drive or track
    s1 = drive.length if s1 is None else s1
    m = LineMap()
    sim = PathSim(m)
    rend = Renderer(track)
    corner_s = corners(drive)
    drift = [0.0, 0.0, 0.0]
    if s0 > 0.5 or drive is not track:
        drift = [rng.normal(0, 0.05), rng.normal(0, 0.05), rng.normal(0, math.radians(3))]
    start_world = track.xy[0]
    jump_left = 0.0
    jump_dir = 0.0
    lat = rng.normal(0, 0.01)
    exc_until, exc_target = -1.0, 0.0
    samples = []
    next_sample = 0.0
    travelled = 0.0
    n = int(abs(s1 - s0) / FRAME_DS)
    frame = 0
    for k in range(n + 1):
        s = s1 - k * FRAME_DS if reverse else s0 + k * FRAME_DS
        # lateral wander (OU) + occasional excursions near corners
        # AR(1) wander, std ~1.5 cm; an excursion pulls it to +-6..20 cm for 0.6 m
        in_exc = (s < exc_until) if not reverse else False
        lat = 0.98 * lat + rng.normal(0, 0.003) + (0.05 * (exc_target - lat) if in_exc else 0.0)
        lat = float(np.clip(lat, -0.25, 0.25))
        near_corner = [c for c in corner_s if 0.0 <= (c - s) * (-1 if reverse else 1) < 0.05]
        if near_corner and excursions and rng.random() < 0.15 and s >= exc_until:
            exc_until = s + 0.6
            exc_target = rng.choice((-1, 1)) * rng.uniform(0.06, 0.2)
        x, y, h = route_pose(drive, s, lat)
        hs = smooth_heading(drive, s) + rng.normal(0, math.radians(1.5))
        if reverse:
            hs += math.pi
        true_pose = (x, y, wrap(hs))
        # odometry drift: slow random walk + jump along the travel direction at sharp corners
        drift[0] += rng.normal(0, 0.0004)
        drift[1] += rng.normal(0, 0.0004)
        drift[2] += rng.normal(0, math.radians(0.03))
        if near_corner and jump_left <= 0:
            jump_left = rng.uniform(0.05, 0.25)
            jump_dir = true_pose[2]
        if jump_left > 0:
            dj = min(jump_left, 0.02)
            jump_left -= dj
            drift[0] += dj * math.cos(jump_dir + drift[2])
            drift[1] += dj * math.sin(jump_dir + drift[2])
        odom_pose = compose(drift, true_pose)
        pitch = pitch_at(drive, s, reverse)
        steady = abs(pitch) < math.radians(1.5) and rng.random() > 0.03
        x_far = vision.X_FAR if steady else 0.50
        if rng.random() > 0.03:                             # missed frames
            bev = rend.bev(true_pose, rng)
            m.update(bev, VALID if steady else VALID_NEAR, odom_pose)
        frame += 1
        if frame % 2 and k < n:
            continue
        sim.replan(odom_pose)
        # the synthetic robot follows the true route, not the node's path: when the replayed
        # path no longer starts at the robot or points backwards, start it afresh (acquire)
        if len(sim.path) >= 2:
            rp0 = sim.to_robot(odom_pose, *sim.path[0])
            rp1 = sim.to_robot(odom_pose, *sim.path[min(5, len(sim.path) - 1)])
            if math.hypot(*rp0) > 0.15 or rp1[0] < rp0[0] - 0.02:
                sim.path = []
                sim.replan(odom_pose)
        travelled = k * FRAME_DS
        is_t = sim.end == "t" and len(sim.path) >= 2
        if travelled < next_sample and not is_t:
            continue
        if travelled >= next_sample:
            next_sample = travelled + SAMPLE_DS
        start_odom = compose(drift, (*start_world, 0.0))[:2] if s0 < 0.5 else \
            compose([0.0, 0.0, 0.0], (*start_world, 0.0))[:2]
        f = FE.extract(m.grid, odom_pose, sim.path, sim.end, pitch, start_odom, x_far)
        cand = odom_to_world(sim.path[-1], odom_pose, true_pose) if is_t else None
        lab = track_route.label(true_pose, cand)
        if keep is not None and not keep(INDEX[lab], is_t):
            continue
        s_route, _ = track.route_pos(true_pose)
        samples.append(dict(f=f, y=INDEX[lab], tcand=int(is_t), s=-1.0 if s_route is None else s_route,
                            map=map_name, run=run_id, reverse=int(reverse),
                            cand_finish=-1.0 if cand is None else float(np.linalg.norm(np.asarray(cand) - track.finish))))
    return samples


def simulate_map(json_path, seed, variants=("forward", "reverse_start")):
    """All synthetic samples of one map (forward drive + a reverse drive onto the start T)."""
    track = Track(json_path)
    rng = np.random.default_rng(seed)
    name = Path(json_path).stem
    out = []
    if "forward" in variants:
        out += simulate(track, rng, 0.0, None, map_name=name, run_id=0)
    if "forward2" in variants:            # second pass with other noise, starting mid-route
        s0 = float(rng.uniform(2.0, max(2.5, track.length - 6.0)))
        out += simulate(track, rng, s0, None, map_name=name, run_id=1)
    if "reverse_start" in variants:
        rev = simulate(track, rng, 0.0, float(rng.uniform(1.6, 2.6)), reverse=True, excursions=False,
                       map_name=name, run_id=2)
        # driving back along the route is only kept where it ends at the start T (START_T)
        out += [r for r in rev if r["y"] == INDEX["START_T"] or r["tcand"]]
    if "markings" in variants:
        # a lost robot following a stub or a cross bar towards the route sees a T whose bar is
        # the route itself (long arms, nothing or the bar's other half beyond): not the finish
        for k, (sx, kind, ex, ey) in enumerate(track.events):
            if kind != "STUB" and not (kind == "CROSSING" and rng.random() < 0.5):
                continue
            m = min(track.markings, key=lambda mk: np.min(np.linalg.norm(mk - np.array([ex, ey]), axis=1)))
            if len(m) != 2 or np.min(np.linalg.norm(m - np.array([ex, ey]), axis=1)) > 0.03:
                continue
            root = np.array([ex, ey])
            far = m[0] if np.linalg.norm(m[0] - root) > np.linalg.norm(m[1] - root) else m[1]
            u = (root - far) / max(np.linalg.norm(root - far), 1e-6)
            drive = Drive([far - 0.8 * u, root - 0.18 * u])
            out += simulate(track, rng, 0.0, None, excursions=False, map_name=name, run_id=10 + k, drive=drive,
                            keep=lambda y, t: t or rng.random() < 0.3)
    return out
