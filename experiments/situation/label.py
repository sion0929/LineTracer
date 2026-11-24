#!/usr/bin/env python3
"""DEV ONLY: ground-truth situation label of the window ahead of a robot pose.

Uses the generator sidecar JSON (centerline, markings, ramps, gaps, loop corners) of a map, so it
must never be imported by the robot package. The class list is the package's
line_follower.situation.SITUATIONS.

  label(track, (x, y, yaw))                world pose -> situation name
  label(track, pose, cand=(cx, cy))        the node's path ends in a T bar at world point cand:
                                           FINISH_T only if cand really is the finish bar
  python3 experiments/situation/label.py MAP.json [--step 0.1]   labels along the route (sanity)

Definitions (window = route arc [s + 0.05, s + 0.85] ahead of the robot's route position s):
  OFF          robot > 10 cm from the route or heading > 60 deg off it (and no T bar faced)
  FINISH_T     the route end is <= 0.90 m ahead (or the finish bar is faced from the route side)
  START_T      not on the route, facing the start bar from the route side (driving back)
  LOOP_CORNER  window overlaps a small loop (loop corner circle or teardrop: self-crossing < 1.3 m apart)
  CROSSING     a cross bar, a big loop marking or a far route self-crossing (connected crossing) in the window
  STUB         a one-sided branch root in the window
  GAP          a painted-line break overlapping the window
  HILL         robot on a ramp or a ramp starts within 0.5 m ahead
  RIGHT_ANGLE  heading changes >= 60 deg within 0.12 m somewhere in [s, s + 0.8]
  ARC          heading changes >= 30 deg over [s, s + 0.8]
  STRAIGHT     otherwise
The first matching class in this order wins.
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src/line_follower"))
from line_follower.situation import SITUATIONS  # noqa: E402

WIN0, WIN1 = 0.05, 0.85
FINISH_AHEAD = 0.90
ON_ROUTE_DIST, ON_ROUTE_COS = 0.10, 0.5
LOOP_MAX_ARC = 1.3
CAND_RADIUS = 0.15


def _cum(xy):
    return np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]


def seg_intersections(a, b, tol=1e-9):
    """All crossings between polyline segments a (n+1 points) and b (m+1 points).

    Returns (i, j, t, u): segment indices and the parameters along each segment."""
    p, r = a[:-1], np.diff(a, axis=0)
    q, s = b[:-1], np.diff(b, axis=0)
    out = []
    step = 400
    for i0 in range(0, len(p), step):
        pp, rr = p[i0:i0 + step, None, :], r[i0:i0 + step, None, :]
        den = rr[..., 0] * s[None, :, 1] - rr[..., 1] * s[None, :, 0]
        qp = q[None] - pp
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (qp[..., 0] * s[None, :, 1] - qp[..., 1] * s[None, :, 0]) / den
            u = (qp[..., 0] * rr[..., 1] - qp[..., 1] * rr[..., 0]) / den
        ok = (np.abs(den) > 1e-12) & (t >= -tol) & (t <= 1 + tol) & (u >= -tol) & (u <= 1 + tol)
        for i, j in zip(*np.nonzero(ok)):
            out.append((i0 + i, j, float(t[i, j]), float(u[i, j])))
    return out


class Track:
    """Route geometry, situation events and painted lines of one generated map (world frame)."""

    def __init__(self, meta):
        if isinstance(meta, (str, Path)):
            meta = json.loads(Path(meta).read_text())
        self.meta = meta
        c = np.asarray(meta["centerline"], float)
        self.xy, self.z = c[:, :2], c[:, 2]
        self.arc = _cum(self.xy)
        self.length = float(self.arc[-1])
        d = np.diff(self.xy, axis=0)
        self.seg_head = np.unwrap(np.arctan2(d[:, 1], d[:, 0]))
        self.head = np.r_[self.seg_head, self.seg_head[-1]]
        self.start, self.finish = self.xy[0], self.xy[-1]
        k0 = int(np.searchsorted(self.arc, 0.06))
        k1 = int(np.searchsorted(self.arc, self.length - 0.06))
        self.start_dir = math.atan2(*(self.xy[k0] - self.xy[0])[::-1])
        self.finish_dir = math.atan2(*(self.xy[-1] - self.xy[k1])[::-1])
        self.markings = [np.asarray(m, float)[:, :2] for m in meta.get("markings", [])]
        self.painted = [np.asarray(p, float)[:, :2] for p in meta.get("painted", [meta["centerline"]])]
        self.ramps = [(r["x"][0], r["x"][-1], r["y"][0], r["y"][1]) for r in
                      meta.get("ramps", [meta["ramp"]] if "ramp" in meta else [])]
        self.gaps = [tuple(g) for g in meta.get("gaps_arc_m", [])]
        self.events = []          # (s, kind, x, y): CROSSING / STUB points on the route
        self.loops = []           # (s0, s1, x, y): small loop spans (LOOP_CORNER)
        self._find_events()
        self.ev_s = np.array([e[0] for e in self.events]) if self.events else np.zeros(0)
        self.ev_kind = [e[1] for e in self.events]

    # ------------------------------------------------------------------ geometry
    def s_at_point(self, i, t):
        return float(self.arc[i] + t * (self.arc[i + 1] - self.arc[i]))

    def dist_to_route(self, p):
        a, d = self.xy[:-1], np.diff(self.xy, axis=0)
        t = np.clip(np.einsum("ij,ij->i", p - a, d) / np.maximum(np.einsum("ij,ij->i", d, d), 1e-12), 0, 1)
        dist = np.linalg.norm(a + t[:, None] * d - p, axis=1)
        k = int(np.argmin(dist))
        return float(dist[k]), k, float(t[k])

    def _find_events(self):
        loop_vertices = []
        for lc in self.meta.get("loop_corners", []) or []:
            v = np.asarray(lc["vertex"], float)
            ks = np.nonzero(np.linalg.norm(self.xy - v, axis=1) < 1e-6)[0]
            if len(ks) >= 2:
                s0, s1 = float(self.arc[ks.min()]), float(self.arc[ks.max()])
                self.loops.append((s0, s1, v[0], v[1]))
                loop_vertices.append(v)
        # route self-crossings
        seen = []
        for i, j, t, u in seg_intersections(self.xy, self.xy):
            si, sj = self.s_at_point(i, t), self.s_at_point(j, u)
            if sj - si < 0.3:
                continue
            pt = self.xy[i] + t * (self.xy[i + 1] - self.xy[i])
            if any(np.linalg.norm(pt - v) < 0.05 for v in loop_vertices):
                continue
            if any(abs(si - a) < 0.1 and abs(sj - b) < 0.1 for a, b in seen):
                continue
            seen.append((si, sj))
            if sj - si <= LOOP_MAX_ARC:
                self.loops.append((si, sj, pt[0], pt[1]))
            else:
                self.events += [(si, "CROSSING", pt[0], pt[1]), (sj, "CROSSING", pt[0], pt[1])]
        # markings: start / finish bars, stubs, cross bars, big loops
        for m in self.markings:
            mid = m.mean(axis=0) if len(m) == 2 else None
            if mid is not None and (np.linalg.norm(mid - self.start) < 0.03 or np.linalg.norm(mid - self.finish) < 0.03):
                continue
            if len(m) == 2:
                dp, kp, tp = self.dist_to_route(m[0])
                dq, kq, tq = self.dist_to_route(m[1])
                if min(dp, dq) < 0.012 and max(dp, dq) > 0.05:
                    k, t, root = (kp, tp, m[0]) if dp < dq else (kq, tq, m[1])
                    self.events.append((self.s_at_point(k, t), "STUB", root[0], root[1]))
                    continue
            hits = []
            for i, j, t, u in seg_intersections(self.xy, m):
                s = self.s_at_point(i, t)
                if all(abs(s - h) > 0.05 for h in hits):
                    hits.append(s)
                    pt = self.xy[i] + t * (self.xy[i + 1] - self.xy[i])
                    self.events.append((s, "CROSSING", pt[0], pt[1]))
        self.events.sort()

    def on_ramp(self, p):
        return any(x0 <= p[0] <= x1 and y0 <= p[1] <= y1 for x0, x1, y0, y1 in
                   [(min(a, b), max(a, b), c, d) for a, b, c, d in self.ramps])

    def head_at(self, s):
        return float(np.interp(s, self.arc, self.head))

    def route_pos(self, pose):
        """(s, distance) of the heading-aligned route point nearest to pose, or (None, d)."""
        x, y, yaw = pose
        a, d = self.xy[:-1], np.diff(self.xy, axis=0)
        p = np.array([x, y])
        t = np.clip(np.einsum("ij,ij->i", p - a, d) / np.maximum(np.einsum("ij,ij->i", d, d), 1e-12), 0, 1)
        dist = np.linalg.norm(a + t[:, None] * d - p, axis=1)
        ok = (dist <= ON_ROUTE_DIST) & (np.cos(self.seg_head - yaw) >= ON_ROUTE_COS)
        if not ok.any():
            return None, float(dist.min())
        k = int(np.argmin(np.where(ok, dist, np.inf)))
        return self.s_at_point(k, t[k]), float(dist[k])

    # ------------------------------------------------------------------ labels
    def _facing(self, pose, point, direction):
        x, y, yaw = pose
        dx, dy = point[0] - x, point[1] - y
        rx, ry = dx * math.cos(yaw) + dy * math.sin(yaw), -dx * math.sin(yaw) + dy * math.cos(yaw)
        return 0.0 <= rx <= FINISH_AHEAD and abs(ry) <= 0.20 and math.cos(yaw - direction) >= 0.7

    def window_label(self, pose, allow_finish=True):
        s, _ = self.route_pos(pose)
        if s is None:
            if allow_finish and self._facing(pose, self.finish, self.finish_dir):
                return "FINISH_T"
            if self._facing(pose, self.start, self.start_dir + math.pi):
                return "START_T"
            return "OFF"
        if allow_finish and self.length - s <= FINISH_AHEAD:
            return "FINISH_T"
        a, b = s + WIN0, s + WIN1
        if any(s0 <= b and s1 >= a for s0, s1, _, _ in self.loops):
            return "LOOP_CORNER"
        if len(self.ev_s):
            inside = [k for k, e in zip(self.ev_kind, self.ev_s) if a <= e <= b]
            if "CROSSING" in inside:
                return "CROSSING"
            if "STUB" in inside:
                return "STUB"
        if any(g0 <= b and g1 >= a for g0, g1 in self.gaps):
            return "GAP"
        if self.ramps:
            sel = (self.arc >= s) & (self.arc <= s + 0.5)
            if self.on_ramp(pose[:2]) or any(self.on_ramp(p) for p in self.xy[sel][::3]):
                return "HILL"
        ss = np.arange(s, min(s + 0.8, self.length) + 1e-9, 0.02)
        if len(ss) >= 2:
            hh = np.interp(ss, self.arc, self.head)
            j = np.minimum(np.arange(len(ss)) + 6, len(ss) - 1)
            if np.max(np.abs(hh[j] - hh)) >= math.radians(60):
                return "RIGHT_ANGLE"
            if np.max(np.abs(hh - hh[0])) >= math.radians(30):
                return "ARC"
        return "STRAIGHT"

    def candidate_kind(self, cand):
        """What is really at the world point where the node's path ended in a T bar."""
        c = np.asarray(cand, float)
        if np.linalg.norm(c - self.finish) < CAND_RADIUS:
            return "FINISH_T"
        if np.linalg.norm(c - self.start) < CAND_RADIUS:
            return "START_T"
        best, kind = CAND_RADIUS, None
        for s, k, x, y in self.events:
            d = math.hypot(c[0] - x, c[1] - y)
            if d < best:
                best, kind = d, k
        for s0, s1, x, y in self.loops:
            d = math.hypot(c[0] - x, c[1] - y)
            if d < best:
                best, kind = d, "LOOP_CORNER"
        return kind

    def label(self, pose, cand=None):
        """Situation name for a world pose; cand = world point of a T-bar path end (or None)."""
        win = self.window_label(pose)
        if cand is None:
            return win
        kind = self.candidate_kind(cand)
        if kind == "FINISH_T":
            return "FINISH_T"
        if win != "FINISH_T":
            return win
        return kind or self.window_label(pose, allow_finish=False)


def label(track, pose, cand=None):
    return track.label(pose, cand)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("map", type=Path, help="generator sidecar JSON (or the SDF next to it)")
    ap.add_argument("--step", type=float, default=0.1)
    a = ap.parse_args()
    tr = Track(a.map.with_suffix(".json"))
    print(f"length {tr.length:.2f} m, events {[(round(s, 2), k) for s, k, _, _ in tr.events]}, "
          f"loops {[(round(s0, 2), round(s1, 2)) for s0, s1, _, _ in tr.loops]}, gaps {tr.gaps}")
    prev = None
    for s in np.arange(0.0, tr.length, a.step):
        k = int(np.searchsorted(tr.arc, s))
        k = min(k, len(tr.xy) - 2)
        pose = (*tr.xy[k], float(tr.seg_head[k]))
        lab = tr.label(pose)
        if lab != prev:
            print(f"{s:6.2f} m  {lab}")
            prev = lab
    assert set(SITUATIONS) >= {"OFF"}


if __name__ == "__main__":
    main()
