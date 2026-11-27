#!/usr/bin/env python3
"""Targeted test tracks: one situation per track, many variants (dev tool, no obstacles).

Each track is: start T -> lead-in straight -> the case -> lead-out straight -> finish T.
The driven centre line comes from a turtle program; distractor paint (crossing bars, parallel
lines, branches, other track pieces) is added as extra painted polylines; ramps can sit on
any straight. After building, every crossing of the path with itself or with other paint is
found numerically and stored with its arc position and angle, so results can be scored per
situation, not per map.

Output per track: <name>.sdf/.stl/.json (bench.py compatible: world, centerline, arc_m,
start_pose, markings, painted, ramps) + "case" (group, parameters, feature arc ranges).

  python3 experiments/worlds/gen_cases.py --out experiments/worlds/suites/cases [--groups a,b] [--preview]
"""
import argparse
import json
import math
import random
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import gen_world as G  # noqa: E402

W = G.LINE_WIDTH
LEAD_IN, LEAD_OUT, MARGIN = 0.90, 0.60, 0.45
RAMP_W = 0.60


# ---------------------------------------------------------------- geometry
def run_program(program, x=0.0, y=0.0, h=0.0, step=0.02):
    """Turtle program -> points (incl. start), list of ramps, list of named marks.

    Commands: ('S', L) straight, ('A', r, deg) arc (deg > 0 left), ('SQ', deg) square
    corner, ('RAMP', up, deck, down, height) a ramp on a straight (total up+deck+down),
    ('MARK', name) remember the current point / heading / arc for distractors and features.
    """
    pts, ramps, marks = [(x, y)], [], {}
    s = 0.0
    for cmd in program:
        if cmd[0] == "MARK":
            marks[cmd[1]] = (x, y, h, s)
            continue
        if cmd[0] == "SQ":
            h += math.radians(cmd[1])
            continue
        if cmd[0] == "RAMP":
            up, deck, down, height = cmd[1:5]
            ramps.append({"x0": x, "y0": y, "heading": h, "up": up, "deck": deck, "down": down,
                          "height": height, "s0": s})
            cmd = ("S", up + deck + down)
        new = G.turtle(x, y, h, [cmd], step)
        for p in new:
            s += math.dist(pts[-1], p)
            pts.append(p)
        x, y = pts[-1]
        if cmd[0] == "A":
            h += math.radians(cmd[2])
    return pts, ramps, marks


def ramp_height(ramp, x, y):
    c, s = math.cos(ramp["heading"]), math.sin(ramp["heading"])
    u = (x - ramp["x0"]) * c + (y - ramp["y0"]) * s
    v = -(x - ramp["x0"]) * s + (y - ramp["y0"]) * c
    if abs(v) > ramp.get("width", RAMP_W) / 2:
        return 0.0
    up, deck, down, hgt = ramp["up"], ramp["deck"], ramp["down"], ramp["height"]
    if u <= 0 or u >= up + deck + down:
        return 0.0
    if u < up:
        return hgt * u / up
    if u < up + deck:
        return hgt
    return hgt * (up + deck + down - u) / down


def seg_intersections(P, Q, skip_adjacent=True, same=False):
    """All crossings between polylines P and Q: (i, j, point, angle_deg)."""
    out = []
    for i in range(len(P) - 1):
        a, b = P[i], P[i + 1]
        for j in range(len(Q) - 1):
            if same and j <= i + 3:
                continue
            c, d = Q[j], Q[j + 1]
            r = (b[0] - a[0], b[1] - a[1])
            q = (d[0] - c[0], d[1] - c[1])
            den = r[0] * q[1] - r[1] * q[0]
            if abs(den) < 1e-12:
                continue
            t = ((c[0] - a[0]) * q[1] - (c[1] - a[1]) * q[0]) / den
            u = ((c[0] - a[0]) * r[1] - (c[1] - a[1]) * r[0]) / den
            if 0 <= t < 1 and 0 <= u < 1:
                ang = abs(math.degrees(math.atan2(den, r[0] * q[0] + r[1] * q[1])))
                ang = min(ang, 180 - ang)
                out.append((i, t, j, u, (a[0] + t * r[0], a[1] + t * r[1]), ang))
    return out


def offset_polyline(pts, d):
    out = []
    for k, (x, y) in enumerate(pts):
        a = pts[max(k - 1, 0)]
        b = pts[min(k + 1, len(pts) - 1)]
        ux, uy = G.unit(b[0] - a[0], b[1] - a[1])
        out.append((x - uy * d, y + ux * d))
    return out


def ray(x, y, heading, length, step=0.02):
    n = max(2, round(length / step))
    return [(x + length * k / n * math.cos(heading), y + length * k / n * math.sin(heading)) for k in range(n + 1)]


def arc_of(pts):
    arc = [0.0]
    for a, b in zip(pts, pts[1:]):
        arc.append(arc[-1] + math.dist(a, b))
    return arc


# ---------------------------------------------------------------- writer
def write_track(out, name, center, paint_extra, ramps, gaps, case, seed, mat=None):
    """Shift everything onto a mat, write SDF/STL/JSON. gaps: list of (s0, s1) on the path.
    mat=(w, h): keep the coordinates as given on a fixed w x h mat at the origin (tile tracks)."""
    allp = center + [p for line in paint_extra for p in line]
    for r in ramps:
        L = r["up"] + r["deck"] + r["down"]
        for u, v in ((0, -RAMP_W / 2), (0, RAMP_W / 2), (L, -RAMP_W / 2), (L, RAMP_W / 2)):
            c, s = math.cos(r["heading"]), math.sin(r["heading"])
            allp.append((r["x0"] + u * c - v * s, r["y0"] + u * s + v * c))
    minx = 0.0 if mat else min(p[0] for p in allp) - MARGIN
    miny = 0.0 if mat else min(p[1] for p in allp) - MARGIN
    shift = lambda p: (p[0] - minx, p[1] - miny)  # noqa: E731
    center = [shift(p) for p in center]
    paint_extra = [[shift(p) for p in line] for line in paint_extra]
    for r in ramps:
        r["x0"], r["y0"] = shift((r["x0"], r["y0"]))
    maxx = mat[0] if mat else max(p[0] for p in allp) - minx + MARGIN
    maxy = mat[1] if mat else max(p[1] for p in allp) - miny + MARGIN
    bounds = [0, 0, maxx, maxy]

    def height(x, y):
        return max([0.0] + [ramp_height(r, x, y) for r in ramps])

    arc = arc_of(center)
    # start / finish T bars
    h0 = math.atan2(center[1][1] - center[0][1], center[1][0] - center[0][0])
    h1 = math.atan2(center[-1][1] - center[-2][1], center[-1][0] - center[-2][0])
    tbars = []
    for (x, y), h in ((center[0], h0), (center[-1], h1)):
        n = (-math.sin(h), math.cos(h))
        tbars.append([(x - 0.10 * n[0], y - 0.10 * n[1]), (x + 0.10 * n[0], y + 0.10 * n[1])])
    painted, cur = [], []
    for k, p in enumerate(center):
        if any(a0 < arc[k] < a1 for a0, a1 in gaps):
            if len(cur) > 1:
                painted.append(cur)
            cur = []
        else:
            cur.append(p)
    if len(cur) > 1:
        painted.append(cur)

    out.mkdir(parents=True, exist_ok=True)
    sdf = out / f"{name}.sdf"
    mesh = sdf.with_suffix(".stl")
    G.write_line_mesh(mesh, [*painted, *tbars, *paint_extra], height)
    tree = ET.parse(G.BRINGUP / "worlds/line_track.sdf")
    world = tree.getroot().find("world")
    world.remove(world.find("gui"))
    track_include, robot = world.findall("include")
    world.remove(track_include)
    spawn = [*center[0], 0.01 + height(*center[0]), 0, 0, h0]
    robot.find("pose").text = " ".join(f"{v:.9g}" for v in spawn)
    track = G.add(world, "model", name="track")
    G.add(track, "static", "true")
    link = G.add(track, "link", name="surface")
    G.surface(link, "mat", (bounds[2], bounds[3], 0.1), (bounds[2] / 2, bounds[3] / 2, -0.05, 0, 0, 0), G.MAT_COLOR)
    k = 0
    for r in ramps:
        c, s = math.cos(r["heading"]), math.sin(r["heading"])
        u0 = 0.0
        for length, top, slope in ((r["up"], r["height"] / 2, math.atan2(r["height"], r["up"])),
                                   (r["deck"], r["height"], 0.0),
                                   (r["down"], r["height"] / 2, -math.atan2(r["height"], r["down"]))):
            if length <= 0:
                continue
            th = r["height"] + 0.015
            um = u0 + length / 2 + math.sin(slope) * th / 2
            pose = (r["x0"] + um * c, r["y0"] + um * s, top - math.cos(slope) * th / 2, 0, -slope, r["heading"])
            G.surface(link, f"ramp_{k}", (length / math.cos(slope), r.get("width", RAMP_W), th), pose, G.MAT_COLOR)
            u0 += length
            k += 1
    visual = G.add(G.add(track, "link", name="line"), "visual", name="visual")
    G.add(visual, "cast_shadows", "false")
    G.add(G.add(G.add(visual, "geometry"), "mesh"), "uri", f"./{mesh.name}")
    G.material(visual, "0 0 0 1", "0.1 0.1 0.1 1")
    ET.indent(tree)
    tree.write(sdf, encoding="utf-8", xml_declaration=True)

    # crossings found numerically (path with itself, path with other paint)
    crossings = []
    for i, t, j, u, p, ang in seg_intersections(center, center, same=True):
        crossings.append({"kind": "self", "s": [arc[i] + t * math.dist(center[i], center[i + 1]),
                                                 arc[j] + u * math.dist(center[j], center[j + 1])],
                          "xy": list(p), "angle_deg": round(ang, 1)})
    for line in paint_extra:
        for i, t, j, u, p, ang in seg_intersections(center, line):
            crossings.append({"kind": "paint", "s": [arc[i] + t * math.dist(center[i], center[i + 1])],
                              "xy": list(p), "angle_deg": round(ang, 1)})
    dedup = []
    for c_ in crossings:                      # a crossing exactly at a vertex is found twice
        if not any(math.dist(c_["xy"], d["xy"]) < 0.02 for d in dedup):
            dedup.append(c_)
    case = dict(case)
    case["crossings"] = dedup
    case["length_m"] = round(arc[-1], 3)
    meta = {
        "version": 1, "world": world.get("name"), "sdf": sdf.name, "seed": seed, "difficulty": case["group"],
        "start_pose": spawn, "centerline": [[x, y, height(x, y)] for x, y in center], "arc_m": arc,
        "markings": [[[x, y, height(x, y)] for x, y in m] for m in [*tbars, *paint_extra]],
        "line_width_m": W, "line_lift_m": G.LINE_LIFT, "mat_bounds": bounds, "obstacles": [],
        "corner_radii_m": [], "ramp": {}, "small_loops": [], "loop_corners": [], "detours": [],
        "gaps_arc_m": [list(g) for g in gaps],
        "painted": [[[x, y, height(x, y)] for x, y in seg] for seg in painted],
        "ramps": [{"x0": r["x0"], "y0": r["y0"], "heading": r["heading"], "up": r["up"], "deck": r["deck"],
                   "down": r["down"], "height_m": r["height"], "s0": r["s0"]} for r in ramps],
        "mirror": False, "case": case,
    }
    sdf.with_suffix(".json").write_text(json.dumps(meta, separators=(",", ":")) + "\n")
    return meta


# ---------------------------------------------------------------- the catalogue
def mirror_prog(prog):
    out = []
    for c in prog:
        if c[0] == "A":
            out.append(("A", c[1], -c[2]))
        elif c[0] == "SQ":
            out.append(("SQ", -c[1]))
        else:
            out.append(c)
    return out


def build(prog, extras=None, gaps=None, feature=None):
    """prog without lead-in/out; extras(marks, pts) -> list of extra painted lines."""
    full = [("S", LEAD_IN), ("MARK", "f0"), *prog, ("MARK", "f1"), ("S", LEAD_OUT)]
    pts, ramps, marks = run_program(full)
    paint = extras(marks, pts) if extras else []
    g = gaps(marks) if gaps else []
    feat = {"s": [round(marks["f0"][3], 3), round(marks["f1"][3], 3)]}
    if feature:
        feat.update(feature)
    return pts, paint, ramps, g, feat


def cases(rng):
    """Yield (group, name, params, prog, extras, gaps). Every case also gets an L/R mirror."""
    J = lambda a, b: rng.uniform(a, b)  # noqa: E731

    # A. geometry --------------------------------------------------------------
    for r in (0.0, 0.05, 0.08, 0.12, 0.18, 0.25, 0.40):
        prog = [("SQ", 90)] if r == 0 else [("A", r, 90)]
        yield "corner90", f"r{int(r * 100):02d}", {"radius": r}, prog + [("S", 0.6)], None, None
    for ang in (30, 45, 60, 90, 120, 135, 150):
        yield "turn_angle", f"a{ang}", {"angle_deg": ang, "radius": 0.0}, [("SQ", ang), ("S", 0.6)], None, None
        yield "turn_angle", f"a{ang}_r08", {"angle_deg": ang, "radius": 0.08}, [("A", 0.08, ang), ("S", 0.6)], None, None
    for r in (0.08, 0.12, 0.18, 0.25):
        for gap in (0.0, 0.10, 0.25):
            prog = [("A", r, 90), ("S", gap), ("A", r, -90), ("S", 0.4)] if gap else [("A", r, 90), ("A", r, -90), ("S", 0.4)]
            yield "s_curve", f"r{int(r * 100):02d}_g{int(gap * 100):02d}", {"radius": r, "straight_between": gap}, prog, None, None
    for r in (0.10, 0.15, 0.20, 0.30):
        yield "hairpin", f"u_r{int(r * 100):02d}", {"radius": r, "kind": "arc"}, [("A", r, 180), ("S", 0.8)], None, None
    for leg in (0.15, 0.25, 0.40):
        yield "hairpin", f"u_sq_leg{int(leg * 100):02d}", {"leg": leg, "kind": "square"}, \
            [("SQ", 90), ("S", leg), ("SQ", 90), ("S", 0.8)], None, None
    for seg in (0.10, 0.20, 0.30):
        for ang in (45, 90):
            prog = []
            for k in range(4):
                prog += [("SQ", ang if k % 2 == 0 else -ang), ("S", seg)]
            yield "zigzag", f"a{ang}_s{int(seg * 100):02d}", {"angle_deg": ang, "segment": seg}, prog + [("S", 0.4)], None, None
    for amp, wl in ((0.05, 0.4), (0.10, 0.5), (0.15, 0.6), (0.08, 0.25)):
        k = 2 * math.pi / wl
        th = math.degrees(math.atan(amp * k))
        r = wl / (4 * math.radians(th)) if th else 1.0
        prog = [("A", r, th)]
        for _ in range(3):
            prog += [("A", r, -2 * th), ("A", r, 2 * th)]
        prog += [("A", r, -th), ("S", 0.3)]
        yield "wavy", f"amp{int(amp * 100):02d}_wl{int(wl * 100):02d}", {"amplitude": amp, "wavelength": wl}, prog, None, None
    yield "spiral", "tightening", {"radii": [0.4, 0.25, 0.15, 0.10]}, \
        [("A", 0.40, 90), ("A", 0.25, 90), ("A", 0.15, 90), ("A", 0.10, 90), ("S", 0.5)], None, None

    # B. crossings and overlapping paint ----------------------------------------
    for ang in (30, 45, 60, 75, 90):
        def bar(m, p, ang=ang):
            x, y, h, _ = m["c"]
            a = h + math.radians(ang)
            return [ray(x - 0.25 * math.cos(a), y - 0.25 * math.sin(a), a, 0.50)]
        yield "cross_bar", f"a{ang}", {"angle_deg": ang, "bar_len": 0.5}, [("S", 0.4), ("MARK", "c"), ("S", 0.6)], bar, None
    for n_bars, spacing in ((2, 0.15), (2, 0.30), (3, 0.20)):
        def bars(m, p, n=n_bars, sp=spacing):
            x, y, h, _ = m["c"]
            out = []
            for k in range(n):
                cx, cy = x + k * sp * math.cos(h), y + k * sp * math.sin(h)
                out.append(ray(cx - 0.25 * math.cos(h + math.pi / 2), cy - 0.25 * math.sin(h + math.pi / 2), h + math.pi / 2, 0.5))
            return out
        yield "multi_cross", f"n{n_bars}_sp{int(spacing * 100):02d}", {"bars": n_bars, "spacing": spacing}, \
            [("S", 0.3), ("MARK", "c"), ("S", 0.3 + n_bars * spacing + 0.3)], bars, None
    for r in (0.08, 0.10, 0.12, 0.15):
        yield "corner_loop", f"r{int(r * 100):02d}", {"radius": r, "note": "official-style 270 deg loop at a corner"}, \
            [("S", r), ("A", r, -270), ("S", r), ("S", 0.6)], None, None
    for ang in (30, 45, 60, 75, 90, 120, 150):
        # turn left by 180+ang (radius r), come back down and cross the already-driven
        # straight at `ang` (acute angle min(ang, 180-ang)), then turn away below it
        r = 0.12
        y1 = r * (1 + math.cos(math.radians(ang)))
        L = y1 / math.sin(math.radians(ang)) + 0.30
        prog = [("S", 0.6), ("A", r, 180 + ang), ("S", L), ("A", r, 180 - ang), ("S", 0.6)]
        yield "self_cross", f"a{ang}", {"crossing_angle_deg": min(ang, 180 - ang), "approach_deg": ang, "radius": r}, prog, None, None
    for off in (0.03, 0.05, 0.08, 0.12, 0.20):
        for side in (1, -1):
            def par(m, p, off=off, side=side):
                x, y, h, _ = m["p0"]
                return [ray(x - side * off * math.sin(h), y + side * off * math.cos(h), h, 0.8)]
            yield "parallel", f"o{int(off * 100):02d}_{'l' if side > 0 else 'r'}", {"offset": off, "side": side, "length": 0.8}, \
                [("S", 0.2), ("MARK", "p0"), ("S", 1.0)], par, None
    for off in (0.04, 0.08, 0.15):
        def par_curve(m, p, off=off):
            i0 = next(k for k, q in enumerate(p) if math.dist(q, m["p0"][:2]) < 0.011)
            i1 = next(k for k, q in enumerate(p) if math.dist(q, m["p1"][:2]) < 0.011)
            return [offset_polyline(p[i0:i1 + 1], off)]
        yield "parallel", f"curve_o{int(off * 100):02d}", {"offset": off, "on": "90 deg arc r 0.25"}, \
            [("MARK", "p0"), ("A", 0.25, 90), ("MARK", "p1"), ("S", 0.6)], par_curve, None
    for ang in (15, 25, 35, 50):
        for side in (1, -1):
            def branch(m, p, ang=ang, side=side):
                x, y, h, _ = m["b"]
                return [ray(x, y, h + side * math.radians(ang), 0.6)]
            yield "y_split", f"a{ang}_{'l' if side > 0 else 'r'}", {"branch_angle_deg": ang, "note": "path goes straight, branch diverges"}, \
                [("S", 0.3), ("MARK", "b"), ("S", 0.9)], branch, None
            def merge(m, p, ang=ang, side=side):
                x, y, h, _ = m["b"]
                a = h + math.pi - side * math.radians(ang)
                return [list(reversed(ray(x, y, a, 0.6)))]
            yield "y_merge", f"a{ang}_{'l' if side > 0 else 'r'}", {"branch_angle_deg": ang, "note": "a line joins the path from behind-side"}, \
                [("S", 0.6), ("MARK", "b"), ("S", 0.6)], merge, None
    for length in (0.08, 0.15, 0.30):
        for kind in ("left", "right", "both"):
            def stub(m, p, L=length, kind=kind):
                x, y, h, _ = m["b"]
                out = []
                if kind in ("left", "both"):
                    out.append(ray(x, y, h + math.pi / 2, L))
                if kind in ("right", "both"):
                    out.append(ray(x, y, h - math.pi / 2, L))
                return out
            yield "t_stub", f"{kind}_{int(length * 100):02d}", {"stub_len": length, "side": kind, "note": "mid-path T, not the finish"}, \
                [("S", 0.4), ("MARK", "b"), ("S", 0.8)], stub, None
    for ang in (20, 35):
        def overlap(m, p, ang=ang):
            # another line shares 0.30 m of the path, then leaves
            x, y, h, _ = m["o0"]
            xb, yb, hb, _ = m["o1"]
            before = list(reversed(ray(x, y, h + math.pi + math.radians(ang), 0.5)))
            shared = ray(x, y, h, math.dist((x, y), (xb, yb)))
            after = ray(xb, yb, hb - math.radians(ang), 0.5)
            return [before + shared[1:] + after[1:]]
        yield "overlap", f"a{ang}", {"angle_deg": ang, "shared_len": 0.30}, \
            [("S", 0.3), ("MARK", "o0"), ("S", 0.30), ("MARK", "o1"), ("S", 0.7)], overlap, None
    for d in (0.0, 0.02, 0.05):
        def kiss(m, p, d=d):
            x, y, h, _ = m["k"]
            n = (-math.sin(h), math.cos(h))
            r = 0.25
            cx, cy = x + n[0] * (r + W + d), y + n[1] * (r + W + d)
            return [[(cx + r * math.cos(a), cy + r * math.sin(a)) for a in [math.tau * i / 120 for i in range(121)]]]
        yield "touching", f"d{int(d * 100):02d}", {"clearance": d, "note": "a circle that touches the path from the side"}, \
            [("S", 0.5), ("MARK", "k"), ("S", 0.8)], kiss, None
    def other_track(m, p):
        x, y, h, _ = m["c"]
        a = h + math.pi / 2
        return [ray(x - 0.8 * math.cos(a), y - 0.8 * math.sin(a), a, 1.6)]
    yield "near_track", "long_crossing", {"note": "another long track crosses at 90 deg"}, [("S", 0.4), ("MARK", "c"), ("S", 0.8)], other_track, None

    # C. paint quality ----------------------------------------------------------
    for g in (0.03, 0.05, 0.10, 0.15, 0.20):
        yield "gap", f"straight_{int(g * 100):02d}", {"gap": g, "on": "straight"}, [("S", 0.4), ("MARK", "g"), ("S", 0.8)], None, \
            (lambda m, g=g: [(m["g"][3], m["g"][3] + g)])
        yield "gap", f"curve_{int(g * 100):02d}", {"gap": g, "on": "arc r 0.25"}, [("A", 0.25, 30), ("MARK", "g"), ("A", 0.25, 60), ("S", 0.5)], None, \
            (lambda m, g=g: [(m["g"][3], m["g"][3] + g)])
    for dash, space in ((0.10, 0.05), (0.05, 0.05), (0.20, 0.10)):
        def dashes(m, dash=dash, space=space):
            s0, out, s = m["d0"][3], [], 0.0
            while s + dash + space < 1.0:
                out.append((s0 + s + dash, s0 + s + dash + space))
                s += dash + space
            return out
        yield "dashed", f"d{int(dash * 100):02d}_s{int(space * 100):02d}", {"dash": dash, "space": space}, \
            [("MARK", "d0"), ("S", 0.5), ("A", 0.3, 60), ("S", 0.4)], None, dashes

    # D. hills (with and without nearby corners) --------------------------------
    for hgt in (0.03, 0.05, 0.07):
        yield "ramp", f"h{int(hgt * 100)}", {"height": hgt}, [("RAMP", 0.44, 0.44, 0.44, hgt), ("S", 0.4)], None, None
    for hgt in (0.03, 0.06):
        for after in (0.0, 0.10, 0.25, 0.45):
            for kind, turn in (("corner90", [("A", 0.10, 90)]), ("uturn", [("A", 0.20, 180)]), ("square", [("SQ", 90)])):
                yield "ramp_then_turn", f"h{int(hgt * 100)}_{kind}_d{int(after * 100):02d}", \
                    {"height": hgt, "turn": kind, "distance_after_ramp": after}, \
                    [("RAMP", 0.44, 0.44, 0.44, hgt), ("S", max(after, 0.001)), *turn, ("S", 0.6)], None, None
        for before in (0.10, 0.30):
            yield "turn_then_ramp", f"h{int(hgt * 100)}_d{int(before * 100):02d}", {"height": hgt, "distance_before_ramp": before}, \
                [("A", 0.10, 90), ("S", before), ("RAMP", 0.44, 0.44, 0.44, hgt), ("S", 0.4)], None, None
    for hgt in (0.03, 0.06):
        def ramp_cross(m, p):
            x, y, h, _ = m["c"]
            a = h + math.pi / 2
            return [ray(x - 0.25 * math.cos(a), y - 0.25 * math.sin(a), a, 0.5)]
        yield "ramp_cross", f"h{int(hgt * 100)}_after", {"height": hgt, "cross": "0.15 m after the ramp"}, \
            [("RAMP", 0.44, 0.44, 0.44, hgt), ("S", 0.15), ("MARK", "c"), ("S", 0.6)], ramp_cross, None
        yield "ramp_gap", f"h{int(hgt * 100)}_deck", {"height": hgt, "gap": 0.08, "on": "ramp deck"}, \
            [("RAMP", 0.44, 0.44, 0.44, hgt), ("S", 0.5)], None, (lambda m: [(LEAD_IN + 0.60, LEAD_IN + 0.68)])
    for hgt in (0.04, 0.07):
        yield "double_ramp", f"h{int(hgt * 100)}", {"height": hgt, "between": 0.30}, \
            [("RAMP", 0.44, 0.30, 0.44, hgt), ("S", 0.30), ("RAMP", 0.44, 0.30, 0.44, hgt), ("S", 0.4)], None, None

    # E. start / finish ---------------------------------------------------------
    for bar in (0.20, 0.40):
        def near_finish_cross(m, p, bar=bar):
            x, y, h, _ = m["c"]
            a = h + math.pi / 2
            return [ray(x - bar / 2 * math.cos(a), y - bar / 2 * math.sin(a), a, bar)]
        yield "finish_like", f"cross_bar{int(bar * 100):02d}", {"bar_len": bar, "note": "a T-like crossing 0.4 m before the finish"}, \
            [("S", 0.6), ("MARK", "c"), ("S", 0.4)], near_finish_cross, None
    yield "finish_like", "after_corner", {"note": "finish T right after a corner"}, [("S", 0.4), ("A", 0.12, 90), ("S", 0.15)], None, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=HERE / "suites" / "cases")
    ap.add_argument("--groups", default="")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--no-mirror", action="store_true")
    ap.add_argument("--preview", action="store_true")
    a = ap.parse_args()
    rng = random.Random(a.seed)
    want = set(filter(None, a.groups.split(",")))
    rows = []
    for group, name, params, prog, extras, gaps in cases(rng):
        if want and group not in want:
            continue
        for mir in ((False,) if a.no_mirror else (False, True)):
            p = mirror_prog(prog) if mir else prog
            ex = extras
            if mir and extras:
                ex = (lambda f: (lambda m, pts: [[(x, -y) for x, y in line] for line in f(
                    {k: (v[0], -v[1], -v[2], v[3]) for k, v in m.items()}, [(x, -y) for x, y in pts])]))(extras)
            center, paint, ramps, gl, feat = build(p, ex, gaps, None)
            full = f"{group}__{name}{'_m' if mir else ''}"
            case = {"group": group, "name": name, "mirror": mir, "params": params, "feature_s": feat["s"]}
            meta = write_track(a.out / "maps", full, center, paint, ramps, gl, case, a.seed)
            rows.append((group, full, meta["case"]["length_m"], len(meta["case"]["crossings"]),
                         json.dumps(params, ensure_ascii=False)))
    a.out.mkdir(parents=True, exist_ok=True)
    (a.out / "list.txt").write_text("".join(f"{(a.out / 'maps' / (r[1] + '.sdf')).relative_to(HERE.parents[1])}\n" for r in rows))
    with (a.out / "index.csv").open("w") as f:
        f.write("group,map,length_m,crossings,params\n")
        for r in rows:
            f.write(f"{r[0]},{r[1]},{r[2]},{r[3]},\"{r[4].replace(chr(34), chr(39))}\"\n")
    groups = {}
    for r in rows:
        groups.setdefault(r[0], []).append(r[1])
    for g, names in groups.items():
        (a.out / f"list_{g}.txt").write_text("".join(f"{(a.out / 'maps' / (n + '.sdf')).relative_to(HERE.parents[1])}\n" for n in names))
    print(f"{len(rows)} tracks in {len(groups)} groups -> {a.out}")
    for g, names in groups.items():
        print(f"  {g:16} {len(names)}")
    if a.preview:
        preview(a.out, rows)


def preview(out, rows):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    groups = {}
    for r in rows:
        groups.setdefault(r[0], []).append(r[1])
    for g, names in groups.items():
        n = len(names)
        cols = min(8, n)
        fig, axs = plt.subplots((n + cols - 1) // cols, cols, figsize=(2.6 * cols, 2.6 * ((n + cols - 1) // cols)), squeeze=False)
        for ax, name in zip(axs.flat, names):
            m = json.loads((out / "maps" / f"{name}.json").read_text())
            for seg in m["painted"]:
                s = list(zip(*[(p[0], p[1]) for p in seg]))
                ax.plot(s[0], s[1], "k-", lw=1.6)
            for seg in m["markings"]:
                s = list(zip(*[(p[0], p[1]) for p in seg]))
                ax.plot(s[0], s[1], "-", color="#c0392b", lw=1.2)
            for r in m["ramps"]:
                c, s_ = math.cos(r["heading"]), math.sin(r["heading"])
                L = r["up"] + r["deck"] + r["down"]
                q = [(r["x0"] + u * c - v * s_, r["y0"] + u * s_ + v * c) for u, v in
                     ((0, -RAMP_W / 2), (L, -RAMP_W / 2), (L, RAMP_W / 2), (0, RAMP_W / 2), (0, -RAMP_W / 2))]
                ax.fill(*zip(*q), color="#e6c26b", alpha=0.5)
            x0, y0 = m["start_pose"][:2]
            ax.plot(x0, y0, "o", color="#2e86c1", ms=4)
            for c_ in m["case"]["crossings"]:
                ax.plot(*c_["xy"], "x", color="#8e44ad", ms=6)
            ax.set_aspect("equal")
            ax.set_title(name.split("__")[1], fontsize=8)
            ax.set_xticks([]); ax.set_yticks([])
        for ax in list(axs.flat)[n:]:
            ax.axis("off")
        fig.suptitle(g)
        fig.tight_layout()
        fig.savefig(out / f"preview_{g}.png", dpi=70)
        plt.close(fig)


if __name__ == "__main__":
    main()
