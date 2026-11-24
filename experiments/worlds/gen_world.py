#!/usr/bin/env python3
"""Generate deterministic Gazebo Harmonic worlds and ground-truth sidecars."""

import argparse
import json
import math
import random
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

BRINGUP = Path(__file__).resolve().parents[2] / "src/line_follower_bringup"
LINE_WIDTH = 0.02
LINE_LIFT = 0.001
MAT_COLOR = "0.82 0.72 0.55 1"
# Arc radii, number of square corners, ramp height, loop count.
DIFFICULTIES = {
    "easy": ((0.25, 0.34), 1, (0.025, 0.038), 1),
    "normal": ((0.14, 0.27), 2, (0.04, 0.06), 1),
    "hard": ((0.08, 0.17), 3, (0.058, 0.075), 2),
}


def polyline(vertices, radii):
    """Sample axis-aligned legs and exact quarter circles at about 2 cm."""
    points = [vertices[0]]
    for index, (x, y) in enumerate(vertices[1:-1], 1):
        previous, following = vertices[index - 1], vertices[index + 1]
        ux, uy = unit(x - previous[0], y - previous[1])
        vx, vy = unit(following[0] - x, following[1] - y)
        radius = radii[index - 1]
        entry = (x - radius * ux, y - radius * uy)
        extend_line(points, entry)
        if radius:
            cx, cy = entry[0] + radius * vx, entry[1] + radius * vy
            count = math.ceil(math.pi * radius / 0.04)
            for step in range(1, count + 1):
                angle = math.pi / 2 * step / count
                points.append((
                    cx + radius * (ux * math.sin(angle) - vx * math.cos(angle)),
                    cy + radius * (uy * math.sin(angle) - vy * math.cos(angle)),
                ))
    extend_line(points, vertices[-1])
    return points


def unit(x, y):
    length = math.hypot(x, y)
    return x / length, y / length


def extend_line(points, end):
    x, y = points[-1]
    dx, dy = end[0] - x, end[1] - y
    count = math.ceil(math.hypot(dx, dy) / 0.02)
    points.extend((x + dx * i / count, y + dy * i / count) for i in range(1, count + 1))


def add(parent, tag, value=None, **attributes):
    element = ET.SubElement(parent, tag, attributes)
    if value is not None:
        element.text = value if isinstance(value, str) else " ".join(f"{v:.9g}" for v in value)
    return element


def material(visual, rgba, specular="0 0 0 1"):
    mat = add(visual, "material")
    add(mat, "ambient", rgba)
    add(mat, "diffuse", rgba)
    add(mat, "specular", specular)


def surface(link, name, size, pose, color):
    for kind in ("collision", "visual"):
        part = add(link, kind, name=f"{name}_{kind}")
        add(part, "pose", pose)
        add(add(add(part, "geometry"), "box"), "size", size)
        if kind == "visual":
            material(part, color)


def write_line_mesh(path, lines, height):
    triangles = []
    for points in lines:
        # Shared miter vertices keep the width constant through square corners.
        closed = points[0] == points[-1]
        strip = []
        for index, (x, y) in enumerate(points):
            before = points[index - 1] if index else points[-2] if closed else None
            after = points[index + 1] if index + 1 < len(points) else points[1] if closed else None
            incoming = unit(x - before[0], y - before[1]) if before else None
            outgoing = unit(after[0] - x, after[1] - y) if after else None
            u, v = incoming or outgoing, outgoing or incoming
            factor = (LINE_WIDTH / 2) / (1 + u[0] * v[0] + u[1] * v[1])
            nx, ny = -(u[1] + v[1]) * factor, (u[0] + v[0]) * factor
            strip.append(tuple(
                (px, py, height(px, py) + LINE_LIFT)
                for px, py in ((x + nx, y + ny), (x - nx, y - ny))
            ))
        for (left, right), (next_left, next_right) in zip(strip, strip[1:]):
            triangles.extend(((left, right, next_left), (right, next_right, next_left)))
    with path.open("wb") as mesh:
        mesh.write(struct.pack("<80sI", b"Generated track line", len(triangles)))
        for a, b, c in triangles:
            ux, uy, uz = (q - p for p, q in zip(a, b))
            vx, vy, vz = (q - p for p, q in zip(a, c))
            normal = (uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx)
            length = math.hypot(*normal)
            mesh.write(struct.pack("<12fH", *(v / length for v in normal), *a, *b, *c, 0))


def turtle(x, y, heading, program, step=0.02):
    """Sample a path from (x, y, heading) through ('S', length) / ('A', radius, degrees) commands.
    Positive degrees turn left. Returns the sampled points (start excluded)."""
    pts = []
    for cmd in program:
        if cmd[0] == "S":
            n = max(1, round(cmd[1] / step))
            for _ in range(n):
                x += cmd[1] / n * math.cos(heading)
                y += cmd[1] / n * math.sin(heading)
                pts.append((x, y))
        else:
            r, deg = cmd[1], cmd[2]
            side = 1.0 if deg > 0 else -1.0
            cx, cy = x - side * r * math.sin(heading), y + side * r * math.cos(heading)
            a0 = math.atan2(y - cy, x - cx)
            n = max(2, round(abs(math.radians(deg)) * r / step))
            for k in range(1, n + 1):
                a = a0 + math.radians(deg) * k / n
                pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
            x, y = pts[-1]
            heading += math.radians(deg)
    return pts


def generate(seed, difficulty, obstacle_count, output, hills=1, mirror=False,
             small_loops=0, crosses=0, gaps=0, loop_corners=0, detours=0, stubs=0):
    """hills: total ramps (1 = original layout; extra ramps go on later straights).
    mirror: flip the world left-right (corners turn the other way, start heading -x).
    small_loops: teardrop loops on a straight (entry and exit collinear: AMBIGUOUS, a robot
    may legitimately go straight; kept for stress tests only).
    loop_corners: official-style small circle - a square corner replaced by a 270 deg turn
    the other way (r 8-12 cm) that crosses the incoming line at 90 deg.
    detours: connected crossing - the path leaves a straight, loops around, crosses the
    already-driven straight at 90 deg, and rejoins the original road further on.
    stubs: short one-sided branches (T junction mid-path) that must not end the run.
    crosses: short perpendicular lines across straights (intersections to drive through).
    gaps: breaks of 5-15 cm in the painted line (the scoring centreline stays continuous).
    With all extras at their defaults the original files are reproduced byte for byte."""
    rng = random.Random(seed)
    extra = random.Random(seed * 7919 + 104729)   # separate stream: originals stay identical
    arc_range, square_count, height_range, loop_count = DIFFICULTIES[difficulty]
    y_start = 0.33 + rng.uniform(-0.03, 0.03)
    rows = [y_start]
    for _ in range(4):
        rows.append(rows[-1] + rng.uniform(0.71, 0.82))
    left = [0.33 + rng.uniform(-0.035, 0.07) for _ in rows]
    right = [2.93 + rng.uniform(-0.13, 0.24) for _ in rows]
    vertices = [(left[0], rows[0])]
    for i in range(len(rows) - 1):
        edge = right[i] if i % 2 == 0 else left[i]
        vertices.extend(((edge, rows[i]), (edge, rows[i + 1])))
    vertices.append((right[-1], rows[-1]))

    square_indices = set(rng.sample(range(len(vertices) - 2), square_count))
    radii = [
        0.0 if i in square_indices else rng.uniform(*arc_range)
        for i in range(len(vertices) - 2)
    ]
    center = polyline(vertices, radii)
    # Explicit samples at the ramp breakpoints keep both STL and label geometry planar.
    ramp_start = left[0] + rng.uniform(0.43, 0.57)
    ramp_length, deck_length = 0.44, 0.44
    xs = [ramp_start, ramp_start + ramp_length,
          ramp_start + ramp_length + deck_length, ramp_start + 2 * ramp_length + deck_length]
    rise = rng.uniform(*height_range)
    center = [p for p in center if p[1] != rows[0] or all(abs(p[0] - x) > 1e-7 for x in xs)]
    for x in xs:
        index = next(i for i, p in enumerate(center) if p[0] > x)
        center.insert(index, (x, rows[0]))

    ramps = [(0, xs, rise)]                        # (row, breakpoints, height)

    def height(x, y):
        h = 0.0
        for row_i, rxs, rh in ramps:
            if abs(y - rows[row_i]) <= 0.30:
                h = max(h, rh * max(0.0, min(1.0, (x - rxs[0]) / ramp_length, (rxs[-1] - x) / ramp_length)))
        return h

    markings = []
    for x, y in (center[0], center[-1]):
        markings.append([(x, y - 0.10), (x, y + 0.10)])
    loops = []
    for row in rng.sample((1, 2, 3), loop_count):
        cx = rng.uniform(1.15, 1.48) if row % 2 else rng.uniform(2.08, 2.35)
        rx = rng.uniform(0.30, 0.42)
        cy = (rows[row] + rows[row + 1]) / 2
        ry = (rows[row + 1] - rows[row]) / 2 + rng.uniform(0.13, 0.20)
        points = [(cx + rx * math.cos(i * math.tau / 160),
                   cy + ry * math.sin(i * math.tau / 160)) for i in range(160)]
        markings.append(points + points[:1])
        loops.append((cx, rx, row))

    # Official-style small circles at square corners.
    corner_loops = []
    sq = [k for k in range(1, len(vertices) - 1) if radii[k - 1] == 0.0]
    extra.shuffle(sq)
    for k in sq[:loop_corners]:
        V = vertices[k]
        idx = next((i for i, p in enumerate(center) if math.dist(p, V) < 1e-9), None)
        if idx is None or idx == 0 or idx == len(center) - 1:
            continue
        ix, iy = unit(V[0] - center[idx - 1][0], V[1] - center[idx - 1][1])
        ox, oy = unit(center[idx + 1][0] - V[0], center[idx + 1][1] - V[1])
        turn = 1.0 if ix * oy - iy * ox > 0 else -1.0        # normal turn direction
        r = extra.uniform(0.08, 0.12)
        heading = math.atan2(iy, ix)
        loop = turtle(V[0], V[1], heading, [("S", r), ("A", r, -turn * 270), ("S", r)])
        # loop ends back at V heading out: the second visit of V is the 90 deg crossing
        if math.dist(loop[-1], V) > 0.02:
            continue
        loop[-1] = V
        center[idx + 1:idx + 1] = loop
        corner_loops.append({"vertex": list(V), "r": r})

    # Extra hills on later straights, clear of corner arcs and loop crossings.
    def row_span(row):
        a, b = vertices[2 * row][0], vertices[2 * row + 1][0]
        sign = math.copysign(1.0, b - a)
        a += sign * (radii[2 * row - 1] if row else 0)
        b -= sign * (radii[2 * row] if row < len(rows) - 1 else 0)
        return min(a, b), max(a, b)

    extra_deck = 0.25                              # shorter top on extra hills so they fit
    span_len = 2 * ramp_length + extra_deck
    for row in extra.sample(range(1, len(rows)), min(max(hills - 1, 0), len(rows) - 1)):
        lo, hi = row_span(row)
        lo, hi = lo + 0.20, hi - 0.20 - span_len
        blocked = [(cx - rx - 0.10, cx + rx + 0.10) for cx, rx, i in loops if row in (i, i + 1)]
        options = [x0 / 100 for x0 in range(int(lo * 100), int(hi * 100) + 1)
                   if all(x0 / 100 + span_len < b0 or x0 / 100 > b1 for b0, b1 in blocked)]
        if not options:
            continue
        x0 = extra.choice(options)
        rxs = [x0, x0 + ramp_length, x0 + ramp_length + extra_deck, x0 + span_len]
        ramps.append((row, rxs, extra.uniform(*height_range)))
        on_row = [i for i, p in enumerate(center) if abs(p[1] - rows[row]) < 1e-9]
        forward = center[on_row[-1]][0] > center[on_row[0]][0]
        for x in rxs:
            seg = [i for i in on_row if (center[i][0] > x if forward else center[i][0] < x)]
            if seg:
                center.insert(seg[0], (x, rows[row]))
                on_row = [i for i, p in enumerate(center) if abs(p[1] - rows[row]) < 1e-9]

    # Tight self-crossing loops (prolate cycloid, one period) inserted into straights.
    # x = x0 + r*t - d*sin t, y = row +/- (d - d*cos t ... ) starting/ending tangent to the row.
    busy = {r_row: [(rxs[0] - 0.25, rxs[-1] + 0.25)] for r_row, rxs, _ in ramps}
    for cx, rx, i in loops:
        for r_row in (i, i + 1):
            busy.setdefault(r_row, []).append((cx - rx - 0.15, cx + rx + 0.15))
    tight = []
    for _ in range(small_loops):
        r_cyc, d_cyc = extra.uniform(0.045, 0.06), extra.uniform(0.16, 0.19)
        width_l = 2 * math.pi * r_cyc
        rows_ok = [r for r in range(1, len(rows))]
        extra.shuffle(rows_ok)
        placed = False
        for row in rows_ok:
            lo, hi = row_span(row)
            lo, hi = lo + 0.30, hi - 0.30 - width_l
            opts = [v / 100 for v in range(int(lo * 100), int(hi * 100) + 1)
                    if all(v / 100 + width_l < b0 or v / 100 > b1 for b0, b1 in busy.get(row, []))]
            if not opts:
                continue
            x0 = extra.choice(opts)
            on_row = [i for i, p in enumerate(center) if abs(p[1] - rows[row]) < 1e-9]
            forward = center[on_row[-1]][0] > center[on_row[0]][0]
            side = -1.0 if row < len(rows) - 1 else -1.0   # hang toward the previous row's gap
            if row == len(rows) - 1:
                side = -1.0
            gap_rows = rows[row] - rows[row - 1]
            if 2 * d_cyc + 0.12 > gap_rows:
                continue
            n = 120
            pts = []
            for k in range(n + 1):
                t = math.pi + 2 * math.pi * k / n
                u = r_cyc * (t - math.pi) - d_cyc * math.sin(t)     # along the row, 0..width_l
                v = d_cyc * (1 + math.cos(t))                        # 0 at both ends, 2d in the middle
                x = x0 + u if forward else x0 + width_l - u
                pts.append((x, rows[row] + side * v))
            a, b = (x0, x0 + width_l)
            keep_before = [i for i in on_row if (center[i][0] < a if forward else center[i][0] > b)]
            keep_after = [i for i in on_row if (center[i][0] > b if forward else center[i][0] < a)]
            cut_from = keep_before[-1] + 1 if keep_before else on_row[0]
            cut_to = keep_after[0] if keep_after else on_row[-1] + 1
            center[cut_from:cut_to] = pts
            busy.setdefault(row, []).append((a - 0.25, b + 0.25))
            busy.setdefault(row - 1, []).append((a - 0.25, b + 0.25))
            tight.append({"row": row, "x": [a, b], "r": r_cyc, "d": d_cyc})
            placed = True
            break
        if not placed:
            break

    # Connected crossings: leave a middle straight, go round, cross the driven straight
    # at 90 deg, come back under it and rejoin the original road.
    detour_list = []
    for _ in range(detours):
        mids = [r_ for r_ in range(1, len(rows) - 1)]
        extra.shuffle(mids)
        for row in mids:
            r = extra.uniform(0.08, 0.11)
            H = min(extra.uniform(2 * r + 0.06, 0.32), rows[row + 1] - rows[row] - 0.22)
            D = min(extra.uniform(2 * r + 0.06, 0.32), rows[row] - rows[row - 1] - 0.22)
            W = extra.uniform(0.35, 0.55)
            E = extra.uniform(0.10, 0.25)
            lo, hi = row_span(row)
            on_row = [i for i, p in enumerate(center) if abs(p[1] - rows[row]) < 1e-9]
            if len(on_row) < 5:
                continue
            forward = center[on_row[-1]][0] > center[on_row[0]][0]
            sgn = 1.0 if forward else -1.0
            need = W + r + E + 2 * r
            # L1 in travel coordinates measured from the row's first point
            first_x = center[on_row[0]][0]
            row_len = abs(center[on_row[-1]][0] - first_x)
            opts = [v / 100 for v in range(int((W + 0.35) * 100), int((row_len - (r + E + r) - 0.35) * 100))]
            # busy[row] already holds this row's hills, the crossing ellipses in the gaps above
            # and below it, and earlier features: the detour's x-span must clear all of them
            def clear(u):
                xa, xb = sorted((first_x + sgn * (u - W), first_x + sgn * (u + r + E + r)))
                return all(xb + 0.1 < b0 or xa - 0.1 > b1 for b0, b1 in busy.get(row, []))
            opts = [u for u in opts if clear(u)]
            if not opts or H < 2 * r + 0.04 or D < 2 * r + 0.04:
                continue
            u1 = extra.choice(opts)
            x1 = first_x + sgn * u1
            heading = 0.0 if forward else math.pi
            left = 1.0 if forward else -1.0       # +v (up) is a left turn when heading +x
            prog = [("A", r, 90 * left), ("S", H - 2 * r), ("A", r, 90 * left), ("S", W - r),
                    ("A", r, 90 * left), ("S", H - r + D - r), ("A", r, 90 * left),
                    ("S", W - 2 * r + r + E), ("A", r, 90 * left), ("S", D - 2 * r), ("A", r, -90 * left)]
            path = turtle(x1, rows[row], heading, prog)
            x_end = path[-1][0]
            if abs(path[-1][1] - rows[row]) > 0.02:
                continue
            path[-1] = (x_end, rows[row])
            cut_a = [i for i in on_row if (center[i][0] <= x1 + 1e-9 if forward else center[i][0] >= x1 - 1e-9)]
            cut_b = [i for i in on_row if (center[i][0] > x_end if forward else center[i][0] < x_end)]
            if not cut_a or not cut_b:
                continue
            center[cut_a[-1] + 1:cut_b[0]] = path
            span = sorted([x1 - sgn * (W + 0.1), x_end + sgn * 0.1])
            for rr in (row - 1, row, row + 1):
                busy.setdefault(rr, []).append((span[0] - 0.1, span[1] + 0.1))
            detour_list.append({"row": row, "x": span, "r": r, "W": W, "H": H, "D": D})
            break

    # One-sided branches (mid-path T junctions) that are not the finish.
    for _ in range(stubs):
        row = extra.randrange(1, len(rows))
        lo, hi = row_span(row)
        opts = [v / 100 for v in range(int((lo + 0.3) * 100), int((hi - 0.3) * 100) + 1)
                if all(not (b0 - 0.1 <= v / 100 <= b1 + 0.1) for b0, b1 in busy.get(row, []))]
        if opts:
            x = extra.choice(opts)
            side = extra.choice((-1, 1))
            length = extra.uniform(0.18, 0.32)
            markings.append([(x, rows[row]), (x, rows[row] + side * length)])
            busy.setdefault(row, []).append((x - 0.2, x + 0.2))

    # Perpendicular crossing bars across straights (drive straight through them).
    for _ in range(crosses):
        row = extra.randrange(1, len(rows))
        lo, hi = row_span(row)
        opts = [v / 100 for v in range(int((lo + 0.3) * 100), int((hi - 0.3) * 100) + 1)
                if all(not (b0 - 0.1 <= v / 100 <= b1 + 0.1) for b0, b1 in busy.get(row, []))]
        if opts:
            x = extra.choice(opts)
            half = extra.uniform(0.15, 0.25)
            markings.append([(x, rows[row] - half), (x, rows[row] + half)])
            busy.setdefault(row, []).append((x - 0.2, x + 0.2))

    # Keep obstacles on level, straight sections with room to drive around them.
    straight_bounds = []
    for row in range(len(rows)):
        a, b = vertices[2 * row][0], vertices[2 * row + 1][0]
        sign = math.copysign(1.0, b - a)
        a += sign * (radii[2 * row - 1] if row else 0)
        b -= sign * (radii[2 * row] if row < len(rows) - 1 else 0)
        straight_bounds.append((min(a, b) + 0.25, max(a, b) - 0.30))
    candidates = [
        (x / 10, row)
        for row in range(1, len(rows))
        for x in range(10, 29)
        if straight_bounds[row][0] <= x / 10 <= straight_bounds[row][1]
        and all(row not in (i, i + 1) or abs(x / 10 - cx) > rx + 0.23
               for cx, rx, i in loops)
        and all(r_row != row or not (rxs[0] - 0.30 <= x / 10 <= rxs[-1] + 0.30)
                for r_row, rxs, _ in ramps[1:])
        and (not (small_loops or crosses)
             or all(not (b0 - 0.1 <= x / 10 <= b1 + 0.1) for b0, b1 in busy.get(row, [])[len(
                 [1 for r_row, _, _ in ramps if r_row == row]) + len(
                 [1 for cx, rx, i in loops if row in (i, i + 1)]):]))
    ]
    rng.shuffle(candidates)
    boxes = []
    while len(boxes) < obstacle_count and candidates:
        x, row = candidates.pop()
        y = rows[row]
        if any(math.hypot(x - box["pose"][0], y - box["pose"][1]) < 0.50 for box in boxes):
            continue
        if boxes and rng.random() > 0.55:
            y += rng.choice((-1, 1)) * rng.uniform(0.22, 0.28)
        size = [rng.uniform(0.11, 0.19), rng.uniform(0.11, 0.18), rng.uniform(0.12, 0.22)]
        boxes.append({
            "name": f"obstacle_{len(boxes) + 1:02}",
            "pose": [x, y, size[2] / 2, 0, 0, rng.uniform(-0.30, 0.30)],
            "size": size,
            "rgba": [rng.uniform(0.18, 0.76) for _ in range(3)] + [1.0],
            "on_line": y == rows[row],
        })
    if len(boxes) < obstacle_count:
        raise ValueError(f"requested {obstacle_count} obstacles, found room for {len(boxes)}")

    arc = [0.0]
    for a_, b_ in zip(center, center[1:]):
        arc.append(arc[-1] + math.dist(a_, b_))
    gap_list = []
    if gaps:
        # only on level straight stretches away from start/finish, hills, loops and corners
        cand = [i for i in range(1, len(center) - 1)
                if 1.0 < arc[i] < arc[-1] - 1.0
                and abs(center[i][1] - center[i - 1][1]) < 1e-9 and abs(center[i][1] - center[i + 1][1]) < 1e-9
                and height(*center[i]) == 0.0
                and not any(abs(center[i][1] - rows[r]) < 1e-9 and b0 - 0.2 <= center[i][0] <= b1 + 0.2
                            for r in range(len(rows)) for b0, b1 in busy.get(r, []))]
        tries = 0
        while len(gap_list) < gaps and cand and tries < 200:
            tries += 1
            i = extra.choice(cand)
            g0 = arc[i]
            g1 = g0 + extra.uniform(0.05, 0.15)
            j = next((k for k in range(i, len(arc)) if arc[k] >= g1), None)
            if j is None or any(abs(center[k][1] - center[i][1]) > 1e-9 for k in range(i, j + 1)):
                continue
            if any(not (g1 + 0.5 < a0 or g0 > a1 + 0.5) for a0, a1 in gap_list):
                continue
            gap_list.append((g0, g1))
        gap_list.sort()
    bounds = [0, 0, max(right) + 0.40, rows[-1] + 0.43]
    spawn_yaw = 0
    if mirror:
        width = bounds[2]
        flat_height = height
        center = [(width - x, y) for x, y in center]
        markings = [[(width - x, y) for x, y in m] for m in markings]
        ramps = [(r, sorted(width - x for x in rxs), rh) for r, rxs, rh in ramps]
        height = lambda x, y: flat_height(width - x, y)  # noqa: E731
        for box in boxes:
            box["pose"][0] = width - box["pose"][0]
            box["pose"][5] = -box["pose"][5]
        spawn_yaw = math.pi
        for t_ in tight:
            t_["x"] = sorted(width - x for x in t_["x"])
        for c_ in corner_loops:
            c_["vertex"][0] = width - c_["vertex"][0]
        for d_ in detour_list:
            d_["x"] = sorted(width - x for x in d_["x"])
    xs, rise = ramps[0][1], ramps[0][2]

    painted = [center]
    if gap_list:
        painted, cur = [], []
        for k, p_ in enumerate(center):
            if any(a0 < arc[k] < a1 for a0, a1 in gap_list):
                if len(cur) > 1:
                    painted.append(cur)
                cur = []
            else:
                cur.append(p_)
        if len(cur) > 1:
            painted.append(cur)

    output.parent.mkdir(parents=True, exist_ok=True)
    mesh = output.with_suffix(".stl")
    write_line_mesh(mesh, [*painted, *markings], height)
    tree = ET.parse(BRINGUP / "worlds/line_track.sdf")
    world = tree.getroot().find("world")
    world.remove(world.find("gui"))
    track_include, robot = world.findall("include")
    world.remove(track_include)
    spawn = [*center[0], 0.01, 0, 0, spawn_yaw]
    robot.find("pose").text = " ".join(f"{v:.9g}" for v in spawn)
    track = add(world, "model", name="track")
    add(track, "static", "true")
    link = add(track, "link", name="surface")
    surface(link, "mat", (bounds[2], bounds[3], 0.1),
            (bounds[2] / 2, bounds[3] / 2, -0.05, 0, 0, 0), MAT_COLOR)
    k = 0
    for r_row, rxs, rh in ramps:
        for start, end, top, slope in zip(
            rxs, rxs[1:], (rh / 2, rh, rh / 2),
            (math.atan2(rh, ramp_length), 0, -math.atan2(rh, ramp_length)),
        ):
            thickness = rh + 0.015
            pose = ((start + end) / 2 + math.sin(slope) * thickness / 2, rows[r_row],
                    top - math.cos(slope) * thickness / 2, 0, -slope, 0)
            surface(link, f"ramp_{k}", ((end - start) / math.cos(slope), 0.60, thickness), pose, MAT_COLOR)
            k += 1
    visual = add(add(track, "link", name="line"), "visual", name="visual")
    add(visual, "cast_shadows", "false")
    add(add(add(visual, "geometry"), "mesh"), "uri", f"./{mesh.name}")
    material(visual, "0 0 0 1", "0.1 0.1 0.1 1")
    for box in boxes:
        model = add(world, "model", name=box["name"])
        add(model, "static", "true")
        add(model, "pose", box["pose"])
        surface(add(model, "link", name="body"), "box", box["size"], (0,) * 6, box["rgba"])
    ET.indent(tree)
    tree.write(output, encoding="utf-8", xml_declaration=True)
    arc = [0.0]
    for a, b in zip(center, center[1:]):
        arc.append(arc[-1] + math.dist(a, b))
    metadata = {
        "version": 1, "world": world.get("name"), "sdf": output.name,
        "seed": seed, "difficulty": difficulty,
        "start_pose": spawn, "centerline": [[x, y, height(x, y)] for x, y in center],
        "arc_m": arc,
        "markings": [[[x, y, height(x, y)] for x, y in points] for points in markings],
        "line_width_m": LINE_WIDTH, "line_lift_m": LINE_LIFT, "mat_bounds": bounds,
        "obstacles": boxes, "corner_radii_m": radii,
        "ramp": {"x": xs, "y": [rows[0] - 0.30, rows[0] + 0.30], "height_m": rise},
    }
    if small_loops or crosses or gaps or loop_corners or detours or stubs:
        metadata["small_loops"] = tight
        metadata["loop_corners"] = corner_loops
        metadata["detours"] = detour_list
        metadata["gaps_arc_m"] = [list(g) for g in gap_list]
        metadata["painted"] = [[[x, y, height(x, y)] for x, y in seg] for seg in painted]
    if hills != 1 or mirror:
        metadata["ramps"] = [{"x": rxs, "y": [rows[r] - 0.30, rows[r] + 0.30], "height_m": rh}
                             for r, rxs, rh in ramps]
        metadata["mirror"] = bool(mirror)
    output.with_suffix(".json").write_text(json.dumps(metadata, separators=(",", ":")) + "\n")
    return metadata


def draw(ax, metadata):
    from matplotlib.patches import Rectangle
    from matplotlib.transforms import Affine2D

    x0, y0, x1, y1 = metadata["mat_bounds"]
    ax.set_facecolor("#d1b88c")
    ax.set(xlim=(x0, x1), ylim=(y0, y1), aspect="equal", xticks=[], yticks=[])
    for ramp in metadata.get("ramps", [metadata["ramp"]]):
        ax.add_patch(Rectangle((ramp["x"][0], ramp["y"][0]), ramp["x"][-1] - ramp["x"][0],
                               ramp["y"][1] - ramp["y"][0], color="#bda06f"))
    for points in [*metadata.get("painted", [metadata["centerline"]]), *metadata["markings"]]:
        x, y, _ = zip(*points)
        ax.plot(x, y, color="#171411", linewidth=1)
    for obstacle in metadata["obstacles"]:
        x, y, _, _, _, yaw = obstacle["pose"]
        width, depth, _ = obstacle["size"]
        patch = Rectangle((x - width / 2, y - depth / 2), width, depth,
                          facecolor=obstacle["rgba"], edgecolor="#442e28", linewidth=0.6)
        patch.set_transform(Affine2D().rotate_around(x, y, yaw) + ax.transData)
        ax.add_patch(patch)
    for point, label in [(metadata["centerline"][0], "S"), (metadata["centerline"][-1], "F")]:
        ax.annotate(label, point[:2], xytext=(3, 3), textcoords="offset points", fontsize=7)
    ax.set_title(f'{metadata["difficulty"]} · {metadata["seed"]} · {metadata["arc_m"][-1]:.1f} m',
                 fontsize=8)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--difficulty", choices=DIFFICULTIES, default="normal")
    parser.add_argument("--obstacles", type=int, default=0)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--hills", type=int, default=1, help="number of ramps (default 1 = original)")
    parser.add_argument("--mirror", action="store_true", help="flip left-right (turns reversed)")
    parser.add_argument("--small-loops", type=int, default=0, help="teardrop loops on straights (ambiguous)")
    parser.add_argument("--loop-corners", type=int, default=0, help="official-style small circles at corners")
    parser.add_argument("--detours", type=int, default=0, help="connected crossings returning to the road")
    parser.add_argument("--stubs", type=int, default=0, help="one-sided branches (mid-path T)")
    parser.add_argument("--crosses", type=int, default=0, help="perpendicular crossing bars")
    parser.add_argument("--gaps", type=int, default=0, help="breaks (5-15 cm) in the painted line")
    parser.add_argument("--preview", action="store_true", help="save a PNG alongside the SDF")
    args = parser.parse_args()
    output = args.output or Path(f"experiments/worlds/generated/{args.difficulty}_{args.seed}.sdf")
    if output.suffix != ".sdf" or args.obstacles < 0:
        parser.error("output must end in .sdf and obstacles must be nonnegative")
    try:
        metadata = generate(args.seed, args.difficulty, args.obstacles, output, args.hills, args.mirror,
                            args.small_loops, args.crosses, args.gaps, args.loop_corners,
                            args.detours, args.stubs)
    except ValueError as error:
        parser.error(str(error))
    if args.preview:
        import matplotlib
        matplotlib.use("Agg")
        from matplotlib import pyplot as plt

        fig, ax = plt.subplots(figsize=(5, 5), layout="constrained")
        draw(ax, metadata)
        fig.savefig(output.with_suffix(".png"), dpi=150)
        plt.close(fig)
    print(f'{output}  {metadata["arc_m"][-1]:.2f} m  {len(metadata["obstacles"])} obstacles')


if __name__ == "__main__":
    main()
