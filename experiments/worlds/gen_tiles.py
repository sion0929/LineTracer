#!/usr/bin/env python3
"""Official-style tile tracks: 15 x 15 tiles of 22 cm (3.3 m mat), like the official PDF track.

Each tile the line passes holds one piece:
  straight     edge middle -> edge middle
  corner (ㄱ)  square 90 deg turn at the tile centre
  arc          quarter circle r = 11 cm between two edge middles (4 of them make a circle)
  crossing     the line passes the tile twice at 90 deg (straight both times)
The start tile (col 2, row 14 in the PDF, heading +x) and the finish get the T bars. A hill like
the official one (2 tiles up, 2 flat, 2 down, 6 cm, ~7.8 deg, 3 tiles wide) sits on a straight run
whose side tiles are free.

  python3 experiments/worlds/gen_tiles.py --n 60 --out experiments/worlds/suites/tiles60 --preview
"""
import argparse
import json
import math
import random
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import gen_cases as GC  # noqa: E402

T, N = 0.22, 15
LO, HI = 1, N - 2                                   # usable tiles: keep the outer ring free like the official track
DIRS = [(1, 0), (0, 1), (-1, 0), (0, -1)]          # +x, +y, -x, -y (index = heading / 90 deg)
HILL = dict(up=0.44, deck=0.44, down=0.44, height=0.06, width=0.66)


def walk(rng, length, max_cross, p_straight=0.8, budget=40000):
    """Random tile path from (1, 1) heading +x. Returns cells, exit dirs, crossing set or None."""
    cells, dirs = [(1, 1)], [0]                      # dirs[k] = direction leaving cells[k]
    used = {(1, 1): 1}
    kind = {}                                        # cell -> 'straight-axis' for single straight visits
    cross = set()
    choices = [None]
    steps = 0

    def options(k):
        c, d_in = cells[k], (dirs[k - 1] if k else 0)
        if k < 4:
            return [0]                               # start straight: the camera cannot see 0.31 m ahead
        if c in cross:
            return [d_in]
        ahead = (c[0] + DIRS[d_in][0], c[1] + DIRS[d_in][1])
        if ahead in used:
            return [d_in]                            # touching an earlier lane only to cross it
        opts = [d_in, (d_in + 1) % 4, (d_in + 3) % 4]
        straight_first = rng.random() < p_straight
        turns = opts[1:]
        rng.shuffle(turns)
        return [d_in, *turns] if straight_first else [*turns, d_in]

    stack = [options(0)]
    while stack:
        steps += 1
        if steps > budget:
            return None
        k = len(cells) - 1
        if k + 1 >= length and cells[-1] not in cross:
            return cells, dirs[:k], cross
        if not stack[-1]:
            # backtrack
            stack.pop()
            c = cells.pop()
            dirs.pop()
            used[c] -= 1
            if used[c] == 0:
                del used[c]
            cross.discard(c) if used.get(c, 0) == 1 else None
            if cells:
                prev = cells[-1]
                kind.pop(prev, None)
            continue
        e = stack[-1].pop(0)
        c = cells[-1]
        d_in = dirs[-1] if k else 0
        dx, dy = DIRS[e]
        n = (c[0] + dx, c[1] + dy)
        if not (LO <= n[0] <= HI and LO <= n[1] <= HI):
            continue
        crossing = False
        if n in used:
            # crossing: the earlier visit went straight across this axis, we go straight over it
            if (used[n] != 1 or kind.get(n) != (1 - e % 2) or len(cross) >= max_cross
                    or n == cells[0]):
                continue
            nn = (n[0] + dx, n[1] + dy)
            if not (LO <= nn[0] <= HI and LO <= nn[1] <= HI) or nn in used:
                continue
            crossing = True

        def clear(cell, skip):
            # no side-by-side lanes: a new tile may not touch earlier tiles (except where it came from)
            for ddx, ddy in DIRS:
                q = (cell[0] + ddx, cell[1] + ddy)
                if q in used and q not in skip:
                    return False
            return True
        if not crossing and not clear(n, {c, (n[0] + dx, n[1] + dy)}):
            continue
        if crossing and not clear((n[0] + dx, n[1] + dy), {n}):
            continue
        if k:
            dirs[-1] = e
        else:
            dirs[0] = e
        if e == d_in and k:
            kind[c] = e % 2                          # straight tile, axis 0 = x, 1 = y
        cells.append(n)
        dirs.append(e)
        used[n] = used.get(n, 0) + 1
        if crossing:
            cross.add(n)
        stack.append(options(len(cells) - 1))
    return None


def find_hill(rng, cells, dirs, cross):
    """A straight run of 6 tiles (+2 straight tiles before/after) whose side tiles are free."""
    occupied = set(cells)
    ok = []
    for k in range(3, len(cells) - 9):
        d = dirs[k - 1]
        run = cells[k - 2:k + 8]                     # 2 straight tiles before and after, like the official hill
        if any(dirs[j] != d for j in range(k - 3, k + 8)) or any(c in cross for c in run):
            continue
        px, py = DIRS[(d + 1) % 4]
        sides = [(c[0] + s * px, c[1] + s * py) for c in cells[k:k + 6] for s in (-1, 1)]
        if any(s in occupied for s in sides) or any(not (LO <= s[0] <= HI and LO <= s[1] <= HI) for s in sides):
            continue
        ok.append(k)
    return rng.choice(ok) if ok else None


def program(cells, dirs, turns, hill_k):
    """Tile path -> turtle program for gen_cases.run_program (starts at the start tile centre)."""
    prog, k_last = [("S", T / 2)], len(cells) - 1
    k = 1
    while k < k_last:
        d_in, d_out = dirs[k - 1], dirs[k]
        if hill_k is not None and k == hill_k:
            prog.append(("RAMP", HILL["up"], HILL["deck"], HILL["down"], HILL["height"]))
            k += 6
            continue
        if d_in == d_out:
            prog.append(("S", T))
        else:
            left = (d_out - d_in) % 4 == 1
            ang = 90 if left else -90
            if turns[k] == "arc":
                prog.append(("A", T / 2, ang))
            else:
                prog += [("S", T / 2), ("SQ", ang), ("S", T / 2)]
        k += 1
    prog.append(("S", T / 2))
    merged = []
    for c in prog:
        if c[0] == "S" and merged and merged[-1][0] == "S":
            merged[-1] = ("S", merged[-1][1] + c[1])
        else:
            merged.append(c)
    return merged


def add_decoys(rng, cells, dirs, cross, hill_k, n):
    """Lines that lead nowhere, to confuse the route choice (painted, not part of the route):
      'cross'  - a short line crossing a straight route tile, 1-2 tiles to each side, dead ends
      'branch' - a 1-3 tile stub leaving a straight route tile sideways (a T with no road)
    They keep the same clearance as the route (no side-by-side lanes) and stay off the hill,
    real crossings and the first/last tiles. Returns (polylines in metres, descriptions)."""
    route = set(cells)
    taken = set()
    lines, desc = [], []
    ks = list(range(6, len(cells) - 4))
    rng.shuffle(ks)
    for k in ks:
        if len(lines) >= n:
            break
        c = cells[k]
        if dirs[k - 1] != dirs[k] or c in cross or c in taken:
            continue
        if hill_k is not None and hill_k - 3 <= k <= hill_k + 8:
            continue
        d = dirs[k]
        kind = "cross" if rng.random() < 0.5 else "branch"
        sides = [1, -1] if kind == "cross" else [rng.choice([1, -1])]
        reach = {s: (rng.randint(1, 2) if kind == "cross" else rng.randint(1, 3)) for s in sides}
        px, py = DIRS[(d + 1) % 4]
        stub = []
        ok = True
        for s in sides:
            prev = c
            for m in range(1, reach[s] + 1):
                q = (c[0] + s * m * px, c[1] + s * m * py)
                if not (LO <= q[0] <= HI and LO <= q[1] <= HI) or q in route or q in taken:
                    ok = False; break
                for ddx, ddy in DIRS:              # no contact with the route or other decoys
                    nb = (q[0] + ddx, q[1] + ddy)
                    if nb != prev and nb != c and (nb in route or nb in taken):
                        ok = False
                stub.append(q); prev = q
            if not ok:
                break
        if not ok:
            continue
        taken.add(c); taken.update(stub)
        e1 = (c[0] + sides[0] * reach[sides[0]] * px, c[1] + sides[0] * reach[sides[0]] * py)
        e0 = (c[0] + sides[-1] * reach[sides[-1]] * px, c[1] + sides[-1] * reach[sides[-1]] * py) if kind == "cross" else c
        a = ((e0[0] + 0.5) * T, (e0[1] + 0.5) * T); b = ((e1[0] + 0.5) * T, (e1[1] + 0.5) * T)
        nseg = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / 0.02))
        lines.append([(a[0] + (b[0] - a[0]) * i / nseg, a[1] + (b[1] - a[1]) * i / nseg) for i in range(nseg + 1)])
        desc.append({"kind": kind, "tile": list(c), "route_index": k, "xy": [list(a), list(b)]})
    return lines, desc


def make(rng, idx, out, length, max_cross, p_arc, p_hill, decoys=0, prefix="tile"):
    want_hill = rng.random() < p_hill
    for _ in range(400):
        r = walk(rng, length, max_cross)
        if not r or len(r[2]) < min(1, max_cross):
            continue
        hill_k = find_hill(rng, *r) if want_hill else None
        if hill_k is not None or not want_hill:
            break
    else:
        return None
    cells, dirs, cross = r
    turns = {k: ("arc" if rng.random() < p_arc else "sq") for k in range(len(cells))}
    prog = program(cells, dirs, turns, hill_k)
    pts, ramps, _ = GC.run_program(prog, x=(cells[0][0] + 0.5) * T, y=(cells[0][1] + 0.5) * T, h=0.0)
    extra, ddesc = add_decoys(rng, cells, dirs, cross, hill_k, rng.randint(1, decoys)) if decoys else ([], [])
    for rp in ramps:
        rp["width"] = HILL["width"]
    n_turn = sum(1 for k in range(1, len(cells) - 1) if dirs[k - 1] != dirs[k])
    n_arc = sum(1 for k in range(1, len(cells) - 1) if dirs[k - 1] != dirs[k] and turns[k] == "arc")
    params = {"tiles": len(cells), "turns": n_turn, "arcs": n_arc, "corners": n_turn - n_arc,
              "crossings": len(cross), "hill": hill_k is not None, "decoys": len(ddesc)}
    name = f"{prefix}_{idx:04d}" if prefix != "tile" else f"tile_{idx:03d}"
    case = {"group": "tiles", "name": name, "mirror": False, "params": params, "feature_s": [0, 0]}
    meta = GC.write_track(out / "maps", name, pts, extra, ramps, [], case, idx, mat=(N * T, N * T))
    meta["case"]["cells"] = [list(c) for c in cells]
    meta["case"]["decoys"] = ddesc
    (out / "maps" / f"{name}.json").write_text(json.dumps(meta, separators=(",", ":")) + "\n")
    return name, meta


def preview(out, names):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    cols = min(6, len(names))
    rows = (len(names) + cols - 1) // cols
    fig, axs = plt.subplots(rows, cols, figsize=(3.0 * cols, 3.1 * rows), squeeze=False)
    for ax, name in zip(axs.flat, names):
        m = json.loads((out / "maps" / f"{name}.json").read_text())
        for g in range(N + 1):
            ax.plot([g * T, g * T], [0, N * T], color="#e3ddd0", lw=0.5)
            ax.plot([0, N * T], [g * T, g * T], color="#e3ddd0", lw=0.5)
        for r in m["ramps"]:
            c, s = math.cos(r["heading"]), math.sin(r["heading"])
            L = r["up"] + r["deck"] + r["down"]
            w = HILL["width"] / 2
            q = [(r["x0"] + u * c - v * s, r["y0"] + u * s + v * c) for u, v in
                 ((0, -w), (L, -w), (L, w), (0, w), (0, -w))]
            ax.fill(*zip(*q), color="#e6c26b", alpha=0.6)
        for seg in m["painted"]:
            ax.plot([p[0] for p in seg], [p[1] for p in seg], "k-", lw=1.4)
        for seg in m["markings"]:
            ax.plot([p[0] for p in seg], [p[1] for p in seg], "-", color="#c0392b", lw=1.6)
        for dd in m["case"].get("decoys", []):
            (x0_, y0_), (x1_, y1_) = dd["xy"]
            ax.plot([x0_, x1_], [y0_, y1_], "-", color="#e67e22", lw=2.2)
        x0, y0 = m["start_pose"][:2]
        ax.plot(x0, y0, "o", color="#2e86c1", ms=5)
        for c_ in m["case"]["crossings"]:
            ax.plot(*c_["xy"], "x", color="#8e44ad", ms=7, mew=2)
        p = m["case"]["params"]
        ax.set_title(f'{name}  {m["case"]["length_m"]:.1f} m  ㄱ{p["corners"]} arc{p["arcs"]} X{p["crossings"]}'
                     + (f' D{p.get("decoys", 0)}' if p.get("decoys") else ''),
                     fontsize=8, family=["Noto Sans CJK KR", "DejaVu Sans"])
        ax.set_xlim(0, N * T); ax.set_ylim(0, N * T); ax.set_aspect("equal")
        ax.set_xticks([]); ax.set_yticks([])
    for ax in list(axs.flat)[len(names):]:
        ax.axis("off")
    fig.tight_layout()
    fig.savefig(out / "preview.png", dpi=80)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=60)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", type=Path, default=HERE / "suites" / "tiles60")
    ap.add_argument("--min-tiles", type=int, default=50)
    ap.add_argument("--max-tiles", type=int, default=72)     # official ~ 71 tiles (15.6 m)
    ap.add_argument("--max-cross", type=int, default=2)      # official has 1
    ap.add_argument("--p-arc", type=float, default=0.5)
    ap.add_argument("--p-hill", type=float, default=0.85)
    ap.add_argument("--decoys", type=int, default=0, help="up to this many dead-end crossings / branches per map")
    ap.add_argument("--prefix", default="tile")
    ap.add_argument("--start", type=int, default=0, help="first map index (split big batches across workers)")
    ap.add_argument("--preview", action="store_true")
    a = ap.parse_args()
    a.out = a.out.resolve()
    rng = random.Random(a.seed)
    names, rows = [], []
    for i in range(a.start, a.start + a.n):
        r = make(rng, i, a.out, rng.randint(a.min_tiles, a.max_tiles), rng.randint(1, a.max_cross),
                 a.p_arc, a.p_hill, a.decoys, a.prefix)
        if r is None:
            print("skip", i)
            continue
        name, meta = r
        names.append(name)
        rows.append((name, meta["case"]["length_m"], meta["case"]["params"]))
    root = HERE.parents[1]
    (a.out / "list.txt").write_text("".join(f"{(a.out / 'maps' / (n + '.sdf')).relative_to(root)}\n" for n in names))
    with (a.out / "index.csv").open("w") as f:
        f.write("map,length_m,tiles,corners,arcs,crossings,hill,decoys\n")
        for n, L, p in rows:
            f.write(f"{n},{L},{p['tiles']},{p['corners']},{p['arcs']},{p['crossings']},{int(p['hill'])},{p.get('decoys', 0)}\n")
    print(f"{len(names)} tile tracks -> {a.out}")
    if a.preview:
        preview(a.out, names)


if __name__ == "__main__":
    main()
