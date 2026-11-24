#!/usr/bin/env python3
"""Compare the line map built by the user's node with the ground-truth line (dev tool).

usage: compare_map.py MAP_DUMP.npz RUN_DIR OUT.png [WORLD.json]

MAP_DUMP.npz  written by run_user_dump.py (map cells + estimated pose, odom frame)
RUN_DIR       bench run folder with telemetry.npz (true pose x, y, yaw over time)
WORLD.json    generator sidecar (adds T bars / loop markings to the ground truth);
              omitted for the official track (centre line only, from gt.npy).

The odom frame starts at the spawn pose, so odom -> world uses the first true pose.
"""
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from telemetry import COLUMNS  # noqa: E402


def dense(poly, step=0.005):
    poly = np.asarray(poly, float)[:, :2]
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        n = max(1, int(np.ceil(np.linalg.norm(b - a) / step)))
        out += [a + (b - a) * k / n for k in range(1, n + 1)]
    return np.array(out)


def nearest(points, ref, chunk=4000):
    try:
        from scipy.spatial import cKDTree
        return cKDTree(ref).query(points)[0]
    except ImportError:
        pass
    d = np.empty(len(points))
    for i in range(0, len(points), chunk):
        p = points[i:i + chunk]
        d[i:i + chunk] = np.sqrt(((p[:, None, :] - ref[None, :, :]) ** 2).sum(-1)).min(1)
    return d


def main(dump, run, out, world_json=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    z = np.load(dump)
    cells, poses = z["cells"].astype(float), z["poses"]
    t = np.load(Path(run) / "telemetry.npz")
    data, centre = t["data"], t["centerline"]
    c = {k: data[:, i] for i, k in enumerate(COLUMNS)}
    x0, y0, yaw0 = c["x"][0], c["y"][0], c["yaw"][0]
    R = np.array([[np.cos(yaw0), -np.sin(yaw0)], [np.sin(yaw0), np.cos(yaw0)]])
    to_world = lambda p: p @ R.T + np.array([x0, y0])  # noqa: E731
    wcells = to_world(cells)
    est = to_world(poses[:, 1:3]) if len(poses) else np.zeros((0, 2))

    lines = [dense(centre[:, :2])]
    if world_json:
        meta = json.loads(Path(world_json).read_text())
        lines += [dense(m) for m in meta.get("markings", []) if len(m) > 1]
    gt = np.vstack(lines)

    err = nearest(wcells, gt)                   # map cell -> nearest true line point
    seen_gt = gt[nearest(gt, wcells) < 0.015]   # true line covered by the map
    ok1, ok3 = (err < 0.01).mean(), (err < 0.03).mean()

    # drift: node's own pose estimate vs the true path, measured spatially (the two logs
    # start at different times, so time matching would inflate it)
    drift = None
    if len(poses) > 5:
        true_xy = np.column_stack((c["x"], c["y"]))
        drift = nearest(est, true_xy)
        k = np.array([np.argmin(((true_xy - e) ** 2).sum(1)) for e in est])
        drift_s = c["s"][k]

    # shape quality independent of global drift: best rigid fit (ICP) of the map onto the truth
    def icp(src, ref, it=30):
        R_, T_ = np.eye(2), np.zeros(2)
        cur = src.copy()
        for _ in range(it):
            try:
                from scipy.spatial import cKDTree
                idx = cKDTree(ref).query(cur)[1]
            except ImportError:
                idx = np.array([np.argmin(((ref - p) ** 2).sum(1)) for p in cur])
            m = ref[idx]
            mc, cc = m.mean(0), cur.mean(0)
            U, _, Vt = np.linalg.svd((cur - cc).T @ (m - mc))
            r = (U @ Vt).T
            if np.linalg.det(r) < 0:
                Vt[-1] *= -1
                r = (U @ Vt).T
            cur = (cur - cc) @ r.T + mc
        return cur
    aligned = icp(wcells, gt)
    err_al = nearest(aligned, gt)

    fig = plt.figure(figsize=(16, 10))
    g = fig.add_gridspec(2, 3, width_ratios=(1.6, 1, 1))
    ax = fig.add_subplot(g[:, 0])
    ax.plot(gt[:, 0], gt[:, 1], ".", color="0.8", ms=2, label="ground truth line")
    for lo, hi, col, lab in ((0, .01, "tab:green", "map <1 cm"), (.01, .03, "tab:orange", "map 1–3 cm"),
                             (.03, 9, "tab:red", "map >3 cm")):
        m = (err >= lo) & (err < hi)
        ax.plot(wcells[m, 0], wcells[m, 1], ".", color=col, ms=1.5, label=f"{lab} ({m.mean():.0%})")
    ax.plot(c["x"], c["y"], "-", color="k", lw=0.8, label="true robot path")
    if len(est):
        ax.plot(est[:, 0], est[:, 1], "--", color="tab:blue", lw=0.8, label="node's estimated path")
    ax.set_aspect("equal")
    ax.legend(fontsize=8, loc="best", markerscale=6)
    ax.set_title("line map (odom→world) over ground truth")

    h = fig.add_subplot(g[0, 1])
    h.hist(err * 100, bins=np.linspace(0, 8, 41), color="tab:blue")
    h.axvline(1, color="tab:green", ls="--")
    h.axvline(3, color="tab:red", ls="--")
    h.set_xlabel("map cell → nearest true line [cm]")
    h.set_ylabel("cells")
    h.set_title(f"map accuracy: <1 cm {ok1:.0%}, <3 cm {ok3:.0%}")

    d = fig.add_subplot(g[0, 2])
    if drift is not None:
        d.plot(drift_s, drift * 100, lw=1)
        d.set_xlabel("distance along line s [m]")
        d.set_ylabel("distance to true path [cm]")
        d.set_title(f"node's pose estimate vs true path (median {np.median(drift)*100:.1f}, max {drift.max()*100:.1f} cm)")
        d.grid(alpha=.3)

    s = fig.add_subplot(g[1, 1:])
    s.axis("off")
    length = float(centre[-1, 2])
    reach = float(c["progress"].max())
    covered = len(seen_gt) / len(gt)
    txt = [f"run: {Path(run).name}   max progress {reach:.2f} / {length:.2f} m ({100*reach/length:.1f}%)",
           f"map cells: {len(cells)}   (cell = {float(z['res'])*1000:.0f} mm)",
           f"cells within 1 cm of a true line: {ok1:.1%}   within 3 cm: {ok3:.1%}",
           f"median cell error {np.median(err)*100:.2f} cm,  p95 {np.percentile(err,95)*100:.2f} cm",
           f"true line covered by the map (≤1.5 cm): {covered:.1%}  (only what the camera has seen so far)",
           ]
    if drift is not None:
        txt.append(f"node pose vs true path (spatial): median {np.median(drift)*100:.1f} cm, max {drift.max()*100:.1f} cm")
    if not world_json:
        txt.append("note: official track GT = centre line only; T bars at start/finish show as >3 cm.")
    txt.append(f"after best rigid alignment (removes global drift): <1 cm {(err_al<.01).mean():.1%}, "
               f"<3 cm {(err_al<.03).mean():.1%}, median {np.median(err_al)*100:.2f} cm")
    s.text(0, 1, "\n".join(txt), va="top", family="monospace", fontsize=10)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    summary = dict(cells=int(len(cells)), within_1cm=round(ok1, 4), within_3cm=round(ok3, 4),
                   median_cm=round(float(np.median(err)) * 100, 2), p95_cm=round(float(np.percentile(err, 95)) * 100, 2),
                   gt_covered=round(covered, 4), max_progress_m=round(reach, 3),
                   drift_max_cm=round(float(drift.max()) * 100, 1) if drift is not None else None,
                   drift_median_cm=round(float(np.median(drift)) * 100, 1) if drift is not None else None,
                   aligned_within_1cm=round(float((err_al < .01).mean()), 4),
                   aligned_within_3cm=round(float((err_al < .03).mean()), 4))
    print(json.dumps(summary))
    return summary


if __name__ == "__main__":
    if len(sys.argv) < 4:
        raise SystemExit(__doc__)
    main(*sys.argv[1:5])
