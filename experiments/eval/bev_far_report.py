#!/usr/bin/env python3
"""BEV range comparison on the official track (dev only, uses ground truth).

Inputs: bench runs of experiments/user_base/run_xfar.py (map_world.npz: map cells placed in the
world with the truth-vs-odom transform at the time, so odometry drift does not count).
GT: the official line mesh (all line area incl. T bars), sampled on a 5 mm grid.
  python3 experiments/eval/bev_far_report.py RUNS_DIR OUT_DIR
"""
import json, struct, sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from scipy.spatial import cKDTree

ROOT = Path(__file__).resolve().parents[2]
for f in (ROOT / "experiments/results/lead/fonts").glob("*.tt*"):
    font_manager.fontManager.addfont(str(f))
fam = [f.name for f in font_manager.fontManager.ttflist if "CJK" in f.name]
plt.rcParams["font.family"] = fam[:1] + ["DejaVu Sans"]
D = Path(sys.argv[1]); OUT = Path(sys.argv[2]); OUT.mkdir(parents=True, exist_ok=True)


def mesh_points(path, step=0.004):
    b = path.read_bytes(); n = struct.unpack("<I", b[80:84])[0]
    a = np.frombuffer(b[84:84 + 50 * n], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("x", "<u2")]))
    pts = []
    for t in a["v"].reshape(-1, 3, 3)[:, :, :2].astype(float):
        area = abs(np.cross(t[1] - t[0], t[2] - t[0])) / 2
        k = max(3, int(area / step ** 2 * 2))
        r = np.random.default_rng(0).random((k, 2)); f = r.sum(1) > 1; r[f] = 1 - r[f]
        pts.append(t[0] + r[:, :1] * (t[1] - t[0]) + r[:, 1:] * (t[2] - t[0]))
    p = np.vstack(pts)
    return np.unique(np.round(p / 0.005), axis=0) * 0.005


GT = mesh_points(ROOT / "src/line_follower_bringup/models/track/meshes/track.stl")
gtree = cKDTree(GT)
centre = np.load(ROOT / "experiments/gt.npy")[:, :2]
# curvature class for GT points: near a corner/arc = heading of the polyline changes within 0.25 m
sys.path.insert(0, str(Path(__file__).resolve().parent))
from line_follower_helpers import turn_mask  # noqa: E402  (tiny helper below, same folder)
TURN = turn_mask(GT, centre)

rows, per = [], {}
for run in sorted(p for p in D.iterdir() if (p / "job.json").exists()):
    if not (run / "map_world.npz").exists() or not (run / "result.json").exists():
        continue
    job = json.loads((run / "job.json").read_text()); r = json.loads((run / "result.json").read_text())
    if r.get("reason", "").startswith("error:"):
        continue                       # infrastructure error, not a drive
    if not job["map"].endswith("worlds/line_track.sdf"):
        continue                       # map metrics use the official line mesh
    z = np.load(run / "map_world.npz")
    xy, dist, tseen, track = z["xy"], z["dist"], z["t"], z["track"]
    err = gtree.query(xy)[0]
    good = err < 0.02
    ctree = cKDTree(xy[good])
    dgt, idx = ctree.query(GT)
    cov = dgt < 0.01
    # seen ahead: GT point recorded before the robot (truth) came within 0.35 m of it
    tr = cKDTree(track[:, 1:3])
    near = tr.query_ball_point(GT, 0.35)
    t_arr = np.array([track[min(l), 0] if l else np.inf for l in near])
    t_seen = np.where(cov, tseen[good][idx], np.inf)
    ahead = cov & (t_seen < t_arr)
    reached = np.isfinite(t_arr)
    # looser: the shape ahead known within 5 cm (far cells are coarse; near frames refine them)
    ok5 = err < 0.05
    d5, i5 = cKDTree(xy[ok5]).query(GT) if ok5.any() else (np.full(len(GT), 9.0), np.zeros(len(GT), int))
    ahead5 = (d5 < 0.03) & (np.where(d5 < 0.03, tseen[ok5][i5] if ok5.any() else np.inf, np.inf) < t_arr)
    rows.append(dict(profile=job["profile"], run=run.name, complete=r["complete"], lap=r["lap_sec"],
                     progress=r["progress_pct"], lat_mean_cm=r["lat_mean"] * 100, lat_p95_cm=r["lat_p95"] * 100,
                     covered=float(cov.mean()), cells=int(len(xy)), cell_in2=float(good.mean()),
                     ahead_turn=float(ahead[TURN & reached].mean()), ahead_all=float(ahead[reached].mean()),
                     ahead5_turn=float(ahead5[TURN & reached].mean()), ahead5_all=float(ahead5[reached].mean()),
                     first_dist_med=float(np.median(dist[good]))))
    if not r["complete"]:
        continue                       # map metrics and figures: completed laps only (same track seen)
    per.setdefault(job["profile"], []).append(dict(xy=xy, good=good, cov=cov, ahead=ahead, reached=reached, name=run.name,
                                                   dist=dist[good], dist_all=dist, err=err))
json.dump(rows, open(OUT / "bev_far_runs.json", "w"), indent=1)
profs = sorted(per)
agg = {}
for p in profs:
    rr = [r for r in rows if r["profile"] == p]
    laps = [r["lap"] for r in rr if r["complete"]]
    agg[p] = {"runs": len(rr), "done": sum(r["complete"] for r in rr), "lap": float(np.mean(laps)) if laps else None,
              **{k: float(np.mean([r[k] for r in rr if r["complete"]])) for k in ("covered", "cell_in2", "ahead_turn", "ahead_all", "ahead5_turn", "ahead5_all",
                                                                 "first_dist_med", "lat_mean_cm", "lat_p95_cm")}}
json.dump(agg, open(OUT / "bev_far_agg.json", "w"), indent=1)
for p in profs:
    a = agg[p]
    print(f'{p} {a["done"]}/{a["runs"]} lap {a["lap"]} cov {a["covered"]:.3f} in2 {a["cell_in2"]:.3f} '
          f'ahead_turn {a["ahead_turn"]:.3f} ahead5_turn {a["ahead5_turn"]:.3f} ahead5_all {a["ahead5_all"]:.3f} dist {a["first_dist_med"]:.2f} '
          f'lat {a["lat_mean_cm"]:.2f}/{a["lat_p95_cm"]:.2f}')

LABELS = json.loads(sys.argv[3]) if len(sys.argv) > 3 else {}
def lab(p):
    if p in LABELS:
        return LABELS[p]
    return f"BEV {int(p[1:]) / 100:.2f} m" if p[1:].isdigit() else p
# fig 1: maps (dark), best-covered complete run per profile
fig, axs = plt.subplots(1, len(profs), figsize=(5.0 * len(profs), 5.6), facecolor="#14181c")
for ax, p in zip(np.atleast_1d(axs), profs):
    m = max(per[p], key=lambda m: m["cov"].mean())
    ax.set_facecolor("#14181c")
    ax.plot(GT[:, 0], GT[:, 1], ".", color="#5c6670", ms=0.8)
    ax.plot(m["xy"][m["good"], 0], m["xy"][m["good"], 1], ".", color="#4a8fe7", ms=0.7)
    ax.plot(m["xy"][~m["good"], 0], m["xy"][~m["good"], 1], ".", color="#e5484d", ms=1.6)
    miss = GT[~m["cov"]]
    ax.plot(miss[:, 0], miss[:, 1], ".", color="#f5a524", ms=1.6)
    ax.set_aspect("equal"); ax.set_xticks([]); ax.set_yticks([])
    for s in ax.spines.values():
        s.set_color("#444")
    ax.set_title(f'{lab(p)}  ·  복원 {agg[p]["covered"]:.1%}  ·  2 cm 밖 {1 - agg[p]["cell_in2"]:.1%}', color="w", fontsize=11)
fig.legend(handles=[plt.Line2D([], [], ls="", marker="o", color=c, label=l) for c, l in
                    (("#5c6670", "정답 선 (트랙 메시)"), ("#4a8fe7", "지도에 기록된 선 (2 cm 이내)"),
                     ("#e5484d", "지도에서 2 cm 밖"), ("#f5a524", "지도에 없는 정답 선"))],
           loc="lower center", ncol=4, facecolor="#14181c", labelcolor="w", edgecolor="#444", fontsize=10)
fig.suptitle("공식 트랙 한 바퀴 선 지도 — BEV 거리별 (규칙 인지 640×480, rule.yaml)", color="w", fontsize=13)
fig.tight_layout(rect=(0, 0.06, 1, 0.95))
fig.savefig(OUT / "bev_far_maps.png", dpi=100, facecolor=fig.get_facecolor())

# fig 2: seen-ahead map vs 0.85 (majority over runs)
base = profs[0]
A0 = np.mean([m["ahead"] for m in per[base]], 0) > 0.5
fig, axs = plt.subplots(1, len(profs) - 1, figsize=(5.0 * (len(profs) - 1), 5.4))
for ax, p in zip(np.atleast_1d(axs), profs[1:]):
    Ap = np.mean([m["ahead"] for m in per[p]], 0) > 0.5
    gain, loss = Ap & ~A0, A0 & ~Ap
    ax.plot(GT[:, 0], GT[:, 1], ".", color="0.85", ms=0.8)
    ax.plot(GT[Ap & A0, 0], GT[Ap & A0, 1], ".", color="#9fb7d6", ms=0.8)
    ax.plot(GT[gain, 0], GT[gain, 1], ".", color="#1f6f4a", ms=2)
    ax.plot(GT[loss, 0], GT[loss, 1], ".", color="#c0392b", ms=2)
    ax.set_aspect("equal"); ax.set_xticks([]); ax.set_yticks([])
    ax.set_title(f"{lab(p)} vs 0.85 m\n미리 본 선 +{gain.mean():.1%}p / −{loss.mean():.1%}p", fontsize=11)
fig.legend(handles=[plt.Line2D([], [], ls="", marker="o", color=c, label=l) for c, l in
                    (("#9fb7d6", "둘 다 미리 봄"), ("#1f6f4a", "새로 미리 보게 된 선"), ("#c0392b", "오히려 못 보게 된 선"),
                     ("0.85", "도착 전엔 안 보임 (바로 아래 등)"))], loc="lower center", ncol=4, fontsize=10)
fig.suptitle("로봇이 도착하기(0.35 m 이내) 전에 지도에 들어온 선 — 실행 과반 기준", fontsize=12)
fig.tight_layout(rect=(0, 0.07, 1, 0.94))
fig.savefig(OUT / "bev_far_ahead.png", dpi=100)

# fig 3: accuracy of map cells by the distance at which they first entered the map
fig, ax = plt.subplots(1, 2, figsize=(12, 4.2))
bins = np.arange(0.3, 1.45, 0.1)
cols = ("#888888", "#4a8fe7", "#1f6f4a", "#c0392b")
for p, col in zip(profs, cols):
    d = np.concatenate([m["dist_all"] for m in per[p]]); e = np.concatenate([m["err"] for m in per[p]])
    k = np.digitize(d, bins) - 1
    acc = [((e[k == i] < 0.02).mean() * 100 if (k == i).sum() > 200 else np.nan) for i in range(len(bins) - 1)]
    cnt = [(k == i).sum() / len(per[p]) for i in range(len(bins) - 1)]
    ax[0].plot(bins[:-1] + 0.05, acc, "-o", ms=3.5, color=col, label=lab(p))
    ax[1].plot(bins[:-1] + 0.05, cnt, "-o", ms=3.5, color=col, label=lab(p))
ax[0].set_title("처음 기록된 거리별 지도 칸 정확도 (2 cm 이내 %)"); ax[0].set_xlabel("로봇에서의 거리 (m)"); ax[0].set_ylim(0, 100)
ax[1].set_title("처음 기록된 거리별 지도 칸 수 (한 바퀴 평균)"); ax[1].set_xlabel("로봇에서의 거리 (m)")
for a in ax:
    a.grid(alpha=.3); a.legend(fontsize=9)
fig.tight_layout(); fig.savefig(OUT / "bev_far_dist.png", dpi=100)
