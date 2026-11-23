#!/usr/bin/env python3
"""Per-run telemetry: everything indexed by distance along the line (x axis = progress s).

bench.py records one row per ~50 ms of simulation time and writes, in each run folder:
  telemetry.npz / telemetry.csv  time series (see COLUMNS)
  telemetry.json                 summary (max progress, speeds, accelerations, errors, ...)
  telemetry.png                  dashboard with progress on the x axis + top-down view

Compare several runs (any mix of run folders or bench output folders):
  python3 experiments/telemetry.py compare OUT.png RUN_OR_BENCH_DIR [...]
Re-plot one run:
  python3 experiments/telemetry.py plot RUN_DIR
"""
import csv
import json
import sys
from pathlib import Path

import numpy as np

COLUMNS = (
    "t",            # sim time since start [s]
    "x", "y", "yaw",  # ground-truth pose (dev tool only, never used by the robot)
    "s",            # projected distance along the ordered centreline [m]
    "progress",     # best s reached so far [m]
    "lat",          # signed lateral error, + = left of travel direction [m]
    "v", "a",       # true forward speed [m/s], acceleration [m/s^2]
    "w", "alpha",   # true yaw rate [rad/s], yaw acceleration [rad/s^2]
    "jerk",         # d a / dt [m/s^3]
    "cmd_v", "cmd_w",  # last /cmd_vel seen [m/s], [rad/s] (nan if none)
    "curv",         # path curvature of the driven trajectory w/v [1/m] (nan when slow)
    "wheel_on",     # 1 if a wheel centre is within line+wheel half-widths of the centreline
)
CORRIDOR_DESIGN, CORRIDOR_LIMIT = 0.10, 0.12


class Telemetry:
    def __init__(self, track, wheel_offset=0.1435, wheel_on_line=0.019):
        self.track, self.wheel_offset, self.wheel_on_line = track, wheel_offset, wheel_on_line
        self.rows = []
        self.progress = 0.0

    def _signed(self, xy, s):
        arc = self.track.arc
        i = int(np.clip(arc.searchsorted(s) - 1, 0, len(self.track.delta) - 1))
        d = self.track.delta[i]
        base = self.track.xy[i] + d * np.clip((s - arc[i]) / self.track.ds[i], 0, 1)
        r = xy - base
        return float(np.sign(d[0] * r[1] - d[1] * r[0]) * np.hypot(*r))

    def add(self, t, x, y, yaw, cmd=None):
        xy = np.array([x, y], dtype=float)
        s, _ = self.track.project(xy, self.progress if self.rows else None)
        self.progress = max(self.progress, s)
        normal = np.array([-np.sin(yaw), np.cos(yaw)])
        on = any(self.track.project(xy + k * self.wheel_offset * normal)[1] < self.wheel_on_line
                 for k in (1.0, -1.0))
        cv, cw = cmd if cmd is not None else (np.nan, np.nan)
        self.rows.append((t, x, y, yaw, s, self.progress, self._signed(xy, s), cv, cw, float(on)))

    def table(self):
        if len(self.rows) < 3:
            return None
        r = np.array(self.rows, dtype=float)
        t, x, y, yaw = r[:, 0], r[:, 1], r[:, 2], np.unwrap(r[:, 3])
        dt = np.gradient(t)
        dt[dt <= 0] = np.nan
        vx, vy = np.gradient(x) / dt, np.gradient(y) / dt
        v = vx * np.cos(yaw) + vy * np.sin(yaw)
        v = _smooth(v, 3)
        a = _smooth(np.gradient(v) / dt, 3)
        w = _smooth(np.gradient(yaw) / dt, 3)
        alpha = _smooth(np.gradient(w) / dt, 3)
        jerk = np.gradient(a) / dt
        curv = np.where(np.abs(v) > 0.02, w / np.where(np.abs(v) > 0.02, v, 1), np.nan)
        cols = dict(t=t, x=x, y=y, yaw=r[:, 3], s=r[:, 4], progress=r[:, 5], lat=r[:, 6], v=v, a=a,
                    w=w, alpha=alpha, jerk=jerk, cmd_v=r[:, 7], cmd_w=r[:, 8], curv=curv,
                    wheel_on=r[:, 9])
        return np.column_stack([cols[c] for c in COLUMNS])

    def save(self, folder, result=None):
        folder = Path(folder)
        data = self.table()
        if data is None:
            return None
        np.savez_compressed(folder / "telemetry.npz", data=data, columns=np.array(COLUMNS),
                            centerline=np.column_stack((self.track.xy, self.track.arc)))
        with (folder / "telemetry.csv").open("w", newline="") as f:
            out = csv.writer(f)
            out.writerow(COLUMNS)
            out.writerows(np.round(data, 5).tolist())
        summary = summarize(data, float(self.track.arc[-1]), result)
        (folder / "telemetry.json").write_text(json.dumps(summary, indent=1, ensure_ascii=False))
        try:
            plot_run(folder, data, np.column_stack((self.track.xy, self.track.arc)), summary)
        except Exception as error:  # plotting must never break scoring
            (folder / "telemetry_plot_error.txt").write_text(repr(error))
        return summary


def _smooth(values, k):
    if len(values) < 2 * k + 1:
        return values
    kernel = np.ones(2 * k + 1) / (2 * k + 1)
    padded = np.pad(np.nan_to_num(values), k, mode="edge")
    return np.convolve(padded, kernel, mode="valid")


def summarize(data, length, result=None):
    c = {name: data[:, i] for i, name in enumerate(COLUMNS)}
    moving = np.abs(c["v"]) > 0.02
    dt = np.diff(c["t"], prepend=c["t"][0])
    q = lambda v, p: float(np.nanpercentile(v, p)) if np.isfinite(v).any() else None
    summary = dict(
        track_length_m=round(length, 3),
        max_progress_m=round(float(c["progress"].max()), 3),
        max_progress_pct=round(100 * float(c["progress"].max()) / length, 2),
        sim_sec=round(float(c["t"][-1] - c["t"][0]), 2),
        v_mean=round(float(np.nanmean(c["v"][moving])), 4) if moving.any() else 0.0,
        v_p95=q(c["v"], 95), v_max=q(c["v"], 100),
        a_p95_abs=q(np.abs(c["a"]), 95), a_max_abs=q(np.abs(c["a"]), 100),
        w_p95_abs=q(np.abs(c["w"]), 95), w_max_abs=q(np.abs(c["w"]), 100),
        lat_p95_abs=q(np.abs(c["lat"]), 95), lat_max_abs=q(np.abs(c["lat"]), 100),
        time_outside_design_corridor_s=round(float(dt[np.abs(c["lat"]) > CORRIDOR_DESIGN].sum()), 2),
        time_outside_wheel_corridor_s=round(float(dt[np.abs(c["lat"]) > CORRIDOR_LIMIT].sum()), 2),
        wheel_on_line_s=round(float(dt[c["wheel_on"] > 0].sum()), 2),
        wheel_on_line_events=int(np.sum(np.diff(c["wheel_on"]) > 0) + (c["wheel_on"][0] > 0)),
        stopped_s=round(float(dt[~moving].sum()), 2),
    )
    if result:
        summary.update({k: result.get(k) for k in ("complete", "lap_sec", "reason", "profile", "map")
                        if k in result})
    return {k: (round(v, 4) if isinstance(v, float) else v) for k, v in summary.items()}


def load(folder):
    z = np.load(Path(folder) / "telemetry.npz", allow_pickle=False)
    return z["data"], z["centerline"]


def plot_run(folder, data=None, centerline=None, summary=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    folder = Path(folder)
    if data is None:
        data, centerline = load(folder)
    if summary is None and (folder / "telemetry.json").is_file():
        summary = json.loads((folder / "telemetry.json").read_text())
    c = {name: data[:, i] for i, name in enumerate(COLUMNS)}
    s = c["s"]
    fig = plt.figure(figsize=(15, 13))
    grid = fig.add_gridspec(6, 2, width_ratios=(2.2, 1))
    axes = [fig.add_subplot(grid[i, 0]) for i in range(6)]
    for ax in axes[1:]:
        ax.sharex(axes[0])
    ax = axes[0]
    ax.axhspan(-CORRIDOR_DESIGN, CORRIDOR_DESIGN, color="tab:green", alpha=0.08, label="design ±0.10")
    for k in (-1, 1):
        ax.axhline(k * CORRIDOR_LIMIT, color="tab:red", lw=0.8, ls="--")
    ax.plot(s, c["lat"], lw=1)
    ax.set_ylabel("lateral [m]")
    ax.legend(loc="upper right", fontsize=8)
    axes[1].plot(s, c["v"], lw=1, label="true v")
    axes[1].plot(s, c["cmd_v"], lw=0.8, alpha=0.7, label="cmd v")
    axes[1].set_ylabel("speed [m/s]")
    axes[1].legend(loc="upper right", fontsize=8)
    axes[2].plot(s, c["a"], lw=1)
    axes[2].set_ylabel("accel [m/s²]")
    axes[3].plot(s, c["w"], lw=1, label="true ω")
    axes[3].plot(s, c["cmd_w"], lw=0.8, alpha=0.7, label="cmd ω")
    axes[3].set_ylabel("yaw rate [rad/s]")
    axes[3].legend(loc="upper right", fontsize=8)
    axes[4].fill_between(s, 0, c["wheel_on"], step="mid", color="tab:red", alpha=0.6)
    axes[4].set_ylabel("wheel on line")
    axes[4].set_ylim(-0.1, 1.1)
    axes[5].plot(s, c["t"] - c["t"][0], lw=1)
    axes[5].set_ylabel("time [s]")
    axes[5].set_xlabel("distance along line s [m]")
    length = float(centerline[-1, 2])
    for ax in axes:
        ax.axvline(float(c["progress"].max()), color="k", lw=0.8, ls=":")
        ax.set_xlim(0, length)
        ax.grid(alpha=0.3)
    top = fig.add_subplot(grid[:4, 1])
    top.plot(centerline[:, 0], centerline[:, 1], color="0.6", lw=3, zorder=1)
    sc = top.scatter(c["x"], c["y"], c=c["v"], s=3, cmap="viridis", zorder=2)
    end = int(np.argmax(c["progress"]))
    top.plot(c["x"][end], c["y"][end], "rx", ms=10)
    top.set_aspect("equal")
    top.set_title("trajectory (colour = speed)")
    fig.colorbar(sc, ax=top, fraction=0.04)
    text = fig.add_subplot(grid[4:, 1])
    text.axis("off")
    if summary:
        text.text(0, 1, "\n".join(f"{k}: {v}" for k, v in summary.items() if k != "map"),
                  va="top", family="monospace", fontsize=8)
    fig.suptitle(f"{folder.name}  max {float(c['progress'].max()):.2f} / {length:.2f} m")
    fig.tight_layout()
    fig.savefig(folder / "telemetry.png", dpi=110)
    plt.close(fig)


def find_runs(paths):
    runs = []
    for p in map(Path, paths):
        if (p / "telemetry.npz").is_file():
            runs.append(p)
        else:
            runs += sorted(q.parent for q in p.rglob("telemetry.npz"))
    return runs


def compare(out, paths):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    runs = find_runs(paths)
    if not runs:
        raise SystemExit("no telemetry.npz found")
    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
    best = []
    for run in runs:
        data, centerline = load(run)
        c = {name: data[:, i] for i, name in enumerate(COLUMNS)}
        label = f"{run.parent.name}/{run.name}"
        axes[0].plot(c["s"], c["lat"], lw=0.8, label=label)
        axes[1].plot(c["s"], c["v"], lw=0.8)
        axes[2].plot(c["s"], c["t"] - c["t"][0], lw=0.8)
        best.append((label, float(c["progress"].max()), float(centerline[-1, 2])))
    axes[0].axhspan(-CORRIDOR_DESIGN, CORRIDOR_DESIGN, color="tab:green", alpha=0.08)
    axes[0].set_ylabel("lateral [m]")
    axes[1].set_ylabel("speed [m/s]")
    axes[2].set_ylabel("time [s]")
    axes[2].set_xlabel("distance along line s [m]")
    axes[0].legend(fontsize=6, ncol=2)
    labels = [b[0] for b in best]
    axes[3].barh(range(len(best)), [100 * b[1] / b[2] for b in best])
    axes[3].set_yticks(range(len(best)), labels, fontsize=6)
    axes[3].set_xlabel("max progress [%]")
    for ax in axes[:3]:
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print(f"wrote {out} ({len(runs)} runs)")


if __name__ == "__main__":
    if len(sys.argv) >= 3 and sys.argv[1] == "plot":
        for run in find_runs(sys.argv[2:]):
            plot_run(run)
            print(f"wrote {run / 'telemetry.png'}")
    elif len(sys.argv) >= 4 and sys.argv[1] == "compare":
        compare(sys.argv[2], sys.argv[3:])
    else:
        raise SystemExit(__doc__)
