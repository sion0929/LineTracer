#!/usr/bin/env python3
"""Route error along the official track per profile, from shadow_route logs (dev only).
  python3 experiments/eval/shadow_profile.py OUT.png DIR1[:label] DIR2[:label] ...
Bins s (true arc position) by 0.25 m; per bin: median |route error| at 0.30 and 0.60 m ahead,
and robot offset to the line; failures marked."""
import json, sys, glob
import numpy as np, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
out = sys.argv[1]; specs = sys.argv[2:]
bins = np.arange(0, 15.75, 0.25)
fig, axs = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
for spec in specs:
    d, _, lab = spec.partition(":"); lab = lab or d.rstrip("/").split("/")[-1]
    E30, E60, OFF = [[] for _ in bins], [[] for _ in bins], [[] for _ in bins]
    fails = []
    for run in sorted(glob.glob(f"{d}/*/")):
        try:
            z = np.load(run + "shadow.npz"); r = json.load(open(run + "result.json"))
        except Exception:
            continue
        if not r["complete"] and r.get("progress_m"):
            fails.append(r["progress_m"])
        k = np.digitize(z["s"], bins) - 1
        for i, b in enumerate(k):
            if 0 <= b < len(bins):
                if np.isfinite(z["err"][i, 1]): E30[b].append(abs(z["err"][i, 1]))
                if np.isfinite(z["err"][i, 3]): E60[b].append(abs(z["err"][i, 3]))
                OFF[b].append(abs(z["robot_off"][i]))
    med = lambda L: [np.median(x) * 100 if len(x) > 5 else np.nan for x in L]  # noqa: E731
    p90 = lambda L: [np.percentile(x, 90) * 100 if len(x) > 5 else np.nan for x in L]  # noqa: E731
    l, = axs[0].plot(bins + 0.125, p90(E30), label=f"{lab} ({len(fails)} fails)")
    axs[1].plot(bins + 0.125, p90(E60), color=l.get_color())
    axs[2].plot(bins + 0.125, p90(OFF), color=l.get_color())
    for f in fails:
        for a in axs: a.axvline(f, color=l.get_color(), ls=":", lw=1)
axs[0].set_ylabel("route err 0.30 m ahead\np90 [cm]"); axs[1].set_ylabel("route err 0.60 m ahead\np90 [cm]")
axs[2].set_ylabel("robot to line\np90 [cm]"); axs[2].set_xlabel("arc position s on the official track [m] (dotted = failure)")
for a in axs: a.grid(alpha=.3); a.set_ylim(0, 15)
axs[0].legend(); fig.tight_layout(); fig.savefig(out, dpi=90); print(out)
