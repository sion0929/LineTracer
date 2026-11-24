#!/usr/bin/env python3
"""random_drive.py 6 --seed 1457145442 with the guarded stack, on ROS domains 70-89.

random_drive.py only allows domains 50-69, so this reuses its functions unchanged: the same
map sample (sample_maps.run), the same bench.py arguments as the committed baseline
(backend run_eval, --off 0.12, --limit 300, --stall 20; --jobs 2 instead of 3) with
--follower experiments/user_base/run_guarded.py, and the same Korean summary.md.

  python3 experiments/results/safety/guarded/random6_guarded.py <output dir> [seed]
"""
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "experiments/eval"))
import random_drive as rd  # noqa: E402


def main():
    out = Path(sys.argv[1]).resolve()
    seed = int(sys.argv[2]) if len(sys.argv) > 2 else 1457145442
    out.mkdir(parents=True, exist_ok=True)
    suites = list(rd.sample_maps.DEFAULT_SUITES)
    _, seed, picked = rd.sample_maps.run(6, suites, seed, "balanced", out / "sample")
    follower = ROOT / "experiments/user_base/run_guarded.py"
    params = os.environ.get("LF_PARAMS") or str(rd.DEFAULT_PARAMS)
    env = dict(os.environ, LF_PARAMS=params, BENCH_DOMAIN_BASE="70", LAB_NAME=os.environ.get("LAB_NAME", "lf_g"),
               GZ_PARTITION="lf_g_drive", ROS_DOMAIN_ID=os.environ.get("ROS_DOMAIN_ID", "23"))
    cmd = [sys.executable, str(rd.BENCH), *[str(ROOT / row["path"]) for _, row in picked],
           "--follower", str(follower), "--jobs", "2", "--off", "0.12", "--limit", "300",
           "--stall", "20", "--backend", "run_eval", "--output", str(out / "bench")]
    t0 = time.time()
    with (out / "bench.log").open("w") as log:
        log.write(" ".join(cmd) + "\n")
        log.flush()
        code = subprocess.call(cmd, env=env, stdout=log, stderr=subprocess.STDOUT)
    results = [__import__("json").loads(p.read_text()) for p in sorted((out / "bench").glob("*/result.json"))]
    info = dict(created=time.strftime("%Y-%m-%d %H:%M:%S"), wall_min=(time.time() - t0) / 60, seed=seed,
                suites=suites, mode="balanced", follower=str(follower.relative_to(ROOT)), params=params,
                backend="run_eval", off=0.12, limit=300, stall=20, jobs=2, domain_base=70,
                partition="lf_g_drive", bench_exit=code)
    summary = rd.summarize(out, info, picked, results, rd.appendix_text(out, picked))
    print(f"완주 {summary['completed']}/{summary['runs']}, 평균 진행률 {summary['mean_progress_pct']}%")


if __name__ == "__main__":
    main()
