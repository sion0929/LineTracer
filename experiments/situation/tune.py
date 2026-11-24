#!/usr/bin/env python3
"""DEV ONLY: tune the per-situation playbook (config/playbook.yaml) in Gazebo by coordinate descent.

Tuning maps: 12 suite maps with seeds >= 1000 that are not in the classifier's held-out set
(collect.GZ_VAL), not in the random_drive --seed 1457145442 sample, and never seeds 0-39 or the
official track. Every candidate config is driven once on every tuning map from the real start
(bench.py, run_eval backend, --off 0.12, LF_PARAMS = config/playbook_rule.yaml, the candidate passed
as -p playbook.config:=...). All candidates of one coordinate go into one bench call (3 at once).

Objective per run (decision_rules.md 3.3, lower is better):
  J = time + W_OFF * max(0, lat_p95 - CUT_TOL) + failure penalty
  time = lap time when complete, else the simulated time until the run ended
  failure penalty = 200 s + 200 s * (1 - progress fraction) when not complete
J of a config = sum over the tuning maps.

  python3 experiments/situation/tune.py step --name r1 --base src/line_follower/config/playbook.yaml \
      --set RIGHT_ANGLE.v_cap=none,0.18,0.12
  python3 experiments/situation/tune.py report      # all rounds -> experiments/results/situation/tuning/summary.md

Each step keeps its configs, bench output and a summary.json under experiments/results/situation/tuning/<name>/
(failed attempts included, nothing is deleted).
"""
import argparse
import copy
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
OUT = ROOT / "experiments/results/situation/tuning"
PARAMS = ROOT / "src/line_follower/config/playbook_rule.yaml"
TUNE_MAPS = {"general200": range(1012, 1020), "official_like50": range(5008, 5012)}
W_OFF, CUT_TOL = 500.0, 0.06


def tuning_maps():
    sys.path.insert(0, str(ROOT / "experiments/worlds"))
    import sample_maps
    out = []
    for suite, seeds in TUNE_MAPS.items():
        for row in sample_maps.load_index(suite):
            if int(row["seed"]) in seeds:
                sample_maps.ensure_map(suite, row)
                out.append(ROOT / row["path"])
    return out


def parse_value(v):
    return None if v == "none" else float(v)


def variant(base, key, value):
    cfg = copy.deepcopy(base)
    sit, param = key.split(".", 1)
    if sit == "finish":
        cfg.setdefault("finish", {})[param] = value
    else:
        row = cfg.setdefault("situations", {}).setdefault(sit, {}) or {}
        if value is None:
            row.pop(param, None)
        else:
            row[param] = value
        cfg["situations"][sit] = row
    return cfg


def score(result, length):
    complete = bool(result.get("complete"))
    t = result.get("lap_sec") if complete else (result.get("sim_sec") or result.get("wall_sec") or 300.0)
    prog = (result.get("progress_m") or 0.0) / max(length, 1e-6)
    off = W_OFF * max(0.0, (result.get("lat_p95") or 0.0) - CUT_TOL)
    fail = 0.0 if complete else 200.0 + 200.0 * (1.0 - min(prog, 1.0))
    return round((t or 0.0) + off + fail, 2)


def step(a):
    base = yaml.safe_load(Path(a.base).read_text())
    folder = OUT / a.name
    (folder / "configs").mkdir(parents=True, exist_ok=False)
    configs = {}
    for spec in a.set:
        key, values = spec.split("=", 1)
        for v in values.split(","):
            tag = f"{key}={v}"
            configs[tag] = variant(base, key, parse_value(v))
    if a.include_base:
        configs["base"] = base
    for spec in a.config or []:
        tag, path = spec.split("=", 1)
        configs[tag] = yaml.safe_load(Path(path).read_text())
    profiles = []
    for i, (tag, cfg) in enumerate(configs.items()):
        path = folder / "configs" / f"c{i:02d}.yaml"
        path.write_text(f"# {tag}\n" + yaml.safe_dump(cfg, sort_keys=False))
        profiles += ["--profile", f"c{i:02d}=-p playbook.config:={path}"]
    maps = tuning_maps() * a.repeat
    env = dict(os.environ, LF_PARAMS=str(PARAMS), BENCH_DOMAIN_BASE="50", LAB_NAME="lf_l",
               GZ_PARTITION=f"lf_l_tune_{a.name}", ROS_DOMAIN_ID=os.environ.get("ROS_DOMAIN_ID", "22"))
    cmd = [sys.executable, str(ROOT / "experiments/bench.py"), *map(str, maps), *profiles,
           "--follower", str(ROOT / "experiments/user_base/run_user.py"), "--jobs", "3", "--off", "0.12",
           "--limit", "300", "--stall", "20", "--output", str(folder / "bench")]
    info = dict(name=a.name, base=str(a.base), started=time.strftime("%Y-%m-%d %H:%M:%S"),
                configs={f"c{i:02d}": tag for i, tag in enumerate(configs)}, maps=[str(m.relative_to(ROOT)) for m in maps],
                repeat=a.repeat)
    (folder / "info.json").write_text(json.dumps(info, indent=1) + "\n")
    with (folder / "bench.log").open("w") as log:
        subprocess.call(cmd, env=env, stdout=log, stderr=subprocess.STDOUT)
    summarize(folder)


def summarize(folder):
    info = json.loads((folder / "info.json").read_text())
    rows = {}
    for res in sorted((folder / "bench").glob("*/result.json")):
        r = json.loads(res.read_text())
        meta = json.loads(Path(r["map"]).with_suffix(".json").read_text())
        rows.setdefault(r["profile"], []).append(dict(
            map=Path(r["map"]).stem, complete=bool(r.get("complete")), progress_pct=r.get("progress_pct"),
            lap_sec=r.get("lap_sec"), lat_p95=r.get("lat_p95"), reason=r.get("reason"),
            J=score(r, meta["arc_m"][-1]), infra=str(r.get("reason", "")).startswith("error")))
    table = {}
    for prof, rs in rows.items():
        good = [x for x in rs if not x["infra"]]
        table[prof] = dict(tag=info["configs"].get(prof, prof), runs=len(good), complete=sum(x["complete"] for x in good),
                           mean_progress=round(sum(x["progress_pct"] or 0 for x in good) / max(len(good), 1), 1),
                           J=round(sum(x["J"] for x in good), 1), infra=len(rs) - len(good), per_map=rs)
    best = min((p for p in table if table[p]["runs"]), key=lambda p: table[p]["J"], default=None)
    (folder / "summary.json").write_text(json.dumps(dict(info, results=table, best=best), indent=1) + "\n")
    for p, t in sorted(table.items()):
        print(f"{p} {t['tag']:40s} runs {t['runs']} complete {t['complete']} progress {t['mean_progress']}% J {t['J']}"
              f"{'  <- best' if p == best else ''}")
    return table, best


def report(a):
    lines = ["# 플레이북 튜닝 기록 (자동 생성, tune.py report)", "",
             "| 단계 | 후보 | 회차 | 완주 | 평균 진행률 | J (낮을수록 좋음) | 선택 |", "|---|---|---|---|---|---|---|"]
    for folder in sorted(p for p in OUT.iterdir() if (p / "info.json").is_file()):
        if not (folder / "summary.json").is_file() and (folder / "bench").is_dir():
            summarize(folder)
        if not (folder / "summary.json").is_file():
            continue
        s = json.loads((folder / "summary.json").read_text())
        for p, t in sorted(s["results"].items()):
            lines.append(f"| {folder.name} | `{t['tag']}` | {t['runs']} | {t['complete']} | {t['mean_progress']}% | "
                         f"{t['J']} | {'**선택**' if p == s['best'] else ''} |")
    (OUT / "summary.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("step")
    s.add_argument("--name", required=True)
    s.add_argument("--base", default=str(ROOT / "src/line_follower/config/playbook.yaml"))
    s.add_argument("--set", action="append", default=[], metavar="SITUATION.param=v1,v2,...")
    s.add_argument("--include-base", action="store_true")
    s.add_argument("--config", action="append", metavar="TAG=PLAYBOOK.yaml", help="add a whole config as a candidate")
    s.add_argument("--repeat", type=int, default=1, help="drive every tuning map this many times per config")
    sub.add_parser("report")
    r = sub.add_parser("summarize")
    r.add_argument("name")
    a = ap.parse_args()
    if a.cmd == "step":
        step(a)
    elif a.cmd == "summarize":
        summarize(OUT / a.name)
    else:
        report(a)


if __name__ == "__main__":
    main()
