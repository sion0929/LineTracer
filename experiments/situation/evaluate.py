#!/usr/bin/env python3
"""Evaluate a follower parameter file on the held-out set and compare modes (dev tool).

Held-out set (never used for training or tuning, TEAM_GUIDE 4-1):
  official   line_track.sdf x3
  seeds      gen_world.py seeds 0-19 (easy 0-6, normal 7-13, hard 14-19, obstacles 0)
  random     random_drive.py 6 --seed 1457145442 (its 6 suite maps are excluded from training)
  finishgate (optional, --finishgate) the safety 5a/5b maps short_t, normal_26_x4, normal_13_x4

  # inside the lab container (ROS sourced), from /ws
  python3 experiments/situation/evaluate.py run --tag rule --params src/line_follower/config/rule.yaml
  python3 experiments/situation/evaluate.py run --tag playbook --params src/line_follower/config/playbook_rule.yaml
  python3 experiments/situation/evaluate.py compare rule playbook      # per-map table of single repetitions
  python3 experiments/situation/evaluate.py pool rule=rule,rule_r2 playbook=playbook_v0,playbook_v0_r2   # -> compare.md

Every run goes through experiments/bench.py (backend run_eval, --off 0.12, --limit 300, --stall 20)
with experiments/user_base/run_user.py and LF_PARAMS, 3 Gazebo runs at once, ROS domains 50-69,
GZ_PARTITION prefix lf_l_. Metrics per run: complete, progress, lap, and the finish verdicts from
the follower log ('finish T detected' = the node fixed a finish point):
  false finish   the node fixed a finish point and the run did not complete (it stopped / left the
                 line on the way to a wrong T)
  finish ok      the node fixed a finish point and the run completed
  missed finish  the run completed without the node fixing a finish point
"""
import argparse
import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "experiments/worlds"))
OUT = ROOT / "experiments/results/situation"
MAPS = ROOT / "experiments/situation/data/maps/eval"
OFFICIAL = ROOT / "src/line_follower_bringup/worlds/line_track.sdf"
BENCH = ROOT / "experiments/bench.py"
RANDOM = ROOT / "experiments/eval/random_drive.py"
FOLLOWER = ROOT / "experiments/user_base/run_user.py"
RANDOM_SEED = 1457145442
FINISHGATE = [ROOT / "experiments/results/safety/maps" / f"{n}.sdf" for n in ("short_t", "normal_26_x4", "normal_13_x4")]


def seed_difficulty(seed):
    return "easy" if seed <= 6 else "normal" if seed <= 13 else "hard"


def eval_maps():
    import gen_world
    out = []
    for seed in range(20):
        d = seed_difficulty(seed)
        path = MAPS / f"{d}_{seed}.sdf"
        if not all(path.with_suffix(e).is_file() for e in (".sdf", ".stl", ".json")):
            gen_world.generate(seed, d, 0, path)
        out.append(path)
    return out


def env_for(tag, params):
    return dict(os.environ, LF_PARAMS=str(Path(params).resolve()), BENCH_DOMAIN_BASE="50", LAB_NAME="lf_l",
                GZ_PARTITION=f"lf_l_{tag}", ROS_DOMAIN_ID=os.environ.get("ROS_DOMAIN_ID", "22"))


def bench(maps, out, env, jobs=3, extra=()):
    cmd = [sys.executable, str(BENCH), *map(str, maps), "--follower", str(FOLLOWER), "--jobs", str(jobs),
           "--off", "0.12", "--limit", "300", "--stall", "20", "--output", str(out), *extra]
    with (out.parent / f"{out.name}.log").open("w") as log:
        log.write(" ".join(cmd) + "\n")
        log.flush()
        return subprocess.call(cmd, env=env, stdout=log, stderr=subprocess.STDOUT)


def follower_log(run):
    for log in (run / "results/legacy.log", run / "run.log"):
        if log.is_file():
            return log.read_text(errors="replace")
    return ""


def run_record(run, group):
    res = json.loads((run / "result.json").read_text())
    log = follower_log(run)
    det = [float(x) for x in re.findall(r"finish T detected \(driven ([0-9.]+) m\)", log)]
    stop = [float(x) for x in re.findall(r"reached finish, stopping \(driven ([0-9.]+) m\)", log)]
    meta = Path(res["map"]).with_suffix(".json")
    length = json.loads(meta.read_text())["arc_m"][-1] if meta.is_file() else 15.6986
    complete = bool(res.get("complete"))
    return dict(group=group, run=run.name, map=Path(res["map"]).stem, complete=complete,
                progress_m=res.get("progress_m"), progress_pct=res.get("progress_pct"), lap_sec=res.get("lap_sec"),
                reason=res.get("reason"), length_m=round(length, 3), finish_detected=det[0] if det else None,
                finish_stop=stop[0] if stop else None, false_finish=bool(det) and not complete,
                finish_ok=bool(det) and complete, missed_finish=complete and not det,
                infra=str(res.get("reason", "")).startswith("error"))


def collect(tag_dir):
    rows = []
    for group in ("official", "seeds", "random", "finishgate"):
        base = tag_dir / group / ("bench" if group == "random" else "")
        if not base.is_dir():
            continue
        for run in sorted(p for p in base.iterdir() if (p / "result.json").is_file()):
            rows.append(run_record(run, group))
    return rows


def stats(rows):
    good = [r for r in rows if not r["infra"]]
    done = [r for r in good if r["complete"]]
    laps = [r["lap_sec"] for r in done if r["lap_sec"] is not None]
    return dict(runs=len(good), infra=len(rows) - len(good), complete=len(done),
                mean_progress_pct=round(sum(r["progress_pct"] or 0 for r in good) / max(len(good), 1), 1),
                mean_lap_sec=round(sum(laps) / len(laps), 1) if laps else None,
                false_finish=sum(r["false_finish"] for r in good), finish_ok=sum(r["finish_ok"] for r in good),
                missed_finish=sum(r["missed_finish"] for r in good))


def summarize(tag_dir):
    rows = collect(tag_dir)
    groups = {g: stats([r for r in rows if r["group"] == g]) for g in ("official", "seeds", "random", "finishgate")}
    groups = {g: s for g, s in groups.items() if s["runs"] or s["infra"]}
    core = [r for r in rows if r["group"] in ("official", "seeds", "random")]
    summary = dict(tag=tag_dir.name, groups=groups, total=stats(core), runs=rows)
    (tag_dir / "summary.json").write_text(json.dumps(summary, indent=1, ensure_ascii=False) + "\n")
    lines = [f"# 평가 요약: {tag_dir.name}", "", "| 묶음 | 회차 | 완주 | 평균 진행률 | 평균 랩 s | 잘못된 완주 | 정상 완주 판정 | 완주했지만 판정 없음 |",
             "|---|---|---|---|---|---|---|---|"]
    for g, s in list(groups.items()) + [("합계(공식+시드+랜덤)", summary["total"])]:
        lines.append(f"| {g} | {s['runs']} | {s['complete']} | {s['mean_progress_pct']}% | {s['mean_lap_sec'] or '-'} | "
                     f"{s['false_finish']} | {s['finish_ok']} | {s['missed_finish']} |")
    lines += ["", "| 묶음 | 회차 | 맵 | 길이 m | 완주 | 진행률 | 랩 s | T 확정(주행 m) | 결과 |", "|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        lines.append(f"| {r['group']} | {r['run']} | {r['map']} | {r['length_m']} | {'O' if r['complete'] else 'X'} | "
                     f"{r['progress_pct']}% | {r['lap_sec'] or '-'} | {r['finish_detected'] or '-'}"
                     f"{' **잘못된 완주**' if r['false_finish'] else ''} | {r['reason']} |")
    (tag_dir / "summary.md").write_text("\n".join(lines) + "\n")
    return summary


def run(a):
    tag_dir = OUT / a.tag
    tag_dir.mkdir(parents=True, exist_ok=True)
    env = env_for(a.tag, a.params)
    info = dict(tag=a.tag, params=str(a.params), started=time.strftime("%Y-%m-%d %H:%M:%S"), groups=a.groups)
    (tag_dir / "info.json").write_text(json.dumps(info, indent=1) + "\n")
    groups = a.groups.split(",")
    if "official" in groups and not (tag_dir / "official").exists():
        bench([OFFICIAL] * 3, tag_dir / "official", env)
    if "seeds" in groups and not (tag_dir / "seeds").exists():
        bench(eval_maps(), tag_dir / "seeds", env)
    if "random" in groups and not (tag_dir / "random").exists():
        cmd = [sys.executable, str(RANDOM), "6", "--seed", str(RANDOM_SEED), "--params", str(Path(a.params).resolve()),
               "--jobs", "3", "--domain-base", "50", "--partition", f"lf_l_rand_{a.tag}", "--output",
               str(tag_dir / "random")]
        with (tag_dir / "random.log").open("w") as log:
            subprocess.call(cmd, env=env, stdout=log, stderr=subprocess.STDOUT)
    if "finishgate" in groups and not (tag_dir / "finishgate").exists():
        bench([m for m in FINISHGATE for _ in range(2)], tag_dir / "finishgate", env)
    s = summarize(tag_dir)
    print(json.dumps(dict(s["groups"], total=s["total"]), indent=1, ensure_ascii=False))


def compare(a):
    tags = a.tags
    sums = {t: summarize(OUT / t) for t in tags}
    lines = ["# 규칙 모드 대비 비교 (보류 평가 세트)", "",
             "| 모드 | 묶음 | 회차 | 완주 | 평균 진행률 | 평균 랩 s | 잘못된 완주 | 정상 완주 판정 | 완주했지만 판정 없음 |",
             "|---|---|---|---|---|---|---|---|---|"]
    for t in tags:
        s = sums[t]
        for g, st in list(s["groups"].items()) + [("합계", s["total"])]:
            lines.append(f"| {t} | {g} | {st['runs']} | {st['complete']} | {st['mean_progress_pct']}% | "
                         f"{st['mean_lap_sec'] or '-'} | {st['false_finish']} | {st['finish_ok']} | {st['missed_finish']} |")
    lines += ["", "## 맵별", "", "| 묶음 | 맵 | " + " | ".join(tags) + " |", "|---|---|" + "---|" * len(tags)]
    keyed = {t: {(r["group"], r["run"]): r for r in sums[t]["runs"]} for t in tags}
    keys = sorted(set().union(*[set(k) for k in keyed.values()]))
    for k in keys:
        cells = []
        for t in tags:
            r = keyed[t].get(k)
            cells.append("-" if r is None else f"{'O' if r['complete'] else 'X'} {r['progress_pct']}%"
                         f"{' FF' if r['false_finish'] else ''}")
        name = next(keyed[t][k]["map"] for t in tags if k in keyed[t])
        lines.append(f"| {k[0]} | {name} ({k[1]}) | " + " | ".join(cells) + " |")
    out = OUT / (a.output or "compare.md")
    out.write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


def pool(a):
    """Pool repetitions: NAME=tag1,tag2 ... -> compare.md (main comparison table)."""
    modes = [spec.split("=", 1) for spec in a.modes]
    runs = {name: [r for t in tags.split(",") for r in summarize(OUT / t)["runs"]] for name, tags in modes}
    lines = ["# 규칙 모드 대비 비교 (보류 평가 세트, 반복 합산)", "",
             "같은 하네스(bench run_eval, --off 0.12, --limit 300, --stall 20, run_user.py + LF_PARAMS, 3개 동시, 도메인 50-69). "
             "핵심 세트 = 공식 트랙 3회 + 시드 0-19 + random_drive 6 --seed 1457145442 = 반복당 29회.", "",
             "| 모드 | 반복(태그) | 묶음 | 회차 | 완주 | 평균 진행률 | 평균 랩 s | 잘못된 완주 | 정상 완주 판정 | 완주했지만 판정 없음 |",
             "|---|---|---|---|---|---|---|---|---|---|"]
    for name, tags in modes:
        for g in ("official", "seeds", "random", "core", "finishgate"):
            rs = [r for r in runs[name] if (r["group"] in ("official", "seeds", "random") if g == "core" else r["group"] == g)]
            if not rs:
                continue
            st = stats(rs)
            lines.append(f"| {name} | {tags} | {'**합계(핵심)**' if g == 'core' else g} | {st['runs']} | {st['complete']} | "
                         f"{st['mean_progress_pct']}% | {st['mean_lap_sec'] or '-'} | {st['false_finish']} | {st['finish_ok']} | "
                         f"{st['missed_finish']} |")
    # paired laps: same map completed by both modes (all repetitions)
    names = [n for n, _ in modes]
    lines += ["", "## 둘 다 완주한 맵의 랩(쌍 비교, 맵별 평균)", "", "| 비교 | 맵 수 | " + " | ".join(names[:2]) + " |", "|---|---|---|---|"]
    if len(names) >= 2:
        a0, a1 = names[0], names[1]
        laps = {}
        for n in (a0, a1):
            for r in runs[n]:
                if r["complete"] and r["lap_sec"] and r["group"] in ("official", "seeds", "random"):
                    laps.setdefault(n, {}).setdefault(r["map"], []).append(r["lap_sec"])
        both = sorted(set(laps.get(a0, {})) & set(laps.get(a1, {})))
        if both:
            m0 = sum(sum(laps[a0][m]) / len(laps[a0][m]) for m in both) / len(both)
            m1 = sum(sum(laps[a1][m]) / len(laps[a1][m]) for m in both) / len(both)
            lines.append(f"| {a0} vs {a1} | {len(both)} ({', '.join(both)}) | {m0:.1f} | {m1:.1f} |")
    lines += ["", "## 잘못된 완주(노드가 종점을 확정했지만 완주하지 못한 회차)", ""]
    for name, _ in modes:
        ff = [r for r in runs[name] if r["false_finish"]]
        lines.append(f"- {name}: {len(ff)}건" + ("" if not ff else ": " + ", ".join(
            f"{r['map']} ({r['group']}, 진행 {r['progress_m']} / {r['length_m']} m, T 확정 주행 {r['finish_detected']} m)" for r in ff)))
    out = OUT / (a.output or "compare.md")
    out.write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run")
    r.add_argument("--tag", required=True)
    r.add_argument("--params", required=True, type=Path)
    r.add_argument("--groups", default="official,seeds,random")
    s = sub.add_parser("summarize")
    s.add_argument("tag")
    c = sub.add_parser("compare")
    c.add_argument("tags", nargs="+")
    c.add_argument("--output")
    pl = sub.add_parser("pool")
    pl.add_argument("modes", nargs="+", metavar="NAME=tag1,tag2")
    pl.add_argument("--output")
    a = ap.parse_args()
    if a.cmd == "run":
        run(a)
    elif a.cmd == "summarize":
        print(json.dumps(summarize(OUT / a.tag)["groups"], indent=1, ensure_ascii=False))
    elif a.cmd == "pool":
        pool(a)
    else:
        compare(a)


if __name__ == "__main__":
    main()
