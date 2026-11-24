#!/usr/bin/env python3
"""Drive a follower on N randomly sampled suite maps with bench.py; Korean summary.md (dev tool).

  # default: package rule mode (run_user.py + config/rule.yaml), 6 maps, 3 Gazebo runs at once
  python3 experiments/eval/random_drive.py 6 --jobs 3
  # reproduce a sample / another follower or parameter file
  python3 experiments/eval/random_drive.py 6 --seed 1234
  LF_PARAMS=/ws/src/line_follower/config/rule_plane.yaml python3 experiments/eval/random_drive.py 10
  python3 experiments/eval/random_drive.py 10 --follower experiments/follower.py --off 0.12 --limit 300

Run inside the lab container (needs ROS 2 + Gazebo). Maps come from
experiments/worlds/sample_maps.py (seed random by default, always printed and saved; default
suites general200, official_like50, hills60, stress50; --with-obstacles adds obstacles100).
Missing map files are regenerated. bench.py drives every map once and scores it against the
ground truth; each run gets its own ROS domain from BENCH_DOMAIN_BASE (default 50 -> 50-69)
and its own GZ_PARTITION (prefix --partition, default lf_p_drive).

Writes into --output (default experiments/results/random/<time>_seed<seed>/):
  sample/maps.txt, maps.csv, sample.json   the sampled maps (sample_maps.py)
  bench/<run>/...                          bench.py output (result.json, telemetry, logs)
  bench.log                                bench.py console output
  summary.md / summary.json                Korean summary: seed, maps, completions, progress,
                                           lap, failure reasons, completion by suite / feature
  by_feature.png                           worlds/summarize_suite.py figure
"""
import argparse
import csv
import json
import os
import re
import subprocess
import sys
import time
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "experiments/worlds"))
import sample_maps  # noqa: E402

BENCH = ROOT / "experiments/bench.py"
SUMMARIZE = ROOT / "experiments/worlds/summarize_suite.py"
DEFAULT_FOLLOWER = ROOT / "experiments/user_base/run_user.py"
DEFAULT_PARAMS = ROOT / "src/line_follower/config/rule.yaml"
FEATURES = (("difficulty", "난이도"), ("hills", "언덕 수"), ("mirror", "좌우 반전"), ("loop_corners", "코너 원"),
            ("detours", "연결 교차로"), ("bars", "교차 막대+곁가지"), ("gaps", "끊긴 길"),
            ("small_loops", "물방울 원"), ("obstacles", "상자"))


def reason_kind(result):
    reason = str(result.get("reason") or "")
    if result.get("complete"):
        return "완주"
    if reason.startswith("error"):
        return "인프라 오류"
    if reason.startswith("off line"):
        return "선 이탈"
    if reason.startswith("stalled"):
        return "정지(진행 없음)"
    if reason.startswith("time limit"):
        return "시간 초과"
    return reason or "알 수 없음"


def follower_finish(run_dir):
    """Driven distance [m] at which the follower declared the finish itself, or None."""
    for log in (run_dir / "results/legacy.log", run_dir / "run.log"):
        if log.is_file():
            m = re.search(r"reached finish, stopping \(driven ([0-9.]+) m\)", log.read_text(errors="replace"))
            if m:
                return float(m.group(1))
    return None


def where(result):
    m = re.search(r"at ([0-9.]+) ?m", str(result.get("reason") or ""))
    return float(m.group(1)) if m else None


def fmt(v, nd=1, unit=""):
    return "-" if v is None else f"{v:.{nd}f}{unit}"


def group_table(title, key, label, runs):
    groups = defaultdict(list)
    for r in runs:
        groups[str(r["row"].get(key, "?"))].append(r)
    lines = [f"| {label} | 맵 | 완주 | 완주율 | 평균 진행률 | 평균 랩 |", "|---|---|---|---|---|---|"]
    for value in sorted(groups, key=lambda v: (len(v), v)):
        items = groups[value]
        done = [r for r in items if r["result"].get("complete")]
        laps = [r["result"]["lap_sec"] for r in done if r["result"].get("lap_sec") is not None]
        prog = sum(r["result"].get("progress_pct") or 0 for r in items) / len(items)
        lines.append(f"| {value} | {len(items)} | {len(done)} | {100 * len(done) / len(items):.0f}% | "
                     f"{prog:.1f}% | {fmt(sum(laps) / len(laps) if laps else None, 1, ' s')} |")
    return [f"### {title}", ""] + lines + [""]


def summarize(out, info, picked, results, appendix):
    by_name = {Path(r["map"]).name: r for r in results}
    runs = []
    for i, (suite, row) in enumerate(picked, 1):
        res = by_name.get(Path(row["path"]).name, {"complete": False, "reason": "error: no result"})
        stop = follower_finish(out / "bench" / res["name"]) if res.get("name") else None
        runs.append(dict(order=i, suite=suite, row=row, result=res, follower_stop=stop))
    good = [r for r in runs if reason_kind(r["result"]) != "인프라 오류"]
    infra = [r for r in runs if r not in good]
    done = [r for r in good if r["result"].get("complete")]
    laps = [r["result"]["lap_sec"] for r in done if r["result"].get("lap_sec") is not None]
    mean_prog = sum(r["result"].get("progress_pct") or 0 for r in good) / max(len(good), 1)
    mean_lap = sum(laps) / len(laps) if laps else None
    lines = [
        "# 랜덤 맵 주행 요약", "",
        f"- 실행: {info['created']} · 소요 {info['wall_min']:.1f}분",
        f"- **시드 {info['seed']}** · 세트 {', '.join(info['suites'])} · 뽑기 {info['mode']} · 맵 {len(picked)}개",
        f"- 재현: `python3 experiments/eval/random_drive.py {len(picked)} --seed {info['seed']} "
        f"--suites {','.join(info['suites'])} --mode {info['mode']}` (같은 follower, LF_PARAMS)",
        f"- follower `{info['follower']}` · LF_PARAMS `{info['params']}`",
        f"- bench: backend {info['backend']}, --off {info['off']}, --limit {info['limit']} s, --stall {info['stall']} s, "
        f"--jobs {info['jobs']}, 도메인 {info['domain_base']}–{info['domain_base'] + 19}, 파티션 `{info['partition']}_*`",
        "",
        "## 결과",
        "",
        f"- **완주 {len(done)}/{len(good)}** ({100 * len(done) / max(len(good), 1):.0f}%)"
        + (f", 인프라 오류 {len(infra)}건은 제외" if infra else ""),
        f"- 평균 진행률 {mean_prog:.1f}%",
        f"- 평균 랩타임(완주한 맵) {fmt(mean_lap, 1, ' s')}",
        "",
        "## 맵별 결과", "",
        "| # | 세트 | 맵 | 난이도 | 언덕 | 반전 | 코너 원 | 교차로 | 막대 | 끊김 | 물방울 | 상자 | 길이 m | 완주 | 진행률 | 랩 s | 결과 |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for r in runs:
        row, res = r["row"], r["result"]
        lines.append(
            f"| {r['order']} | {r['suite']} | {Path(row['path']).stem} | {row['difficulty']} | {row['hills']} | "
            f"{row['mirror']} | {row.get('loop_corners', '-')} | {row.get('detours', '-')} | {row.get('bars', '-')} | "
            f"{row.get('gaps', '-')} | {row.get('small_loops', '-')} | {row['obstacles']} | {row['length_m']} | "
            f"{'O' if res.get('complete') else 'X'} | {fmt(res.get('progress_pct'), 1, '%')} | "
            f"{fmt(res.get('lap_sec'), 1)} | {res.get('reason', '-')} |")
    kinds = defaultdict(list)
    for r in runs:
        if not r["result"].get("complete"):
            kinds[reason_kind(r["result"])].append(r)
    lines += ["", "## 실패 원인", ""]
    if not kinds:
        lines.append("- 실패 없음")
    for kind, items in sorted(kinds.items(), key=lambda kv: -len(kv[1])):
        spots = ", ".join(f"{Path(r['row']['path']).stem} ({fmt(where(r['result']), 2, ' m')}, "
                          f"{fmt(r['result'].get('progress_pct'), 0, '%')})" for r in items)
        lines.append(f"- **{kind}** {len(items)}건: {spots}")
        early = [r for r in items if r["follower_stop"] is not None]
        if early:
            lines.append("  - 노드가 종점 T를 봤다고 판단하고 스스로 멈춤(follower 로그 'reached finish'): " + ", ".join(
                f"{Path(r['row']['path']).stem} (주행 {r['follower_stop']:.2f} m, 실제 끝까지 "
                f"{fmt(float(r['row']['length_m']) - (r['result'].get('progress_m') or 0), 2, ' m')} 남음)"
                for r in early) + ". finish_min_dist 이후 보이는 T자·막대·곁가지를 종점으로 본 조기 종점 판정이다.")
    lines += ["", "## 세트별·요소별 완주율", ""]
    lines += group_table("세트", "suite", "세트", [dict(r, row=dict(r["row"], suite=r["suite"])) for r in good])
    for key, label in FEATURES:
        lines += group_table(label, key, label, good)
    if appendix:
        lines += ["## 부록: worlds/summarize_suite.py 출력", "", "```", appendix.rstrip(), "```", ""]
        if (out / "by_feature.png").is_file():
            lines += ["![요소별 완주율](by_feature.png)", ""]
    (out / "summary.md").write_text("\n".join(lines) + "\n")
    summary = dict(info, completed=len(done), runs=len(good), infra_errors=len(infra),
                   mean_progress_pct=round(mean_prog, 2), mean_lap_sec=None if mean_lap is None else round(mean_lap, 2),
                   failures={k: [Path(r["row"]["path"]).stem for r in v] for k, v in kinds.items()},
                   maps=[dict(order=r["order"], suite=r["suite"], map=r["row"]["path"],
                              complete=bool(r["result"].get("complete")),
                              progress_pct=r["result"].get("progress_pct"), lap_sec=r["result"].get("lap_sec"),
                              reason=r["result"].get("reason")) for r in runs])
    (out / "summary.json").write_text(json.dumps(summary, indent=1, ensure_ascii=False) + "\n")
    return summary


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("n", type=int, nargs="?", default=6, help="number of maps (default 6)")
    ap.add_argument("--suites", default=",".join(sample_maps.DEFAULT_SUITES))
    ap.add_argument("--with-obstacles", action="store_true", help="add obstacles100")
    ap.add_argument("--seed", type=int, help="sampling seed (default random, printed and saved)")
    ap.add_argument("--mode", choices=("balanced", "pool"), default="balanced")
    ap.add_argument("--follower", type=Path, default=DEFAULT_FOLLOWER)
    ap.add_argument("--params", help="LF_PARAMS for run_user.py (default: $LF_PARAMS or config/rule.yaml)")
    ap.add_argument("--jobs", type=int, default=3, help="Gazebo runs at once (max 3 here)")
    ap.add_argument("--off", type=float, default=0.12)
    ap.add_argument("--limit", type=float, default=300, help="time limit per run [s]")
    ap.add_argument("--stall", type=float, default=20)
    ap.add_argument("--backend", choices=("auto", "run_eval", "obstacle"), default="auto",
                    help="auto: obstacle backend when a sampled map has boxes, else run_eval (team default)")
    ap.add_argument("--domain-base", type=int, default=int(os.environ.get("BENCH_DOMAIN_BASE") or 50))
    ap.add_argument("--partition", default="lf_p_drive", help="GZ_PARTITION prefix")
    ap.add_argument("--output", type=Path)
    ap.add_argument("--summarize-only", type=Path, help="rebuild summary.md for an existing output dir")
    a = ap.parse_args()
    if not 1 <= a.jobs <= 3:
        ap.error("--jobs must be 1-3 (at most 3 Gazebo simulations at once)")
    if a.domain_base < 50 or a.domain_base + 19 > 69:   # bench.py uses a block of 20 domains
        ap.error("--domain-base must keep the runs inside ROS domains 50-69")

    if a.summarize_only:
        out = a.summarize_only.resolve()
        info = json.loads((out / "summary.json").read_text())
        picked = [(row["suite"], row) for row in csv.DictReader((out / "sample/maps.csv").open())]
        results = [json.loads(p.read_text()) for p in sorted((out / "bench").glob("*/result.json"))]
        summarize(out, {k: info[k] for k in info if k not in ("maps", "failures")}, picked, results,
                  appendix_text(out, picked))
        print((out / "summary.md").read_text())
        return 0

    suites = [s for s in a.suites.split(",") if s]
    if a.with_obstacles and "obstacles100" not in suites:
        suites.append("obstacles100")
    seed = a.seed if a.seed is not None else sample_maps.secrets.randbelow(2 ** 31)
    out = (a.output or ROOT / "experiments/results/random" / f"{time.strftime('%Y%m%d-%H%M%S')}_seed{seed}").resolve()
    out.mkdir(parents=True, exist_ok=True)
    print(f"random drive: seed {seed}, output {out}", flush=True)
    _, seed, picked = sample_maps.run(a.n, suites, seed, a.mode, out / "sample")
    params = a.params or os.environ.get("LF_PARAMS") or str(DEFAULT_PARAMS)
    backend = a.backend
    if backend == "auto":
        backend = "obstacle" if any(int(row["obstacles"]) for _, row in picked) else "run_eval"
    env = dict(os.environ, LF_PARAMS=params, BENCH_DOMAIN_BASE=str(a.domain_base),
               LAB_NAME=os.environ.get("LAB_NAME", "lf_p"), GZ_PARTITION=a.partition,
               ROS_DOMAIN_ID=os.environ.get("ROS_DOMAIN_ID", "22"))
    cmd = [sys.executable, str(BENCH), *[str(ROOT / row["path"]) for _, row in picked],
           "--follower", str(a.follower), "--jobs", str(a.jobs), "--off", str(a.off), "--limit", str(a.limit),
           "--stall", str(a.stall), "--backend", backend, "--output", str(out / "bench")]
    t0 = time.time()
    with (out / "bench.log").open("w") as log:
        log.write(" ".join(cmd) + "\n")
        log.flush()
        proc = subprocess.Popen(cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        for line in proc.stdout:
            log.write(line)
            print(line, end="", flush=True)
        code = proc.wait()
    results = [json.loads(p.read_text()) for p in sorted((out / "bench").glob("*/result.json"))]
    info = dict(created=time.strftime("%Y-%m-%d %H:%M:%S"), wall_min=(time.time() - t0) / 60, seed=seed,
                suites=suites, mode=a.mode, follower=str(Path(a.follower).resolve().relative_to(ROOT))
                if Path(a.follower).resolve().is_relative_to(ROOT) else str(a.follower),
                params=params, backend=backend, off=a.off, limit=a.limit, stall=a.stall, jobs=a.jobs,
                domain_base=a.domain_base, partition=a.partition, bench_exit=code)
    summary = summarize(out, info, picked, results, appendix_text(out, picked))
    print(f"\n완주 {summary['completed']}/{summary['runs']}, 평균 진행률 {summary['mean_progress_pct']}%, "
          f"평균 랩 {summary['mean_lap_sec']} s -> {out / 'summary.md'}")
    return 0 if code in (0, 1) else code


def appendix_text(out, picked):
    """worlds/summarize_suite.py over the bench output (feature table + PNG)."""
    indexes = sorted({str(sample_maps.SUITES_DIR / suite / "index.csv") for suite, _ in picked})
    try:
        res = subprocess.run([sys.executable, str(SUMMARIZE), str(out / "bench"), *indexes,
                              "--png", str(out / "by_feature.png")], capture_output=True, text=True, timeout=300)
        return res.stdout + (res.stderr if res.returncode else "")
    except (OSError, subprocess.SubprocessError) as error:
        return f"summarize_suite.py failed: {error}"


if __name__ == "__main__":
    sys.exit(main())
