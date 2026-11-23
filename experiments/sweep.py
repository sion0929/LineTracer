#!/usr/bin/env python3
"""Optimize the follower on a fixed map set using a resumable Optuna study."""

import argparse
from datetime import date
import fcntl
import hashlib
import json
from pathlib import Path
import shlex
import signal
import statistics
import sys

from bench import BRINGUP, HERE, MAX_JOBS, atomic_json, run_matrix

PARAMETERS = {
    "vmax": (0.16, 0.42),
    "alat": (0.10, 0.35),
    "ld-min": (0.035, 0.085),
    "ld-gain": (0.15, 0.55),
    "accel": (0.30, 0.90),
    "waccel": (2.0, 6.0),
}
DEFAULTS = dict(zip(PARAMETERS, (0.22, 0.25, 0.06, 0.4, 0.6, 4.0)))


class TrialFailure(RuntimeError):
    """An infrastructure error prevented evaluation of a parameter set."""


def fingerprint(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def configuration(args):
    maps = []
    for path in args.maps:
        if path == BRINGUP / "worlds/line_track.sdf":
            assets = [path, HERE / "gt.npy", BRINGUP / "models/track/meshes/track.stl"]
        else:
            assets = [path, path.with_suffix(".stl"), path.with_suffix(".json")]
        maps.append({"path": str(path), "sha256": [fingerprint(file) for file in assets]})
    return {
        "version": 1, "maps": maps, "limit": args.limit, "stall": args.stall,
        "follower": str(args.follower), "follower_sha256": fingerprint(args.follower),
        "bench_sha256": fingerprint(HERE / "bench.py"),
        "seed": args.seed, "fixed_args": shlex.split(args.follower_args),
    }


def metrics_for(results, limit):
    laps = [r["lap_sec"] for r in results if r["complete"]]
    lateral = statistics.mean(r["lat_p95"] for r in results)
    mean_lap = statistics.mean(laps) if laps else None
    pace = 0.0 if mean_lap is None else 1 - min(1, mean_lap / limit)
    accuracy = 1 - min(1, lateral / 0.08)
    # Secondary scores total less than 1, so an extra finish always wins.
    value = len(laps) + 0.79 * pace + 0.20 * accuracy
    return value, {
        "finish_count": len(laps), "finish_rate": len(laps) / len(results),
        "mean_lap_sec": mean_lap, "mean_p95_m": lateral,
        "mean_progress_pct": statistics.mean(r["progress_pct"] for r in results),
        "collision_count": sum(r["collision_count"] for r in results),
        "slip_count": sum(r["slip_count"] for r in results),
    }


def save_summary(study, complete_state, output, total):
    trials = study.get_trials(deepcopy=False)
    successes = sorted((t for t in trials if t.state == complete_state), key=lambda t: -t.value)
    lines = [
        "# 라인트레이서 파라미터 스윕",
        "",
        f"유효 trial {len(successes)}/{total} · 전체 기록 {len(trials)} · 평가 맵 {len(study.user_attrs['config']['maps'])}개",
        "",
        "| Trial | 상태 | 점수 | 완주율 | 평균 랩타임 s | 평균 횡오차 p95 m | 평균 진행률 | 충돌 | 슬립 |",
        "| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for trial in trials:
        if trial.state == complete_state:
            m = trial.user_attrs
            lap = "-" if m["mean_lap_sec"] is None else f'{m["mean_lap_sec"]:.2f}'
            lines.append(f'| {trial.number} | {trial.state.name} | {trial.value:.4f} | '
                         f'{m["finish_rate"]:.0%} | {lap} | {m["mean_p95_m"]:.4f} | '
                         f'{m["mean_progress_pct"]:.2f}% | {m["collision_count"]} | {m["slip_count"]} |')
        else:
            lines.append(f"| {trial.number} | {trial.state.name} | - | - | - | - | - | - | - |")
    lines.extend(["", "## 상위 5개", "",
                  "| Trial | 점수 | " + " | ".join(PARAMETERS) + " |",
                  "| ---: | ---: | " + " | ".join("---:" for _ in PARAMETERS) + " |"])
    for trial in successes[:5]:
        values = " | ".join(f"{trial.params[name]:.4f}" for name in PARAMETERS)
        lines.append(f"| {trial.number} | {trial.value:.4f} | {values} |")
    lines.extend(["", "개별 맵 결과는 각 `trial_NNNN/results.csv`에 저장된다.", ""])
    temp = output / "summary.tmp"
    temp.write_text("\n".join(lines))
    temp.replace(output / "summary.md")


def run(args, optuna):
    state = optuna.trial.TrialState
    opts = configuration(args)
    args.output.mkdir(parents=True, exist_ok=True)
    if any(arg.split("=")[0].removeprefix("--") in PARAMETERS for arg in opts["fixed_args"]):
        raise ValueError("--follower-args must not override tuned parameters")
    with (args.output / ".sweep.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise ValueError(f"a sweep is already running in {args.output}") from error
        db = args.output / "study.sqlite3"
        study = optuna.create_study(
            study_name="follower", storage=f"sqlite:///{db}", direction="maximize",
            load_if_exists=True,
        )
        if "config" in study.user_attrs and study.user_attrs["config"] != opts:
            raise ValueError("map set, follower, scoring code or trial settings changed; use a new output directory")
        study.set_user_attr("config", opts)
        atomic_json(args.output / "config.json", opts)
        running = study.get_trials(deepcopy=False, states=(state.RUNNING,))
        for trial in running:
            study.tell(trial.number, state=state.FAIL)
        if not study.trials:
            study.enqueue_trial(DEFAULTS)
        save_summary(study, state.COMPLETE, args.output, args.trials)

        def objective(trial):
            follower_args = list(opts["fixed_args"])
            for name, (low, high) in PARAMETERS.items():
                follower_args.extend((f"--{name}", str(trial.suggest_float(name, low, high))))
            path = args.output / f"trial_{trial.number:04d}"
            results = run_matrix(args.maps, {"optuna": follower_args}, path, jobs=args.jobs,
                                 limit=args.limit, stall=args.stall, follower=args.follower)
            failed = [r["reason"] for r in results if r["reason"].startswith("error:")]
            if failed:
                trial.set_user_attr("errors", failed)
                raise TrialFailure(f"{path}: {failed[0]}")
            value, metrics = metrics_for(results, args.limit)
            for key, metric in metrics.items():
                trial.set_user_attr(key, metric)
            return value

        errors = 0
        try:
            while len(study.get_trials(deepcopy=False, states=(state.COMPLETE,))) < args.trials:
                # A trial-specific seed prevents repeats caused by resetting a sampler on restart.
                study.sampler = optuna.samplers.TPESampler(seed=(args.seed + len(study.trials)) % (2 ** 32))
                study.optimize(objective, n_trials=1, catch=(TrialFailure,))
                errors = errors + 1 if study.trials[-1].state == state.FAIL else 0
                save_summary(study, state.COMPLETE, args.output, args.trials)
                if errors >= 3:
                    raise TrialFailure("three consecutive trial infrastructure failures")
        finally:
            save_summary(study, state.COMPLETE, args.output, args.trials)
    print(f"{args.output / 'summary.md'} · {len(study.get_trials(states=(state.COMPLETE,)))} complete")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("maps", nargs="+", type=Path)
    parser.add_argument("--output", type=Path, default=HERE / "sweeps" / date.today().isoformat())
    parser.add_argument("--trials", type=int, default=10, help="total number of successful trials including prior runs")
    parser.add_argument("--jobs", type=int, default=1, help="parallel maps within each trial")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--limit", type=float, default=300)
    parser.add_argument("--stall", type=float, default=20)
    parser.add_argument("--follower", type=Path, default=HERE / "follower.py")
    parser.add_argument("--follower-args", default="", help="quoted flags to keep fixed for every trial")
    args = parser.parse_args()
    if args.trials < 1 or not 1 <= args.jobs <= MAX_JOBS or min(args.limit, args.stall) <= 0:
        parser.error(f"trials and time limits must be positive and jobs must be in 1–{MAX_JOBS}")
    try:
        import optuna
    except ImportError:
        parser.error("Optuna is required. See experiments/sweeps/README.md.")
    args.output = args.output.resolve()
    args.maps = [path.resolve() for path in args.maps]
    args.follower = args.follower.resolve()

    def interrupted(_number, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupted)
    try:
        run(args, optuna)
    except (OSError, ValueError, TrialFailure) as error:
        parser.exit(1, f"{error}\n")
    except KeyboardInterrupt:
        parser.exit(130, f"Interrupted. Resume from {args.output / 'study.sqlite3'}\n")


if __name__ == "__main__":
    sys.exit(main())
