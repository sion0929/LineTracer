#!/usr/bin/env python3
"""Score Isaac Drive runs on the official grading sections.

Input folders contain one directory per run (for example V0_01) with result.json
and trajectory.npy.  trajectory.npy columns are
[sim_s, x_m, y_m, candidate_s_m, lateral_error_m].
"""
import argparse
import itertools
import json
import math
import re
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))
from sections import GRADED_ACC, NAMES  # noqa: E402

RUN_SUFFIX = re.compile(r"_\d+$")


def profile_name(run_name):
    return RUN_SUFFIX.sub("", run_name)


def load_ends():
    return np.asarray(json.loads((HERE / "sections.json").read_text())["ends_s"], float)


def section_stats(traj, ends):
    traj = np.asarray(traj, float)
    if traj.size == 0:
        return None
    s = traj[:, 3]
    lat = traj[:, 4]
    finite_s = s[np.isfinite(s)]
    best = float(finite_s.max()) if len(finite_s) else float("nan")
    out = []
    lo = 0.0
    for hi in ends:
        rows = (s >= lo) & (s < hi) & np.isfinite(lat)
        length = hi - lo
        reached = bool(np.isfinite(best) and best >= lo + 0.8 * length)
        mean_cm = float(np.mean(np.abs(lat[rows])) * 100) if rows.any() else float("nan")
        out.append({"reached": reached, "mean_cm": mean_cm, "samples": int(rows.sum())})
        lo = hi
    return out


def run_pass_count(stats):
    return sum(1 for item in stats if item["reached"] and np.isfinite(item["mean_cm"])
               and item["mean_cm"] <= 2.0)


def load_run(run):
    result = json.loads((run / "result.json").read_text())
    traj = np.load(run / "trajectory.npy")
    return result, traj


def collect(folders, ends):
    runs = []
    seen = set()
    for folder in folders:
        folder = Path(folder)
        if not folder.is_dir():
            continue
        candidates = [folder] if (folder / "result.json").is_file() else []
        candidates.extend(sorted(p for p in folder.iterdir() if p.is_dir()))
        for run in candidates:
            if run in seen or not (run / "result.json").is_file() or not (run / "trajectory.npy").is_file():
                continue
            seen.add(run)
            try:
                result, traj = load_run(run)
            except (OSError, ValueError, KeyError):
                continue
            stats = section_stats(traj, ends)
            if stats is None:
                continue
            runs.append({"run": run.name, "profile": profile_name(run.name),
                         "result": result, "sections": stats,
                         "passes": run_pass_count(stats)})
    return runs


def summarize(runs, ends):
    n = len(runs)
    complete = sum(1 for run in runs if run["result"].get("complete") is True)
    laps = [float(run["result"]["lap_sec"]) for run in runs
            if run["result"].get("lap_sec") is not None and np.isfinite(run["result"]["lap_sec"])]
    section_means, section_stds, reach = [], [], []
    for index in range(len(ends)):
        errors = [run["sections"][index]["mean_cm"] for run in runs
                  if run["sections"][index]["reached"] and np.isfinite(run["sections"][index]["mean_cm"])]
        section_means.append(float(np.mean(errors)) if errors else float("nan"))
        section_stds.append(float(np.std(errors)) if errors else float("nan"))
        reach.append(sum(1 for run in runs if run["sections"][index]["reached"]))
    return {"runs": n, "complete": complete, "mean_lap_sec": float(np.mean(laps)) if laps else None,
            "section_mean_cm": section_means, "section_std_cm": section_stds,
            "mean_pass_sections": float(np.mean([run["passes"] for run in runs])) if runs else None,
            "reach": reach}


def fisher_one_sided(a_success, a_total, b_success, b_total):
    a_fail = a_total - a_success
    b_fail = b_total - b_success
    n = a_total + b_total
    successes = a_success + b_success
    low = max(0, a_total - (n - successes))
    high = min(a_total, successes)
    numerator = 0
    for x in range(int(np.clip(a_success, low, high)), high + 1):
        numerator += math.comb(a_total, x) * math.comb(b_total, successes - x)
    denominator = math.comb(n, successes)
    return numerator / denominator if denominator else float("nan")


def section_header():
    cells = []
    for index, name in enumerate(NAMES, 1):
        label = f"S{index}"
        if index <= len(GRADED_ACC) and GRADED_ACC[index - 1] == "X":
            label = f"**{label}**"
        cells.append(f"{label} {name}")
    return "| " + " | ".join(cells) + " |"


def render(report):
    profiles = report["profiles"]
    lines = ["# Isaac section comparison", "",
             f"Official graded sections: {GRADED_ACC} (X = failed official grading, bold)", "",
             "| Profile | Complete | Mean lap (s) | " + section_header()[2:-2] + " | Sections <= 2 cm |",
             "|---|" + "---:|" * (len(NAMES) + 2)]
    for name, summary in profiles.items():
        cells = []
        for mean, std in zip(summary["section_mean_cm"], summary["section_std_cm"]):
            cells.append("—" if not np.isfinite(mean) else f"{mean:.2f} ± {std:.2f}")
        lap = "—" if summary["mean_lap_sec"] is None else f"{summary['mean_lap_sec']:.2f}"
        passes = "—" if summary["mean_pass_sections"] is None else f"{summary['mean_pass_sections']:.1f} / {len(NAMES)}"
        lines.append("| " + " | ".join([name, f"{summary['complete']}/{summary['runs']}", lap,
                                        *cells, passes]) + " |")
    lines += ["", "Section reach (runs reached / runs):", "",
              "| Profile | " + " | ".join(f"S{index}" for index in range(1, len(NAMES) + 1)) + " |",
              "|---|" + "---:|" * len(NAMES)]
    for name, summary in profiles.items():
        lines.append("| " + " | ".join([name, *(f"{count}/{summary['runs']}" for count in summary["reach"])]) + " |")
    if len(profiles) >= 2:
        lines += ["", "Completion Fisher exact, one-sided P(A completion > B completion):", "",
                  "| A | B | A complete | B complete | p |", "|---|---|---|---|---:|"]
        ordered = sorted(profiles, key=lambda name: (-profiles[name]["complete"] / profiles[name]["runs"], name))
        for a_name, b_name in itertools.combinations(ordered, 2):
            a, b = profiles[a_name], profiles[b_name]
            p = fisher_one_sided(a["complete"], a["runs"], b["complete"], b["runs"])
            lines.append(f"| {a_name} | {b_name} | {a['complete']}/{a['runs']} "
                         f"| {b['complete']}/{b['runs']} | {p:.4g} |")
    lines += ["", "Per-run results:", "",
              "| Profile | Run | Complete | Reason | Progress % | Lap s | Sections <= 2 cm |",
              "|---|---|---:|---|---:|---:|---:|"]
    for run in report["runs"]:
        result = run["result"]
        lap = result.get("lap_sec")
        progress = result.get("progress_pct")
        lines.append("| " + " | ".join([
            run["profile"], run["run"], "yes" if result.get("complete") else "no",
            str(result.get("reason", "")).replace("|", "\\|"),
            "—" if progress is None else f"{progress:.2f}",
            "—" if lap is None else f"{lap:.2f}", f"{run['passes']} / {len(NAMES)}"]) + " |")
    return "\n".join(lines) + "\n"


def build_report(folders):
    ends = load_ends()
    runs = collect(folders, ends)
    grouped = {}
    for run in runs:
        grouped.setdefault(run["profile"], []).append(run)
    return {"sections": ends.tolist(), "names": NAMES, "graded": GRADED_ACC, "runs": runs,
            "profiles": {name: summarize(items, ends) for name, items in sorted(grouped.items())}}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("folders", nargs="+", type=Path)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    report = build_report(args.folders)
    text = render(report) if report["runs"] else "No runs with result.json and trajectory.npy found.\n"
    if args.out:
        args.out.write_text(text)
    print(text, end="")
    return 0 if report["runs"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
