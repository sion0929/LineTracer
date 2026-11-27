#!/usr/bin/env python3
"""Copy finished Isaac runs into a small, shareable planning-log folder.

    python3 experiments/isaac/export_planning_logs.py BATCH_DIR/runs experiments/planning_logs/<PROFILE>

Keeps per run: perception_log/ (meta.json + chunks), trajectory.npy, result.json, job.json.
Absolute repo paths in job.json / result.json become repo-relative; Isaac logs, USD and textures
are left out. Runs without a perception log or a result are skipped.
"""
import json
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def rel(v):
    if isinstance(v, str):
        for pre in (str(ROOT) + '/', '/ws/'):
            if v.startswith(pre):
                return v[len(pre):]
        return v
    if isinstance(v, list):
        return [rel(x) for x in v]
    if isinstance(v, dict):
        return {k: rel(x) for k, x in v.items()}
    return v


def main():
    src, dst = Path(sys.argv[1]), Path(sys.argv[2])
    dst.mkdir(parents=True, exist_ok=True)
    n = 0
    for run in sorted(p for p in src.iterdir() if p.is_dir()):
        log = run / 'perception_log'
        if not (log / 'meta.json').exists() or not (run / 'result.json').exists():
            print('skip', run.name)
            continue
        out = dst / run.name
        if out.exists():
            shutil.rmtree(out)
        shutil.copytree(log, out / 'perception_log')
        shutil.copy2(run / 'trajectory.npy', out / 'trajectory.npy')
        for name in ('job.json', 'result.json'):
            (out / name).write_text(json.dumps(rel(json.loads((run / name).read_text())), indent=1) + '\n')
        n += 1
    print(n, 'runs ->', dst)


if __name__ == '__main__':
    main()
