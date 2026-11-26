#!/usr/bin/env python3
"""Collect the decoy1000 suite with one isolated worker per map attempt."""
import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import zipfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
SUITE = ROOT / "experiments/worlds/suites/decoy1000"
ISAAC = Path("/home/rvlab/isaacsim/python.sh")
SPLITS = {"train": range(900), "val": range(900, 950), "test": range(950, 1000)}
GB = 1_000_000_000


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--frames", type=int, default=40)
    parser.add_argument("--split", choices=(*SPLITS, "all"), required=True)
    parser.add_argument("--limit", type=int, help="Select the first N maps, including resumed maps")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--shard", default="0/1", help="K/N: only maps whose index %% N == K (parallel workers)")
    parser.add_argument("--timeout-minutes", type=float, default=10)
    parser.add_argument("--budget-gb", type=float, default=60)
    parser.add_argument("--min-free-gb", type=float, default=1)
    parser.add_argument("--gpu", default="0", help="nvidia-smi GPU index or UUID to monitor")
    parser.add_argument("--max-vram-mib", type=int, default=19500)
    parser.add_argument("--max-gpu-util", type=int, default=80)
    parser.add_argument("--gpu-wait-minutes", type=float, default=10)
    parser.add_argument("--gpu-poll-seconds", type=float, default=5)
    args = parser.parse_args()
    for name in ("timeout_minutes", "gpu_poll_seconds", "budget_gb"):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            parser.error(f"--{name.replace('_', '-')} must be finite and positive")
    for name in ("min_free_gb", "gpu_wait_minutes"):
        value = getattr(args, name)
        if not math.isfinite(value) or value < 0:
            parser.error(f"--{name.replace('_', '-')} must be finite and nonnegative")
    if args.frames < 1 or (args.limit is not None and args.limit < 1):
        parser.error("--frames and --limit must be positive")
    if args.max_vram_mib < 0 or not 0 <= args.max_gpu_util <= 100:
        parser.error("invalid GPU thresholds")
    args.out = args.out.resolve()
    return args


def output_bytes(out):
    return sum(path.lstat().st_size for path in out.rglob("*") if path.is_file())


def disk_stop(args, size):
    free = shutil.disk_usage(args.out).free
    if size >= args.budget_gb * GB:
        return f"budget reached ({size} bytes, limit {args.budget_gb:g} GB)"
    if free <= args.min_free_gb * GB:
        return f"free disk too low ({free} bytes, reserve {args.min_free_gb:g} GB)"
    return None


def completed_frames(directory):
    try:
        frames = json.loads((directory / "summary.json").read_text())["frames"]
    except (FileNotFoundError, ValueError, KeyError, TypeError):
        return None
    return frames if type(frames) is int and frames > 0 else None


def wait_for_gpu(args, log):
    deadline = time.monotonic() + args.gpu_wait_minutes * 60
    command = ["nvidia-smi", "-i", args.gpu, "--query-gpu=memory.used,utilization.gpu",
               "--format=csv,noheader,nounits"]
    while True:
        try:
            result = subprocess.check_output(command, text=True, timeout=5, stderr=log)
            memory, util = map(int, result.strip().split(","))
        except (OSError, subprocess.SubprocessError, ValueError) as exc:
            raise RuntimeError(f"GPU query failed: {exc}") from exc
        print(f"GPU sample: {memory} MiB, {util}%", file=log, flush=True)
        if memory <= args.max_vram_mib and util <= args.max_gpu_util:
            return
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise RuntimeError(f"GPU wait expired at {memory} MiB, {util}%")
        time.sleep(min(remaining, args.gpu_poll_seconds))


def fake_capture(directory, frames):
    directory.mkdir(parents=True)
    # A valid one-element uint8 NumPy array, packed into an NPZ without NumPy.
    header = b"{'descr': '|u1', 'fortran_order': False, 'shape': (1,), }"
    header += b" " * (-(10 + len(header) + 1) % 64) + b"\n"
    array = b"\x93NUMPY\x01\x00" + len(header).to_bytes(2, "little") + header + b"\x00"
    records = []
    for index in range(min(3, frames)):
        name = f"{index:06d}.npz"
        with zipfile.ZipFile(directory / name, "w") as archive:
            archive.writestr("image.npy", array)
        records.append(json.dumps({"file": name}))
    (directory / "frames.jsonl").write_text("\n".join(records) + "\n")
    (directory / "summary.json").write_text(json.dumps({"frames": len(records), "dry_run": True}))
    print(f"fake capture wrote {len(records)} frames")


def capture(args, directory, log):
    if args.dry_run:
        command = [sys.executable, str(Path(__file__).resolve()), "_fake_capture",
                   str(directory), str(args.frames)]
    else:
        command = [str(ISAAC), str(HERE / "isaac_rand_sample.py"), str(directory.parent),
                   directory.name, "--frames", str(args.frames)]
    worker = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                              start_new_session=True)
    try:
        code = worker.wait(timeout=args.timeout_minutes * 60)
    finally:
        # Also reap a surviving launcher descendant after a timeout or interruption.
        try:
            os.killpg(worker.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        worker.wait()
    if code:
        raise RuntimeError(f"worker exited with status {code}")
    frames = completed_frames(directory)
    if frames is None:
        raise RuntimeError("worker did not write a valid summary.json")
    return frames


def collect(args):
    splits = SPLITS if args.split == "all" else (args.split,)
    shard_k, shard_n = (int(v) for v in args.shard.split("/"))
    selected = [(split, f"decoy_{index:04d}") for split in splits for index in SPLITS[split]
                if index % shard_n == shard_k]
    selected = selected[:args.limit]
    available = {path.stem for path in SUITE.glob("part*/maps/decoy_*.json")}
    missing = [name for _, name in selected if name not in available]
    if missing:
        raise RuntimeError(f"missing map JSON: {', '.join(missing)}")
    if not args.dry_run and not os.access(ISAAC, os.X_OK):
        raise RuntimeError(f"Isaac launcher unavailable: {ISAAC}")

    dry_marker = args.out / ".dry-run"
    if args.dry_run:
        if not dry_marker.exists() and any((args.out / split).exists() for split in SPLITS):
            raise RuntimeError("use a separate output directory for dry-run")
        dry_marker.touch()
    elif dry_marker.exists():
        raise RuntimeError("use a separate output directory for Isaac capture")

    logs = args.out / "logs"
    logs.mkdir(exist_ok=True)
    pending = []
    done = failed = frames = attempted = 0
    seconds = 0.0
    for split, name in selected:
        directory = args.out / split / name
        count = completed_frames(directory)
        if count is None:
            pending.append(directory)
        else:
            done += 1
            frames += count
            print(f"SKIP {split}/{name} (complete)", flush=True)

    def progress(status, reason=None):
        remaining = len(selected) - done - failed
        mean = seconds / attempted if attempted else None
        snapshot = {
            "split": args.split, "dry_run": args.dry_run, "status": status, "reason": reason,
            "selected": len(selected), "done": done, "failed": failed, "frames": frames,
            "remaining": remaining, "bytes": output_bytes(args.out),
            "mean_seconds_per_map": mean, "eta_seconds": mean * remaining if mean is not None
            else (0 if not remaining else None),
        }
        tag = "" if args.shard == "0/1" else "_" + args.shard.replace("/", "of")
        temp = args.out / f".progress{tag}.json.tmp"
        temp.write_text(json.dumps(snapshot, indent=2) + "\n")
        temp.replace(args.out / f"progress{tag}.json")

    for directory in pending:
        label = f"{directory.parent.name}/{directory.name}"
        started = time.monotonic()
        errors = []
        log_path = logs / f"{directory.name}.log"
        for attempt in (1, 2):
            reason = disk_stop(args, output_bytes(args.out))
            if reason:
                progress("stopped", reason)
                print(f"STOP before {label}: {reason}", flush=True)
                return 2
            with log_path.open("a") as log:
                print(f"Attempt {attempt}", file=log, flush=True)
                try:
                    if not args.dry_run:
                        wait_for_gpu(args, log)
                        reason = disk_stop(args, output_bytes(args.out))
                        if reason:
                            progress("stopped", reason)
                            print(f"STOP before {label}: {reason}", flush=True)
                            return 2
                    if directory.exists():
                        shutil.rmtree(directory)
                        print(f"REDO {label} (partial)", flush=True)
                    count = capture(args, directory, log)
                except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
                    errors.append(str(exc))
                    print(errors[-1], file=log, flush=True)
                    # A failed worker might have written its marker before exiting.
                    (directory / "summary.json").unlink(missing_ok=True)
                else:
                    done += 1
                    frames += count
                    print(f"DONE {label} ({count} frames)", flush=True)
                    break
        else:
            failed += 1
            with (args.out / "failures.jsonl").open("a") as failures:
                failures.write(json.dumps({"split": directory.parent.name,
                                           "map": directory.name, "errors": errors}) + "\n")
            print(f"FAILED {label}: {errors[-1]}", flush=True)
        attempted += 1
        seconds += time.monotonic() - started
        progress("running")

    progress("finished")
    print(f"Finished: done={done} failed={failed} frames={frames}", flush=True)
    return 1 if failed else 0


def terminate(signum, _frame):
    raise SystemExit(128 + signum)


def main():
    args = parse_args()
    signal.signal(signal.SIGTERM, terminate)
    try:
        args.out.mkdir(parents=True, exist_ok=True)
        with (args.out / f".collect{'' if args.shard == '0/1' else '_' + args.shard.replace('/', 'of')}.lock").open("a") as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as exc:
                raise RuntimeError(f"collector already running for {args.out}") from exc
            return collect(args)
    except (OSError, RuntimeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    if len(sys.argv) == 4 and sys.argv[1] == "_fake_capture":
        fake_capture(Path(sys.argv[2]), int(sys.argv[3]))
    else:
        sys.exit(main())
