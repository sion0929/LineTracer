# Collect decoy1000 with Isaac Sim

Run from the repository root on the Isaac host. Measure throughput on the first 5 maps using `--split train --limit 5`, then remove `--limit` to resume the full training split or switch to `--split all`.

```bash
python3 experiments/isaac/collect_decoy.py \
  --out /home/rvlab/isaac_datasets/26_robotprogramming/decoy1000 \
  --frames 40 --split all
```

Train contains indices 0–899, validation 900–949, and test 950–999. The driver visits maps in index order. `--limit` selects the first N maps in that order, including completed maps. Each attempt launches a fresh Isaac process with a 10-minute timeout by default. The standalone sampler still defaults to 10 frames.

Rerun with the same output path and frame count to resume. A readable `summary.json` with a positive frame count marks a completed map. A partial map directory is deleted before its next capture. A failed attempt gets one retry. Once both attempts fail, the driver appends the errors to `failures.jsonl` and moves on. The failure log retains history across runs.

The launch gate polls GPU 0 via `nvidia-smi` when memory usage exceeds 19500 MiB or utilization exceeds 80%. Configure it with `--gpu`, `--max-vram-mib`, `--max-gpu-util`, `--gpu-wait-minutes` (default 10 per attempt), and `--gpu-poll-seconds` (default 5). The GPU selector controls monitoring. GPU load is checked before launch, and the process timeout applies once capture starts.

The driver checks output size against `--budget-gb` (default 60 decimal GB) and available disk against `--min-free-gb` (default 1). These are launch gates. A map already in progress can take the total past the budget.

Monitor `OUT/progress.json` for selected, completed, failed, and remaining maps, frames, output bytes, mean seconds per attempted map, and ETA. Counts cover the current selection. Timing includes waits and retries during the current invocation. Inspect `OUT/logs/decoy_NNNN.log` for individual attempts. Exit status 0 indicates completion, 1 indicates a failure, and 2 indicates a disk or budget stop.

## Dry-run

```bash
out=$(mktemp -d /tmp/decoy-collect.XXXXXX)
python3 experiments/isaac/collect_decoy.py --out "$out" --frames 40 --split train --limit 5 --dry-run
```

Dry-run writes up to three tiny NPZ files and completion metadata per map. It skips GPU checks and uses its own output directory.
