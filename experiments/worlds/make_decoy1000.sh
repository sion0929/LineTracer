#!/usr/bin/env bash
# Rebuild the decoy1000 map suite (1000 tile maps with decoys, 840 with a hill) and its
# train/val/test split folders. Deterministic: same seeds -> byte-identical json/sdf/stl
# (checked 2026-11-27 on part0). Takes about 1-2 minutes, ~150 MB.
#   bash experiments/worlds/make_decoy1000.sh            # all four parts + splits
#   bash experiments/worlds/make_decoy1000.sh 0          # only part0 (maps 0000-0249)
set -euo pipefail
cd "$(dirname "$0")/../.."
S=experiments/worlds/suites
parts=${1:-0 1 2 3}
for p in $parts; do
  python3 experiments/worlds/gen_tiles.py --n 250 --start $((p * 250)) --seed $((1000 + p * 250)) \
    --decoys 3 --prefix decoy --out $S/decoy1000/part$p
done
cat $S/decoy1000/part*/list.txt > $S/decoy1000/list.txt
# split by map index: train 0000-0899, val 0900-0949, test 0950-0999 (relative symlinks)
for split in train val test; do mkdir -p $S/decoy1000_$split/maps; done
for f in $S/decoy1000/part*/maps/decoy_*.json; do
  n=$(basename "$f" .json); i=$((10#${n#decoy_}))
  if (( i < 900 )); then split=train; elif (( i < 950 )); then split=val; else split=test; fi
  ln -sfn "../../decoy1000/$(basename "$(dirname "$(dirname "$f")")")/maps/$n.json" $S/decoy1000_$split/maps/$n.json
done
echo "decoy1000: $(ls $S/decoy1000/part*/maps/*.json | wc -l) maps; train $(ls $S/decoy1000_train/maps | wc -l), val $(ls $S/decoy1000_val/maps | wc -l), test $(ls $S/decoy1000_test/maps | wc -l)"
