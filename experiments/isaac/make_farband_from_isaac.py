#!/usr/bin/env python3
"""Convert completed decoy1000 maps into far-band training archives."""
import argparse
import json
from pathlib import Path
from tempfile import NamedTemporaryFile

import numpy as np

DATASET = Path("/home/rvlab/isaac_datasets/26_robotprogramming/decoy1000")
OUTPUT = Path(__file__).resolve().parents[1] / "farband/data_isaac"
BAND = slice(256, 320)
SPLITS = {"train": range(900), "val": range(900, 950)}


def npz_frames(folder):
    """Read the complete map's manifest in capture order."""
    summary = json.loads((folder / "summary.json").read_text())
    records = [json.loads(line) for line in (folder / "frames.jsonl").read_text().splitlines()]
    files = [folder / record["file"] for record in records]
    if not files or len(files) != summary["frames"] or len(set(files)) != len(files):
        raise ValueError(f"Invalid frame count or duplicate frame in {folder}")
    for file in files:
        if file.parent != folder or file.suffix != ".npz" or not file.is_file():
            raise ValueError(f"Invalid frame in {folder}: {file}")
    return files


def convert_map(folder, output):
    files = npz_frames(folder)
    x = np.empty((len(files), 64, 640, 3), dtype=np.uint8)
    y = np.empty((len(files), 64, 640), dtype=np.uint8)
    for i, path in enumerate(files):
        with np.load(path, allow_pickle=False) as frame:
            rgb, classes = frame["rgb"], frame["classes"]
            if rgb.shape != (480, 640, 3) or classes.shape != (480, 640):
                raise ValueError(f"Unexpected camera dimensions in {path}")
            if rgb.dtype != np.uint8 or classes.dtype != np.uint8:
                raise ValueError(f"Expected uint8 RGB and classes in {path}")
            x[i] = rgb[BAND]
            y[i] = 255 * (classes[BAND] == 1)
    # The trainer holds out the last stems across ALL input folders (tile_* sorts first).
    target = output / f"z_{folder.name}.npz"
    with NamedTemporaryFile(dir=output, suffix=".tmp", delete=False) as temp:
        temporary = Path(temp.name)
        try:
            np.savez_compressed(temp, x=x, y=y)
            temp.close()
            temporary.replace(target)
        finally:
            temporary.unlink(missing_ok=True)
    return len(files)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, default=DATASET)
    parser.add_argument("--out", type=Path, default=OUTPUT)
    parser.add_argument("--split", choices=("train", "val", "all"), default="all")
    args = parser.parse_args()
    splits = SPLITS if args.split == "all" else (args.split,)
    args.out.mkdir(parents=True, exist_ok=True)
    for split in splits:
        folders = [args.data / split / f"decoy_{seed:04d}" for seed in SPLITS[split]]
        complete = [folder for folder in folders if (folder / "summary.json").is_file()]
        frames = sum(convert_map(folder, args.out) for folder in complete)
        print(f"{split}: {len(complete)} maps, {frames} frames; "
              f"{len(folders) - len(complete)} maps pending", flush=True)


if __name__ == "__main__":
    main()
