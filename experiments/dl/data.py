"""Read completed per-map datasets and maintain map-level split boundaries."""

import json
from pathlib import Path

import cv2


def split_maps(dataset, split):
    root = Path(dataset) / split
    maps = []
    for path in sorted(root.glob("*/summary.json")):
        summary = json.loads(path.read_text())
        records = [json.loads(line) for line in (path.parent / "frames.jsonl").read_text().splitlines()]
        if not records or len(records) != summary["frames"]:
            raise ValueError(f"Invalid frame manifest in {path.parent}")
        if (path.parent / "gazebo.log").exists() and ".incomplete." in path.parent.name:
            continue
        maps.append((path.parent, summary, records))
    if not maps:
        raise ValueError(f"No completed maps in {root}")
    return maps


def check_disjoint(*splits):
    seen = set()
    for maps in splits:
        identities = set()
        for directory, summary, _ in maps:
            identity = ("official",) if summary["map_seed"] is None else ("random", summary["map_seed"])
            if identity in seen or identity in identities:
                raise ValueError(f"Map appears more than once: {directory} ({identity})")
            identities.add(identity)
        seen.update(identities)


def read_pair(directory, filename):
    image = cv2.imread(str(directory / "image" / filename), cv2.IMREAD_COLOR)
    mask = cv2.imread(str(directory / "mask" / filename), cv2.IMREAD_GRAYSCALE)
    if image is None or mask is None:
        raise OSError(f"Missing image or mask: {directory}/{filename}")
    return image, mask
