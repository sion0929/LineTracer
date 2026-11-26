#!/usr/bin/env python3
"""Pick flat poses on the route of each sample map (some facing decoys) -> pairs/<map>_poses.json."""
import json, math, random, sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "experiments/farband"))
import numpy as np
from capture import ramp_h  # noqa: E402
for k in range(10):
    name = f"decoy_{k:04d}"
    m = json.loads((ROOT / f"experiments/worlds/suites/decoy1000/part0/maps/{name}.json").read_text())
    C = np.array([p[:2] for p in m["centerline"]]); arc = np.array(m["arc_m"]); ramps = m["ramps"]
    rng = random.Random(k)
    targets = [d["route_index"] * 0.22 - 0.7 for d in m["case"].get("decoys", [])]   # ~0.7 m before a decoy
    poses = []
    tries = 0
    while len(poses) < 6 and tries < 500:
        tries += 1
        s = targets.pop(0) if targets else rng.uniform(0.3, arc[-1] - 0.5)
        i = int(np.clip(np.searchsorted(arc, s), 3, len(C) - 4))
        x, y = C[i]
        if ramp_h(ramps, np.array([x, x + 0.6]), np.array([y, y])).max() > 0 or \
           ramp_h(ramps, np.array([x]), np.array([y + 0.6])).max() > 0:
            continue
        yaw = math.atan2(C[i + 3][1] - C[i - 3][1], C[i + 3][0] - C[i - 3][0]) + rng.uniform(-0.15, 0.15)
        poses.append({"position": [float(x), float(y), 0.0],
                      "quaternion_xyzw": [0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)], "yaw": yaw})
    (Path(__file__).parent / "pairs" / f"{name}_poses.json").write_text(json.dumps(poses))
    print(name, len(poses))
