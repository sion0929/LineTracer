#!/usr/bin/env python3
"""Isaac side: render pairs/<map>_poses.json with d-isaac capture_map (RTX, headless).
Run with Isaac's python: $isaac/python.sh experiments/isaac/isaac_pairs.py decoy_0000 [...]"""
import json, sys, types
from pathlib import Path
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, "/home/rvlab/orca/workspaces/26_robotprogramming_project/d-isaac/experiments/isaac")
sys.path.insert(0, str(HERE))                     # our json_to_usd (rotated ramps) wins
from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "renderer": "RaytracedLighting", "multi_gpu": False})
import capture as cap                              # noqa: E402
from json_to_usd import convert                    # noqa: E402
args = types.SimpleNamespace(frames=6, subframes=2, batch_pause=0.3, image_noise=False, mat_texture=False,
                             max_bytes=2_000_000_000, parallel=1)
for name in sys.argv[1:]:
    js = ROOT / f"experiments/worlds/suites/decoy1000/part0/maps/{name}.json"
    data = json.loads(js.read_text())
    poses = json.loads((HERE / "pairs" / f"{name}_poses.json").read_text())
    usd = HERE / "sample_usd" / f"{name}.usdc"
    if not usd.exists():
        convert(js, usd)
    out = HERE / "pairs" / name / "isaac"; out.mkdir(parents=True, exist_ok=True)
    args.frames = len(poses)
    cap.capture_map(usd, data, out, args, cap.camera_spec(), 0, paired_poses=poses)
    print(name, "ok", flush=True)
app.close()
