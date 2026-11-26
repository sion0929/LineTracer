#!/usr/bin/env python3
"""Isaac domain-randomized sample frames (lights, mat texture, line ink, noise, random poses).
  $isaac/python.sh experiments/isaac/isaac_rand_sample.py OUT_DIR decoy_0000 [...] [--frames N]"""
import argparse, json, sys, types
from pathlib import Path
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, "/home/rvlab/orca/workspaces/26_robotprogramming_project/d-isaac/experiments/isaac")
sys.path.insert(0, str(HERE))
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("out_dir", type=Path)
parser.add_argument("maps", nargs="+")
parser.add_argument("--frames", type=int, default=10)
cli = parser.parse_args()
if cli.frames < 1:
    parser.error("--frames must be positive")
from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "renderer": "RaytracedLighting", "multi_gpu": False})
import capture as cap                              # noqa: E402
from json_to_usd import convert                    # noqa: E402
out_root = cli.out_dir
args = types.SimpleNamespace(frames=cli.frames, subframes=2, batch_pause=0.3, image_noise=True, mat_texture=True,
                             max_bytes=2_000_000_000, parallel=2)
for name in cli.maps:
    js = next((ROOT / "experiments/worlds/suites/decoy1000").glob(f"part*/maps/{name}.json"))
    data = json.loads(js.read_text())
    usd = HERE / "sample_usd" / f"{name}.usdc"
    if not usd.exists():
        convert(js, usd)
    out = out_root / name; out.mkdir(parents=True, exist_ok=True)
    cap.capture_map(usd, data, out, args, cap.camera_spec(), 0)
    print(name, "ok", flush=True)
app.close()
