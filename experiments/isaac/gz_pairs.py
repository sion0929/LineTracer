#!/usr/bin/env python3
"""Gazebo side of the same-pose comparison: teleport to pairs/<map>_poses.json, save RGB frames.
Run in the lab container: python3 experiments/isaac/gz_pairs.py decoy_0000 [...]"""
import json, math, os, subprocess, sys, time
from pathlib import Path
import cv2, numpy as np
ROOT = Path(__file__).resolve().parents[2]
# usage: gz_pairs.py MAP [MAP...]  or  gz_pairs.py --sdf WORLD.sdf --poses MAP --out DIR
argv = sys.argv[1:]
jobs = []
if argv[:1] == ["--sdf"]:
    jobs.append((Path(argv[1]), argv[3], Path(argv[5])))
else:
    for name in argv:
        jobs.append((ROOT / f"experiments/worlds/suites/decoy1000/part0/maps/{name}.sdf", name,
                     Path(__file__).parent / "pairs" / name / "gazebo"))
for sdf, name, out in jobs:
    poses = json.loads((Path(__file__).parent / "pairs" / f"{name}_poses.json").read_text())
    env = dict(os.environ, GZ_PARTITION=f"gzpair_{os.getpid()}_{name}",
               GZ_SIM_RESOURCE_PATH=str(ROOT / "src/line_follower_bringup/models"))
    os.environ.update(env)
    sim = subprocess.Popen(["gz", "sim", "-s", "-r", "--headless-rendering", str(sdf)],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env)
    from gz.transport13 import Node
    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.pose_pb2 import Pose
    from gz.msgs10.boolean_pb2 import Boolean
    node = Node(); frames = []
    node.subscribe(Image, "/camera/image_raw", lambda m: frames.append((time.monotonic(), np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3).copy())) or frames.__delitem__(slice(0, -2)))
    t = time.monotonic()
    while not frames and time.monotonic() - t < 90:
        time.sleep(0.2)
    out.mkdir(parents=True, exist_ok=True)
    for k, p in enumerate(poses):
        req = Pose(); req.name = "turtlebot3_waffle_pi"
        req.position.x, req.position.y, req.position.z = p["position"][0], p["position"][1], 0.01
        req.orientation.z, req.orientation.w = p["quaternion_xyzw"][2], p["quaternion_xyzw"][3]
        node.request("/world/line_track/set_pose", req, Pose, Boolean, 2000)
        t_set = time.monotonic()
        while time.monotonic() - t_set < 3 and (not frames or frames[-1][0] < t_set + 0.4):
            time.sleep(0.02)
        cv2.imwrite(str(out / f"{k:02d}.png"), cv2.cvtColor(frames[-1][1], cv2.COLOR_RGB2BGR))
    sim.terminate(); sim.wait(10)
    print(name, len(poses))
