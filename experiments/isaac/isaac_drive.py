#!/usr/bin/env python3
"""Render an Isaac track and drive it with commands from a TCP ROS relay."""

import argparse
from contextlib import ExitStack
import json
import math
import os
from pathlib import Path
import random
import signal
import socket
import sys
import time
import traceback
import xml.etree.ElementTree as ET

from drive_core import BRINGUP, DT, HEIGHT, HZ, WIDTH, Replay, Score, drive, load_map

CAPTURE_DIR = Path("/home/rvlab/orca/workspaces/26_robotprogramming_project/d-isaac/experiments/isaac")


def camera_spec():
    """Read the same SDF camera link and intrinsics as d-isaac capture.py."""
    model = ET.parse(BRINGUP / "models/turtlebot3_waffle_pi/model.sdf")
    link = model.find(".//link[@name='camera_rgb_frame']")
    camera = link.find("sensor/camera")
    resolution = tuple(int(camera.findtext(f"image/{side}")) for side in ("width", "height"))
    if resolution != (WIDTH, HEIGHT):
        raise ValueError(f"SDF camera does not match protocol resolution: {resolution}")
    return (list(map(float, link.findtext("pose").split())), resolution,
            float(camera.findtext("horizontal_fov")),
            tuple(float(camera.findtext(f"clip/{bound}")) for bound in ("near", "far")))


class Renderer:
    def __init__(self, usd, args, cleanup):
        import carb
        import numpy as np
        import omni.replicator.core as rep
        import omni.usd
        from pxr import Gf, UsdGeom, UsdLux

        context = omni.usd.get_context()
        if not context.open_stage(str(usd.resolve())):
            raise RuntimeError(f"Unable to open {usd}")
        stage = context.get_stage()
        key = UsdLux.DistantLight.Define(stage, "/World/Key")
        key.CreateAngleAttr(8.0)
        key.CreateIntensityAttr(1800.0)
        fill = UsdLux.DomeLight.Define(stage, "/World/Fill")
        fill.CreateIntensityAttr(350.0)
        if args.lights:
            rng = random.Random(args.seed)
            key.GetIntensityAttr().Set(rng.uniform(900, 2400))
            fill.GetIntensityAttr().Set(rng.uniform(120, 700))
            UsdGeom.XformCommonAPI(key).SetRotate(
                Gf.Vec3f(rng.uniform(10, 65), rng.uniform(-45, 45), 0))
        if args.texture:
            texture_dir = Path(os.environ.get("ISAAC_CAPTURE_DIR", CAPTURE_DIR))
            if not (texture_dir / "mat_texture.py").is_file():
                raise FileNotFoundError(f"mat_texture.py missing from {texture_dir}")
            sys.path.append(str(texture_dir))
            from mat_texture import apply

            apply(stage, args.seed, args.out)
        carb.settings.get_settings().set("/omni/replicator/captureOnPlay", False)

        link, resolution, hfov, clipping = camera_spec()
        focal_length = 12.0
        aperture = 2 * focal_length * math.tan(hfov / 2)
        camera = UsdGeom.Camera.Define(stage, "/World/Camera_0")
        camera.CreateFocalLengthAttr(focal_length)
        camera.CreateHorizontalApertureAttr(aperture)
        camera.CreateVerticalApertureAttr(aperture * HEIGHT / WIDTH)
        camera.CreateClippingRangeAttr(clipping)
        self.camera = camera.MakeMatrixXform()

        product = rep.create.render_product(str(camera.GetPath()), resolution)
        cleanup.callback(product.destroy)
        self.rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        self.rgb.attach(product)
        cleanup.callback(self.rgb.detach, product)

        # Reuse d-isaac capture.py's USD row-vector transform convention.
        self.mount = Gf.Matrix4d().SetRotate(
            Gf.Rotation(Gf.Vec3d(1, 0, 0), math.degrees(link[3])) *
            Gf.Rotation(Gf.Vec3d(0, 1, 0), math.degrees(link[4])) *
            Gf.Rotation(Gf.Vec3d(0, 0, 1), math.degrees(link[5])))
        self.mount.SetTranslateOnly(Gf.Vec3d(*link[:3]))
        self.optical = Gf.Matrix4d(0, -1, 0, 0, 0, 0, 1, 0, -1, 0, 0, 0, 0, 0, 0, 1)
        self.gf, self.rep, self.np = Gf, rep, np

    def __call__(self, pose, tick):
        gf = self.gf
        rotation = (gf.Rotation(gf.Vec3d(0, 1, 0), -math.degrees(pose.pitch)) *
                    gf.Rotation(gf.Vec3d(0, 0, 1), math.degrees(pose.yaw)))
        robot = gf.Matrix4d().SetRotate(rotation)
        robot.SetTranslateOnly(gf.Vec3d(pose.x, pose.y, pose.z))
        self.camera.Set(self.optical * self.mount * robot)
        self.rep.orchestrator.step(rt_subframes=2, delta_time=DT)
        rgb = self.np.asarray(self.rgb.get_data())
        if rgb.shape not in {(HEIGHT, WIDTH, 3), (HEIGHT, WIDTH, 4)} or rgb.dtype != self.np.uint8:
            raise RuntimeError(f"Incomplete RGB annotator output at tick {tick}: {rgb.shape}, {rgb.dtype}")
        return rgb[..., :3].tobytes()


def video_sampler(args, cleanup):
    import cv2
    import numpy as np

    path = args.out / "camera.mp4"
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"),
                             HZ / args.video_every, (WIDTH, HEIGHT))
    cleanup.callback(writer.release)
    if not writer.isOpened():
        raise OSError(f"Cannot open MP4 writer for {path}")

    def save(tick, rgb):
        if tick % args.video_every == 0:
            image = np.frombuffer(rgb, np.uint8).reshape(HEIGHT, WIDTH, 3)
            writer.write(cv2.cvtColor(image, cv2.COLOR_RGB2BGR))

    return save


def write_result(args, result, score, reason, wall_seconds):
    import numpy as np

    trajectory = np.asarray(score.history if score else [], dtype=float).reshape(-1, 5)
    np.save(args.out / "trajectory.npy", trajectory)
    complete = bool(score and score.complete and reason == "finished")
    progress = score.progress if score else 0.0
    result.update(complete=complete, reason=reason, progress_m=round(progress, 3),
                  progress_pct=round(100 * progress / score.arc[-1], 2) if score else 0.0,
                  lap_sec=None, sim_sec=0.0, wall_sec=round(wall_seconds, 3),
                  lat_mean=None, lat_p95=None, lat_max=None)
    if len(trajectory):
        errors = trajectory[:, 4]
        result.update(sim_sec=round(float(trajectory[-1, 0]), 3),
                      lat_mean=round(float(errors.mean()), 4),
                      lat_p95=round(float(np.percentile(errors, 95)), 4),
                      lat_max=round(float(errors.max()), 4))
        if complete and score.moved_at is not None:
            result["lap_sec"] = round(float(trajectory[-1, 0]) - score.moved_at, 3)
    pending = args.out / "result.json.tmp"
    pending.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    pending.replace(args.out / "result.json")
    print("RESULT", json.dumps(result), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--bind", default="172.17.0.1")
    parser.add_argument("--port", type=int, default=18990)
    parser.add_argument("--max-sim-seconds", type=float, default=300)
    parser.add_argument("--replay-speed", type=float, help="follow the ordered centre line at V m/s")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--lights", action="store_true")
    parser.add_argument("--texture", action="store_true")
    parser.add_argument("--odom-position-std", type=float, default=0, help="metres per axis")
    parser.add_argument("--odom-yaw-std", type=float, default=0, help="radians")
    parser.add_argument("--video-every", type=int, default=0, help="save MP4 every N frames (0 disables)")
    args = parser.parse_args()
    if not 0 < args.port < 65536 or args.max_sim_seconds <= 0 or args.video_every < 0:
        parser.error("invalid port, sim time or video interval")
    if not all(math.isfinite(v) and v >= 0 for v in
               (args.max_sim_seconds, args.odom_position_std, args.odom_yaw_std)):
        parser.error("sim time and noise parameters must be finite and noise must be nonnegative")
    if args.replay_speed is not None and (
            not math.isfinite(args.replay_speed) or args.replay_speed <= 0):
        parser.error("--replay-speed must be finite and positive")
    args.out = args.out.resolve()
    args.out.mkdir(parents=True, exist_ok=True)
    # Prevent concurrent or repeated writes even when launching directly.
    try:
        (args.out / ".isaac-drive").touch(exist_ok=False)
    except FileExistsError:
        parser.error("use a fresh --out directory")
    if (args.out / "result.json").exists():
        parser.error("use a fresh --out directory")

    profile = os.environ.get("ISAAC_DRIVE_PROFILE", "external-relay")
    domain = os.environ.get("ROS_DOMAIN_ID")
    result = dict(name=args.out.name, map=args.map, backend="isaac_kinematic",
                  profile=profile, args=sys.argv[1:], domain=int(domain) if domain else None,
                  seed=args.seed, hz=HZ, lights=args.lights, texture=args.texture,
                  replay_speed=args.replay_speed)
    app = score = None
    started = time.monotonic()
    reason = "error: interrupted"

    def interrupted(_signal, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupted)
    try:
        track = load_map(args.map)
        score = Score(track.centerline, track.arc, args.max_sim_seconds)
        replay = (Replay(score, track.ramps, track.clearance, args.replay_speed)
                  if args.replay_speed is not None else None)
        with ExitStack() as cleanup:
            server = cleanup.enter_context(socket.socket(socket.AF_INET, socket.SOCK_STREAM))
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((args.bind, args.port))

            # SimulationApp forwards sys.argv to Kit. Keep harness options out of Kit.
            sys.argv[:] = sys.argv[:1]
            from isaacsim import SimulationApp
            app = SimulationApp({"headless": True, "renderer": "RaytracedLighting", "multi_gpu": False})
            from json_to_usd import convert

            usd = args.out / "track.usdc"
            convert(track.source, usd)
            renderer = Renderer(usd, args, cleanup)
            sample = video_sampler(args, cleanup) if args.video_every else None
            # First RTX frames compile shaders and can take minutes; do them before the relay connects
            # so its per-frame socket timeout only covers steady-state rendering.
            warm = time.monotonic()
            for _ in range(int(os.environ.get("ISAAC_WARMUP_FRAMES", "8"))):
                renderer.rep.orchestrator.step(rt_subframes=2, delta_time=DT)
            print(f"Renderer warm-up {time.monotonic() - warm:.1f}s", flush=True)
            server.listen(1)
            server.settimeout(180)
            print(f"Listening on {args.bind}:{args.port}", flush=True)
            peer, address = server.accept()
            cleanup.enter_context(peer)
            peer.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            peer.settimeout(30)
            print(f"Relay connected from {address}", flush=True)
            started = time.monotonic()
            reason = drive(peer, renderer, track.start, track.ramps, score,
                           clearance=track.clearance, sample=sample,
                           noise=random.Random(args.seed), position_std=args.odom_position_std,
                           yaw_std=args.odom_yaw_std, replay=replay,
                           relative_odom=replay is not None or profile == "map_dump")
    except (Exception, KeyboardInterrupt) as error:
        traceback.print_exc()
        reason = f"error: {error or type(error).__name__}"
    finally:
        try:
            write_result(args, result, score, reason, time.monotonic() - started)
        finally:
            if app is not None:
                app.close()
    return int(reason.startswith("error:"))


if __name__ == "__main__":
    sys.exit(main())
