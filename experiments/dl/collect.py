"""Collect Gazebo camera images and geometric class masks. Development only."""

import argparse
from collections import deque
from contextlib import contextmanager
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
from xml.etree import ElementTree as ET

import cv2
import numpy as np

from .geometry import HEIGHT, WIDTH, box, quaternion_matrix, read_stl, render_mask

EXPERIMENTS = Path(__file__).resolve().parents[1]
BRINGUP = EXPERIMENTS.parent / "src/line_follower_bringup"
MODEL = "turtlebot3_waffle_pi"
ENV = {"LAB_NAME": "lf_b", "ROS_DOMAIN_ID": "12", "GZ_PARTITION": "lf_b"}


def terrain_geometry(track):
    terrain = []
    for visual in track.findall("./link[@name='surface']/visual"):
        pose = np.fromstring(visual.findtext("pose"), sep=" ")
        size = np.fromstring(visual.findtext("geometry/box/size"), sep=" ")
        terrain.append((box(pose[:3], size, pose[3:]), 0))
    return terrain


def official_geometry():
    path = BRINGUP / "models/track"
    terrain = terrain_geometry(ET.parse(path / "model.sdf").getroot().find("model"))
    line = read_stl(path / "meshes/track.stl")
    centerline = np.load(EXPERIMENTS / "gt.npy")[:, :2]
    # The ordered official GT array stores arc length in its third column.
    segments = np.load(EXPERIMENTS / "segments.npy").reshape(-1, 3)
    nearest = np.linalg.norm(centerline[:, None, :] - segments[None, :, :2], axis=2).argmin(1)
    path_xyz = np.column_stack((centerline, segments[nearest, 2] - 0.001))
    return terrain + [(line, 1)], path_xyz


def random_geometry(world):
    metadata = json.loads(world.with_suffix(".json").read_text())
    if metadata["version"] != 1 or metadata["sdf"] != world.name:
        raise ValueError("Expected the Codex-3 version 1 SDF/JSON/STL bundle")
    scene = ET.parse(world).getroot().find("world")
    track = scene.find("./model[@name='track']")
    terrain = terrain_geometry(track)
    line_uri = track.findtext("./link[@name='line']/visual/geometry/mesh/uri")
    line = read_stl(world.parent / line_uri)
    obstacles = metadata["obstacles"]
    boxes = [(box(item["pose"][:3], item["size"], item["pose"][3:]), 2) for item in obstacles]
    return terrain + [(line, 1)] + boxes, np.array(metadata["centerline"]), metadata


def sampled_poses(path, count, rng, obstacles=()):
    delta = np.diff(path, axis=0)
    length = np.linalg.norm(delta[:, :2], axis=1)
    valid = length > 1e-6
    start, delta, length = path[:-1][valid], delta[valid], length[valid]
    cumulative = np.r_[0, np.cumsum(length)]
    distance = (np.arange(count) + rng.random(count)) * cumulative[-1] / count
    centers = np.array([item["pose"][:2] for item in obstacles]).reshape(-1, 2)
    radii = np.array([np.linalg.norm(item["size"][:2]) / 2 + 0.20 for item in obstacles])
    if len(centers):
        fractions = (((centers[:, None] - start[None, :, :2]) * delta[None, :, :2]).sum(2)
                     / length[None] ** 2).clip(0, 1)
        projections = start[None, :, :2] + fractions[:, :, None] * delta[None, :, :2]
        nearest = np.linalg.norm(projections - centers[:, None], axis=2).argmin(1)
        arc = cumulative[nearest] + length[nearest] * fractions[np.arange(len(centers)), nearest]
        biased = count // 3
        distance[:biased] = np.clip(rng.choice(arc, biased) - rng.uniform(0.45, 1.2, biased),
                                    0, np.nextafter(cumulative[-1], 0))
    rng.shuffle(distance)
    for pos in distance:
        for attempt in range(100):
            sample = pose_at_arc(start, delta, length, cumulative, pos, rng)
            if np.all(np.linalg.norm(centers - sample[0][:2], axis=1) > radii):
                yield sample
                break
            pos = rng.uniform(0, cumulative[-1])
        else:
            raise ValueError("Could not find a collision-free sample")


def pose_at_arc(start, delta, length, cumulative, pos, rng):
    index = np.searchsorted(cumulative, pos, side="right") - 1
    dx, dy, dz = delta[index]
    xyz = start[index] + delta[index] * (pos - cumulative[index]) / length[index]
    heading = math.atan2(dy, dx)
    offset = rng.uniform(-0.1, 0.1)
    angle = rng.uniform(-40, 40)
    xyz[:2] += offset * np.array([-math.sin(heading), math.cos(heading)])
    xyz[2] += 0.03
    yaw = heading + math.radians(angle)
    pitch = -math.atan2(dz, length[index]) * math.cos(yaw - heading)
    return xyz, yaw, pitch, offset, angle


def msg_stamp(message):
    return message.header.stamp.sec + message.header.stamp.nsec / 1e9


class Capture:
    def __init__(self, world):
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.image_pb2 import Image
        from gz.msgs10.pose_pb2 import Pose
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node

        self.world_prefix = f"/world/{world}"
        self.Pose = Pose
        self.Boolean = Boolean
        self.node = Node()
        self.frames = deque(maxlen=8)
        self.poses = deque(maxlen=80)
        self.ready = threading.Condition()
        if not self.node.subscribe(Image, "/camera/image_raw", self.on_frame):
            raise RuntimeError("Gazebo image subscription failed")
        if not self.node.subscribe(Pose_V, self.world_prefix + "/dynamic_pose/info", self.on_pose):
            raise RuntimeError("Gazebo pose subscription failed")

    def on_frame(self, image):
        with self.ready:
            self.frames.append((msg_stamp(image), image))
            self.ready.notify()

    def on_pose(self, poses):
        for pose in poses.pose:
            if pose.name == MODEL:
                with self.ready:
                    self.poses.append((msg_stamp(poses), pose))
                    self.ready.notify()
                break

    def synchronized(self, after, timeout, requested=None):
        deadline = time.monotonic() + timeout
        with self.ready:
            while (remaining := deadline - time.monotonic()) > 0:
                if self.frames and self.poses:
                    stamp, image = self.frames[-1]
                    if stamp >= after:
                        pose_stamp, pose = min(self.poses, key=lambda p: abs(p[0] - stamp))
                        xyz = np.array([pose.position.x, pose.position.y, pose.position.z])
                        if abs(pose_stamp - stamp) <= 0.045 and (
                            requested is None or np.linalg.norm(xyz[:2] - requested[:2]) < 0.04
                        ):
                            return stamp, image, pose
                self.ready.wait(remaining)
        raise TimeoutError(f"No synchronized Gazebo camera/pose after sim time {after:.3f}")

    def teleport(self, xyz, yaw, pitch, settle):
        pose = self.Pose(name=MODEL)
        pose.position.x, pose.position.y, pose.position.z = xyz
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        pose.orientation.x, pose.orientation.y = -sy * sp, cy * sp
        pose.orientation.z, pose.orientation.w = sy * cp, cy * cp
        with self.ready:
            before = self.poses[-1][0]
        success, reply = self.node.request(
            self.world_prefix + "/set_pose", pose, self.Pose, self.Boolean, 3000,
        )
        if not success or not reply.data:
            raise RuntimeError(f"Gazebo refused set_pose for {MODEL}")
        return self.synchronized(before + settle, 10, xyz)

    def close(self):
        self.node.unsubscribe("/camera/image_raw")
        self.node.unsubscribe(self.world_prefix + "/dynamic_pose/info")


def to_bgr(image):
    if (image.width, image.height) != (WIDTH, HEIGHT):
        raise ValueError(f"Unexpected camera resolution {image.width}x{image.height}")
    pixels = np.frombuffer(image.data, dtype=np.uint8).reshape(HEIGHT, image.step)
    pixels = pixels[:, :WIDTH * 3].reshape(HEIGHT, WIDTH, 3)
    if image.pixel_format_type == 3:  # RGB_INT8
        return cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
    if image.pixel_format_type == 8:  # BGR_INT8
        return pixels
    raise ValueError(f"Unsupported pixel format: {image.pixel_format_type}")


@contextmanager
def simulation(world, logfile):
    env = dict(os.environ, **ENV)
    resources = [str(BRINGUP / "models"), str(world.parent)]
    if env.get("GZ_SIM_RESOURCE_PATH"):
        resources.append(env["GZ_SIM_RESOURCE_PATH"])
    env["GZ_SIM_RESOURCE_PATH"] = os.pathsep.join(resources)
    with logfile.open("w") as output:
        sim = subprocess.Popen(
            ["gz", "sim", "-s", "-r", "--headless-rendering", str(world)],
            env=env, stdout=output, stderr=subprocess.STDOUT, start_new_session=True,
        )
        try:
            yield
        finally:
            try:
                os.killpg(sim.pid, signal.SIGINT)
                sim.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(sim.pid, signal.SIGKILL)
                sim.wait()
            except ProcessLookupError:
                sim.wait()


def collect(args, world, world_name, meshes, centerline, metadata):
    if any(os.environ.get(key, default) != default for key, default in ENV.items()):
        raise ValueError(f"Unexpected lab isolation values. Use {ENV}")
    os.environ.update(ENV)
    output = args.out
    output.mkdir(parents=True, exist_ok=False)
    for child in ("image", "mask"):
        (output / child).mkdir()
    cv2.setNumThreads(1)
    rng = np.random.default_rng(args.seed)
    totals = np.zeros(3, dtype=np.int64)
    with simulation(world, output / "gazebo.log"):
        capture = Capture(world_name)
        try:
            capture.synchronized(0, 60)
            with (output / "frames.jsonl").open("w") as manifest:
                for index, sample in enumerate(
                    sampled_poses(centerline, args.count, rng, metadata.get("obstacles", ()))
                ):
                    xyz, yaw, pitch, offset, angle = sample
                    stamp, message, pose = capture.teleport(xyz, yaw, pitch, args.settle)
                    position = np.array([pose.position.x, pose.position.y, pose.position.z])
                    q = pose.orientation
                    rotation = quaternion_matrix(q.x, q.y, q.z, q.w)
                    mask = render_mask(meshes, position, rotation)
                    image = to_bgr(message)
                    filename = f"{index:06d}.png"
                    for directory, array in (("image", image), ("mask", mask)):
                        if not cv2.imwrite(str(output / directory / filename), array):
                            raise OSError(f"Write failed for {directory}/{filename}")
                    counts = np.bincount(mask.ravel(), minlength=3)
                    totals += counts
                    manifest.write(json.dumps({
                        "file": filename, "sim_stamp": stamp, "position": position.tolist(),
                        "quaternion_xyzw": [q.x, q.y, q.z, q.w],
                        "offset": offset, "yaw_offset_deg": angle, "class_pixels": counts.tolist(),
                    }) + "\n")
                    if (index + 1) % 100 == 0:
                        manifest.flush()
                        print(f"{output.name}: {index + 1}/{args.count}", flush=True)
        finally:
            capture.close()
    summary = {
        "map": world.stem, "world": str(world), "capture_seed": args.seed,
        "map_seed": metadata.get("seed"), "difficulty": metadata.get("difficulty"),
        "frames": args.count, "class_pixels": totals.tolist(),
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", type=Path, help="Codex-3 generated SDF with adjacent JSON/STL")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--count", type=int, required=True)
    parser.add_argument("--seed", type=int, default=11)
    parser.add_argument("--settle", type=float, default=0.30, help="Seconds of simulation time")
    args = parser.parse_args()
    if args.count < 1 or args.settle < 0.1:
        parser.error("count must be positive and settle must be at least 0.1 seconds")
    if args.world is None:
        world = BRINGUP / "worlds/line_track.sdf"
        meshes, centerline = official_geometry()
        metadata = {"world": "line_track"}
    else:
        world = args.world.resolve()
        meshes, centerline, metadata = random_geometry(world)
    collect(args, world, metadata["world"], meshes, centerline, metadata)


if __name__ == "__main__":
    main()
