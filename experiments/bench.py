#!/usr/bin/env python3
"""Evaluate follower profiles against ordered track metadata in isolated Gazebo runs."""

import argparse
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack, contextmanager
import csv
import fcntl
import json
import math
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import threading
from types import SimpleNamespace
import time
import traceback
from uuid import uuid4
import xml.etree.ElementTree as ET

import numpy as np

from telemetry import Telemetry

HERE = Path(__file__).resolve().parent
BRINGUP = HERE.parent / "src/line_follower_bringup"
OFF_SECONDS = 0.6
FINISH_TOLERANCE = 0.12
# Waffle Pi wheels: centres at y=±0.1435 (separation 0.287), width 0.018; line half-width 0.010.
WHEEL_OFFSET = 0.1435
WHEEL_ON_LINE = 0.010 + 0.009
MAX_JOBS = 16
BENCH_DOMAINS = range(30, 102)
LAB_DOMAIN_BASE = {"lf_a": 30, "lf_b": 50, "lf_c": 70, "line_follower_lab": 90}
COLUMNS = (
    "name", "map", "profile", "backend", "complete", "progress_m", "progress_pct", "lap_sec",
    "sim_sec", "wall_sec", "lat_p95", "lat_mean", "lat_max", "collision_count",
    "avoidance_attempts", "avoided_count", "slip_count", "odom_windows",
    "wheel_line_events", "wheel_line_sec", "max_progress_m", "reason",
    "domain", "partition",
)


class Track:
    def __init__(self, world):
        self.path = Path(world).resolve()
        sidecar = self.path.with_suffix(".json")
        if self.path == BRINGUP / "worlds/line_track.sdf":
            gt = np.load(HERE / "gt.npy")
            self.world, self.xy, self.arc, self.obstacles = "line_track", gt[:, :2], gt[:, 2], []
        else:
            metadata = json.loads(sidecar.read_text())
            self.world = metadata["world"]
            self.xy = np.array(metadata["centerline"], dtype=float)[:, :2]
            self.arc = np.array(metadata["arc_m"], dtype=float)
            self.obstacles = metadata["obstacles"]
        self.delta = np.diff(self.xy, axis=0)
        self.ds = np.diff(self.arc)
        self.norm2 = np.einsum("ij,ij->i", self.delta, self.delta)
        if (len(self.xy) < 2 or len(self.xy) != len(self.arc)
                or not np.isfinite(self.xy).all() or not np.isfinite(self.arc).all()
                or self.arc[0] != 0 or np.any(self.norm2 <= 0) or np.any(self.ds <= 0)):
            raise ValueError(f"{world}: invalid ordered centerline")
        self.blockers = {}
        for box in self.obstacles:
            if box["on_line"]:
                along, _ = self.project(box["pose"][:2])
                self.blockers[box["name"]] = (along, np.array(box["pose"][:2]))

    def project(self, xy, progress=None):
        if progress is None:
            lo, hi = 0, len(self.delta)
        else:
            lo = max(0, self.arc.searchsorted(progress - 0.35) - 1)
            hi = min(len(self.delta), self.arc.searchsorted(progress + 0.45))
        span = slice(lo, hi)
        delta = self.delta[span]
        relative = xy - self.xy[:-1][span]
        t = np.einsum("ij,ij->i", relative, delta) / self.norm2[span]
        if progress is None:
            t = np.clip(t, 0, 1)
        else:
            t = np.clip(t, np.maximum(0, (progress - 0.35 - self.arc[:-1][span]) / self.ds[span]),
                        np.minimum(1, (progress + 0.45 - self.arc[:-1][span]) / self.ds[span]))
        distance = np.linalg.norm(relative - t[:, None] * delta, axis=1)
        best = distance.argmin()
        return float(self.arc[lo + best] + t[best] * self.ds[lo + best]), float(distance[best])

    def evaluation_world(self, folder):
        if not self.obstacles:
            return self.path
        tree = ET.parse(self.path)
        world = tree.getroot().find("world")
        ET.SubElement(world, "plugin", filename="gz-sim-contact-system", name="gz::sim::systems::Contact")
        for uri in world.iter("uri"):
            if uri.text.startswith("./"):
                uri.text = str(self.path.parent / uri.text)
        for box in self.obstacles:
            link = world.find(f'./model[@name="{box["name"]}"]/link')
            sensor = ET.SubElement(link, "sensor", name="bench_contact", type="contact")
            ET.SubElement(sensor, "always_on").text = "true"
            ET.SubElement(ET.SubElement(sensor, "contact"), "collision").text = "box_collision"
        output = folder / "eval.sdf"
        tree.write(output, encoding="utf-8", xml_declaration=True)
        return output


def contact_topic(world, name):
    return f"/world/{world}/model/{name}/link/body/sensor/bench_contact/contact"


def stamp(header_stamp):
    return header_stamp.sec + header_stamp.nsec * 1e-9


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def angle_distance(a, b):
    return abs(math.remainder(a - b, math.tau))


class SlipMonitor:
    """Compare wheel odometry and true displacement over matched ~250 ms windows."""

    def __init__(self):
        self.previous = None
        self.events = []
        self.active = None
        self.windows = 0

    def observe(self, truth, odom):
        if odom is None or abs(odom[0] - truth[0]) > 0.055:
            return
        if self.previous is None:
            self.previous = truth, odom
            return
        before, wheel_before = self.previous
        elapsed = truth[0] - before[0]
        if elapsed < 0.25:
            return
        self.previous = truth, odom
        wheel_elapsed = odom[0] - wheel_before[0]
        if elapsed > 0.45 or abs(wheel_elapsed - elapsed) > 0.06:
            self.active = None
            return
        self.windows += 1
        linear = abs(math.dist(truth[1:4], before[1:4]) / elapsed
                     - math.dist(odom[1:4], wheel_before[1:4]) / wheel_elapsed)
        angular = abs(angle_distance(truth[4], before[4]) / elapsed
                      - angle_distance(odom[4], wheel_before[4]) / wheel_elapsed)
        if linear < 0.10 and angular < 0.65:
            self.active = None
            return
        if self.active is None:
            self.active = {"start_sim_sec": round(before[0], 3), "windows": 0,
                           "peak_linear_mps": 0.0, "peak_angular_radps": 0.0}
            self.events.append(self.active)
        self.active["end_sim_sec"] = round(truth[0], 3)
        self.active["windows"] += 1
        self.active["peak_linear_mps"] = round(max(self.active["peak_linear_mps"], linear), 3)
        self.active["peak_angular_radps"] = round(max(self.active["peak_angular_radps"], angular), 3)


class Score:
    def __init__(self, track, options):
        self.track, self.options = track, options
        self.progress = self.last_progress = 0.0
        self.off_since = self.moved_at = None
        self.avoidance = {}
        self.history = []
        self.complete = False
        self.wheel_events, self.wheel_sec = 0, 0.0
        self._wheel_on, self._wheel_t = False, None

    def wheels(self, now, x, y, heading):
        """Count a wheel touching the centreline (rising edges) and the time spent touching."""
        normal = np.array([-np.sin(heading), np.cos(heading)])
        centre = np.array([x, y])
        on = any(self.track.project(centre + side * WHEEL_OFFSET * normal)[1] < WHEEL_ON_LINE
                 for side in (1.0, -1.0))
        if on and self._wheel_on and self._wheel_t is not None:
            self.wheel_sec += now - self._wheel_t
        if on and not self._wheel_on:
            self.wheel_events += 1
        self._wheel_on, self._wheel_t = on, now

    def observe(self, now, xy):
        candidate, error = self.track.project(xy, self.progress)
        if self.moved_at is None and candidate > 0.02:
            self.moved_at = now
        self.history.append((now, *xy, candidate, error))
        bypass = False
        for name, (along, box_xy) in self.track.blockers.items():
            if (along - 0.60 <= candidate <= along + 0.46
                    and np.linalg.norm(xy - box_xy) <= 0.65
                    and self.options["off"] < error <= 0.38):
                self.avoidance.setdefault(name, False)
                bypass = True
            if name in self.avoidance and candidate >= along + 0.22 and error <= self.options["off"]:
                self.avoidance[name] = True
        if error > self.options["off"] and not bypass:
            if self.off_since is None:
                self.off_since = now
            if now - self.off_since > OFF_SECONDS:
                return f"off line at {candidate:.2f} m"
        else:
            self.off_since = None
            if candidate > self.progress + 0.003:
                self.progress, self.last_progress = candidate, now
        if self.progress >= self.track.arc[-1] - FINISH_TOLERANCE:
            self.complete = True
            return "finished"
        if now - self.last_progress > self.options["stall"]:
            return f"stalled at {self.progress:.2f} m"
        if now >= self.options["limit"]:
            return "time limit"
        return None


@contextmanager
def process(command, log):
    proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        yield proc
    finally:
        for signum, timeout in ((signal.SIGINT, 3), (signal.SIGTERM, 2), (signal.SIGKILL, 2)):
            try:
                os.killpg(proc.pid, signum)
            except ProcessLookupError:
                break
            try:
                proc.wait(timeout)
            except subprocess.TimeoutExpired:
                continue
        proc.wait()


def read_pose(position, orientation, time_sec):
    return time_sec, position.x, position.y, position.z, yaw(orientation)


def atomic_json(path, value):
    temp = path.with_suffix(".tmp")
    temp.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    temp.replace(path)


def run_legacy(job, folder, track, sim, probe, truth, odom, collisions, rclpy, cmd=(None,)):
    """Run the original scorer with this worker's world and output directory."""
    import run_eval

    np.save(folder / "gt.npy", np.column_stack((track.xy, track.arc)))
    name = "legacy"
    out = folder / "results"
    score, slip = Score(track, job), SlipMonitor()
    tele = Telemetry(track, WHEEL_OFFSET, WHEEL_ON_LINE)
    stopping = threading.Event()
    faults = []
    follower = None
    monitor = None

    def collect():
        cursor = origin = truth[-1][0]
        try:
            while not stopping.is_set():
                rclpy.spin_once(probe, timeout_sec=0.05)
                current = truth[-1]
                if current[0] - cursor < 0.049:
                    continue
                cursor = current[0]
                wheel = min(odom, key=lambda sample: abs(sample[0] - cursor), default=None)
                slip.observe(current, wheel)
                # Monitor detours observed during the run; run_eval owns termination.
                score.observe(cursor - origin, current[1:3])
                score.wheels(cursor - origin, current[1], current[2], current[4])
                tele.add(cursor - origin, current[1], current[2], current[4], cmd[0])
        except Exception as error:
            faults.append(error)

    def spawn(command, **kwargs):
        nonlocal follower, monitor
        if command[:3] == ["ros2", "launch", "line_follower_bringup"]:
            return sim
        if command[:2] == [sys.executable, str(folder / "follower.py")]:
            flags = command[2:] if Path(job["follower"]) == HERE / "follower.py" else job["args"]
            follower = subprocess.Popen([sys.executable, job["follower"], *flags], **kwargs)
            monitor = threading.Thread(target=collect)
            monitor.start()
            return follower
        raise RuntimeError(f"unsupported run_eval subprocess: {command!r}")

    # This module instance is isolated in the per-job subprocess. The source
    # and Python's process-global subprocess module are untouched.
    saved = run_eval.HERE, run_eval.subprocess, sys.argv
    run_eval.HERE = folder
    run_eval.subprocess = SimpleNamespace(Popen=spawn, STDOUT=subprocess.STDOUT,
                                          TimeoutExpired=subprocess.TimeoutExpired)
    sys.argv = [
        str(HERE / "run_eval.py"), "--name", name, "--limit", str(job["limit"]),
        "--stall", str(job["stall"]), "--off", str(job["off"]),
        "--", *job["args"],
    ]
    try:
        run_eval.main()
    finally:
        run_eval.HERE, run_eval.subprocess, sys.argv = saved
        stopping.set()
        if monitor:
            monitor.join()
    if faults:
        raise RuntimeError(f"telemetry collection failed: {faults[0]}")
    result = json.loads((out / f"{name}.json").read_text())
    if follower is not None and follower.returncode not in (0, -signal.SIGINT, -signal.SIGTERM,
                                                           -signal.SIGKILL, 130):
        raise RuntimeError(f"follower exited with status {follower.returncode}; see results/legacy.log")
    trajectory = out / f"{name}_traj.npy"
    if trajectory.is_file():
        (folder / "trajectory.npy").symlink_to(trajectory.relative_to(folder))
    contacts = collisions.copy()
    result.pop("name")
    result.update(
        collision_count=len(contacts), collided=sorted(contacts),
        avoidance_attempts=len(score.avoidance),
        avoided_count=sum(passed and key not in contacts for key, passed in score.avoidance.items()),
        slip_count=len(slip.events), slip_events=slip.events, odom_windows=slip.windows,
        sim_sec=round(score.history[-1][0], 3) if score.history else None,
        wheel_line_events=score.wheel_events, wheel_line_sec=round(score.wheel_sec, 3),
    )
    summary = tele.save(folder, dict(result, profile=job["profile"], map=job["map"]))
    if summary:
        result["max_progress_m"] = summary["max_progress_m"]
    return result


def evaluate(job_path):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    from gz.transport13 import Node
    from gz.msgs10.contacts_pb2 import Contacts
    from gz.msgs10.pose_v_pb2 import Pose_V

    job = json.loads(job_path.read_text())
    folder = job_path.parent
    result = {key: job[key] for key in ("name", "map", "profile", "args")}
    result.update(complete=False, domain=int(os.environ["ROS_DOMAIN_ID"]),
                  partition=os.environ["GZ_PARTITION"], backend=job.get("backend", "obstacle"))
    resources = str(BRINGUP / "models")
    if os.environ.get("GZ_SIM_RESOURCE_PATH"):
        resources += os.pathsep + os.environ["GZ_SIM_RESOURCE_PATH"]
    os.environ["GZ_SIM_RESOURCE_PATH"] = resources
    os.environ["ROS_LOG_DIR"] = str(folder / "ros")
    truth, odom = deque(maxlen=150), deque(maxlen=150)
    collisions = set()
    cmd = [None]

    def receive_cmd(message):
        cmd[0] = (message.linear.x, message.angular.z)

    def receive(message):
        for pose in message.pose:
            if pose.name == "turtlebot3_waffle_pi":
                truth.append(read_pose(pose.position, pose.orientation, stamp(message.header.stamp)))
                break

    def receive_odom(message):
        pose = message.pose.pose
        header_stamp = message.header.stamp
        time_sec = header_stamp.sec + header_stamp.nanosec * 1e-9
        odom.append(read_pose(pose.position, pose.orientation, time_sec))

    def listen_for_contact(message, name):
        for event in message.contact:
            entities = event.collision1.name, event.collision2.name
            if any("turtlebot3_waffle_pi" in entity for entity in entities):
                collisions.add(name)

    def interrupted(_number, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupted)
    try:
        track = Track(job["map"])
        world = track.evaluation_world(folder)
        gz = Node()
        if not gz.subscribe(Pose_V, f"/world/{track.world}/dynamic_pose/info", receive):
            raise RuntimeError("cannot subscribe to Gazebo ground truth")
        for box in track.obstacles:
            name = box["name"]
            if not gz.subscribe(Contacts, contact_topic(track.world, name),
                                lambda msg, name=name: listen_for_contact(msg, name)):
                raise RuntimeError(f"cannot subscribe to {name} contacts")
        rclpy.init(args=[], signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
        try:
            probe = rclpy.create_node("bench_probe")
            probe.create_subscription(Odometry, "/odom", receive_odom, qos_profile_sensor_data)
            probe.create_subscription(Twist, "/cmd_vel", receive_cmd, 10)
            with (folder / "run.log").open("w") as log, ExitStack() as stack:
                sim = stack.enter_context(process(["gz", "sim", "-s", "-r", "--headless-rendering",
                                                   str(world)], log))
                bridge = stack.enter_context(process(
                    ["ros2", "run", "ros_gz_bridge", "parameter_bridge", "--ros-args", "-p",
                     f"config_file:={BRINGUP / 'config/ros_gz_bridge.yaml'}"], log))
                camera = stack.enter_context(process(
                    ["ros2", "run", "ros_gz_image", "image_bridge", "/camera/image_raw"], log))
                deadline = time.monotonic() + job["startup"]
                expected_contacts = {contact_topic(track.world, box["name"]) for box in track.obstacles}
                while True:
                    rclpy.spin_once(probe, timeout_sec=0.05)
                    camera_ready = probe.count_publishers("/camera/image_raw") > 0
                    missing = expected_contacts.difference(gz.topic_list())
                    if truth and odom and camera_ready and not missing:
                        break
                    if time.monotonic() >= deadline or any(p.poll() is not None for p in (sim, bridge, camera)):
                        raise RuntimeError(f"startup failed: pose={bool(truth)}, odom={bool(odom)}, "
                                           f"camera={camera_ready}, missing contact topics={sorted(missing)}")
                if result["backend"] == "run_eval":
                    result.update(run_legacy(job, folder, track, sim, probe, truth, odom, collisions, rclpy, cmd))
                else:
                    score, slip = Score(track, job), SlipMonitor()
                    tele = Telemetry(track, WHEEL_OFFSET, WHEEL_ON_LINE)
                    origin = cursor = truth[-1][0]
                    start = last_update = time.monotonic()
                    follower = stack.enter_context(process([sys.executable, job["follower"], *job["args"]], log))
                    while True:
                        rclpy.spin_once(probe, timeout_sec=0.05)
                        now_wall = time.monotonic()
                        if now_wall - last_update > 25 or now_wall - start > max(60, job["limit"] * 8):
                            raise RuntimeError("Gazebo pose stalled or exceeded the wall-clock cap")
                        if sim.poll() is not None or follower.poll() is not None:
                            raise RuntimeError("Gazebo or follower exited before scoring finished")
                        current = truth[-1]
                        if current[0] - cursor < 0.049:
                            continue
                        cursor = current[0]
                        last_update = now_wall
                        wheel = min(tuple(odom), key=lambda sample: abs(sample[0] - cursor), default=None)
                        slip.observe(current, wheel)
                        reason = score.observe(cursor - origin, current[1:3])
                        score.wheels(cursor - origin, current[1], current[2], current[4])
                        tele.add(cursor - origin, current[1], current[2], current[4], cmd[0])
                        if reason:
                            break
                    # Keep the contact subscriber alive while Gazebo flushes notifications for the last tick.
                    time.sleep(0.12)
                    contacts = collisions.copy()
                    trajectory = np.array(score.history)
                    np.save(folder / "trajectory.npy", trajectory)
                    errors = trajectory[:, 4]
                    result.update(
                        complete=score.complete, reason=reason, progress_m=round(score.progress, 3),
                        progress_pct=round(100 * score.progress / track.arc[-1], 2),
                        lap_sec=round(trajectory[-1, 0] - score.moved_at, 3) if score.complete else None,
                        sim_sec=round(trajectory[-1, 0], 3), wall_sec=round(last_update - start, 3),
                        lat_p95=round(float(np.percentile(errors, 95)), 4),
                        lat_mean=round(float(errors.mean()), 4), lat_max=round(float(errors.max()), 4),
                        collision_count=len(contacts), collided=sorted(contacts),
                        avoidance_attempts=len(score.avoidance),
                        avoided_count=sum(passed and name not in contacts
                                          for name, passed in score.avoidance.items()),
                        slip_count=len(slip.events), slip_events=slip.events, odom_windows=slip.windows,
                        wheel_line_events=score.wheel_events, wheel_line_sec=round(score.wheel_sec, 3),
                    )
                    summary = tele.save(folder, dict(result, profile=job["profile"], map=job["map"]))
                    if summary:
                        result["max_progress_m"] = summary["max_progress_m"]
        finally:
            rclpy.shutdown()
    except (Exception, KeyboardInterrupt) as error:
        with (folder / "run.log").open("a") as log:
            traceback.print_exc(file=log)
        result.update(reason=f"error: {error or type(error).__name__}")
    atomic_json(folder / "result.json", result)
    return int(result["reason"].startswith("error:"))


def job_contexts(count):
    """Assign each run its own domain and a partition namespaced to the matrix."""
    coordinator = int(os.environ.get("ROS_DOMAIN_ID") or "13")
    prefix = os.environ.get("GZ_PARTITION") or os.environ.get("LAB_NAME") or "lf_c"
    # BENCH_DOMAIN_BASE lets several bench processes share one container without
    # colliding (e.g. 30, 50, 70 ...). Each run still gets its own domain.
    # Containers share the Docker bridge, so DDS discovery crosses containers:
    # every lab gets its own block of 20 domains (lf_a 30-49, lf_b 50-69, lf_c/lead 70-89).
    lab = os.environ.get("LAB_NAME", "")
    base = int(os.environ.get("BENCH_DOMAIN_BASE") or LAB_DOMAIN_BASE.get(lab, BENCH_DOMAINS.start))
    stop = min(base + 20, BENCH_DOMAINS.stop) if not os.environ.get("BENCH_DOMAIN_BASE") or lab else BENCH_DOMAINS.stop
    domains = [domain for domain in range(base, stop) if domain != coordinator]
    if not 1 <= count <= len(domains):
        raise ValueError(f"matrix must have 1–{len(domains)} runs for distinct ROS domains")
    batch = uuid4().hex[:12]
    return [(domain, f"{prefix}_{batch}_{index:03d}")
            for index, domain in enumerate(domains[:count], 1)]


def run_matrix(maps, profiles, output, jobs=1, limit=300, stall=20, off=0.08,
               startup=60, follower=HERE / "follower.py", backend="run_eval"):
    if not 1 <= jobs <= MAX_JOBS or min(limit, stall, off, startup) <= 0:
        raise ValueError(f"jobs must be in 1–{MAX_JOBS} and timing/offset limits must be positive")
    if backend not in ("run_eval", "obstacle"):
        raise ValueError(f"unknown scoring backend {backend!r}")
    maps = [Path(p).resolve(strict=True) for p in maps]
    for path in maps:
        track = Track(path)
        if backend == "run_eval" and track.world != "line_track":
            raise ValueError(f"{path}: run_eval requires the world name line_track; use --backend obstacle")
    contexts = iter(job_contexts(len(maps) * len(profiles)))
    follower = Path(follower).resolve(strict=True)
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    inputs = []
    for i, path in enumerate(maps, 1):
        for j, (profile, args) in enumerate(profiles.items(), 1):
            name = f"{i:02d}_{j:02d}"
            domain, partition = next(contexts)
            folder = output / name
            folder.mkdir(exist_ok=False)
            job_path = folder / "job.json"
            atomic_json(job_path, dict(
                name=name, map=str(path), profile=profile, args=args, limit=limit,
                stall=stall, off=off, startup=startup, follower=str(follower), backend=backend,
                domain=domain, partition=partition,
            ))
            inputs.append((job_path, domain, partition))
    stopping = threading.Event()

    def run_job(job_path, domain, partition):
        # Concurrent invocations in the same container must never reuse a domain.
        with Path(f"/tmp/line-follower-bench-{domain}.lock").open("a") as lock:
            while not stopping.is_set():
                try:
                    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    break
                except BlockingIOError:
                    stopping.wait(0.25)
            env = dict(os.environ, ROS_DOMAIN_ID=str(domain), GZ_PARTITION=partition,
                       LAB_NAME=os.environ.get("LAB_NAME", "lf_c"), ROS2CLI_NO_DAEMON="1")
            if stopping.is_set():
                return
            with (job_path.parent / "worker.log").open("w") as log:
                proc = subprocess.Popen([sys.executable, __file__, "--run-job", str(job_path)],
                                        stdout=log, stderr=subprocess.STDOUT, env=env,
                                        start_new_session=True, pass_fds=(lock.fileno(),))
                while proc.poll() is None and not stopping.wait(0.25):
                    pass
                if proc.poll() is None:
                    proc.terminate()
                proc.wait()

    with ThreadPoolExecutor(max_workers=min(jobs, len(inputs))) as pool:
        futures = [pool.submit(run_job, *job) for job in inputs]
        try:
            for future in futures:
                future.result()
        finally:
            stopping.set()
    results = []
    for job_path, domain, partition in inputs:
        result_path = job_path.with_name("result.json")
        if result_path.is_file():
            results.append(json.loads(result_path.read_text()))
        else:
            job = json.loads(job_path.read_text())
            results.append(dict(name=job["name"], map=job["map"], profile=job["profile"],
                                backend=job["backend"], domain=domain, partition=partition,
                                complete=False, reason="error: worker exited, see worker.log"))
    with (output / "results.csv").open("w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=COLUMNS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(results)
    return results


def print_table(results):
    print(f'{"map":25} {"profile":14} {"done":5} {"prog %":>7} {"lap s":>7} {"p95 m":>7} '
          f'{"hits":>4} {"avoid":>5} {"slips":>5}  reason')
    for result in results:
        print(f'{Path(result["map"]).stem:25} {result["profile"]:14} '
              f'{"yes" if result["complete"] else "no":5} '
              + " ".join(f"{str(result[key]) if result.get(key) is not None else '-':>{width}}"
                         for key, width in zip(
                             ("progress_pct", "lap_sec", "lat_p95", "collision_count",
                              "avoided_count", "slip_count"), (7, 7, 7, 4, 5, 5)))
              + f'  {result["reason"]}')


def main():
    if len(sys.argv) == 3 and sys.argv[1] == "--run-job":
        return evaluate(Path(sys.argv[2]))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("maps", nargs="+", type=Path)
    parser.add_argument("--profile", action="append", default=[], metavar="NAME=ARGS",
                        help="repeat to compare profiles, e.g. 'fast=--vmax 0.3'")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--jobs", type=int, default=1)
    parser.add_argument("--limit", type=float, default=300)
    parser.add_argument("--stall", type=float, default=20)
    parser.add_argument("--off", type=float, default=0.08)
    parser.add_argument("--startup", type=float, default=60)
    parser.add_argument("--follower", type=Path, default=HERE / "follower.py")
    parser.add_argument("--backend", choices=("run_eval", "obstacle"), default="run_eval",
                        help="score with run_eval.py (default) or allow bounded obstacle detours")
    options = parser.parse_args()
    profiles = {}
    try:
        for spec in options.profile or ["default="]:
            name, args = spec.split("=", 1)
            if not name or name in profiles:
                raise ValueError(f"empty or duplicate profile {name!r}")
            profiles[name] = shlex.split(args)
        results = run_matrix(options.maps, profiles, options.output, options.jobs, options.limit,
                             options.stall, options.off, options.startup, options.follower, options.backend)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    print_table(results)
    return int(any(result["reason"].startswith("error:") for result in results))


if __name__ == "__main__":
    sys.exit(main())
