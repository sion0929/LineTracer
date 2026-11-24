#!/usr/bin/env python3
"""Safety fault injection for the line_follower package (rule mode) in headless Gazebo.

Every scenario starts its own Gazebo server, ROS bridges and the unchanged follower
(experiments/user_base/run_user.py with LF_PARAMS=src/line_follower/config/rule.yaml),
injects one fault (or drives a crafted map) and measures what the robot really does from
the Gazebo ground-truth pose (/world/<world>/dynamic_pose/info, dev tool only).

Run inside the lab container from /ws (paths below are relative to the workspace):

  python3 experiments/eval/safety_faults.py list
  python3 experiments/eval/safety_faults.py run sigkill camera_loss --repeat 2 --jobs 2
  python3 experiments/eval/safety_faults.py run all --repeat 2 --jobs 2
  python3 experiments/eval/safety_faults.py run obstacles --repeat 2 --jobs 2   # G2 boxes via bench.py
  python3 experiments/eval/safety_faults.py summarize

Guarded stack (follower -> /line_follower/cmd_raw -> safety_guard -> /cmd_vel, started by
experiments/user_base/run_guarded.py): add --stack guarded. Follower faults (sigint, sigterm,
sigkill) then hit only the follower process and the guard must stop the robot; guard_* faults
hit only the guard, stack_* the run_guarded.py wrapper (forwards to both, like Ctrl-C).

  python3 experiments/eval/safety_faults.py run sigint camera_loss --stack guarded --gz-prefix lf_g \
      --install install_g --repeat 2 --jobs 2
  python3 experiments/eval/safety_faults.py summarize --stack guarded
  python3 experiments/eval/safety_faults.py compare          # raw vs guarded markdown table

Isolation: each run gets its own ROS_DOMAIN_ID from --domains (default 84-89, checked to be
empty first) and its own GZ_PARTITION (<gz-prefix>_<scenario>_<id>, default prefix lf_s).
Results go to experiments/results/safety/runs/<scenario>/<stamp>_r<k>/{result.json,trace.npz,*.log}
(guarded: experiments/results/safety/guarded/runs/...).
"""

import argparse
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack
import fcntl
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import threading
import time
import traceback
from uuid import uuid4
import xml.etree.ElementTree as ET

import numpy as np

HERE = Path(__file__).resolve().parent
EXP = HERE.parent
WS = EXP.parent
sys.path.insert(0, str(EXP))
sys.path.insert(0, str(EXP / "worlds"))

import bench  # noqa: E402  (Track, process, read_pose, stamp, atomic_json, run_matrix)
import gen_world  # noqa: E402

BRINGUP = WS / "src/line_follower_bringup"
OFFICIAL = BRINGUP / "worlds/line_track.sdf"
RUN_USER = EXP / "user_base/run_user.py"
RUN_GUARDED = EXP / "user_base/run_guarded.py"
RULE_YAML = WS / "src/line_follower/config/rule.yaml"
OUT = EXP / "results/safety"
MAPS = OUT / "maps"
RUNS = OUT / "runs"
GUARDED_OUT = OUT / "guarded"
STACK_RUNS = {"raw": RUNS, "guarded": GUARDED_OUT / "runs"}
INSTALL = WS / "install_s"
ROBOT = "turtlebot3_waffle_pi"
# Real TurtleBot3 Waffle Pi limits (ROBOTIS e-Manual / turtlebot3 teleop constants).
HW_V, HW_W = 0.26, 1.82
G2_SUBSET = ["easy_20_obs1", "easy_22_obs3", "normal_26_obs1", "normal_28_obs3",
             "normal_30_obs2", "hard_33_obs2"]

# --------------------------------------------------------------------------------------
# scenarios
# --------------------------------------------------------------------------------------
# world: 'official' | name of a crafted map (see build_maps) | 'gen:<difficulty>:<seed>:<crosses>'
# trigger: ('progress', s) = GT progress along the line >= s m; ('after', t) = sim seconds after
#          the follower started.  fault: what happens at the trigger.  window: sim seconds
#          observed after the fault.  duration: sim seconds observed after follower start
#          (no-fault scenarios).  end='finish': stop observing 3 s after 'reached finish'.
SCENARIOS = {
    "nominal": dict(item="7", world="official", end="finish", duration=140,
                    doc="full official lap, no fault: commanded vs true v/w"),
    "sigint": dict(item="1", world="official", trigger=("progress", 7.85), fault="follower_sigint",
                   window=8, doc="SIGINT the follower on the fastest straight (~0.5 m/s)"),
    "sigterm": dict(item="1", world="official", trigger=("progress", 7.85), fault="follower_sigterm",
                    window=8, doc="SIGTERM the follower (docker stop / process manager)"),
    "sigkill": dict(item="1", world="official", trigger=("progress", 7.85), fault="follower_sigkill",
                    window=8, doc="SIGKILL the follower (= crash): does Gazebo keep the last /cmd_vel?"),
    "sigkill_slow": dict(item="1", world="official", trigger=("progress", 4.0), fault="follower_sigkill",
                         window=8, doc="SIGKILL at normal speed (v_max 0.25 window)"),
    "bridge_kill": dict(item="1+", world="official", trigger=("progress", 7.85), fault="bridge_sigkill",
                        window=8, doc="SIGKILL the ros_gz parameter_bridge (cmd_vel/odom/imu/clock)"),
    "camera_loss": dict(item="2", world="official", trigger=("progress", 7.85), fault="camera_sigkill",
                        window=40, doc="SIGKILL the image bridge: blind distance/time, does it stop?"),
    "camera_loss_slow": dict(item="2", world="official", trigger=("progress", 4.0), fault="camera_sigkill",
                             window=40, doc="camera loss at normal speed"),
    "odom_freeze": dict(item="3", world="official", relay="odom", trigger=("progress", 4.0),
                        fault="relay_stop", window=30, doc="/odom frozen (relay SIGSTOP), camera alive"),
    "imu_freeze": dict(item="3+", world="official", split_imu=True, trigger=("progress", 4.0),
                       fault="imubridge_stop", window=30,
                       doc="/imu on its own parameter_bridge, SIGSTOP at 4.0 m (odom+camera alive)"),
    "imu_bridge_ctrl": dict(item="3+", world="official", split_imu=True, duration=60,
                            doc="control for imu_freeze: same split /imu bridge, never frozen"),
    # superseded: the Python relay alone already breaks the follower (see imu_relay_ctrl)
    "imu_freeze_relay": dict(item="3+", world="official", relay="imu", trigger=("progress", 4.0),
                             fault="relay_stop", window=30, doc="(superseded) /imu via Python relay, SIGSTOP"),
    "imu_relay_ctrl": dict(item="3+", world="official", relay="imu", duration=60,
                           doc="control for imu_freeze_relay: /imu through the Python relay, never frozen"),
    "deadend": dict(item="4", world="deadend", duration=90,
                    doc="2 m line that just ends (no T): creep, spin, never stop?"),
    "blank_start": dict(item="4", world="blank", duration=60,
                        doc="robot starts 1.8 m from the only line: search behaviour"),
    "short_t": dict(item="5", world="short_t", duration=150, end="finish",
                    doc="8.0 m map with a proper finish T (< finish_min_dist 14 m)"),
    "cross_late_a": dict(item="5", world="gen:normal:26:4", duration=200, end="finish",
                         doc="gen normal_26 --crosses 4: + crossing at 15.13 m of 16.04 m"),
    "cross_late_b": dict(item="5", world="gen:normal:13:4", duration=200, end="finish",
                         doc="gen normal_13 --crosses 4: + crossing at 14.98 m of 15.80 m"),
    "far": dict(item="9", world="far", duration=90,
                doc="7 m straight out and back (path > 4.7 m from start): line-map grid edge"),
    "camera_late": dict(item="8", world="official", camera_delay=10.0, duration=50,
                        doc="odom ready, first image 10 s later: does it move blind before seeing?"),
    "launch_default": dict(item="8", launch=True, start_delay=20.0, duration=50,
                           doc="ros2 launch line_follower (gui:=false): 20 s wall timer"),
    "launch_nodelay": dict(item="8", launch=True, start_delay=0.0, duration=50,
                           doc="start_delay:=0 = simulator 20 s slower than the timer"),
    # item 6: G2 boxes with contact sensors, NO off-line termination (bench ends runs at 8 cm)
    "obs_easy_22": dict(item="6", world="g2:easy_22_obs3", obstacles=True, duration=200, end="finish",
                        stall=25, doc="G2 easy_22_obs3: on-line boxes at s=4.38/7.40/8.00 m"),
    "obs_normal_28": dict(item="6", world="g2:normal_28_obs3", obstacles=True, duration=200, end="finish",
                          stall=25, doc="G2 normal_28_obs3: on-line 4.39/7.19 m, beside-line 9.55 m"),
    "obs_normal_30": dict(item="6", world="g2:normal_30_obs2", obstacles=True, duration=200, end="finish",
                          stall=25, doc="G2 normal_30_obs2: on-line boxes at 10.94/14.98 m"),
    "obs_easy_20": dict(item="6", world="g2:easy_20_obs1", obstacles=True, duration=200, end="finish",
                        stall=25, doc="G2 easy_20_obs1: on-line box at 10.79 m"),
    "nobox_easy_22": dict(item="6", world="gen:easy:22:0", duration=200, end="finish", stall=25,
                          doc="control: easy_22 geometry (identical centreline/markings) without boxes"),
    "nobox_normal_28": dict(item="6", world="gen:normal:28:0", duration=200, end="finish", stall=25,
                            doc="control: normal_28 geometry without boxes"),
    # guarded stack only (--stack guarded)
    "guard_sigint": dict(item="1g", world="official", trigger=("progress", 7.85), fault="guard_sigint",
                         window=8, stack="guarded", doc="SIGINT the safety guard on the fastest straight"),
    "guard_sigkill": dict(item="1g", world="official", trigger=("progress", 7.85), fault="guard_sigkill",
                          window=8, stack="guarded",
                          doc="SIGKILL the safety guard (no zero possible: gz keeps the last command)"),
    "stack_sigint": dict(item="1g", world="official", trigger=("progress", 7.85), fault="stack_sigint",
                         window=8, stack="guarded",
                         doc="SIGINT run_guarded.py: forwarded to follower and guard (Ctrl-C)"),
    "gate_hold": dict(item="8g", world="official", gate_delay=20.0, duration=45, stack="guarded",
                      doc="guard start gate 20 s while the follower runs and ramps from t=0"),
}
ORDER = list(SCENARIOS)


# --------------------------------------------------------------------------------------
# maps
# --------------------------------------------------------------------------------------
def _write_world(name, center, markings, spawn, bounds):
    """Flat crafted world from the official template (same robot, lights, physics)."""
    MAPS.mkdir(parents=True, exist_ok=True)
    out = MAPS / f"{name}.sdf"
    mesh = out.with_suffix(".stl")
    gen_world.write_line_mesh(mesh, [list(map(tuple, center)), *markings], lambda x, y: 0.0)
    tree = ET.parse(OFFICIAL)
    world = tree.getroot().find("world")
    world.remove(world.find("gui"))
    track_include, robot = world.findall("include")
    world.remove(track_include)
    robot.find("pose").text = " ".join(f"{v:.9g}" for v in spawn)
    track = gen_world.add(world, "model", name="track")
    gen_world.add(track, "static", "true")
    link = gen_world.add(track, "link", name="surface")
    w, h = bounds[2] - bounds[0], bounds[3] - bounds[1]
    gen_world.surface(link, "mat", (w, h, 0.1), (bounds[0] + w / 2, bounds[1] + h / 2, -0.05, 0, 0, 0),
                      gen_world.MAT_COLOR)
    visual = gen_world.add(gen_world.add(track, "link", name="line"), "visual", name="visual")
    gen_world.add(visual, "cast_shadows", "false")
    gen_world.add(gen_world.add(gen_world.add(visual, "geometry"), "mesh"), "uri", f"./{mesh.name}")
    gen_world.material(visual, "0 0 0 1", "0.1 0.1 0.1 1")
    ET.indent(tree)
    tree.write(out, encoding="utf-8", xml_declaration=True)
    arc = [0.0]
    for a, b in zip(center, center[1:]):
        arc.append(arc[-1] + math.dist(a, b))
    meta = {"version": 1, "world": world.get("name"), "sdf": out.name, "crafted": name,
            "start_pose": list(spawn), "centerline": [[x, y, 0.0] for x, y in center], "arc_m": arc,
            "markings": [[[x, y, 0.0] for x, y in m] for m in markings], "mat_bounds": list(bounds),
            "obstacles": []}
    out.with_suffix(".json").write_text(json.dumps(meta, separators=(",", ":")) + "\n")
    return out


def _path(x, y, heading, program):
    return [(x, y), *gen_world.turtle(x, y, heading, program)]


def _tbar(p, heading, half=0.10):
    nx, ny = -math.sin(heading), math.cos(heading)
    return [(p[0] - half * nx, p[1] - half * ny), (p[0] + half * nx, p[1] + half * ny)]


def build_maps():
    """Crafted maps (flat, official mat colour, 2 cm line) + generated maps. Idempotent."""
    made = {}
    # deadend: a 2 m line that simply ends, no T anywhere
    c = _path(0.5, 0.5, 0.0, [("S", 2.0)])
    made["deadend"] = _write_world("deadend", c, [], (0.5, 0.5, 0.01, 0, 0, 0), (0, 0, 4.0, 1.5))
    # blank: the only line is 1.8 m away from the robot
    c = _path(0.5, 2.8, 0.0, [("S", 2.5)])
    made["blank"] = _write_world("blank", c, [], (1.0, 1.0, 0.01, 0, 0, 0), (0, 0, 3.5, 3.5))
    # short_t: S-shape 8.0 m with start and finish T (shorter than finish_min_dist = 14 m)
    c = _path(0.5, 0.5, 0.0, [("S", 2.2), ("A", 0.38, 180), ("S", 2.2), ("A", 0.38, -180), ("S", 1.2)])
    made["short_t"] = _write_world("short_t", c, [_tbar(c[0], 0.0), _tbar(c[-1], 0.0)],
                                   (0.5, 0.5, 0.01, 0, 0, 0), (0, 0, 3.6, 2.5))
    # far: 7 m out, U-turn, 7 m back (15.2 m, finish T); |x - x0| reaches 7.4 m
    c = _path(0.5, 0.5, 0.0, [("S", 7.0), ("A", 0.38, 180), ("S", 7.0)])
    made["far"] = _write_world("far", c, [_tbar(c[0], 0.0), _tbar(c[-1], math.pi)],
                               (0.5, 0.5, 0.01, 0, 0, 0), (0, 0, 8.4, 1.9))
    return made


def world_for(spec):
    """(sdf path, map json path or None) for a scenario world spec."""
    if spec == "official":
        return OFFICIAL, None
    if spec.startswith("g2:"):
        name = spec[3:]
        g2_maps()
        out = MAPS / "g2" / f"{name}.sdf"
        return out, out.with_suffix(".json")
    if spec.startswith("gen:"):
        _, difficulty, seed, crosses = spec.split(":")
        out = MAPS / f"{difficulty}_{seed}_x{crosses}.sdf"
        if not out.with_suffix(".json").is_file():
            gen_world.generate(int(seed), difficulty, 0, out, crosses=int(crosses))
        return out, out.with_suffix(".json")
    out = MAPS / f"{spec}.sdf"
    if not out.is_file():
        build_maps()
    return out, out.with_suffix(".json")


def g2_maps():
    paths = []
    for name in G2_SUBSET:
        difficulty, seed, obstacles = name.split("_")
        out = MAPS / "g2" / f"{name}.sdf"
        if not out.with_suffix(".json").is_file():
            gen_world.generate(int(seed), difficulty, int(obstacles[3:]), out)
        paths.append(out)
    return paths


# --------------------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------------------
def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def domain_busy(domain):
    """True if some other participant already publishes on this ROS domain."""
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS2CLI_NO_DAEMON="1")
    try:
        out = subprocess.run(["ros2", "topic", "list", "--no-daemon", "--spin-time", "2"], env=env,
                             capture_output=True, text=True, timeout=20).stdout.split()
    except subprocess.TimeoutExpired:
        return True
    return bool(set(out) - {"/parameter_events", "/rosout"})


def relay_main(src, dst, kind):
    """Tiny topic relay (pause it with SIGSTOP to freeze the topic)."""
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    if kind == "odom":
        from nav_msgs.msg import Odometry as T
    else:
        from sensor_msgs.msg import Imu as T
    rclpy.init()
    node = rclpy.create_node(f"safety_relay_{kind}")
    pub = node.create_publisher(T, dst, qos_profile_sensor_data)
    node.create_subscription(T, src, pub.publish, qos_profile_sensor_data)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass


class Progress:
    """Monotonic progress along the ordered centreline (same window rule as bench.Score)."""

    def __init__(self, track):
        self.track, self.s = track, 0.0

    def update(self, xy):
        cand, err = self.track.project(np.asarray(xy, float), self.s)
        if err < 0.15 and cand > self.s:
            self.s = cand
        return self.s, self.track.project(np.asarray(xy, float))[1]


# --------------------------------------------------------------------------------------
# one run (executed in a child process with its own ROS domain / Gazebo partition)
# --------------------------------------------------------------------------------------
def run_job(job_path):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import Image
    from rosgraph_msgs.msg import Clock
    from gz.transport13 import Node as GzNode
    from gz.msgs10.pose_v_pb2 import Pose_V

    job = json.loads(Path(job_path).read_text())
    folder = Path(job_path).parent
    sc = SCENARIOS[job["scenario"]]
    stack = job.get("stack", "raw")
    guarded = stack == "guarded"
    install = Path(job.get("install") or INSTALL)
    res = dict(scenario=job["scenario"], item=sc["item"], doc=sc["doc"], rep=job["rep"], stack=stack,
               domain=int(os.environ["ROS_DOMAIN_ID"]), partition=os.environ["GZ_PARTITION"],
               world=job["world"], ok=False)
    os.environ["GZ_SIM_RESOURCE_PATH"] = os.pathsep.join(
        filter(None, [str(BRINGUP / "models"), os.environ.get("GZ_SIM_RESOURCE_PATH")]))
    os.environ["ROS_LOG_DIR"] = str(folder / "ros")
    os.environ["LF_PARAMS"] = str(RULE_YAML)
    pidfile = folder / "pids.json"
    if guarded:
        os.environ.update(LF_GUARDED_PIDS=str(pidfile), LF_START_DELAY=str(sc.get("gate_delay", 0.0)))
    world_path = Path(job["world"])
    track = bench.Track(world_path)
    mat = (0.0, 0.0, 3.3, 3.3)
    if job.get("map_json"):
        meta = json.loads(Path(job["map_json"]).read_text())
        mat = tuple(meta.get("mat_bounds", mat))[:4]
    res["path_length_m"] = round(float(track.arc[-1]), 3)

    truth = deque(maxlen=4000)
    gt_rows, cmd_rows, odom_rows = [], [], []
    first = {}
    wall0 = time.time()

    def on_pose(msg):
        for p in msg.pose:
            if p.name == ROBOT:
                truth.append((bench.stamp(msg.header.stamp), p.position.x, p.position.y,
                              p.position.z, yaw_of(p.orientation)))
                break

    def latest_sim():
        return truth[-1][0] if truth else float("nan")

    def on_cmd(m):
        now = time.time() - wall0
        cmd_rows.append((now, latest_sim(), m.linear.x, m.angular.z))
        first.setdefault("cmd_wall", now)
        if abs(m.linear.x) > 1e-6 or abs(m.angular.z) > 1e-6:
            first.setdefault("nonzero_cmd_wall", now)
            first.setdefault("nonzero_cmd_sim", latest_sim())

    def on_odom(m):
        now = time.time() - wall0
        p = m.pose.pose
        odom_rows.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.position.x,
                          p.position.y, yaw_of(p.orientation)))
        first.setdefault("odom_wall", now)

    def on_clock(_m):
        first.setdefault("clock_wall", time.time() - wall0)

    image_sub = [None]
    cand_rows, safety_events = [], []

    def on_cand(m):
        cand_rows.append((time.time() - wall0, latest_sim(), m.linear.x, m.angular.z))

    def on_safety(m):
        try:
            st = json.loads(m.data)
        except ValueError:
            return
        if not safety_events or (safety_events[-1]["state"], safety_events[-1]["reason"].split(" (")[0]) != \
                (st.get("state"), str(st.get("reason", "")).split(" (")[0]):
            sim = latest_sim()                   # NaN before the first ground-truth pose
            safety_events.append(dict(wall=round(time.time() - wall0, 3), sim=round(sim, 3) if sim == sim else None,
                                      state=st.get("state"), reason=str(st.get("reason", ""))))

    def on_image(_m):
        first.setdefault("image_wall", time.time() - wall0)
        first.setdefault("image_sim", latest_sim())

    gz = GzNode()
    if not gz.subscribe(Pose_V, f"/world/{track.world}/dynamic_pose/info", on_pose):
        raise RuntimeError("cannot subscribe to ground truth")
    contacts = {}           # box name -> [first sim time, message count]
    if sc.get("obstacles"):
        from gz.msgs10.contacts_pb2 import Contacts
        world_path = track.evaluation_world(folder)       # + contact sensors on every box

        def on_contact(msg, name):
            for ev in msg.contact:
                if any(ROBOT in e for e in (ev.collision1.name, ev.collision2.name)):
                    rec = contacts.setdefault(name, [latest_sim(), 0])
                    rec[1] += 1
                    break
        for box in track.obstacles:
            if not gz.subscribe(Contacts, bench.contact_topic(track.world, box["name"]),
                                lambda m, n=box["name"]: on_contact(m, n)):
                raise RuntimeError(f"cannot subscribe to contacts of {box['name']}")
    rclpy.init(args=[], signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    probe = rclpy.create_node("safety_probe")
    probe.create_subscription(Twist, "/cmd_vel", on_cmd, 50)
    probe.create_subscription(Odometry, "/odom", on_odom, qos_profile_sensor_data)
    probe.create_subscription(Clock, "/clock", on_clock, qos_profile_sensor_data)
    image_sub[0] = probe.create_subscription(Image, "/camera/image_raw", on_image, qos_profile_sensor_data)
    if guarded:
        from std_msgs.msg import String
        probe.create_subscription(Twist, "/line_follower/cmd_raw", on_cand, 50)
        probe.create_subscription(String, "/line_follower/safety", on_safety, 50)

    def spin(sec=0.02):
        rclpy.spin_once(probe, timeout_sec=sec)
        if image_sub[0] is not None and "image_wall" in first:
            probe.destroy_subscription(image_sub[0])     # only the first frame matters
            image_sub[0] = None

    procs = {}
    fault = dict(kind=sc.get("fault"))
    logs = {}
    try:
        with ExitStack() as stack:
            def start(name, cmd):
                logs[name] = stack.enter_context((folder / f"{name}.log").open("w"))
                procs[name] = stack.enter_context(bench.process(cmd, logs[name]))
                return procs[name]

            if sc.get("launch"):
                launch_cmd = (f"source {install}/setup.bash && exec ros2 launch line_follower "
                              f"line_follower.launch.py gui:=false start_delay:={sc['start_delay']}")
                wall0 = time.time()
                start("launch", ["bash", "-c", launch_cmd])
            else:
                start("sim", ["gz", "sim", "-s", "-r", "--headless-rendering", str(world_path)])
                bridge_cfg = BRINGUP / "config/ros_gz_bridge.yaml"
                if sc.get("split_imu"):
                    import yaml
                    entries = [e for e in yaml.safe_load(bridge_cfg.read_text()) if e["ros_topic_name"] != "/imu"]
                    bridge_cfg = folder / "bridge_without_imu.yaml"
                    bridge_cfg.write_text(yaml.safe_dump(entries))
                    start("imubridge", ["ros2", "run", "ros_gz_bridge", "parameter_bridge",
                                        "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU"])
                start("bridge", ["ros2", "run", "ros_gz_bridge", "parameter_bridge", "--ros-args", "-p",
                                 f"config_file:={bridge_cfg}"])
                if not sc.get("camera_delay"):
                    start("camera", ["ros2", "run", "ros_gz_image", "image_bridge", "/camera/image_raw"])
                deadline = time.monotonic() + 90
                while True:
                    spin(0.05)
                    cam_ok = sc.get("camera_delay") or probe.count_publishers("/camera/image_raw") > 0
                    if truth and odom_rows and cam_ok:
                        break
                    if time.monotonic() > deadline or any(p.poll() is not None for p in procs.values()):
                        raise RuntimeError(f"startup failed: pose={bool(truth)} odom={bool(odom_rows)}")
                follower_args = []
                if sc.get("relay"):
                    kind = sc["relay"]
                    src = "/odom" if kind == "odom" else "/imu"
                    start("relay", [sys.executable, __file__, "--relay", src, f"{src}_relay", kind])
                    follower_args = ["-r", f"{src}:={src}_relay"]
                    os.environ["LF_GUARD_ARGS"] = " ".join(follower_args)    # guard watches the same input
                    time.sleep(1.0)
                res["stack_started_wall"] = round(time.time() - wall0, 3)
                start("follower", [sys.executable, str(RUN_GUARDED if guarded else RUN_USER), *follower_args])

            # ---------------- observation loop ----------------
            t_start = None          # sim time the follower was started (launch: first GT sample)
            prog = Progress(track)
            cursor = -1.0
            last_wall = time.monotonic()
            wall_start = time.monotonic()
            finish_seen_at = None
            best_prog, best_prog_t = 0.0, None
            end_reason = None
            next_log_check = 0.0
            while True:
                spin(0.02)
                if not truth:
                    if time.monotonic() - wall_start > 120:
                        raise RuntimeError("no ground truth")
                    continue
                cur = truth[-1]
                if t_start is None:
                    t_start = cur[0]
                    res["t_start_sim"] = round(t_start, 3)
                if cur[0] - cursor < 0.02:
                    if time.monotonic() - last_wall > 30:
                        raise RuntimeError("Gazebo ground truth stalled")
                    if time.monotonic() - wall_start > 900:
                        raise RuntimeError("wall-clock cap")
                    continue
                cursor = cur[0]
                last_wall = time.monotonic()
                s, lat = prog.update(cur[1:3])
                gt_rows.append((*cur, s, lat, time.time() - wall0))
                t_rel = cur[0] - t_start

                # camera started late on purpose
                if sc.get("camera_delay") and "camera" not in procs and t_rel >= sc["camera_delay"]:
                    start("camera", ["ros2", "run", "ros_gz_image", "image_bridge", "/camera/image_raw"])
                    res["camera_started_sim"] = round(cur[0], 3)

                # fault trigger
                trig = sc.get("trigger")
                if trig and "t" not in fault:
                    hit = (trig[0] == "progress" and s >= trig[1]) or (trig[0] == "after" and t_rel >= trig[1])
                    if hit:
                        fault.update(t=cur[0], wall=time.time() - wall0, progress=round(s, 3))
                        k = fault["kind"]
                        target = {"follower": "follower", "bridge": "bridge", "camera": "camera",
                                  "relay": "relay", "imubridge": "imubridge", "guard": "guard",
                                  "stack": "stack"}[k.split("_")[0]]
                        sig = {"sigint": signal.SIGINT, "sigterm": signal.SIGTERM, "sigkill": signal.SIGKILL,
                               "stop": signal.SIGSTOP}[k.split("_")[1]]
                        if guarded and target in ("follower", "guard"):
                            # one child of run_guarded.py only; the other keeps running
                            fault["pid"] = json.loads(pidfile.read_text())[target]
                            os.kill(fault["pid"], sig)
                        elif target == "stack":
                            os.kill(procs["follower"].pid, sig)        # the wrapper: it forwards
                        else:
                            os.killpg(procs[target].pid, sig)

                # end conditions
                if time.time() - wall0 > next_log_check:
                    next_log_check = time.time() - wall0 + 1.0
                    if sc.get("end") == "finish" and finish_seen_at is None:
                        text = (folder / ("launch.log" if sc.get("launch") else "follower.log")).read_text(
                            errors="replace")
                        if "reached finish" in text:
                            finish_seen_at = cur[0]
                if "t" in fault and cur[0] >= fault["t"] + sc["window"]:
                    end_reason = "window"
                elif not sc.get("trigger") and t_rel >= sc.get("duration", 60):
                    end_reason = "duration"
                elif sc.get("trigger") and "t" not in fault and t_rel >= 240:
                    end_reason = "trigger never reached"
                elif finish_seen_at is not None and cur[0] >= finish_seen_at + 3.0:
                    end_reason = "finished"
                if best_prog_t is None or s > best_prog + 0.05:
                    best_prog, best_prog_t = s, cur[0]
                if not end_reason and sc.get("stall") and s > 0.1 and cur[0] - best_prog_t > sc["stall"]:
                    end_reason = f"stalled {sc['stall']} s at {best_prog:.2f} m"
                if end_reason:
                    break
                if "t" in fault and "follower" in procs and "exit_wall" not in fault:
                    if fault.get("pid"):
                        try:
                            os.kill(fault["pid"], 0)
                            gone = False
                        except ProcessLookupError:
                            gone = True
                    else:
                        gone = procs["follower"].poll() is not None
                    if gone:
                        fault["exit_wall"] = time.time() - wall0
                        fault["exit_after_s"] = round(fault["exit_wall"] - fault["wall"], 3)
                if "follower" in procs and procs["follower"].poll() is not None and "t" not in fault:
                    end_reason = f"follower exited early ({procs['follower'].returncode})"
                    break

            # follower exit after the fault (signals)
            if "follower" in procs:
                f = procs["follower"]
                if "t" in fault and fault["kind"].startswith(("follower", "stack")) and not fault.get("pid"):
                    try:
                        f.wait(10)
                    except subprocess.TimeoutExpired:
                        pass
                res["follower_returncode"] = f.poll()
            for name in ("relay", "imubridge"):
                if fault.get("kind", "") and fault["kind"].startswith(name) and procs[name].poll() is None:
                    os.killpg(procs[name].pid, signal.SIGKILL)       # SIGSTOPped: SIGINT would hang
            res["end_reason"] = end_reason
            res["ok"] = True
    except (Exception, KeyboardInterrupt) as error:
        (folder / "error.log").write_text(traceback.format_exc())
        res["error"] = f"{type(error).__name__}: {error}"
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass

    gt = np.array(gt_rows, float).reshape(-1, 8)            # t x y z yaw progress lateral wall
    cmd = np.array(cmd_rows, float).reshape(-1, 4)          # wall sim v w
    odom = np.array(odom_rows, float).reshape(-1, 4)
    cand = np.array(cand_rows, float).reshape(-1, 4)        # wall sim v w (guarded: follower candidate)
    np.savez_compressed(folder / "trace.npz", gt=gt, cmd=cmd, odom=odom, cand=cand)
    res["fault"] = fault
    if guarded:
        res["safety_events"] = safety_events[:200]
        log_text = (folder / "follower.log").read_text(errors="replace") if (folder / "follower.log").is_file() else ""
        res["children_exit"] = {n: int(c) for n, c in re.findall(r"run_guarded: (\w+) exited with code (-?\d+)", log_text)}
    res["first"] = {k: round(v, 3) for k, v in first.items() if v == v}
    flog = folder / ("launch.log" if sc.get("launch") else "follower.log")
    text = flog.read_text(errors="replace") if flog.is_file() else ""
    m = re.findall(r"finish T detected \(driven ([0-9.]+) m\)", text)
    res["finish_T_detected_travel"] = [float(v) for v in m]
    m = re.findall(r"reached finish, stopping \(driven ([0-9.]+) m\)", text)
    res["reached_finish_travel"] = [float(v) for v in m]
    if sc.get("launch"):
        m = re.search(r"\[follower_node-\d+\] \[INFO\] \[([0-9.]+)\] \[follower_node\]: line follower started", text)
        if m:
            res["first"]["follower_started_wall"] = round(float(m.group(1)) - wall0, 3)
    try:
        res.update(analyse(gt, cmd, res, sc, mat, track))
        res.update(analyse_guard(gt, cmd, cand, res))
        if sc.get("obstacles"):
            res["boxes"] = boxes_report(gt, track, contacts)
            res["collision_count"] = sum(1 for b in res["boxes"] if b["contact"])
            res["passed_without_contact"] = sum(1 for b in res["boxes"] if b["passed"] and not b["contact"])
    except Exception as error:
        res["analysis_error"] = repr(error)
    bench.atomic_json(folder / "result.json", res)
    return 0 if res["ok"] else 1


def boxes_report(gt, track, contacts):
    """Per box: contact, closest approach, passed it along the line, deviation before it."""
    out = []
    maxprog = float(gt[:, 5].max()) if len(gt) else 0.0
    for box in track.obstacles:
        bxy = np.array(box["pose"][:2])
        s_box, off = track.project(bxy)
        d = np.hypot(gt[:, 1] - bxy[0], gt[:, 2] - bxy[1])
        k = int(d.argmin())
        near = (gt[:, 5] > s_box - 1.0) & (gt[:, 5] < s_box) & (d < 1.2)
        c = contacts.get(box["name"])
        out.append(dict(
            name=box["name"], on_line=bool(box["on_line"]), s_m=round(float(s_box), 2),
            offset_m=round(float(off), 3), size=[round(v, 3) for v in box["size"]],
            contact=c is not None, first_contact_sim=round(c[0], 2) if c else None,
            contact_msgs=c[1] if c else 0, closest_center_m=round(float(d[k]), 3),
            closest_at_sim=round(float(gt[k, 0]), 2), reached=maxprog >= s_box - 0.5,
            passed=maxprog >= s_box + 0.30,
            max_lateral_last_1m_before=round(float(gt[near, 6].max()), 3) if near.any() else None))
    return out


def speed_series(gt, span=0.2):
    """True planar speed and yaw rate from GT over ~span seconds (centred differences)."""
    t = gt[:, 0]
    n = len(t)
    v = np.zeros(n)
    w = np.zeros(n)
    if n < 3:
        return v, w
    dt_med = max(1e-3, float(np.median(np.diff(t))))
    k = max(1, int(round(span / dt_med / 2)))
    yaw = np.unwrap(gt[:, 4])
    for i in range(n):
        a, b = max(0, i - k), min(n - 1, i + k)
        if t[b] - t[a] <= 0:
            continue
        v[i] = math.hypot(gt[b, 1] - gt[a, 1], gt[b, 2] - gt[a, 2]) / (t[b] - t[a])
        w[i] = (yaw[b] - yaw[a]) / (t[b] - t[a])
    return v, w


def stop_time(t, v, w, t0, hold=1.0):
    """First time >= t0 at which the robot is at rest (v<1 cm/s, |w|<0.05) for `hold` s, else None."""
    rest = (v < 0.01) & (np.abs(w) < 0.05)
    since = None
    for ti, r in zip(t, rest):
        if ti < t0:
            continue
        if r:
            since = ti if since is None else since
            if ti - since >= hold:
                return since
        else:
            since = None
    return None


def analyse_guard(gt, cmd, cand, res):
    """Stop latency / distance after a fault and the start-gate ramp (both stacks)."""
    out = {}
    if len(gt) < 5:
        return out
    t = gt[:, 0]
    v, w = speed_series(gt)
    fault = res.get("fault", {})
    if fault.get("t") is not None:
        zero = (np.abs(cmd[:, 2]) < 1e-9) & (np.abs(cmd[:, 3]) < 1e-9) if len(cmd) else np.zeros(0, bool)
        after = cmd[:, 0] >= fault["wall"] if len(cmd) else np.zeros(0, bool)
        hit = np.nonzero(zero & after)[0]
        out["cmd_first_zero_after_fault_s"] = round(float(cmd[hit[0], 0] - fault["wall"]), 3) if len(hit) else None
        nonzero_after = np.nonzero(~zero & after)[0]
        out["cmd_last_nonzero_after_fault_s"] = (round(float(cmd[nonzero_after[-1], 0] - fault["wall"]), 3)
                                                 if len(nonzero_after) else None)
        ts = stop_time(t, v, w, fault["t"], hold=1.0)
        if ts is not None:
            i0, i1 = int(np.searchsorted(t, fault["t"])), int(np.searchsorted(t, ts))
            seg = gt[i0:i1 + 1]
            out["stop_path_m"] = round(float(np.hypot(np.diff(seg[:, 1]), np.diff(seg[:, 2])).sum()), 3)
            out["stop_disp_m"] = round(float(np.hypot(*(gt[min(i1, len(gt) - 1), 1:3] - gt[i0, 1:3]))), 3)
    x0, y0 = gt[0, 1], gt[0, 2]
    moving = np.nonzero(np.hypot(gt[:, 1] - x0, gt[:, 2] - y0) > 0.02)[0]
    if res.get("stack_started_wall") is not None and len(moving):
        out["first_motion_after_stack_start_s"] = round(float(gt[moving[0], 7] - res["stack_started_wall"]), 2)
    if len(cmd):
        nz = np.nonzero((np.abs(cmd[:, 2]) > 1e-9) | (np.abs(cmd[:, 3]) > 1e-9))[0]
        if len(nz):
            k = nz[0]
            if res.get("stack_started_wall") is not None:
                out["first_nonzero_cmd_after_stack_start_s"] = round(float(cmd[k, 0] - res["stack_started_wall"]), 2)
            out["first_nonzero_cmd"] = [round(float(cmd[k, 2]), 4), round(float(cmd[k, 3]), 4)]
            win = cmd[k - 1 if k else k:][cmd[k - 1 if k else k:, 0] <= cmd[k, 0] + 2.0]
            out["first_2s_max_dv_per_msg"] = round(float(np.abs(np.diff(win[:, 2])).max()), 4) if len(win) > 1 else None
            if len(cand):
                j = int(np.argmin(np.abs(cand[:, 0] - cmd[k, 0])))
                out["candidate_at_first_nonzero"] = [round(float(cand[j, 2]), 4), round(float(cand[j, 3]), 4)]
    if len(moving):
        i = moving[0]
        first = (t >= t[i]) & (t <= t[i] + 1.5)
        if first.sum() > 3:
            out["first_1p5s_gt_max_accel"] = round(float(np.abs(np.diff(v[first]) / np.maximum(np.diff(t[first]), 1e-3)).max()), 2)
    return out


def analyse(gt, cmd, res, sc, mat, track):
    out = {}
    if len(gt) < 5:
        return out
    t = gt[:, 0]
    v, w = speed_series(gt)
    yaw = np.unwrap(gt[:, 4])
    x0, y0 = gt[0, 1], gt[0, 2]
    moving = np.nonzero(np.hypot(gt[:, 1] - x0, gt[:, 2] - y0) > 0.02)[0]
    out["first_motion_sim"] = round(float(t[moving[0]]), 3) if len(moving) else None
    out["first_motion_wall"] = round(float(gt[moving[0], 7]), 3) if len(moving) else None
    out["max_progress_m"] = round(float(gt[:, 5].max()), 3)
    out["sim_observed_s"] = round(float(t[-1] - t[0]), 2)
    fault = res.get("fault", {})
    t0 = fault.get("t")
    if t0 is None:      # no fault: observe from the first motion
        t0 = float(t[moving[0]]) if len(moving) else float(t[0])
    i0 = int(np.searchsorted(t, t0))
    i0 = min(i0, len(t) - 1)
    pre = (t >= t0 - 0.5) & (t < t0)
    out["v_at_t0"] = round(float(v[pre].mean()), 3) if pre.any() else None
    out["w_at_t0"] = round(float(w[pre].mean()), 3) if pre.any() else None
    p0 = gt[i0, 1:3]

    def disp_at(dt):
        j = int(np.searchsorted(t, t0 + dt))
        if j >= len(t) or t[-1] < t0 + dt - 0.05:
            return None
        return round(float(np.hypot(*(gt[j, 1:3] - p0))), 3)

    for dt in (1, 2, 5):
        out[f"disp_{dt}s"] = disp_at(dt)
    seg = gt[i0:]
    steps = np.hypot(np.diff(seg[:, 1]), np.diff(seg[:, 2]))
    out["path_after_t0_m"] = round(float(steps.sum()), 3)
    out["disp_end_m"] = round(float(np.hypot(*(seg[-1, 1:3] - p0))), 3)
    out["yaw_turned_after_t0_rad"] = round(float(np.abs(np.diff(yaw[i0:])).sum()), 2)
    ts = stop_time(t, v, w, t0)
    out["t_stop_after_t0"] = round(ts - t0, 2) if ts is not None else None
    tail = t >= t[-1] - 1.0
    out["v_end"] = round(float(v[tail].mean()), 3)
    out["w_end"] = round(float(w[tail].mean()), 3)
    out["max_lateral_after_t0_m"] = round(float(seg[:, 6].max()), 3)
    out["progress_at_t0"] = round(float(gt[i0, 5]), 3)
    out["progress_end"] = round(float(gt[-1, 5]), 3)
    out["z_min_after_t0"] = round(float(seg[:, 3].min()), 3)
    xs, ys = seg[:, 1], seg[:, 2]
    off = (xs < mat[0]) | (ys < mat[1]) | (xs > mat[2]) | (ys > mat[3])
    out["left_mat"] = bool(off.any())
    if off.any():
        out["left_mat_after_s"] = round(float(seg[np.argmax(off), 0] - t0), 2)
    # blind/still driving: time after t0 during which GT speed > 2 cm/s or |w| > 0.1
    dts = np.diff(t[i0:], prepend=t[i0])
    active = (v[i0:] > 0.02) | (np.abs(w[i0:]) > 0.1)
    out["moving_time_after_t0_s"] = round(float(dts[active].sum()), 2)
    out["observed_after_t0_s"] = round(float(t[-1] - t0), 2)
    # /cmd_vel after the fault
    if len(cmd):
        after = cmd[cmd[:, 0] >= fault["wall"]] if fault.get("wall") is not None else cmd
        out["cmd_msgs_after_t0"] = int(len(after))
        if len(after):
            out["cmd_last"] = [round(float(after[-1, 2]), 3), round(float(after[-1, 3]), 3)]
            out["cmd_zero_after_t0"] = bool(((np.abs(after[:, 2]) < 1e-9) & (np.abs(after[:, 3]) < 1e-9)).any())
            out["cmd_last_wall_after_t0"] = round(float(after[-1, 0] - (fault.get("wall") or after[0, 0])), 2)
        before = cmd[cmd[:, 0] < fault["wall"]] if fault.get("wall") is not None else cmd[:0]
        if len(before):
            out["cmd_before_t0"] = [round(float(before[-1, 2]), 3), round(float(before[-1, 3]), 3)]
        # whole-run command statistics (item 7)
        cv, cw = cmd[:, 2], cmd[:, 3]
        out["cmd_v_max"] = round(float(cv.max()), 3)
        out["cmd_w_absmax"] = round(float(np.abs(cw).max()), 3)
        out["cmd_frac_v_over_hw"] = round(float((cv > HW_V + 1e-6).mean()), 3)
        out["cmd_frac_w_over_hw"] = round(float((np.abs(cw) > HW_W + 1e-6).mean()), 3)
        mov = cv > 0.05
        if mov.any():
            k_cmd = cw[mov] / cv[mov]
            k_clip = np.clip(cw[mov], -HW_W, HW_W) / np.minimum(cv[mov], HW_V)
            rel = np.abs(k_clip - k_cmd) / np.maximum(np.abs(k_cmd), 0.5)
            out["cmd_frac_curvature_changed_by_hw_clamp_gt25pct"] = round(float((rel > 0.25).mean()), 3)
    out["gt_v_max"] = round(float(v.max()), 3)
    out["gt_w_absmax"] = round(float(np.abs(w).max()), 3)
    out["gt_frac_v_over_hw"] = round(float((v > HW_V).mean()), 3)
    out["gt_frac_w_over_hw"] = round(float((np.abs(w) > HW_W).mean()), 3)
    if res.get("camera_started_sim") is not None:
        tc = res["camera_started_sim"]
        pre_cam = gt[t <= tc]
        if len(pre_cam):
            out["blind_before_camera_disp_m"] = round(float(np.hypot(*(pre_cam[-1, 1:3] - gt[0, 1:3]))), 3)
            out["blind_before_camera_path_m"] = round(float(np.hypot(np.diff(pre_cam[:, 1]),
                                                                     np.diff(pre_cam[:, 2])).sum()), 3)
            out["blind_before_camera_yaw_rad"] = round(float(np.abs(np.diff(yaw[:len(pre_cam)])).sum()), 2)
    first = res.get("first", {})
    if "image_sim" in first and first.get("image_sim") == first.get("image_sim"):
        pre_img = gt[t <= first["image_sim"]]
        if len(pre_img):
            out["moved_before_first_image_m"] = round(float(np.hypot(*(pre_img[-1, 1:3] - gt[0, 1:3]))), 3)
    if sc.get("launch") and len(moving):
        # wall time (from `ros2 launch`) of the first GT motion: map the sim time through cmd rows
        out["first_motion_sim_from_first_gt"] = round(float(t[moving[0]] - t[0]), 2)
    return out


# --------------------------------------------------------------------------------------
# orchestration
# --------------------------------------------------------------------------------------
def pick_domain(domains, held):
    for d in domains:
        if d in held:
            continue
        lock = open(f"/tmp/lf_s-safety-{d}.lock", "a")
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            lock.close()
            continue
        if domain_busy(d):
            fcntl.flock(lock, fcntl.LOCK_UN)
            lock.close()
            continue
        return d, lock
    return None, None


def run_scenarios(names, repeat, jobs, domains, stack="raw", gz_prefix="lf_s", install=None):
    if jobs > 2:
        raise SystemExit("at most 2 simulations at once")
    wrong = [n for n in names if SCENARIOS[n].get("stack", stack) != stack]
    if wrong:
        raise SystemExit(f"scenarios {wrong} need --stack {SCENARIOS[wrong[0]]['stack']}")
    build_maps()
    stamp_ = time.strftime("%Y%m%d-%H%M%S")
    queue = []
    for rep in range(1, repeat + 1):
        for name in names:
            sc = SCENARIOS[name]
            world, meta = world_for(sc.get("world", "official"))
            folder = STACK_RUNS[stack] / name / f"{stamp_}_r{rep}"
            folder.mkdir(parents=True, exist_ok=False)
            job = dict(scenario=name, rep=rep, world=str(world), map_json=str(meta) if meta else None,
                       stack=stack, install=str(install) if install else None)
            bench.atomic_json(folder / "job.json", job)
            queue.append(folder)
    held = set()
    guard = threading.Lock()

    def one(folder):
        while True:
            with guard:
                d, lock = pick_domain(domains, held)
                if d is not None:
                    held.add(d)
            if d is not None:
                break
            time.sleep(2)
        try:
            job = json.loads((folder / "job.json").read_text())
            env = dict(os.environ, ROS_DOMAIN_ID=str(d), ROS2CLI_NO_DAEMON="1",
                       GZ_PARTITION=f"{gz_prefix}_{job['scenario']}_{uuid4().hex[:8]}")
            t = time.monotonic()
            with (folder / "worker.log").open("w") as log:
                proc = subprocess.Popen([sys.executable, __file__, "--run-job", str(folder / "job.json")],
                                        stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
                proc.wait()
            r = json.loads((folder / "result.json").read_text()) if (folder / "result.json").is_file() else {}
            print(f"[{time.strftime('%H:%M:%S')}] {stack} {job['scenario']:16s} r{job['rep']} dom {d} "
                  f"{time.monotonic() - t:5.0f}s ok={r.get('ok')} end={r.get('end_reason')} "
                  f"err={r.get('error', '')}", flush=True)
        finally:
            with guard:
                held.discard(d)
            fcntl.flock(lock, fcntl.LOCK_UN)
            lock.close()

    with ThreadPoolExecutor(max_workers=jobs) as pool:
        list(pool.map(one, queue))


def run_obstacles(repeat, jobs, stack="raw", gz_prefix="lf_s"):
    """G2 subset with bench.py --backend obstacle (rule mode), repeated `repeat` times."""
    if jobs > 2:
        raise SystemExit("at most 2 simulations at once")
    maps = g2_maps() * repeat
    os.environ.update(LF_PARAMS=str(RULE_YAML), LAB_NAME=gz_prefix, BENCH_DOMAIN_BASE="70",
                      ROS_DOMAIN_ID=os.environ.get("ROS_DOMAIN_ID", "23"), GZ_PARTITION=f"{gz_prefix}_g2")
    out = STACK_RUNS[stack] / "obstacles" / time.strftime("%Y%m%d-%H%M%S")
    results = bench.run_matrix(maps, {"rule": []}, out, jobs=jobs, limit=300, stall=20, off=0.08,
                               startup=90, follower=RUN_GUARDED if stack == "guarded" else RUN_USER,
                               backend="obstacle")
    bench.print_table(results)
    for r in results:
        r["scenario"], r["item"] = "obstacles", "6"
    bench.atomic_json(out / "obstacles.json", results)


def summarize(stack="raw"):
    runs = STACK_RUNS[stack]
    rows = []
    for path in sorted(runs.glob("*/*/result.json")):
        r = json.loads(path.read_text())
        rows.append(r)
    rows.sort(key=lambda r: (ORDER.index(r["scenario"]) if r["scenario"] in ORDER else 99, r["rep"]))
    obstacles = []
    for path in sorted(runs.glob("obstacles/*/*/result.json")):
        obstacles.append(json.loads(path.read_text()))
    keys = ("v_at_t0", "disp_1s", "disp_5s", "path_after_t0_m", "t_stop_after_t0", "v_end", "w_end",
            "yaw_turned_after_t0_rad", "max_lateral_after_t0_m", "progress_at_t0", "progress_end",
            "max_progress_m", "left_mat", "cmd_zero_after_t0", "cmd_last", "follower_returncode",
            "finish_T_detected_travel", "reached_finish_travel", "end_reason",
            "cmd_first_zero_after_fault_s", "stop_path_m", "first_motion_after_stack_start_s",
            "first_nonzero_cmd", "candidate_at_first_nonzero", "first_2s_max_dv_per_msg")
    for r in rows:
        print(f"{r['scenario']:16s} r{r['rep']} ok={r.get('ok')} " +
              " ".join(f"{k}={r.get(k)}" for k in keys if k in r), r.get("error", ""))
    for r in obstacles:
        print(f"obstacles {Path(r['map']).stem:16s} complete={r.get('complete')} prog={r.get('progress_m')}"
              f" hits={r.get('collision_count')} attempts={r.get('avoidance_attempts')}"
              f" avoided={r.get('avoided_count')} reason={r.get('reason')}")
    bench.atomic_json((OUT if stack == "raw" else GUARDED_OUT) / "all_results.json",
                      {"runs": rows, "obstacles": obstacles})


def compare(names=None):
    """Markdown table: raw stack (results/safety/runs) vs guarded stack, mean (min-max) per scenario."""
    keys = (("disp_1s", "1 s 이동 m"), ("disp_5s", "5 s 이동 m"), ("t_stop_after_t0", "정지까지 s"),
            ("stop_path_m", "정지 거리 m"), ("cmd_first_zero_after_fault_s", "첫 0 명령 s"),
            ("path_after_t0_m", "관측 창 총 경로 m"), ("yaw_turned_after_t0_rad", "관측 창 회전 rad"),
            ("observed_after_t0_s", "관측 창 s"), ("left_mat", "매트 이탈"))

    def load(stack):
        out = {}
        for path in sorted(STACK_RUNS[stack].glob("*/*/result.json")):
            r = json.loads(path.read_text())
            trace = path.with_name("trace.npz")
            if "stop_path_m" not in r and "cmd_first_zero_after_fault_s" not in r and trace.is_file():
                d = np.load(trace)       # runs recorded before analyse_guard existed: same metrics
                r.update(analyse_guard(d["gt"], d["cmd"], d["cand"] if "cand" in d.files else np.zeros((0, 4)), r))
            if r.get("end_reason") == "trigger never reached":
                skipped.append(f"{stack}/{r['scenario']} r{r['rep']}")   # no fault happened
            elif r.get("ok"):
                out.setdefault(r["scenario"], []).append(r)
        return out

    skipped = []

    def cell(rows, key):
        vals = [r.get(key) for r in rows]
        if key == "left_mat":
            return f"{sum(bool(v) for v in vals)}/{len(vals)}"
        nums = [v for v in vals if isinstance(v, (int, float))]
        miss = len(vals) - len(nums)
        if not nums:
            return "없음" if miss else "-"
        txt = f"{np.mean(nums):.2f}" if len(nums) == 1 else f"{np.mean(nums):.2f} ({min(nums):.2f}–{max(nums):.2f})"
        return txt + (f", {miss}회 없음" if miss else "")

    raw, guarded = load("raw"), load("guarded")
    names = names or [n for n in ORDER if n in guarded]
    print("| 시나리오 | 스택 | 회수 | " + " | ".join(label for _, label in keys) + " |")
    print("|---|---|---|" + "---|" * len(keys))
    for name in names:
        for stack, data in (("raw", raw), ("guarded", guarded)):
            rows = data.get(name, [])
            if rows:
                print(f"| {name} | {stack} | {len(rows)} | " + " | ".join(cell(rows, k) for k, _ in keys) + " |")
    if skipped:
        print(f"\n(trigger never reached, left out: {', '.join(skipped)})")


def main():
    if len(sys.argv) >= 2 and sys.argv[1] == "--run-job":
        return run_job(sys.argv[2])
    if len(sys.argv) >= 2 and sys.argv[1] == "--relay":
        return relay_main(*sys.argv[2:5])
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list")
    sub.add_parser("maps")
    sm = sub.add_parser("summarize")
    sm.add_argument("--stack", choices=("raw", "guarded"), default="raw")
    cp = sub.add_parser("compare")
    cp.add_argument("scenarios", nargs="*")
    r = sub.add_parser("run")
    r.add_argument("scenarios", nargs="+", help="scenario names, 'all' or 'obstacles'")
    r.add_argument("--repeat", type=int, default=2)
    r.add_argument("--jobs", type=int, default=2)
    r.add_argument("--domains", default="84-89", help="ROS domain range for the runs (inclusive)")
    r.add_argument("--stack", choices=("raw", "guarded"), default="raw",
                   help="raw: run_user.py (follower publishes /cmd_vel); guarded: run_guarded.py")
    r.add_argument("--gz-prefix", default="lf_s", help="GZ_PARTITION prefix")
    r.add_argument("--install", type=Path, default=INSTALL, help="install space for the launch scenarios")
    a = ap.parse_args()
    if a.cmd == "list":
        for name in ORDER:
            only = f" [{SCENARIOS[name]['stack']} only]" if SCENARIOS[name].get("stack") else ""
            print(f"{name:16s} item {SCENARIOS[name]['item']:3s} {SCENARIOS[name]['doc']}{only}")
        print(f"{'obstacles':16s} item 6   G2 subset {G2_SUBSET} via bench.py --backend obstacle")
        return 0
    if a.cmd == "maps":
        print(build_maps(), g2_maps())
        return 0
    if a.cmd == "summarize":
        summarize(a.stack)
        return 0
    if a.cmd == "compare":
        compare(a.scenarios or None)
        return 0
    lo, hi = (int(v) for v in a.domains.split("-"))
    names = ([n for n in ORDER if SCENARIOS[n].get("stack", a.stack) == a.stack]
             if a.scenarios == ["all"] else a.scenarios)
    if "obstacles" in names:
        run_obstacles(a.repeat, a.jobs, a.stack, a.gz_prefix)
        names = [n for n in names if n != "obstacles"]
    unknown = set(names) - set(SCENARIOS)
    if unknown:
        ap.error(f"unknown scenarios {sorted(unknown)}")
    if names:
        run_scenarios(names, a.repeat, a.jobs, list(range(lo, hi + 1)), a.stack, a.gz_prefix,
                      a.install.resolve())
    return 0


if __name__ == "__main__":
    sys.exit(main())
