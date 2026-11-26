"""CPU-side kinematics, track scoring and lockstep transport for Isaac Drive."""

from bisect import bisect_left, bisect_right
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

HZ = 30
DT = 1 / HZ
WIDTH, HEIGHT = 640, 480
FRAME_BYTES = WIDTH * HEIGHT * 3
G = 9.81
FRAME_HEADER = struct.Struct("!4sIQ23d")
COMMAND_PACKET = struct.Struct("!4sI2d")
FRAME_MAGIC, COMMAND_MAGIC = b"IDF2", b"IDC1"
EXPERIMENTS = Path(__file__).resolve().parents[1]
BRINGUP = EXPERIMENTS.parent / "src/line_follower_bringup"


@dataclass(frozen=True)
class Command:
    v: float
    w: float


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    z: float
    pitch: float  # nose-up positive
    yaw: float


@dataclass(frozen=True)
class DriveMap:
    source: Path
    centerline: list
    arc: list
    ramps: list
    start: Pose
    clearance: float


def load_map(name):
    if name == "official":
        import numpy as np

        source = BRINGUP / "worlds/line_track.sdf"
        world = ET.parse(source).getroot().find("world")
        robot = next(item for item in world.findall("include")
                     if item.findtext("uri") == "model://turtlebot3_waffle_pi")
        route = np.load(EXPERIMENTS / "gt.npy")
        # Nominal top surface of hill_ramp_up, hill_top and hill_ramp_down
        # in src/line_follower_bringup/models/track/model.sdf.
        data = dict(start_pose=list(map(float, robot.findtext("pose").split())),
                    centerline=route[:, :2].tolist(), arc_m=route[:, 2].tolist(),
                    ramps=[dict(x=[0.66, 1.10, 1.54, 1.98], y=[0, 0.66], height_m=0.06)])
    else:
        source = Path(name).resolve()
        if source.suffix != ".json":
            raise ValueError("--map must be a JSON path or 'official'")
        data = json.loads(source.read_text())
        if data.get("obstacles"):
            raise ValueError("Isaac Drive supports line-only maps with zero obstacles")
    ramps = data.get("ramps", [data["ramp"]] if data.get("ramp") else [])
    x, y, z, _, _, yaw = data["start_pose"]
    start = Pose(x, y, z, ground_pitch(ramps, x, y, yaw), yaw)
    return DriveMap(source, data["centerline"], data["arc_m"], ramps, start,
                    z - ground_height(ramps, x, y))


def ground_height(ramps, x, y):
    """Scalar version of farband/capture.py's ramp_h, including legacy ramps."""
    height = 0.0
    for ramp in ramps:
        if "heading" in ramp:
            dx, dy = x - ramp["x0"], y - ramp["y0"]
            c, s = math.cos(ramp["heading"]), math.sin(ramp["heading"])
            along, across = dx * c + dy * s, -dx * s + dy * c
            if abs(across) > ramp.get("width", 0.66) / 2:
                continue
            up, deck, down = ramp["up"], ramp["deck"], ramp["down"]
            a, b, c, d = 0, up, up + deck, up + deck + down
        else:
            if not ramp["y"][0] <= y <= ramp["y"][1]:
                continue
            along = x
            a, b, c, d = ramp["x"]
        if a <= along <= d:
            fraction = ((along - a) / (b - a) if along < b else
                        (d - along) / (d - c) if along > c else 1.0)
            height = max(height, fraction * ramp["height_m"])
    return height


def ground_pitch(ramps, x, y, yaw):
    # Centered 2 cm baseline blends the pitch at ramp breakpoints.
    dx, dy = 0.01 * math.cos(yaw), 0.01 * math.sin(yaw)
    return math.atan2(ground_height(ramps, x + dx, y + dy) -
                      ground_height(ramps, x - dx, y - dy), 0.02)


def advance(pose, cmd, ramps, clearance):
    vmax = float(os.environ.get('ISAAC_MAX_V', '0.26'))   # Waffle Pi spec; Gazebo comparisons use the controller's limits
    wmax = float(os.environ.get('ISAAC_MAX_W', '1.82'))
    v = max(-vmax, min(vmax, cmd.v))
    w = max(-wmax, min(wmax, cmd.w))
    half_turn = w * DT / 2
    heading = pose.yaw + half_turn
    distance = v * DT * (math.sin(half_turn) / half_turn if half_turn else 1.0)
    x = pose.x + distance * math.cos(heading)
    y = pose.y + distance * math.sin(heading)
    yaw = math.remainder(pose.yaw + 2 * half_turn, math.tau)
    return (Pose(x, y, clearance + ground_height(ramps, x, y),
                 ground_pitch(ramps, x, y, yaw), yaw), Command(v, w))


def orientation(yaw, pitch):
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    # ROS quaternion xyzw for Rz(yaw) @ Ry(-pitch).
    return (sy * sp, -cy * sp, sy * cp, cy * cp)


class Score:
    """Ordered segment projection and bench thresholds, measured in sim time."""

    def __init__(self, centerline, arc, limit):
        self.arc = arc
        if len(arc) < 2 or len(arc) != len(centerline) or arc[0] != 0:
            raise ValueError("Invalid ordered centerline")
        self.segments = []
        for i, (a, b) in enumerate(zip(centerline, centerline[1:])):
            dx, dy = b[0] - a[0], b[1] - a[1]
            norm2 = dx * dx + dy * dy
            ds = arc[i + 1] - arc[i]
            if not all(map(math.isfinite, (*a[:2], *b[:2], ds))) or norm2 <= 0 or ds <= 0:
                raise ValueError(f"Invalid centerline segment {i}")
            self.segments.append((a[0], a[1], dx, dy, norm2, ds))
        self.limit = limit
        self.progress = self.last_progress = 0.0
        self.off_since = self.moved_at = None
        self.complete = False
        self.history = []

    def observe(self, now, x, y):
        low, high = self.progress - 0.35, self.progress + 0.45
        first = max(0, bisect_left(self.arc, low) - 1)
        last = min(len(self.segments), bisect_left(self.arc, high))
        best = math.inf
        candidate = self.progress
        for i in range(first, last):
            ax, ay, dx, dy, norm2, ds = self.segments[i]
            arc = self.arc[i]
            t = ((x - ax) * dx + (y - ay) * dy) / norm2
            t = max(0.0, (low - arc) / ds, min(1.0, (high - arc) / ds, t))
            error = math.hypot(x - ax - t * dx, y - ay - t * dy)
            if error < best:
                best, candidate = error, arc + t * ds
        self.history.append((now, x, y, candidate, best))
        if self.moved_at is None and candidate > 0.02:
            self.moved_at = now
        if best > 0.12:
            if self.off_since is None:
                self.off_since = now
            if now - self.off_since > 0.6 + 1e-12:
                return f"off line >0.12m at {candidate:.2f}m"
        else:
            self.off_since = None
            if candidate > self.progress + 0.003:
                self.progress, self.last_progress = candidate, now
        # Same endpoint tolerance as bench. Cap it for unusually short inputs.
        if self.progress >= self.arc[-1] - min(0.12, self.arc[-1] / 2):
            self.complete = True
            return "finished"
        if now - self.last_progress > 20:
            return f"stalled at {self.progress:.2f}m"
        if now >= self.limit:
            return "time limit"
        return None


class Replay:
    """Sample the ordered polyline at a fixed simulated speed."""

    def __init__(self, score, ramps, clearance, speed):
        if not math.isfinite(speed) or speed <= 0:
            raise ValueError("Replay speed must be finite and positive")
        self.score = score
        self.ramps = ramps
        self.clearance = clearance
        self.speed = speed

    def at_tick(self, tick):
        arc = self.score.arc
        distance = min(tick * self.speed / HZ, arc[-1])
        i = min(bisect_right(arc, distance) - 1, len(arc) - 2)
        ax, ay, dx, dy, _, length = self.score.segments[i]
        fraction = (distance - arc[i]) / length
        x, y = ax + fraction * dx, ay + fraction * dy
        yaw = math.atan2(dy, dx)
        z = self.clearance + ground_height(self.ramps, x, y)
        return Pose(x, y, z, ground_pitch(self.ramps, x, y, yaw), yaw), distance

    def observe(self, tick, pose, distance):
        score = self.score
        now = tick / HZ
        score.progress = distance
        score.history.append((now, pose.x, pose.y, distance, 0.0))
        if score.moved_at is None and distance > 0.02:
            score.moved_at = now
        if distance == score.arc[-1]:
            score.complete = True
            return "finished"
        if now >= score.limit:
            return "time limit"
        return None


def local_xy(x, y, ox, oy, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return (x - ox) * c + (y - oy) * s, -(x - ox) * s + (y - oy) * c


@dataclass(frozen=True)
class Frame:
    tick: int
    stamp_ns: int
    position: tuple
    odom_q: tuple
    twist: Command
    imu_q: tuple
    angular_velocity: tuple
    acceleration: tuple
    rgb: bytes
    truth: tuple  # world x, y, z, yaw


def read_exact(peer, size):
    buffer = bytearray(size)
    view = memoryview(buffer)
    while view:
        received = peer.recv_into(view)
        if not received:
            if len(view) == size:
                raise EOFError("Peer disconnected between packets")
            raise ValueError("Truncated TCP packet")
        view = view[received:]
    return bytes(buffer)


def send_frame(peer, frame):
    if len(frame.rgb) != FRAME_BYTES:
        raise ValueError(f"Expected {FRAME_BYTES} RGB bytes")
    peer.sendall(FRAME_HEADER.pack(
        FRAME_MAGIC, frame.tick, frame.stamp_ns, *frame.position, *frame.odom_q,
        frame.twist.v, frame.twist.w, *frame.imu_q, *frame.angular_velocity,
        *frame.acceleration, *frame.truth))
    peer.sendall(frame.rgb)


def receive_frame(peer):
    magic, tick, stamp, *values = FRAME_HEADER.unpack(read_exact(peer, FRAME_HEADER.size))
    if magic != FRAME_MAGIC or not all(map(math.isfinite, values)):
        raise ValueError("Invalid frame header")
    try:
        rgb = read_exact(peer, FRAME_BYTES)
    except EOFError as error:
        raise ValueError("Missing RGB payload") from error
    return Frame(tick, stamp, tuple(values[:3]), tuple(values[3:7]), Command(*values[7:9]),
                 tuple(values[9:13]), tuple(values[13:16]), tuple(values[16:19]), rgb,
                 tuple(values[19:23]))


def send_command(peer, tick, cmd):
    peer.sendall(COMMAND_PACKET.pack(COMMAND_MAGIC, tick, cmd.v, cmd.w))


def receive_command(peer, tick):
    magic, echoed, v, w = COMMAND_PACKET.unpack(read_exact(peer, COMMAND_PACKET.size))
    if magic != COMMAND_MAGIC or echoed != tick:
        raise ValueError("Invalid command magic or tick")
    if not math.isfinite(v) or not math.isfinite(w):
        raise ValueError("Non-finite command")
    return Command(v, w)


def drive(peer, render, start, ramps, score, *, clearance=0.0, sample=None,
          noise=None, position_std=0.0, yaw_std=0.0, replay=None, relative_odom=False):
    """Exchange one frame and one command per tick, starting at the spawn or route origin."""
    pose = start
    distance = 0.0
    applied = cmd = Command(0.0, 0.0)
    tick = 0
    while True:
        previous, prior_cmd = pose, applied
        if replay is not None:
            pose, next_distance = replay.at_tick(tick)
            if tick:
                applied = Command((next_distance - distance) / DT,
                                  math.remainder(pose.yaw - previous.yaw, math.tau) / DT)
            distance = next_distance
            if tick == 0:
                previous = pose
        elif tick:
            pose, applied = advance(pose, cmd, ramps, clearance)
        x, y, z, pitch, yaw = pose.x, pose.y, pose.z, pose.pitch, pose.yaw
        odom_yaw = yaw
        if position_std:
            x, y, z = (value + noise.gauss(0, position_std) for value in (x, y, z))
        if yaw_std:
            odom_yaw += noise.gauss(0, yaw_std)
        if relative_odom:
            x, y = local_xy(x, y, start.x, start.y, start.yaw)
            z -= start.z
            odom_yaw -= start.yaw
        rgb = render(pose, tick)
        frame = Frame(tick, round(tick * 1_000_000_000 / HZ), (x, y, z),
                      orientation(odom_yaw, 0), applied, orientation(yaw, pitch),
                      (applied.w * math.sin(pitch), (previous.pitch - pitch) / DT,
                       applied.w * math.cos(pitch)),
                      ((applied.v - prior_cmd.v) / DT + G * math.sin(pitch),
                       applied.v * applied.w, G * math.cos(pitch)), rgb,
                      (pose.x, pose.y, pose.z, pose.yaw))
        send_frame(peer, frame)
        cmd = receive_command(peer, tick)
        reason = (replay.observe(tick, pose, distance) if replay is not None
                  else score.observe(tick / HZ, pose.x, pose.y))
        if sample is not None:
            sample(tick, rgb)
        if reason:
            return reason
        tick += 1
