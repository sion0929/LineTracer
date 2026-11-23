#!/usr/bin/env python3
"""Experimental map-based line follower (camera + /odom only).

Pipeline per camera frame:
  1. threshold dark pixels in the ground part of the image
  2. project pixels to the ground plane (flat-ground pinhole model) and then to the odom frame
  3. write them into a rolling odom-frame grid (latest observation wins), so the 0.3 m blind
     zone in front of the robot is filled by what the camera saw a moment ago
  4. trace the line through the grid from the robot forward, preferring the straightest
     continuation at branches (crossing -> straight, L corner -> the only branch, T -> end)
  5. pure pursuit along the traced path, speed limited by curvature ahead and remaining length
Publishes geometry_msgs/Twist on the topic given by --cmd-topic.
"""
import argparse
import math
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu

from racing_line import nearest_station, plan_path

# camera geometry (model.sdf): 0.076 m ahead of axle, 0.103 m above ground, hfov 1.085595, 640x480
CAM_X, CAM_H, W, H = 0.076, 0.103, 640, 480
F = (W / 2) / math.tan(1.085595 / 2)
CX, CY = (W - 1) / 2, (H - 1) / 2
MAX_RANGE = 0.75         # [m] forward distance used for mapping
RES = 0.005              # [m] grid cell
GRID = 1200              # cells per side (6 m)
ORIGIN = -1.5            # [m] odom coordinate of cell 0


def ground_lut():
    v0 = int(math.ceil(CY + CAM_H * F / MAX_RANGE))
    vs, us = np.mgrid[v0:H, 0:W:2]
    depth = CAM_H * F / (vs - CY)
    x = CAM_X + depth
    y = -(us - CX) * depth / F
    return v0, us.astype(np.int32), vs.astype(np.int32), x.ravel(), y.ravel()


RAY_V0 = 262
_rv, _ru = np.mgrid[RAY_V0:H, 0:W:2]
RAY_U, RAY_V = _ru.astype(np.int32), _rv.astype(np.int32)
RAYS = np.stack([np.ones(_ru.size), -(_ru.ravel() - CX) / F, -(_rv.ravel() - CY) / F])   # body frame x fwd, y left, z up


def project(pitch, roll):
    """Ground points (x fwd, y left from the axle) for every ray given body pitch (+ nose down) and roll."""
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    dx, dy, dz = RAYS
    y1 = cr * dy - sr * dz; z1 = sr * dy + cr * dz
    x2 = cp * dx + sp * z1; z2 = -sp * dx + cp * z1
    ok = z2 < -1e-3
    t = np.where(ok, CAM_H / np.maximum(-z2, 1e-3), 0)
    gx, gy = CAM_X + t * x2, t * y1
    ok &= (gx - CAM_X <= MAX_RANGE) & (gx > CAM_X)
    return gx, gy, ok


class Follower(Node):
    def __init__(self, a):
        super().__init__('experimental_follower')
        self.a = a
        self.v0, self.us, self.vs, self.lx, self.ly = ground_lut()
        self.grid = np.full((GRID, GRID), -1, np.int8)     # -1 unknown, 0 ground, 1 line (derived)
        self.score = np.zeros((GRID, GRID), np.int16)      # evidence: dark +HIT, light -MISS, clamped
        self.pose = None
        self.hist = []
        self.trail = []
        self.last_odom = None
        self.path = None
        self.gap_origin = None
        self.camera_line = None
        self.camera_tangent = None
        self.camera_points = None
        self.camera_near_support = False
        self.fresh_cells = None
        self.recovery_turn = None
        self.recovery_yaw = None
        self.recovery_path = None
        self.recovery_axis = None
        self.camera_advance = None
        self.trusted_path = None
        self.motion_anchor = None
        self.reanchors = 0
        self.cmd = Twist()
        self.v_prev = 0.0
        self.w_prev = 0.0
        self.cmds = []
        self.t0 = time.monotonic()
        self.finished = False
        self.pub = self.create_publisher(Twist, a.cmd_topic, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 50)
        self.create_subscription(Imu, '/imu', self.on_imu, 200)
        self.imu = []            # (t, unwrapped imu yaw)
        self.yaw_offset = None   # odom-frame yaw = imu yaw + offset
        self.terrain = (0.0, 0.0)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, qos_profile_sensor_data)
        self.create_timer(0.02, lambda: self.pub.publish(self.cmd))
        self.frames = 0

    # ---------------------------------------------------------------- inputs
    def on_odom(self, m):
        q = m.pose.pose.orientation
        oyaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        ox, oy = m.pose.pose.position.x, m.pose.pose.position.y
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.a.imu_yaw and not self.imu:
            return
        if self.a.imu_yaw:
            if self.yaw_offset is None:
                self.yaw_offset = oyaw - self.imu[-1][1]
            yaw = self.imu_yaw(t) + self.yaw_offset
        else:
            yaw = oyaw
        if self.pose is None or not self.a.imu_yaw:
            x, y = ox, oy
        else:
            # distance travelled from wheel odometry, direction from the gyro (odom yaw slips in turns)
            ds = (ox - self.last_odom[0]) * math.cos(self.last_odom[2]) + (oy - self.last_odom[1]) * math.sin(self.last_odom[2])
            mid = self.pose[2] + 0.5 * math.atan2(math.sin(yaw - self.pose[2]), math.cos(yaw - self.pose[2]))
            x, y = self.pose[0] + ds * math.cos(mid), self.pose[1] + ds * math.sin(mid)
        self.last_odom = (ox, oy, oyaw)
        self.pose = (x, y, yaw)
        self.hist.append((t, *self.pose))
        self.trail.append((t, *self.pose))
        if len(self.hist) > 200:
            del self.hist[:100]

    def on_imu(self, m):
        q = m.orientation
        y = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if self.imu:
            prev = self.imu[-1][1]
            y = prev + math.atan2(math.sin(y - prev), math.cos(y - prev))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.imu:
            k = min(1.0, (t - self.imu[-1][0]) / self.a.terrain_tau)
            self.terrain = (self.terrain[0] + k * (pitch - self.terrain[0]), self.terrain[1] + k * (roll - self.terrain[1]))
        else:
            self.terrain = (pitch, roll)
        self.imu.append((t, y, pitch - self.terrain[0], roll - self.terrain[1]))
        if len(self.imu) > 1000:
            del self.imu[:500]

    def imu_yaw(self, t):
        h = self.imu
        if t >= h[-1][0]:
            return h[-1][1]
        for k in range(len(h) - 1, 0, -1):
            if h[k - 1][0] <= t:
                r = (t - h[k - 1][0]) / max(h[k][0] - h[k - 1][0], 1e-6)
                return h[k - 1][1] + r * (h[k][1] - h[k - 1][1])
        return h[0][1]

    def tilt_at(self, t):
        """Dynamic (high-passed) pitch/roll at time t."""
        if not self.imu or not self.a.pitch_comp:
            return 0.0, 0.0
        best = min(self.imu[-60:], key=lambda e: abs(e[0] - t))
        return best[2], best[3]

    def pose_at(self, t):
        """Odometry pose interpolated at time t (capture stamp of an image)."""
        h = self.hist
        if not h or t >= h[-1][0]:
            return self.pose
        for k in range(len(h) - 1, 0, -1):
            if h[k - 1][0] <= t:
                t0, x0, y0, a0 = h[k - 1]; t1, x1, y1, a1 = h[k]
                r = (t - t0) / max(t1 - t0, 1e-6)
                da = math.atan2(math.sin(a1 - a0), math.cos(a1 - a0))
                return x0 + r * (x1 - x0), y0 + r * (y1 - y0), a0 + r * da
        return h[0][1:]

    def on_image(self, msg):
        if self.pose is None:
            return
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)
        gray = img[..., :3].mean(axis=2) if img.shape[2] >= 3 else img[..., 0]
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pitch, roll = self.tilt_at(stamp)
        gx, gy, ok0 = project(pitch, roll)
        dark = (gray[RAY_V, RAY_U].ravel() < self.a.dark)[ok0]
        lx, ly = gx[ok0], gy[ok0]
        x, y, yaw = self.pose_at(stamp - self.a.latency)
        c, s = math.cos(yaw), math.sin(yaw)
        wx = x + c * lx - s * ly
        wy = y + s * lx + c * ly
        ix = ((wx - ORIGIN) / RES).astype(np.int32)
        iy = ((wy - ORIGIN) / RES).astype(np.int32)
        ok = (ix >= 0) & (ix < GRID) & (iy >= 0) & (iy < GRID)
        gi, gj = iy[ok & ~dark], ix[ok & ~dark]
        li, lj = iy[ok & dark], ix[ok & dark]
        if self.a.reactive_recovery:
            self.fresh_cells = (gi, gj, li, lj)
            line_x, line_y = lx[dark], ly[dark]
            self.camera_points = (line_x, line_y)
            self.camera_line, self.camera_tangent = self.visible_line(line_x, line_y)
            near_line = (line_x > 0.29) & (line_x < 0.35) & (np.abs(line_y) < 0.075)
            self.camera_near_support = np.count_nonzero(near_line) >= 16
        sc = self.score
        sc[gi, gj] = np.maximum(sc[gi, gj] - self.a.miss, -self.a.clamp)
        sc[li, lj] = np.minimum(sc[li, lj] + self.a.hit, self.a.clamp)
        for (a_, b_) in ((gi, gj), (li, lj)):
            self.grid[a_, b_] = (sc[a_, b_] > 0).astype(np.int8)
        self.frames += 1
        self.control()

    @staticmethod
    def visible_line(x, y):
        """Select the nearest band of projected line pixels for camera-only steering."""
        visible = (x > 0.24) & (x < 0.65) & (abs(y) < 0.30)
        if np.count_nonzero(visible) < 15:
            return None, None
        x, y = x[visible], y[visible]
        near = x <= np.quantile(x, 0.20) + 0.05
        if np.count_nonzero(near) < 10:
            return None, None
        target = (float(np.median(x[near])), float(np.median(y[near])))
        points = np.column_stack((x, y))
        local = points[np.linalg.norm(points - target, axis=1) < 0.12]
        if len(local) < 10:
            return target, None
        values, vectors = np.linalg.eigh(np.cov(local, rowvar=False))
        tangent = vectors[:, -1] if values[-1] > max(0.0003, 5 * values[0]) else None
        return target, tangent

    def reanchor_local_map(self):
        """Replace stale local evidence with the latest camera projection."""
        if self.fresh_cells is None:
            return
        x, y, _ = self.pose
        ix, iy = math.floor((x - ORIGIN) / RES), math.floor((y - ORIGIN) / RES)
        radius = math.ceil(0.34 / RES)
        x0, x1 = max(0, ix - radius), min(GRID, ix + radius + 1)
        y0, y1 = max(0, iy - radius), min(GRID, iy + radius + 1)
        if x0 < x1 and y0 < y1:
            dx = ORIGIN + (np.arange(x0, x1) + 0.5) * RES - x
            dy = ORIGIN + (np.arange(y0, y1) + 0.5) * RES - y
            local = dy[:, None] ** 2 + dx ** 2 <= 0.34 ** 2
            self.score[y0:y1, x0:x1][local] = 0
            self.grid[y0:y1, x0:x1][local] = -1
        gi, gj, li, lj = self.fresh_cells
        self.score[gi, gj] = -self.a.miss
        self.grid[gi, gj] = 0
        self.score[li, lj] = self.a.hit
        self.grid[li, lj] = 1
        self.path = None
        self.reanchors += 1

    def stalled(self):
        x, y, yaw = self.pose
        now = time.monotonic()
        if self.motion_anchor is not None:
            start_x, start_y, start_yaw, since = self.motion_anchor
            turned = abs(math.remainder(yaw - start_yaw, 2 * math.pi)) >= 0.16
            if math.hypot(x - start_x, y - start_y) < 0.018 and not turned:
                return now - since > 1.25
        self.motion_anchor = (x, y, yaw, now)
        return False

    def react_to_camera(self, turn_hint=0.0, path=None):
        if self.recovery_turn is None:
            self.recovery_turn = math.copysign(1.0, turn_hint or self.w_prev or 1.0)
            self.recovery_yaw = self.pose[2]
            self.camera_advance = None
            if path is not None and math.dist(self.pose[:2], path[-1]) < 0.10:
                heading = self.pose[2] + turn_hint
                if len(path) >= 3:
                    self.recovery_axis = (path[0], heading)
                tail = self.gap_path(path[-1], self.pose[2] + turn_hint)
                if tail is not None and math.dist(self.pose[:2], tail[0]) <= 0.20:
                    self.recovery_path = tail
            if self.recovery_path is None and self.a.racing_line:
                self.recovery_path = self.cached_recovery_path()
                if self.recovery_path is not None:
                    self.trusted_path = None
            self.reanchor_local_map()
        limit = min(self.a.wmax, 0.5)
        if self.recovery_path is not None and self.follow_recovery_path(limit):
            self.camera_advance = None
            return
        x, y, yaw = self.pose
        yaw_from_start = math.remainder(yaw - self.recovery_yaw, 2 * math.pi)
        camera_angle = math.atan2(self.camera_line[1], self.camera_line[0]) if self.camera_line is not None else None
        exhausted = self.camera_advance is not None and math.dist((x, y), self.camera_advance[:2]) >= 0.075
        # A distant line can still look centered after the local line turns out of view.
        same_direction = exhausted and camera_angle is not None and abs(
            math.remainder(yaw + camera_angle - self.camera_advance[2], 2 * math.pi)
        ) < 0.42
        sector = 1.60 if exhausted else 1.35
        if camera_angle is None or same_direction or abs(yaw_from_start + camera_angle) > sector:
            if yaw_from_start * self.recovery_turn >= 1.10:
                self.recovery_turn = -self.recovery_turn
            self.set_motion(0.0, self.recovery_turn * min(limit, 0.34))
            return
        if self.recovery_axis is not None and self.camera_tangent is not None:
            approach = self.corner_approach()
            if approach is not None:
                self.recovery_axis = None
                self.recovery_path = approach
                if self.follow_recovery_path(limit):
                    self.camera_advance = None
                    return
        if abs(camera_angle) < 0.25 and (self.camera_advance is None or exhausted or self.camera_near_support):
            self.camera_advance = (x, y, yaw + camera_angle)
        self.steer_recovery(camera_angle, limit, 0.045, 0.25)

    def corner_approach(self):
        """Intersect the last mapped approach with the visible exit of a corner."""
        x, y, yaw = self.pose
        origin, heading = self.recovery_axis
        incoming = np.array((math.cos(heading), math.sin(heading)))
        rotation = np.array(((math.cos(yaw), -math.sin(yaw)), (math.sin(yaw), math.cos(yaw))))
        outgoing = rotation @ self.camera_tangent
        cross = incoming[0] * outgoing[1] - incoming[1] * outgoing[0]
        if abs(cross) < 0.55:
            return None
        observed = np.array((x, y)) + rotation @ self.camera_line - origin
        along = (observed[0] * outgoing[1] - observed[1] * outgoing[0]) / cross
        current = np.dot((x, y) - origin, incoming)
        distance = along - current
        if not 0.065 <= distance <= 0.36:
            return None
        start = origin + current * incoming
        return np.linspace(start, start + (distance - 0.025) * incoming,
                           math.ceil(distance / self.a.step))

    def camera_continues(self):
        """Check for a longitudinal line beyond a mapped terminal bar."""
        if not self.camera_near_support or self.camera_line is None:
            return False
        target_x, target_y = self.camera_line
        if abs(math.atan2(target_y, target_x)) >= 0.25:
            return False
        if self.camera_tangent is not None and abs(self.camera_tangent[0]) > 0.85:
            return True
        # A crossing can obscure the local tangent. Require the same forward
        # stripe to appear in three separated camera distance bands.
        if self.camera_points is None:
            return False
        x, y = self.camera_points
        slopes = np.linspace(-0.45, 0.45, 19)
        supported = np.ones(len(slopes), dtype=bool)
        for low, high in ((0.29, 0.35), (0.37, 0.43), (0.47, 0.53)):
            band = (x > low) & (x < high)
            expected = target_y + slopes[:, None] * (x[band] - target_x)
            supported &= np.count_nonzero(np.abs(y[band] - expected) < 0.027, axis=1) >= 16
        return bool(supported.any())

    def cached_recovery_path(self):
        """Keep a short, ordered piece of the last local path through a blind turn."""
        if self.trusted_path is None:
            return None
        x, y, yaw = self.pose
        count = math.ceil(0.22 / self.a.step)
        nearby = self.trusted_path[:count] - (x, y)
        distance = np.hypot(nearby[:, 0], nearby[:, 1])
        i = int(np.argmin(distance))
        path = self.trusted_path[i:i + count + 1]
        if distance[i] > 0.09 or len(path) < 6:
            return None
        dx, dy = path[5] - (x, y)
        if abs(math.remainder(math.atan2(dy, dx) - yaw, 2 * math.pi)) > 1.25:
            return None
        return path

    def follow_recovery_path(self, limit):
        x, y, yaw = self.pose
        path = self.recovery_path
        nearby = path[:12] - (x, y)
        distance = np.hypot(nearby[:, 0], nearby[:, 1])
        i = int(np.argmin(distance))
        if distance[i] < 0.065:
            path = self.recovery_path = path[i:]
            if len(path) == 1 and distance[i] <= 0.025:
                self.recovery_path = None
                return False
            target = path[min(4, len(path) - 1)]
        else:
            target = path[0]
        dx, dy = target - (x, y)
        angle = math.remainder(math.atan2(dy, dx) - yaw, 2 * math.pi)
        self.steer_recovery(angle, limit, min(0.05, math.hypot(dx, dy) * 0.8), 0.60)
        return True

    def steer_recovery(self, angle, limit, speed, max_angle):
        w = max(-limit, min(limit, 1.8 * angle))
        if abs(angle) > 0.10:
            self.recovery_turn = math.copysign(1.0, angle)
        v = min(speed, self.a.vmax) if abs(angle) < max_angle else 0.0
        self.set_motion(v, w)

    # ---------------------------------------------------------------- tracing
    def line_at(self, px, py):
        ix = int((px - ORIGIN) / RES)
        iy = int((py - ORIGIN) / RES)
        if 0 <= ix < GRID and 0 <= iy < GRID:
            return self.grid[iy, ix]
        return -1

    def snap(self, px, py, radius=0.03):
        """Move a point onto the centroid of nearby line cells (None if no line within radius)."""
        n = int(radius / RES)
        ix, iy = int((px - ORIGIN) / RES), int((py - ORIGIN) / RES)
        if not (n <= ix < GRID - n and n <= iy < GRID - n):
            return None
        win = self.grid[iy - n:iy + n + 1, ix - n:ix + n + 1] == 1
        if not win.any():
            return None
        yy, xx = np.nonzero(win)
        return ORIGIN + (ix - n + xx.mean() + .5) * RES, ORIGIN + (iy - n + yy.mean() + .5) * RES

    def start_point(self, x, y, yaw):
        """Where the trace starts: the previous path point nearest to the robot (snapped onto the
        line), else the first line cell found in a forward cone."""
        if self.path is not None and len(self.path) > 1:
            d = np.hypot(self.path[:, 0] - x, self.path[:, 1] - y)
            i = int(np.argmin(d))
            if d[i] < 0.15:
                j = min(i + 2, len(self.path) - 1)
                k = max(j - 2, 0)
                h = math.atan2(self.path[j, 1] - self.path[k, 1], self.path[j, 0] - self.path[k, 0])
                s = self.snap(self.path[i, 0], self.path[i, 1])
                if s is not None:
                    return s[0], s[1], h
        for r in np.arange(0.0, 0.7, RES):
            for off in np.arange(0.0, 0.16, 0.01):
                for sgn in (1, -1):
                    px = x + r * math.cos(yaw) - sgn * off * math.sin(yaw)
                    py = y + r * math.sin(yaw) + sgn * off * math.cos(yaw)
                    if self.line_at(px, py) == 1:
                        s = self.snap(px, py) or (px, py)
                        return s[0], s[1], yaw
        return None

    def trace(self, px, py, h, probe=None):
        step = self.a.step
        r = self.a.probe if probe is None else probe
        angles = np.radians(np.arange(-105, 106, 5))
        pts = [(px, py)]
        end_reason = 'length'
        total = 0.0
        while total < self.a.horizon:
            hits = []
            for da in angles:
                st = self.line_at(px + r * math.cos(h + da), py + r * math.sin(h + da))
                hits.append(st)
            hits = np.array(hits)
            if (hits == -1).sum() > len(hits) // 2:
                end_reason = 'unknown'
                break
            on = hits == 1
            clusters, cur = [], []
            for da, o in zip(angles, on):
                if o:
                    cur.append(da)
                elif cur:
                    clusters.append(cur); cur = []
            if cur:
                clusters.append(cur)
            if not clusters:
                end_reason = 'end'
                break
            centers = [float(np.mean(cl)) for cl in clusters]
            best = min(centers, key=abs)
            if abs(best) > math.radians(25) and self.tee_arms(px, py, h, clusters, centers):
                tail_x, tail_y = pts[max(0, len(pts) - 9)]
                incoming = math.atan2(py - tail_y, px - tail_x) if len(pts) > 1 else h
                continuation = self.forward_continuation(px, py, incoming, angles)
                if continuation is None:
                    if self.transverse_bar(px, py, incoming, angles):
                        end_reason = 'tee'
                        break
                else:
                    best = math.remainder(incoming + continuation - h, 2 * math.pi)
            h = h + best
            px, py = px + step * math.cos(h), py + step * math.sin(h)
            pts.append((px, py))
            total += step
        return np.array(pts), end_reason

    def tee_arms(self, px, py, heading, clusters, centers):
        """Check that both sides extend past short line artifacts at corners."""
        support = set()
        reach = 0.065
        for cluster, center in zip(clusters, centers):
            if abs(center) <= math.radians(25):
                continue
            if any(self.line_at(px + reach * math.cos(heading + angle),
                                py + reach * math.sin(heading + angle)) == 1 for angle in cluster):
                support.add(math.copysign(1, center))
        return len(support) == 2

    def forward_continuation(self, px, py, heading, angles):
        """Check for a straight arm beyond a noisy intersection."""
        forward = angles[abs(angles) <= math.radians(25)]
        hits = []
        for reach in (0.065, 0.11):
            hits = [offset for offset in forward
                    if self.line_at(px + reach * math.cos(heading + offset),
                                    py + reach * math.sin(heading + offset)) == 1]
            if not hits:
                return None
        return min(hits, key=abs)

    def transverse_bar(self, px, py, heading, angles):
        """Require evidence on both sides of the incoming line."""
        side = angles[angles >= math.radians(55)]
        reach = 0.075
        return all(any(self.line_at(px + reach * math.cos(heading + sign * offset),
                                    py + reach * math.sin(heading + sign * offset)) == 1
                       for offset in side) for sign in (-1, 1))

    def gap_path(self, endpoint, heading):
        """Find a nearby fragment with enough mapped line to continue tracing."""
        end_x, end_y = endpoint
        angles = np.radians([0, 10, -10, 20, -20, 30, -30, 40, -40, 50, -50, 60, -60])
        for radius in (0.06, 0.085, 0.11, 0.135, 0.16):
            for offset in angles:
                bearing = heading + offset
                px = end_x + radius * math.cos(bearing)
                py = end_y + radius * math.sin(bearing)
                if self.line_at(px, py) != 1:
                    continue
                seed = self.snap(px, py, radius=0.015)
                if seed is None:
                    continue
                tail, _ = self.trace(*seed, bearing)
                if len(tail) > math.ceil(0.09 / self.a.step) and np.linalg.norm(tail[-1] - tail[0]) > 0.055:
                    return tail
        return None

    def recover_gap(self, pts, heading):
        x, y, yaw = self.pose
        if self.gap_origin is not None and math.dist((x, y), self.gap_origin) >= 0.14:
            return False
        tail = self.gap_path(pts[-1], heading)
        if tail is None:
            return False
        target = tail[0]
        distance = math.dist((x, y), target)
        if not 0.025 < distance <= 0.20:
            return False
        if self.gap_origin is None:
            self.gap_origin = (x, y)
        bearing = math.atan2(target[1] - y, target[0] - x)
        angle = math.remainder(bearing - yaw, 2 * math.pi)
        limit = min(self.a.wmax, 0.5)
        angular = max(-limit, min(limit, 1.5 * angle))
        speed = min(self.a.vmax, 0.045, distance * 0.5) if abs(angle) < 0.65 else 0.0
        self.set_motion(speed, angular)
        return True

    # ---------------------------------------------------------------- control
    def control(self):
        x, y, yaw = self.pose
        if self.finished or time.monotonic() - self.t0 < self.a.delay:
            self.cmd = Twist()
            return
        sp = self.start_point(x, y, yaw)
        if sp is None:
            if self.a.reactive_recovery and self.frames > 5:
                self.react_to_camera()
                return
            self.cmd = Twist()
            return
        pts, reason = self.trace(*sp)
        min_steps = math.ceil(self.a.stop_dist / self.a.step)
        if reason in ('end', 'unknown') and len(pts) <= min_steps:
            heading_error = math.remainder(yaw - sp[2], 2 * math.pi)
            if 1e-3 < abs(heading_error) < 0.35:
                # A small heading change can shift arc samples across sparse cells.
                retry, retry_reason = self.trace(sp[0], sp[1], yaw)
                if len(retry) > min_steps:
                    pts, reason = retry, retry_reason
            if len(pts) <= min_steps and math.hypot(sp[0] - x, sp[1] - y) < 0.08:
                # Nearby fragments can have a small gap after wheel slip.
                # Use a wider probe only when the usual trace cannot reach the lookahead.
                for probe in (0.045, 0.08):
                    if probe <= self.a.probe:
                        continue
                    retry, retry_reason = self.trace(*sp, probe)
                    if len(retry) > min_steps:
                        pts, reason = retry, retry_reason
                        break
        if (reason == 'tee' and self.a.reactive_recovery
                and (len(pts) - 1) * self.a.step <= self.a.stop_dist
                and math.dist((x, y), pts[-1]) < self.a.stop_dist
                and self.camera_continues()):
            self.recovery_path = None
            self.react_to_camera()
            return
        if len(pts) < 2:
            if reason == 'tee' and math.hypot(sp[0] - x, sp[1] - y) < self.a.stop_dist:
                self.finished = True
            elif self.a.reactive_recovery:
                self.react_to_camera(math.remainder(sp[2] - yaw, 2 * math.pi))
                return
            self.cmd = Twist()
            return
        self.path = pts
        # robot-frame path
        c, s = math.cos(yaw), math.sin(yaw)
        rx = c * (pts[:, 0] - x) + s * (pts[:, 1] - y)
        ry = -s * (pts[:, 0] - x) + c * (pts[:, 1] - y)
        seg = np.hypot(np.diff(rx), np.diff(ry))
        arc = np.r_[0, np.cumsum(seg)]
        remaining = arc[-1]
        racing = self.a.racing_line and self.racing_supported(pts, arc)
        lookahead_speed = min(self.v_prev, 0.10) if self.a.racing_line else self.v_prev
        ld = self.a.ld_min + self.a.ld_gain * max(lookahead_speed, 0.0)
        i = min(int(np.searchsorted(arc, ld)), len(rx) - 1)
        tx, ty = rx[i], ry[i]
        alpha = math.atan2(ty, tx)
        if remaining >= 0.12:
            self.gap_origin = None
        near_end = remaining < self.a.stop_dist and np.hypot(rx[-1], ry[-1]) < self.a.stop_dist
        if reason == 'tee' and near_end:
            self.get_logger().info('terminal T reached; stopping')
            self.finished = True
            self.cmd = Twist()
            return
        if self.a.reactive_recovery:
            stalled = self.stalled() if self.recovery_turn is None else False
            support_radius = self.a.racing_corridor + 0.02 if racing else 0.065
            supported = remaining >= 0.16 and np.hypot(rx[0], ry[0]) < support_radius
            turn_too_far = abs(alpha) > self.a.rotate_angle
            lost = stalled or near_end or turn_too_far or (self.recovery_turn is not None and not supported)
            if reason != 'tee' and lost:
                hint = math.atan2(ry[-1] - ry[0], rx[-1] - rx[0])
                self.react_to_camera(alpha if turn_too_far else hint, None if turn_too_far else pts)
                return
            if self.recovery_turn is not None:
                self.recovery_turn = None
                self.recovery_yaw = None
                self.recovery_path = None
                self.recovery_axis = None
                self.motion_anchor = None
        if self.a.racing_line and remaining >= 0.16 and np.hypot(rx[0], ry[0]) < 0.09 and reason != 'tee':
            self.trusted_path = pts
        racing_lookahead = self.a.ld_min + self.a.ld_gain * max(self.v_prev, 0.0)
        if racing and self.track_racing(pts, racing_lookahead):
            return
        guarded = self.a.racing_line
        if reason in ('end', 'unknown') and near_end:
            dx, dy = rx[-1] - rx[0], ry[-1] - ry[0]
            angle = math.atan2(dy, dx)
            if self.a.gap_recovery and self.recover_gap(pts, yaw + angle):
                return
            # A short mapped curve can terminate at the edge of the camera view.
            # Turn toward its tangent so the continuation can enter the image.
            w = 0.0
            if math.hypot(dx, dy) >= 0.02 and 0.15 <= abs(angle) <= 1.2:
                w = max(-0.4, min(0.4, 1.5 * angle))
            self.set_motion(0.0, w)
            return
        # curvature ahead from heading change over the next window
        hd = np.unwrap(np.arctan2(np.diff(ry), np.diff(rx)))
        k_ahead = 0.0
        win = max(1, int(self.a.preview / self.a.step))
        if len(hd) > win:
            k_ahead = float(np.max(np.abs(hd[win:] - hd[:-win]))) / self.a.preview
        elif len(hd) > 1:
            k_ahead = abs(hd[-1] - hd[0]) / max(arc[-1], 1e-3)
        v = min(self.a.vmax, 0.10) if guarded else self.a.vmax
        lateral_accel = min(self.a.alat, 0.10) if guarded else self.a.alat
        if k_ahead > 1e-3:
            v = min(v, math.sqrt(lateral_accel / k_ahead))
        if k_ahead >= 3.0:
            v = min(v, 0.10)
        if reason != 'length':
            v = min(v, math.sqrt(2 * self.a.decel * max(0.0, remaining - 0.03)))
        if abs(alpha) > self.a.rotate_angle:
            v = 0.0
            w = math.copysign(min(self.a.wmax, self.a.k_rot * abs(alpha)), alpha)
        else:
            v = min(v, self.v_prev + self.a.accel * 0.033)
            l2 = tx * tx + ty * ty
            curv = 2 * ty / max(l2, 1e-4)
            w = max(-self.a.wmax, min(self.a.wmax, v * curv + self.a.k_head * alpha * (v < 0.02)))
            if abs(w) >= self.a.wmax:
                v = min(v, self.a.wmax / max(abs(curv), 1e-3))
        self.set_motion(v, w)

    @staticmethod
    def racing_supported(points, arc):
        """Keep the proven centerline controller near abrupt turns or a short map."""
        if arc[-1] < 0.24:
            return False
        stations = np.linspace(0.0, 0.24, 4)
        sample = np.column_stack((np.interp(stations, arc, points[:, 0]),
                                  np.interp(stations, arc, points[:, 1])))
        segments = np.diff(sample, axis=0)
        heading = np.unwrap(np.arctan2(segments[:, 1], segments[:, 0]))
        return bool(np.max(np.abs(np.diff(heading))) < 0.24)

    def track_racing(self, centerline, lookahead):
        plan = plan_path(
            centerline, corridor=self.a.racing_corridor, vmax=self.a.vmax,
            lateral_accel=self.a.alat, accel=self.a.accel, braking=self.a.decel_cmd,
            wmax=self.a.wmax, initial_speed=self.v_prev, corner_vmax=0.10,
            reaction_time=max(0.0, self.a.latency) + 1 / 30,
        )
        x, y, yaw = self.pose
        station = nearest_station(plan, (x, y), prefix=0.10)
        target_s = min(station + lookahead, plan.arc[-1])
        tx = np.interp(target_s, plan.arc, plan.points[:, 0]) - x
        ty = np.interp(target_s, plan.arc, plan.points[:, 1]) - y
        forward = math.cos(yaw) * tx + math.sin(yaw) * ty
        lateral = -math.sin(yaw) * tx + math.cos(yaw) * ty
        angle = math.atan2(lateral, forward)
        if abs(angle) > self.a.rotate_angle:
            return False
        curvature = 2 * lateral / max(tx * tx + ty * ty, 1e-4)
        magnitude = max(abs(curvature), 1e-6)
        v = min(float(np.interp(station, plan.arc, plan.speed)), self.a.wmax / magnitude,
                math.sqrt(self.a.alat / magnitude))
        if magnitude >= 3.0:
            v = min(v, 0.10)
        self.set_motion(v, v * curvature)
        return True

    def set_motion(self, v, w):
        # Rate limits apply to both tracking and recovery commands.
        dt = 1.0 / 30
        v = min(v, self.v_prev + self.a.accel * dt); v = max(v, self.v_prev - self.a.decel_cmd * dt)
        w = min(w, self.w_prev + self.a.waccel * dt); w = max(w, self.w_prev - self.a.waccel * dt)
        self.v_prev, self.w_prev = v, w
        self.cmd = Twist()
        self.cmd.linear.x = float(v)
        self.cmd.angular.z = float(w)
        self.cmds.append((self.pose[0], self.pose[1], v, w))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--cmd-topic', default='/cmd_vel')
    p.add_argument('--delay', type=float, default=0.0)
    p.add_argument('--dark', type=float, default=80)
    p.add_argument('--vmax', type=float, default=0.10)
    p.add_argument('--wmax', type=float, default=0.8)
    p.add_argument('--alat', type=float, default=0.10)
    p.add_argument('--accel', type=float, default=0.30)
    p.add_argument('--decel', type=float, default=0.5)
    p.add_argument('--ld-min', type=float, default=0.05)
    p.add_argument('--ld-gain', type=float, default=0.2)
    p.add_argument('--preview', type=float, default=0.15)
    p.add_argument('--rotate-angle', type=float, default=1.0)
    p.add_argument('--k-rot', type=float, default=3.0)
    p.add_argument('--k-head', type=float, default=1.0)
    p.add_argument('--step', type=float, default=0.01)
    p.add_argument('--probe', type=float, default=0.03)
    p.add_argument('--horizon', type=float, default=0.8)
    p.add_argument('--stop-dist', type=float, default=0.06)
    p.add_argument('--gap-recovery', action='store_true',
                   help='try a bounded low-speed approach to a supported nearby line fragment')
    p.add_argument('--reactive-recovery', action=argparse.BooleanOptionalAction, default=True,
                   help='refresh the local map and steer using the latest camera observation when tracking fails')
    p.add_argument('--racing-line', action='store_true',
                   help='plan a low-time trajectory inside the mapped line corridor')
    p.add_argument('--racing-corridor', type=float, default=0.10)
    p.add_argument('--dump', default='')
    p.add_argument('--latency', type=float, default=0.04)
    p.add_argument('--imu-yaw', type=int, default=1)
    p.add_argument('--pitch-comp', type=int, default=1)
    p.add_argument('--hit', type=int, default=3)
    p.add_argument('--waccel', type=float, default=1.6)
    p.add_argument('--decel-cmd', type=float, default=0.40)
    p.add_argument('--miss', type=int, default=1)
    p.add_argument('--clamp', type=int, default=12)
    p.add_argument('--terrain-tau', type=float, default=1.0)
    a, _ = p.parse_known_args()
    if not 0.0 < a.racing_corridor <= 0.10:
        p.error('--racing-corridor must be in (0, 0.10] m')
    if a.racing_line and min(a.vmax, a.alat, a.accel, a.decel_cmd, a.wmax) <= 0.0:
        p.error('racing-line speed, acceleration, braking and angular limits must be positive')
    rclpy.init()
    n = Follower(a)
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        if a.dump:
            np.savez_compressed(a.dump, grid=n.grid, path=n.path if n.path is not None else np.zeros((0, 2)),
                                pose=np.array(n.pose or (0, 0, 0)), trail=np.array(n.trail), cmds=np.array(n.cmds),
                                reanchors=n.reanchors)
        try:
            n.pub.publish(Twist())
        except Exception:
            pass


if __name__ == '__main__':
    main()
