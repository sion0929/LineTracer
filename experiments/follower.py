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
        sc = self.score
        sc[gi, gj] = np.maximum(sc[gi, gj] - self.a.miss, -self.a.clamp)
        sc[li, lj] = np.minimum(sc[li, lj] + self.a.hit, self.a.clamp)
        for (a_, b_) in ((gi, gj), (li, lj)):
            self.grid[a_, b_] = (sc[a_, b_] > 0).astype(np.int8)
        self.frames += 1
        self.control()

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
                end_reason = 'tee'
                break
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

    # ---------------------------------------------------------------- control
    def control(self):
        x, y, yaw = self.pose
        if self.finished or time.monotonic() - self.t0 < self.a.delay:
            self.cmd = Twist()
            return
        sp = self.start_point(x, y, yaw)
        if sp is None:
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
        if len(pts) < 2:
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
        near_end = remaining < self.a.stop_dist and np.hypot(rx[-1], ry[-1]) < self.a.stop_dist
        if reason == 'tee' and near_end:
            self.get_logger().info('terminal T reached; stopping')
            self.finished = True
            self.cmd = Twist()
            return
        if reason in ('end', 'unknown') and near_end:
            # A short mapped curve can terminate at the edge of the camera view.
            # Turn toward its tangent so the continuation can enter the image.
            dx, dy = rx[-1] - rx[0], ry[-1] - ry[0]
            angle = math.atan2(dy, dx)
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
        v = self.a.vmax
        if k_ahead > 1e-3:
            v = min(v, math.sqrt(self.a.alat / k_ahead))
        if reason != 'length':
            v = min(v, math.sqrt(2 * self.a.decel * max(0.0, remaining - 0.03)))
        # lookahead target
        ld = self.a.ld_min + self.a.ld_gain * max(self.v_prev, 0.0)
        i = int(np.searchsorted(arc, ld))
        i = min(i, len(rx) - 1)
        tx, ty = rx[i], ry[i]
        alpha = math.atan2(ty, tx)
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
    p.add_argument('--vmax', type=float, default=0.22)
    p.add_argument('--wmax', type=float, default=1.5)
    p.add_argument('--alat', type=float, default=0.25)
    p.add_argument('--accel', type=float, default=0.6)
    p.add_argument('--decel', type=float, default=0.5)
    p.add_argument('--ld-min', type=float, default=0.06)
    p.add_argument('--ld-gain', type=float, default=0.4)
    p.add_argument('--preview', type=float, default=0.15)
    p.add_argument('--rotate-angle', type=float, default=1.0)
    p.add_argument('--k-rot', type=float, default=3.0)
    p.add_argument('--k-head', type=float, default=1.0)
    p.add_argument('--step', type=float, default=0.01)
    p.add_argument('--probe', type=float, default=0.03)
    p.add_argument('--horizon', type=float, default=0.8)
    p.add_argument('--stop-dist', type=float, default=0.06)
    p.add_argument('--dump', default='')
    p.add_argument('--latency', type=float, default=0.04)
    p.add_argument('--imu-yaw', type=int, default=1)
    p.add_argument('--pitch-comp', type=int, default=1)
    p.add_argument('--hit', type=int, default=3)
    p.add_argument('--waccel', type=float, default=4.0)
    p.add_argument('--decel-cmd', type=float, default=0.8)
    p.add_argument('--miss', type=int, default=1)
    p.add_argument('--clamp', type=int, default=12)
    p.add_argument('--terrain-tau', type=float, default=1.0)
    a, _ = p.parse_known_args()
    rclpy.init()
    n = Follower(a)
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        if a.dump:
            np.savez_compressed(a.dump, grid=n.grid, path=n.path if n.path is not None else np.zeros((0, 2)),
                                pose=np.array(n.pose or (0, 0, 0)), trail=np.array(n.trail), cmds=np.array(n.cmds))
        try:
            n.pub.publish(Twist())
        except Exception:
            pass


if __name__ == '__main__':
    main()
