"""Line follower node.

camera image --(vision)--> ground line mask --(line_map)--> line map in the odom frame
line map --(trace)--> path ahead of the robot --(pure pursuit)--> /cmd_vel

Only robot topics are used: /camera/image_raw, /odom, /cmd_vel.
"""
import math
import numpy as np
import cv2
import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange
from sensor_msgs.msg import Image, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from . import vision
from .line_map import LineMap, RES, C0
from .perception import make_perception
from .control import make_controller


REST_PITCH = math.radians(0.33)   # nose-up tilt of the robot standing on flat ground


def rng(lo, hi, desc=''):
    return ParameterDescriptor(
        description=desc,
        floating_point_range=[FloatingPointRange(from_value=lo, to_value=hi, step=0.0)])


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def path_len(pts):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts[:-1], pts[1:]))


class FollowerNode(Node):
    def __init__(self):
        super().__init__('follower_node')
        self.declare_parameter('v_max', 0.25, rng(0.05, 1.5, 'max linear speed [m/s]'))
        self.declare_parameter('v_min', 0.06, rng(0.02, 0.5, 'min linear speed [m/s]'))
        self.declare_parameter('a_lat', 0.25, rng(0.05, 3.0, 'allowed lateral acceleration in curves [m/s^2]'))
        self.declare_parameter('a_dec', 0.3, rng(0.1, 1.0, 'braking deceleration before curves [m/s^2]'))
        self.declare_parameter('a_acc', 0.6, rng(0.1, 1.0, 'acceleration [m/s^2]'))
        self.declare_parameter('v_hill', 0.25, rng(0.05, 1.5, 'max linear speed on the hill [m/s]'))
        self.declare_parameter('omega_max', 3.0, rng(0.5, 8.0, 'max angular speed [rad/s]'))
        self.declare_parameter('lookahead_min', 0.10, rng(0.04, 0.5, 'pure pursuit lookahead at zero speed [m]'))
        self.declare_parameter('lookahead_gain', 0.30, rng(0.0, 2.0, 'lookahead growth per m/s [s]'))
        self.declare_parameter('finish_min_dist', 10.0, rng(0.0, 30.0, 'travel before the finish T is accepted [m]'))
        # Track-specific speed plan: [s_from, s_to, v_max, ...] with s = distance driven [m].
        # Outside these windows v_max applies. Tuned for the provided line_track world.
        self.declare_parameter('speed_profile', [0.0], ParameterDescriptor(
            description='flat list of (s_from, s_to, v) triples, s = distance driven [m]'))
        self.declare_parameter('debug_image', False)
        # plugins: who sees the line, who drives (see TEAM_GUIDE.md)
        self.declare_parameter('perception', 'rule')    # rule | dl
        self.declare_parameter('controller', 'rule')    # rule | learned

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.dbg_pub = self.create_publisher(Image, '/line_follower/debug', 1)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data)

        self.map = LineMap()
        self.pose = None            # latest (x, y, yaw) in odom
        self.hist = []              # (stamp, pose) to look up the pose at image time
        self.imu_yaw = None
        self.yaw_off = None
        self.odom_xy, self.odom_yaw = None, 0.0
        self.imu = []               # (stamp, nose-up pitch [rad])
        self.rock = 0.0             # filtered |pitch rate| [rad/s]
        self.pitch_f = 0.0          # lightly filtered pitch, used to notice the hill
        self.travel = 0.0
        self.path = []              # line ahead of the robot, odom frame, ~2 cm spacing
        self.path_end = 'open'
        self.t_point = None         # finish T position (odom)
        self.done = False
        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self.blind = 0.0            # distance driven without any path
        self.last_t = None
        self.last_obstacle = None   # latest BEV obstacle mask from the perception plugin (or None)
        self.last_observation = None  # latest PerceptionOutput (line, valid, obstacle) + .pitch
        self.perception = make_perception(self.get_parameter('perception').value, self)
        self.controller = make_controller(self.get_parameter('controller').value, self)
        self.create_timer(0.02, self.control)
        self.get_logger().info(
            f"line follower started (perception={self.get_parameter('perception').value}, "
            f"controller={self.get_parameter('controller').value})")

    # ---- frames --------------------------------------------------------
    def to_odom(self, x, y, pose=None):
        px, py, th = pose or self.pose
        return (px + x * math.cos(th) - y * math.sin(th), py + x * math.sin(th) + y * math.cos(th))

    def to_robot(self, X, Y, pose=None):
        px, py, th = pose or self.pose
        dx, dy = X - px, Y - py
        return (dx * math.cos(th) + dy * math.sin(th), -dx * math.sin(th) + dy * math.cos(th))

    def pose_at(self, ts):
        """Odometry pose interpolated at time ts."""
        h = self.hist
        if not h:
            return self.pose
        if ts >= h[-1][0]:
            # image newer than the last odometry message: extrapolate with the command
            dt = min(0.1, ts - h[-1][0])
            x, y, th = h[-1][1]
            th2 = th + self.w_cmd * dt
            return (x + self.v_cmd * dt * math.cos(th + 0.5 * self.w_cmd * dt),
                    y + self.v_cmd * dt * math.sin(th + 0.5 * self.w_cmd * dt), wrap(th2))
        for (t0, p0), (t1, p1) in zip(reversed(h[:-1]), reversed(h[1:])):
            if t0 <= ts <= t1:
                u = (ts - t0) / max(1e-6, t1 - t0)
                return (p0[0] + u * (p1[0] - p0[0]), p0[1] + u * (p1[1] - p0[1]),
                        wrap(p0[2] + u * wrap(p1[2] - p0[2])))
        return h[0][1]

    def on_odom(self, msg):
        """Dead reckoning: distance from the wheel odometry, heading from the IMU.

        The wheels slip on the hill, which corrupts the heading of /odom; the IMU does not.
        """
        p = msg.pose.pose
        q = p.orientation
        oyaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if self.pose is None:
            self.pose = (p.position.x, p.position.y, oyaw)
            self.yaw_off = None
        else:
            dx, dy = p.position.x - self.odom_xy[0], p.position.y - self.odom_xy[1]
            ds = math.hypot(dx, dy)
            if dx * math.cos(oyaw) + dy * math.sin(oyaw) < 0.0:
                ds = -ds
            if self.imu_yaw is not None:
                if self.yaw_off is None:
                    self.yaw_off = wrap(self.pose[2] - self.imu_yaw)
                yaw = wrap(self.imu_yaw + self.yaw_off)
            else:
                yaw = wrap(self.pose[2] + wrap(oyaw - self.odom_yaw))
            ym = self.pose[2] + 0.5 * wrap(yaw - self.pose[2])
            self.pose = (self.pose[0] + ds * math.cos(ym), self.pose[1] + ds * math.sin(ym), yaw)
            self.travel += abs(ds)
        self.odom_xy, self.odom_yaw = (p.position.x, p.position.y), oyaw
        self.hist.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, self.pose))
        if len(self.hist) > 100:
            del self.hist[:50]

    def on_imu(self, msg):
        q = msg.orientation
        self.imu_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        pitch = -math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
        self.pitch_f += 0.1 * (pitch - self.pitch_f)                # ~50 ms at 200 Hz
        self.rock += 0.05 * (abs(msg.angular_velocity.y) - self.rock)
        self.imu.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, pitch))
        if len(self.imu) > 400:
            del self.imu[:200]

    def pitch_at(self, ts):
        """Camera pitch relative to the ground at time ts.

        On flat ground this is the IMU pitch (the body rocks when accelerating / braking).
        On the hill the ground itself is tilted, so the IMU says nothing about the angle
        between camera and ground: the rest value is used there.
        """
        if not self.imu or abs(self.pitch_f) > math.radians(3.0):
            return REST_PITCH
        k = min(range(len(self.imu) - 1, max(-1, len(self.imu) - 80), -1), key=lambda i: abs(self.imu[i][0] - ts))
        return self.imu[k][1]

    # ---- perception + path ----------------------------------------------
    def on_image(self, msg):
        if self.pose is None or self.done:
            return
        if msg.encoding in ('rgb8', 'bgr8'):
            img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
            bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else img
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        elif msg.encoding == 'mono8':
            gray = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width)
            bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        else:
            return
        if gray.shape != (vision.H, vision.W):
            gray = cv2.resize(gray, (vision.W, vision.H))
            bgr = cv2.resize(bgr, (vision.W, vision.H))
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # while the body rocks or the robot is on the hill, only trust the near ground
        steady = self.rock < 0.08 and abs(self.pitch_f) < math.radians(3.0)
        out = self.perception.process(bgr, gray, self.pitch_at(ts), vision.X_FAR if steady else 0.50)
        self.last_obstacle = out.obstacle
        out.pitch = self.pitch_at(ts)
        self.last_observation = out
        self.map.update(out.line, out.valid, self.pose_at(ts))
        self.replan()
        if self.get_parameter('debug_image').value:
            self.publish_debug()

    def prune(self):
        """Drop path points the axle has passed, measured along the path itself.

        A point is passed when the robot is nearer to the next point than to this one,
        seen along the segment between them (works for sharp corners and loops).
        """
        px, py = self.pose[0], self.pose[1]
        n = 0
        while n + 1 < len(self.path) and n < 30:
            (ax, ay), (bx, by) = self.path[n], self.path[n + 1]
            ux, uy = bx - ax, by - ay
            if (px - ax) * ux + (py - ay) * uy <= 0.5 * (ux * ux + uy * uy):
                break
            n += 1
        if n:
            del self.path[:n]

    def acquire(self, x0=0.0):
        """Look for the line in front of the robot, starting x0 metres ahead of the axle."""
        best = None
        for dx in np.arange(x0, 0.62, 0.02):
            for dy in (0.0, 0.03, -0.03, 0.06, -0.06, 0.10, -0.10, 0.15, -0.15, 0.2, -0.2):
                X, Y = self.to_odom(dx, dy)
                if self.map.hit(X, Y):
                    best = self.map.snap(X, Y, 0.03) or (X, Y)
                    break
            if best:
                break
        if best is None:
            return None, None
        bx, by = self.to_robot(*best)
        n = max(1, int(math.hypot(bx, by) / 0.02))
        link = [self.to_odom(bx * i / n, by * i / n) for i in range(1, n)]
        return link, best

    def replan(self):
        self.prune()
        if self.t_point is not None:
            return                          # final approach: the path to the T is fixed
        lock = []
        acc = 0.0
        for i, p in enumerate(self.path):
            if i:
                acc += math.hypot(p[0] - self.path[i - 1][0], p[1] - self.path[i - 1][1])
            lock.append(p)
            if acc >= 0.08:
                break
        if len(lock) >= 3 and acc >= 0.04:
            anchor = self.map.snap(*lock[-1], 0.03)
            if anchor is None:
                return                      # map has nothing here (yet): keep the remembered path
            h = math.atan2(lock[-1][1] - lock[-3][1], lock[-1][0] - lock[-3][0])
        else:
            lock, anchor = self.acquire()
            if anchor is None:
                return
            h = self.pose[2]
        pts, end = self.map.trace(anchor[0], anchor[1], h)
        old_rest = max(0.0, path_len(self.path) - path_len(lock))
        if end == 'open' and path_len(pts) < old_rest - 0.10:
            return                          # the map lost part of the line: trust the memory
        new = lock[:-1] + pts if lock else pts
        if end == 'open' and path_len(pts) < 0.25:
            # the remembered line stops just ahead: if the camera sees a line right now
            # (the remembered one may be displaced by odometry error), follow what is seen
            link, anchor = self.acquire(vision.X_NEAR)
            if anchor is not None and min(math.hypot(anchor[0] - p[0], anchor[1] - p[1]) for p in new) < 0.10:
                pts2, end2 = self.map.trace(anchor[0], anchor[1], self.pose[2])
                if path_len(pts2) > path_len(pts) + 0.15 or end2 == 't':
                    new, pts, end = link + pts2, pts2, end2
        self.path = new
        self.path_end = end
        if end == 't':
            # the controller plugin may judge the finish itself (None = use the legacy rule)
            verdict = self.controller.confirm_finish(new, self.last_observation)
            if verdict is None:
                verdict = self.travel > self.get_parameter('finish_min_dist').value
            if verdict:
                # finish T seen: from now on drive this path to the T and stop there
                self.get_logger().info(f'finish T detected (driven {self.travel:.2f} m)')
                self.t_point = pts[-1]

    # ---- control --------------------------------------------------------
    def stop(self):
        self.v_cmd = self.w_cmd = 0.0
        self.cmd_pub.publish(Twist())

    def control(self):
        if self.pose is None:
            return
        if self.done:
            self.cmd_pub.publish(Twist())
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.02 if self.last_t is None else min(0.05, max(0.0, now - self.last_t))
        self.last_t = now

        v_min = self.get_parameter('v_min').value

        if self.t_point is not None:
            tx, ty = self.to_robot(*self.t_point)
            if tx < 0.01 and math.hypot(tx, ty) < 0.15:
                self.get_logger().info(f'reached finish, stopping (driven {self.travel:.2f} m)')
                self.done = True
                self.stop()
                return

        self.prune()
        rp = [self.to_robot(*p) for p in self.path]
        cmd = Twist()
        if len(rp) < 2:
            # nothing known ahead: creep straight for a while, then turn to search
            self.blind += self.v_cmd * dt
            creep = self.blind < 0.35
            self.v_cmd = v_min if creep else 0.0
            self.w_cmd = 0.0 if creep else 0.8
            cmd.linear.x, cmd.angular.z = self.v_cmd, self.w_cmd
            self.cmd_pub.publish(cmd)
            return
        self.blind = 0.0

        # drive: the controller plugin turns the path ahead into (v, w)
        v, w = self.controller.compute(rp, dt)
        self.v_cmd, self.w_cmd = v, w
        cmd.linear.x, cmd.angular.z = v, w
        self.cmd_pub.publish(cmd)

    # ---- debug ----------------------------------------------------------
    def publish_debug(self):
        r = 200
        j0 = int(round(self.pose[0] / RES)) + C0
        i0 = int(round(self.pose[1] / RES)) + C0
        crop = self.map.grid[i0 - r:i0 + r, j0 - r:j0 + r]
        if crop.shape != (2 * r, 2 * r):
            return
        dbg = cv2.cvtColor(crop, cv2.COLOR_GRAY2BGR)
        if self.map.last is not None:           # area seen by the last frame (dark blue) / line in it (cyan)
            ox, oy, obs, seen = self.map.last
            full = np.zeros((2 * r, 2 * r, 3), np.uint8)
            ys, xs = np.nonzero(seen)
            u, v = xs + ox - j0 + r, ys + oy - i0 + r
            ok = (u >= 0) & (u < 2 * r) & (v >= 0) & (v < 2 * r)
            full[v[ok], u[ok]] = np.where(obs[ys[ok], xs[ok]][:, None], (255, 255, 0), (90, 0, 0))
            dbg = cv2.max(dbg // 2 + full // 2, full // 2)
            dbg[crop > 0] = np.maximum(dbg[crop > 0], 160)
        for X, Y in self.path:
            cv2.circle(dbg, (int(round(X / RES)) + C0 - j0 + r, int(round(Y / RES)) + C0 - i0 + r), 1, (0, 255, 0), -1)
        hx, hy = math.cos(self.pose[2]), math.sin(self.pose[2])
        cv2.arrowedLine(dbg, (r, r), (int(r + 30 * hx), int(r + 30 * hy)), (0, 0, 255), 2)
        dbg = cv2.flip(dbg, 0)              # +y up
        cv2.putText(dbg, f'v{self.v_cmd:.2f} w{self.w_cmd:.2f} s{self.travel:.1f} {self.path_end}',
                    (4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
        out = Image()
        out.header.stamp = self.get_clock().now().to_msg()
        out.height, out.width = dbg.shape[:2]
        out.encoding = 'bgr8'
        out.step = out.width * 3
        out.data = dbg.tobytes()
        self.dbg_pub.publish(out)


def main():
    rclpy.init()
    node = FollowerNode()
    # rclpy occasionally fails to deserialise one incoming message under heavy load
    # ("Unable to convert call argument '0' to Python object"). Losing that single message
    # is harmless; letting it kill the node ends the run. Drop it and keep spinning.
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    dropped = 0
    try:
        while rclpy.ok():
            try:
                executor.spin_once(timeout_sec=0.1)
            except RuntimeError as error:
                if 'Unable to convert' not in str(error) or dropped > 1000:
                    raise
                dropped += 1
                node.get_logger().warning(f'dropped one undecodable message ({dropped})')
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
