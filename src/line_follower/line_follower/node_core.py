"""Shared pieces of the line follower nodes (single follower_node and the split nodes).

Used by follower_node.py (everything in one process) and by line_follower/split/ (perception,
map, planning and control as separate ROS nodes, docs/architecture/NODE_SPLIT.md), so both
modes run the same code:

    declare_follower_parameters(node)  every follower_node parameter, on any node
    Frames                             to_odom / to_robot
    PoseEstimator                      dead reckoning from /odom + /imu (on_odom, on_imu, pose_at, ...)
    ControlLoop                        the 50 Hz control tick (finish, prune, creep / search, controller)
    CallbackTiming                     callback processing time p50 / p95 logged every 10 s
    spin(node, finalize)               executor loop that survives an undecodable message

No ROS import is needed for the math (pure-python tests import this module without rclpy).
"""
import math
import time

try:
    from geometry_msgs.msg import Twist
    from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor
except ImportError:          # pure-python tests: only the math below is used
    Twist = FloatingPointRange = ParameterDescriptor = None

from .perception.stage import REST_PITCH


def rng(lo, hi, desc=''):
    return ParameterDescriptor(
        description=desc,
        floating_point_range=[FloatingPointRange(from_value=lo, to_value=hi, step=0.0)])


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def path_len(pts):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts[:-1], pts[1:]))


def declare_follower_parameters(node):
    """Declare every follower_node parameter on `node` (same names, defaults, ranges)."""
    node.declare_parameter('v_max', 0.25, rng(0.05, 1.5, 'max linear speed [m/s]'))
    node.declare_parameter('v_min', 0.06, rng(0.02, 0.5, 'min linear speed [m/s]'))
    node.declare_parameter('a_lat', 0.25, rng(0.05, 3.0, 'allowed lateral acceleration in curves [m/s^2]'))
    node.declare_parameter('a_dec', 0.3, rng(0.1, 1.0, 'braking deceleration before curves [m/s^2]'))
    node.declare_parameter('a_acc', 0.6, rng(0.1, 1.0, 'acceleration [m/s^2]'))
    node.declare_parameter('v_hill', 0.25, rng(0.05, 1.5, 'max linear speed on the hill [m/s]'))
    node.declare_parameter('omega_max', 3.0, rng(0.5, 8.0, 'max angular speed [rad/s]'))
    node.declare_parameter('lookahead_min', 0.10, rng(0.04, 0.5, 'pure pursuit lookahead at zero speed [m]'))
    node.declare_parameter('lookahead_gain', 0.30, rng(0.0, 2.0, 'lookahead growth per m/s [s]'))
    node.declare_parameter('finish_min_dist', 10.0, rng(0.0, 30.0, 'travel before the finish T is accepted [m]'))
    node.declare_parameter('steady_rock', 0.08, rng(0.0, 1.0, 'max body rocking for the full camera range'))
    node.declare_parameter('steady_pitch_deg', 3.0, rng(0.0, 20.0, 'max |pitch| for the full camera range [deg]'))
    node.declare_parameter('x_far_unsteady', 0.50, rng(0.3, 1.0, 'camera range while rocking / on a hill [m]'))
    node.declare_parameter('hill_memory', False)        # remember climbed hills; flat-ground frames write no line there
    node.declare_parameter('slope_y_half', 0.0, rng(0.0, 0.6, 'on a climb / descent keep line cells only within this lateral distance [m], 0 = off'))
    node.declare_parameter('x_far_slope', 0.0, rng(0.0, 1.0, 'camera range on a slope only (|pitch| >= steady_pitch_deg) [m], 0 = use x_far_unsteady'))
    node.declare_parameter('rock_v_cap', 0.0, rng(0.0, 1.0, 'speed cap while the body rocks on flat ground [m/s], 0 = off'))
    node.declare_parameter('restore_model', '')            # ONNX of the line-restoration net ('' = off)
    node.declare_parameter('restore_replace', False)
    node.declare_parameter('restore_gate', 0.0, rng(0.0, 1.0, 'replace mode: keep the map path while the net agrees with it (mean route prob along it >= gate); 0 = always replace'))       # follow the learned route every frame (not only when the path runs out)
    node.declare_parameter('restore_trigger', 0.45, rng(0.0, 2.0, 'restore when the open path is shorter than this [m]'))
    node.declare_parameter('restore_len', 0.4, rng(0.0, 1.5, 'longest restored stretch [m]'))
    node.declare_parameter('restore_thresh', 0.6, rng(0.0, 1.0, 'follow the predicted route only where its probability exceeds this'))
    node.declare_parameter('trace_wide', False)           # follow acute corners (100-150 deg) when nothing else continues
    node.declare_parameter('imu_slip', False)             # fuse encoder and IMU speed; use the IMU while the wheels slip
    node.declare_parameter('imu_slip_thresh', 0.04, rng(0.0, 0.5, 'encoder speed above the IMU speed that counts as wheel slip [m/s]'))
    node.declare_parameter('imu_slip_rate', 0.0, rng(0.0, 5.0, 'apply the slip fix only while |pitch rate| exceeds this (tipping over a crest) [rad/s], 0 = always'))
    node.declare_parameter('imu_slip_k', 1.5, rng(0.0, 20.0, 'pull of the IMU speed toward the encoder speed [1/s]'))
    node.declare_parameter('imu_floor', False)            # on/near ramps project onto the floor tracked with the IMU
    node.declare_parameter('imu_floor_min_h', 0.008, rng(0.0, 0.1, 'elevation above which the floor model is used [m]'))
    node.declare_parameter('map_skip_pitch_deg', 0.0, rng(0.0, 30.0, 'do not write the map while |pitch| exceeds this (tipping over a ramp edge) [deg], 0 = off'))
    node.declare_parameter('deck_xfar', 0.0, rng(0.0, 1.0, 'camera range while standing on a raised deck (climbed, not yet descended) [m], 0 = off'))
    node.declare_parameter('deck_model', '')              # '' = off, 'symmetric' = model the deck edge from the climb
    node.declare_parameter('deck_edge_offset', 0.0, rng(-0.5, 0.5, 'deck edge distance correction for the symmetric model [m]'))
    node.declare_parameter('climb_slip', 0.0, rng(0.0, 0.3, 'fraction of /odom distance lost to wheel slip while climbing'))
    node.declare_parameter('scan_match', False)          # nudge the pose so fresh line pixels sit on the map (scanmatch.py)
    node.declare_parameter('scan_gain', 0.5, rng(0.0, 1.0, 'fraction of the matched correction applied per frame'))
    node.declare_parameter('relink_radius', 0.0, rng(0.0, 0.2, 'look this far for the line when the remembered path no longer matches the map [m], 0 = off'))
    node.declare_parameter('terrain_calib', False)      # rescale widths so the near ground is z = 0 (helps blurred renders / Isaac, hurts Gazebo)
    node.declare_parameter('terrain_hill', False)
    node.declare_parameter('terrain_down_only', False)
    node.declare_parameter('terrain_bend_z', 0.0, rng(0.0, 0.1, 'on flat ground too, use the profile when it bends this much within the view [m], 0 = off'))  # use the profile only while going downhill (nose down)       # on a slope, project with the line-width height profile (terrain.py)
    node.declare_parameter('x_far_terrain', 0.50, rng(0.3, 1.5, 'camera range on a slope when terrain_hill is on [m]'))
    # dual resolution (2026-11-25): near ground from a low-resolution image, the far band
    # (far_split..BEV end, set LF_X_FAR) from the full 640x480 rows near the horizon,
    # written add-only (never erases, overwritten once seen from close by). Off by default.
    node.declare_parameter('far_band', False)
    node.declare_parameter('far_split', 0.85)
    node.declare_parameter('near_scale', 1.0)
    node.declare_parameter('freeze_passed', False)  # keep the map of the road already driven (no rewrites)
    node.declare_parameter('freeze_radius', 0.15)    # [m] around the axle, marked as driven
    node.declare_parameter('map_mode', 'legacy')
    node.declare_parameter('perception_log', '')   # dir for the per-frame perception log ('' = env LF_PERCEPTION_LOG or off)
    node.declare_parameter('map_l_hit', 0.85)
    node.declare_parameter('map_l_miss', -0.4)
    node.declare_parameter('map_l_min', -2.0)
    node.declare_parameter('map_l_max', 3.5)
    node.declare_parameter('map_l_thresh', 0.6)
    node.declare_parameter('far_add_only', True)   # far band only fills unseen line (False: overwrite like near)
    node.declare_parameter('far_thin', 0.0)       # [m] far band: cut across-runs up to this long to their centre
    node.declare_parameter('far_model', '')      # far-band U-Net ONNX (far_band.py); '' = threshold
    node.declare_parameter('far_add_unsteady', False)   # rocking on flat ground: far range add-only (fills unseen line, never erases)
    # Track-specific speed plan: [s_from, s_to, v_max, ...] with s = distance driven [m].
    # Outside these windows v_max applies. Tuned for the provided line_track world.
    node.declare_parameter('speed_profile', [0.0], ParameterDescriptor(
        description='flat list of (s_from, s_to, v) triples, s = distance driven [m]'))
    node.declare_parameter('debug_image', False)
    # plugins: who sees the line, who drives (see TEAM_GUIDE.md)
    node.declare_parameter('perception', 'rule')    # rule | hybrid | depth | dl
    # rule.max_width_m: remove rule-mask components thicker than this [m] (0 = off)
    node.declare_parameter('rule.max_width_m', 0.0, rng(0.0, 0.5,
                                                          'rule mask: drop components thicker than this [m], 0 = off'))
    node.declare_parameter('hybrid.mode', 'rule_gated')  # 'and' | 'rule_gated'
    node.declare_parameter('hybrid.gate_px', 2)          # cells from the depth line that gate a rule component
    node.declare_parameter('controller', 'rule')    # rule | learned
    node.declare_parameter('planner', 'rule')       # rule
    # control (role 3): pure pursuit + PID trim on the cross-track error (0 = plain PP)
    node.declare_parameter('pid_kp', 0.0, rng(0.0, 50.0, 'steering per metre of cross-track error [rad/s/m]'))
    node.declare_parameter('pid_ki', 0.0, rng(0.0, 50.0, 'integral gain [rad/s/(m s)]'))
    node.declare_parameter('pid_kd', 0.0, rng(0.0, 10.0, 'derivative gain [rad/s/(m/s)]'))
    # line restoration: 'path' = extend the planned path when it runs out (legacy),
    # 'map' = perception writes the restored route into the map it hands to planning
    node.declare_parameter('restore_mode', 'path')
    node.declare_parameter('restore_min_x', 0.2, rng(0.0, 2.0, "map mode: restore only from frontier points at least this far ahead [m]"))
    node.declare_parameter('restore_period', 0.2, rng(0.0, 2.0, "map mode: run the restoration net at most this often [s]"))


class Frames:
    """odom <-> robot frame with self.pose = (x, y, yaw)."""

    def to_odom(self, x, y, pose=None):
        px, py, th = pose or self.pose
        return (px + x * math.cos(th) - y * math.sin(th), py + x * math.sin(th) + y * math.cos(th))

    def to_robot(self, X, Y, pose=None):
        px, py, th = pose or self.pose
        dx, dy = X - px, Y - py
        return (dx * math.cos(th) + dy * math.sin(th), -dx * math.sin(th) + dy * math.cos(th))


class PoseEstimator(Frames):
    """Dead reckoning (wheel distance, IMU heading) and the IMU pitch history.

    Pose state lives on the node; pose_at extrapolates with the last command (v_cmd, w_cmd)."""

    def init_pose_state(self):
        self.pose = None            # latest (x, y, yaw) in odom
        self.hist = []              # (stamp, pose) to look up the pose at image time
        self.imu_yaw = None
        self.yaw_off = None
        self.odom_xy, self.odom_yaw = None, 0.0
        self.imu = []               # (stamp, nose-up pitch [rad])
        self.rock = 0.0             # filtered |pitch rate| [rad/s]
        self.pitch_f = 0.0          # lightly filtered pitch, used to notice the hill
        self.travel = 0.0
        self.rocking = False        # body rocking on flat ground (camera range is cut)
        self.v_cmd = 0.0
        self.w_cmd = 0.0

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
            t_od = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            dt_od = t_od - getattr(self, 't_odom', t_od)
            self.t_odom = t_od
            if self.get_parameter('imu_slip').value and 0.0 < dt_od < 0.2:
                self.v_enc = ds / dt_od
                v_imu = getattr(self, 'v_imu', self.v_enc)
                self.slipping = self.v_enc - v_imu > self.get_parameter('imu_slip_thresh').value
                rate = self.get_parameter('imu_slip_rate').value
                if rate > 0.0 and getattr(self, 'gyro_peak', 0.0) < rate:
                    self.slipping = False          # steady slope: trust the wheels
                if self.slipping and ds > 0.0:
                    # wheels turn faster than the body moves: trust the IMU speed
                    ds = max(0.0, min(ds, v_imu * dt_od))
                    self.slip_events = getattr(self, 'slip_events', 0) + 1
            slip = self.get_parameter('climb_slip').value
            if slip > 0.0 and self.pitch_f > math.radians(2.0) and ds > 0.0:
                # climbing: the wheels spin a little, /odom overstates the distance (~6-7 %
                # measured against ground truth on 4-9 deg ramps). Left uncorrected it shifts
                # the map along the track, which becomes a sideways offset after a turn.
                ds *= 1.0 - slip
            if self.imu_yaw is not None:
                if self.yaw_off is None:
                    self.yaw_off = wrap(self.pose[2] - self.imu_yaw)
                yaw = wrap(self.imu_yaw + self.yaw_off)
            else:
                yaw = wrap(self.pose[2] + wrap(oyaw - self.odom_yaw))
            ym = self.pose[2] + 0.5 * wrap(yaw - self.pose[2])
            self.pose = (self.pose[0] + ds * math.cos(ym), self.pose[1] + ds * math.sin(ym), yaw)
            self.travel += abs(ds)
            # height of the wheels above the floor, from the IMU pitch along the way
            pitch_now = self.imu[-1][1] if self.imu else 0.0
            self.elev = max(0.0, getattr(self, 'elev', 0.0) + ds * math.sin(pitch_now - REST_PITCH))
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
        self.gyro_y = abs(msg.angular_velocity.y)
        self.gyro_peak = max(self.gyro_y, 0.97 * getattr(self, 'gyro_peak', 0.0))   # ~0.15 s hold at 200 Hz
        t_imu = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.get_parameter('imu_slip').value:
            # forward speed from the IMU: specific force minus the gravity share along the body
            # x axis (nose up -> +g sin(pitch)), pulled slowly toward the encoder speed
            last = getattr(self, 't_imu', None)
            self.t_imu = t_imu
            if last is not None and 0.0 < t_imu - last < 0.05:
                dt = t_imu - last
                a = msg.linear_acceleration.x - 9.81 * math.sin(pitch)
                v = getattr(self, 'v_imu', 0.0) + a * dt
                if not getattr(self, 'slipping', False):
                    v += self.get_parameter('imu_slip_k').value * (getattr(self, 'v_enc', 0.0) - v) * dt
                self.v_imu = v
        self.imu.append((t_imu, pitch))
        if len(self.imu) > 400:
            del self.imu[:200]

    def imu_pitch_at(self, ts):
        """Raw IMU body pitch nearest to time ts (nose up > 0)."""
        if not self.imu:
            return REST_PITCH
        k = min(range(len(self.imu) - 1, max(-1, len(self.imu) - 80), -1), key=lambda i: abs(self.imu[i][0] - ts))
        return self.imu[k][1]

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


class ControlLoop:
    """The control tick (role 3 host): finish check, prune, creep / search, controller plugin.

    Needs on the node: pose, path, path_end, t_point, travel, rocking, v_cmd / w_cmd,
    controller, cmd_pub, prune(), to_robot(), get_parameter / get_clock / get_logger."""

    def init_control_state(self):
        self.done = False
        self.blind = 0.0            # distance driven without any path
        self.last_t = None

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
            self.controller.reset()
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
        cap = self.get_parameter('rock_v_cap').value
        if cap > 0.0 and self.rocking and v > cap:
            # perception is degraded (near range only): slow down until the body settles,
            # gently, so the braking itself does not keep the body rocking
            v = max(cap, self.v_cmd - 0.4 * dt)
        self.v_cmd, self.w_cmd = v, w
        cmd.linear.x, cmd.angular.z = v, w
        self.cmd_pub.publish(cmd)


class CallbackTiming:
    """Wall time spent in each callback; logs p50 / p95 per callback every `period` s
    (one line per node)."""

    def __init__(self, node, period=10.0):
        self.node = node
        self.samples = {}
        node.create_timer(period, self.report)

    def wrap(self, name, fn):
        samples = self.samples.setdefault(name, [])

        def timed(*args):
            t0 = time.perf_counter()
            try:
                return fn(*args)
            finally:
                samples.append(time.perf_counter() - t0)
        return timed

    def report(self):
        parts = []
        for name, s in self.samples.items():
            if not s:
                continue
            q = sorted(s)
            parts.append(f'{name} n={len(q)} p50={1e3 * q[len(q) // 2]:.2f} '
                         f'p95={1e3 * q[min(len(q) - 1, int(0.95 * len(q)))]:.2f}')
            s.clear()
        if parts:
            self.node.get_logger().info('callback ms: ' + ' | '.join(parts))


def spin(node, finalize=()):
    """Spin until shutdown. rclpy occasionally fails to deserialise one incoming message under
    heavy load ("Unable to convert call argument '0' to Python object"). Losing that single
    message is harmless; letting it kill the node ends the run. Drop it and keep spinning.
    finalize: callables run (errors ignored) before the node is destroyed."""
    import rclpy
    import rclpy.executors
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
        for fn in finalize:
            try:
                fn()
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
