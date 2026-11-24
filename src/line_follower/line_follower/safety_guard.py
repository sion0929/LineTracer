"""Safety guard: the only /cmd_vel publisher.

follower_node --(/line_follower/cmd_raw, command candidate)--> safety_guard --> /cmd_vel

The guard passes the candidate on only while every rule below holds, otherwise it publishes
zero. It watches the robot topics the follower depends on (/odom, /imu, /camera/image_raw);
internal topics are its candidate input and its status output (/line_follower/safety).

Rules (all limits and times are parameters, see config/safety_sim.yaml / safety_real.yaml):
  start gate    zero until `start_delay` s of WALL time since the guard started (the 20 s rule
                is a stopwatch rule; sim time may run faster than real time) and until /odom,
                /imu and /camera/image_raw have all been received.
  watchdog      zero while the candidate is older than `cmd_timeout` (follower crashed or hung)
                or a sensor is stale: no message for `<name>_timeout`, header stamp not
                advancing for that long (driver republishing an old sample), bit-identical IMU
                samples for `frozen_timeout` (real and simulated IMUs are noisy), or an odometry
                pose that does not change for `frozen_timeout` while the guard commands motion.
                Ages are in ROS time (sim time in Gazebo, so a slow simulation is not a fault);
                a ROS clock that stops advancing for `clock_timeout` s of wall time is a fault
                of its own (/clock bridge dead).
  resume        a watchdog stop ends after every input has been healthy for `recover_time`:
                a hiccup must not end a competition run. With `latch_after` > 0 (real robot) a
                stop that lasts longer than that latches. The output then ramps up from zero.
  latch         a non-finite candidate, or turning in place for more than `spin_budget` seconds
                in total without `spin_reset_distance` of real travel in between (short arcs
                between spins do not restart the budget), latches zero until restart.
  limits        |v| <= max_linear and |w| <= max_angular, scaled TOGETHER so the commanded path
                curvature w/v is kept; then acceleration limits (0 = off) that move (v, w)
                along the straight line to the target, which again keeps w/v when starting
                from or stopping to zero. Safety stops are immediate (no deceleration ramp).
  exit          on SIGINT / SIGTERM zero is published several times (and DDS acknowledged)
                before the ROS context is shut down.
"""
import json
import math
import signal
import threading
import time

import rclpy
import rclpy.executors
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import String

CMD_TOPIC = '/line_follower/cmd_raw'
STATUS_TOPIC = '/line_follower/safety'
SENSORS = ('odom', 'imu', 'image')
SPIN_MIN_W = 0.05       # |w| below this is not turning [rad/s]
MOVE_EPS = 1e-3         # |v| [m/s] or |w| [rad/s] above this counts as a motion command
STATUS_PERIOD = 0.2     # periodic status [s wall]

# name: (default = simulation value, lower bound, upper bound, description)
PARAMS = {
    'start_delay': (20.0, 0.0, 300.0, 'no motion before this many WALL seconds after start [s]'),
    'cmd_timeout': (0.3, 0.02, 5.0, 'zero when the command candidate is older than this [s]'),
    'odom_timeout': (0.5, 0.02, 5.0, 'zero when /odom is older than this [s]'),
    'imu_timeout': (0.5, 0.02, 5.0, 'zero when /imu is older than this [s]'),
    'image_timeout': (1.0, 0.02, 5.0, 'zero when /camera/image_raw is older than this [s]'),
    'frozen_timeout': (1.0, 0.0, 10.0,
                       'zero when IMU samples (always) or the odometry pose (while moving) stay '
                       'bit-identical this long; 0 = off [s]'),
    'clock_timeout': (2.0, 0.1, 10.0, 'zero when the ROS clock stops for this much wall time [s]'),
    'recover_time': (0.3, 0.0, 10.0, 'inputs must be healthy this long before driving again [s]'),
    'latch_after': (0.0, 0.0, 600.0, 'a watchdog stop longer than this latches; 0 = never [s]'),
    'max_linear': (1.0, 0.0, 5.0, 'max |v| [m/s] (v and w scaled together)'),
    'max_angular': (4.0, 0.0, 20.0, 'max |w| [rad/s] (v and w scaled together)'),
    'max_linear_accel': (1.0, 0.0, 20.0, 'max speed-up rate [m/s^2]; 0 = off'),
    'max_linear_decel': (0.0, 0.0, 20.0, 'max slow-down rate [m/s^2]; 0 = off'),
    'max_angular_accel': (0.0, 0.0, 200.0, 'max |dw/dt| [rad/s^2]; 0 = off'),
    'spin_budget': (30.0, 0.0, 3600.0, 'turning in place longer than this latches zero; 0 = off [s]'),
    'spin_reset_distance': (0.3, 0.0, 10.0, 'travel (not turning in place) that restarts the spin budget [m]'),
    'spin_linear': (0.02, 0.0, 0.5, 'turning with |v| below this is turning in place [m/s]'),
    'spin_radius': (0.05, 0.0, 1.0, 'turning with radius |v/w| below this is turning in place [m]'),
    'rate': (50.0, 5.0, 500.0, 'watchdog / output rate [Hz wall]'),
}


def scale_to_limits(v, w, max_v, max_w):
    """Scale (v, w) by one common factor so |v| <= max_v and |w| <= max_w (keeps w/v)."""
    k = 1.0
    if abs(v) > max_v:
        k = min(k, max_v / abs(v))
    if abs(w) > max_w:
        k = min(k, max_w / abs(w))
    return (v, w) if k >= 1.0 else (v * k, w * k)


def rate_limit(v0, w0, v1, w1, dt, lin_acc, lin_dec, ang_acc):
    """Move from (v0, w0) towards (v1, w1) along the straight line, as far as the limits
    allow in dt (0 = no limit). From or to zero this keeps the curvature w/v."""
    dv, dw = v1 - v0, w1 - w0
    k = 1.0
    if dv:
        speeding_up = v0 * v1 >= 0.0 and abs(v1) > abs(v0)
        lim = lin_acc if speeding_up else lin_dec
        if lim > 0.0:
            k = min(k, lim * dt / abs(dv))
    if dw and ang_acc > 0.0:
        k = min(k, ang_acc * dt / abs(dw))
    if k >= 1.0:
        return v1, w1
    return v0 + k * dv, w0 + k * dw


def stamp_of(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class Input:
    """Freshness bookkeeping for one input (ROS-time seconds)."""

    def __init__(self):
        self.recv = None            # last receipt
        self.stamp = None           # newest header stamp seen
        self.stamp_adv = None       # receipt time of the last message whose stamp advanced
        self.content = None         # last content key (odom / imu)
        self.changed = None         # receipt time of the last content change
        self.frozen = False         # odom: content found frozen, until it changes again


class SafetyGuard(Node):
    def __init__(self, wall=time.monotonic, **kwargs):
        super().__init__('safety_guard', **kwargs)
        for name, (default, lo, hi, desc) in PARAMS.items():
            self.declare_parameter(name, default, ParameterDescriptor(
                description=desc,
                floating_point_range=[FloatingPointRange(from_value=lo, to_value=hi, step=0.0)]))
        self._wall = wall
        self.t0_wall = wall()
        self.inputs = {name: Input() for name in ('cmd',) + SENSORS}
        self.cand = None                        # latest candidate (v, w)
        self.state, self.reason = 'GATE', 'starting'
        self.gate_open = False
        self.latched = None                     # reason, once latched
        self.healthy_since = None
        self.stale_since = None
        self.last_problem = ''
        self.out_v = self.out_w = 0.0
        self.out_t = None
        self.moving_since = None
        self.spin_time = 0.0                    # turning in place since the last real travel [s]
        self.spin_travel = 0.0                  # travel since the last turning in place [m]
        self.last_ros = None
        self.ros_change_wall = self.t0_wall
        self.status_wall = None

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
        self.create_subscription(Twist, CMD_TOPIC, self.on_cmd, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, qos_profile_sensor_data)
        # wall-clock timer: keeps running (and publishing zero) when /clock stops
        self.timer = self.create_timer(1.0 / self.p('rate'), self.tick,
                                       clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(
            f"safety guard up: sole /cmd_vel publisher, candidate {CMD_TOPIC}, start gate "
            f"{self.p('start_delay'):.1f} s, limits v {self.p('max_linear'):.2f} m/s "
            f"w {self.p('max_angular'):.2f} rad/s")

    def p(self, name):
        return self.get_parameter(name).value

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9, self._wall()

    # ---- inputs -----------------------------------------------------------
    def _seen(self, name, now, stamp=None, content=None):
        i = self.inputs[name]
        i.recv = now
        if stamp is not None and (i.stamp is None or stamp > i.stamp):
            i.stamp, i.stamp_adv = stamp, now
        if content is not None and content != i.content:
            i.content, i.changed, i.frozen = content, now, False

    def on_odom(self, msg):
        p, q, t = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        key = (p.x, p.y, p.z, q.x, q.y, q.z, q.w, t.linear.x, t.angular.z)
        self._seen('odom', self._now()[0], stamp_of(msg), key)

    def on_imu(self, msg):
        q, a, w = msg.orientation, msg.linear_acceleration, msg.angular_velocity
        key = (q.x, q.y, q.z, q.w, a.x, a.y, a.z, w.x, w.y, w.z)
        self._seen('imu', self._now()[0], stamp_of(msg), key)

    def on_image(self, msg):
        self._seen('image', self._now()[0], stamp_of(msg))

    def on_cmd(self, msg):
        now, wall = self._now()
        self._seen('cmd', now)
        values = (msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z)
        if not all(math.isfinite(x) for x in values):
            self._latch(f'non-finite command {values}')
        self.cand = (msg.linear.x, msg.angular.z)
        self._advance(now, wall)

    def tick(self):
        now, wall = self._now()
        self._advance(now, wall)
        if self.status_wall is None or wall - self.status_wall >= STATUS_PERIOD:
            self._publish_status(now, wall)

    # ---- rules ------------------------------------------------------------
    def _problems(self, now, wall):
        """Every reason not to drive right now (empty list = all inputs healthy)."""
        out = []
        if self.last_ros != now:
            self.last_ros, self.ros_change_wall = now, wall
        if wall - self.ros_change_wall > self.p('clock_timeout'):
            out.append(f'clock stalled ({wall - self.ros_change_wall:.2f} s wall)')
        frozen = self.p('frozen_timeout')
        for name in SENSORS:
            i, timeout = self.inputs[name], self.p(f'{name}_timeout')
            if i.recv is None:
                out.append(f'{name}: no data yet')
            elif now - i.recv > timeout:
                out.append(f'{name} timeout ({now - i.recv:.2f} s)')
            elif i.stamp_adv is None or now - i.stamp_adv > timeout:
                out.append(f'{name} stamp frozen')
            elif frozen > 0.0 and name == 'imu' and now - i.changed > frozen:
                out.append(f'imu frozen (identical samples for {now - i.changed:.2f} s)')
            elif frozen > 0.0 and name == 'odom':
                if (not i.frozen and self.moving_since is not None and now - self.moving_since > frozen
                        and now - i.changed > frozen):
                    i.frozen = True
                if i.frozen:
                    out.append('odom frozen (pose unchanged while driving)')
        c = self.inputs['cmd']
        if c.recv is None:
            out.append('no command yet')
        elif now - c.recv > self.p('cmd_timeout'):
            out.append(f'command timeout ({now - c.recv:.2f} s)')
        return out

    def _evaluate(self, now, wall):
        """(state, reason) for this instant; updates the gate / recovery bookkeeping."""
        problems = self._problems(now, wall)
        if problems:
            self.healthy_since = None
            self.last_problem = problems[0]
        elif self.healthy_since is None:
            self.healthy_since = now
        if self.latched:
            return 'LATCHED', self.latched
        if not self.gate_open:
            waiting = []
            left = self.p('start_delay') - (wall - self.t0_wall)
            if left > 0.0:
                waiting.append(f'start delay ({left:.1f} s left)')
            waiting += [x for x in problems if x.split()[0].rstrip(':') in SENSORS + ('clock',)]
            if waiting:
                return 'GATE', '; '.join(waiting)
            self.gate_open = True
            self.get_logger().info('start gate open')
        if problems:
            if self.stale_since is None:
                self.stale_since = now
            seen = all(self.inputs[n].recv is not None for n in ('cmd',) + SENSORS)
            limit = self.p('latch_after')
            if limit > 0.0 and seen and now - self.stale_since > limit:
                self._latch(f'input lost for more than {limit:.1f} s: {problems[0]}')
                return 'LATCHED', self.latched
            return 'STOP', problems[0]
        self.stale_since = None
        if self.state != 'RUN' and now - self.healthy_since < self.p('recover_time'):
            return 'STOP', f'recovering from: {self.last_problem}' if self.last_problem else 'starting'
        return 'RUN', ''

    def _latch(self, reason):
        if not self.latched:
            self.latched = reason
            self.get_logger().error(f'LATCHED until restart: {reason}')

    def _is_spin(self, v, w):
        return abs(w) > SPIN_MIN_W and abs(v) <= max(self.p('spin_linear'), self.p('spin_radius') * abs(w))

    def _advance(self, now, wall):
        """Evaluate the rules, compute the output and publish it."""
        state, reason = self._evaluate(now, wall)
        v = w = 0.0
        if state == 'RUN' and self.cand is not None:
            tv, tw = scale_to_limits(*self.cand, self.p('max_linear'), self.p('max_angular'))
            dt = 0.0 if self.out_t is None else min(0.1, max(0.0, now - self.out_t))
            v, w = rate_limit(self.out_v, self.out_w, tv, tw, dt, self.p('max_linear_accel'),
                              self.p('max_linear_decel'), self.p('max_angular_accel'))
            budget = self.p('spin_budget')
            if self._is_spin(v, w):
                self.spin_time += dt
                self.spin_travel = 0.0
                if budget > 0.0 and self.spin_time > budget:
                    self._latch(f'spin budget exceeded (turning in place for {budget:.1f} s '
                                f'without {self.p("spin_reset_distance"):.2f} m of travel)')
                    state, reason, v, w = 'LATCHED', self.latched, 0.0, 0.0
            elif abs(v) > MOVE_EPS:
                self.spin_travel += abs(v) * dt
                if self.spin_travel >= self.p('spin_reset_distance'):
                    self.spin_time = 0.0
        moving = abs(v) > MOVE_EPS or abs(w) > MOVE_EPS
        if not moving:
            self.moving_since = None
        elif self.moving_since is None:
            self.moving_since = now
        self.out_v, self.out_w, self.out_t = v, w, now
        self._set_state(state, reason, now, wall)
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = v, w
        self.cmd_pub.publish(cmd)

    def _set_state(self, state, reason, now, wall):
        if state == self.state and (state == 'RUN' or reason.split(' (')[0] == self.reason.split(' (')[0]):
            self.reason = reason
            return
        head = state if state == self.state else f'{self.state} -> {state}'
        text = f'safety {head}' + (f': {reason}' if reason else '')
        if state in ('RUN', 'GATE'):
            self.get_logger().info(text)
        else:
            self.get_logger().warning(text)
        self.state, self.reason = state, reason
        self._publish_status(now, wall)

    def _publish_status(self, now, wall):
        self.status_wall = wall
        age = {name: (None if i.recv is None else round(now - i.recv, 3)) for name, i in self.inputs.items()}
        msg = String()
        msg.data = json.dumps(dict(
            state=self.state, reason=self.reason, latched=bool(self.latched), t=round(now, 3),
            v=round(self.out_v, 4), w=round(self.out_w, 4),
            cand=None if self.cand is None else [round(x, 4) if math.isfinite(x) else str(x) for x in self.cand],
            spin=round(self.spin_time, 2), age=age))
        self.status_pub.publish(msg)

    # ---- exit -------------------------------------------------------------
    def shutdown_zero(self):
        """Last words: zero /cmd_vel several times while the context is still valid."""
        self.timer.cancel()
        self.out_v = self.out_w = 0.0
        self.state, self.reason = 'EXIT', 'shutdown'
        try:
            now, wall = self._now()
            self._publish_status(now, wall)
        except Exception:
            pass
        sent = publish_zeros(self.cmd_pub)
        self.get_logger().info(f'exit: {sent} zero commands published on /cmd_vel')
        return sent


# ---- shared exit helpers (also used by follower_node.main) -------------------
def install_stop_handlers():
    """SIGINT / SIGTERM only set the returned event, so the node can publish zero before
    the ROS context goes away. rclpy's own handlers would shut the context down first.
    Call after rclpy.init(); also removes rclpy's handlers if a wrapper re-enabled them."""
    try:
        rclpy.signals.uninstall_signal_handlers()
    except Exception:
        pass
    stop = threading.Event()

    def handler(signum, _frame):
        stop.set()
    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)
    return stop


def publish_zeros(publisher, repeats=5, interval=0.02, ack_timeout=0.5):
    """Publish a zero Twist `repeats` times and wait for the reliable readers to ACK it."""
    sent = 0
    for k in range(repeats):
        try:
            publisher.publish(Twist())
            sent += 1
        except Exception:
            break
        if k + 1 < repeats:
            time.sleep(interval)
    try:
        publisher.wait_for_all_acked(Duration(seconds=ack_timeout))
    except Exception:
        pass
    return sent


def spin_until_stopped(node, stop):
    """Spin until SIGINT / SIGTERM. rclpy occasionally fails to deserialise one incoming
    message under heavy load ("Unable to convert call argument"); dropping that single
    message is harmless, letting it kill the node is not."""
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    dropped = 0
    try:
        while rclpy.ok() and not stop.is_set():
            try:
                executor.spin_once(timeout_sec=0.05)
            except RuntimeError as error:
                if 'Unable to convert' not in str(error) or dropped > 1000:
                    raise
                dropped += 1
                node.get_logger().warning(f'dropped one undecodable message ({dropped})')
    finally:
        executor.remove_node(node)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stop = install_stop_handlers()
    node = SafetyGuard()
    try:
        spin_until_stopped(node, stop)
    finally:
        node.shutdown_zero()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
