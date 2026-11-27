"""Line follower node.

camera image --(vision)--> ground line mask --(line_map)--> line map in the odom frame
line map --(trace)--> path ahead of the robot --(pure pursuit)--> /cmd_vel

Only robot topics are used: /camera/image_raw, /odom, /cmd_vel.

Everything runs in this one process. The same code split into four nodes (perception, map,
planning, control) lives in line_follower/split/ (docs/architecture/NODE_SPLIT.md); the
parameters, state estimation and control tick are shared through node_core.py.
"""
import math
import os
from pathlib import Path
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
from . import line_map
from . import restore
from .line_map import RES, C0
from .perception import make_perception
from .control import make_controller
from . import terrain
from .scanmatch import ScanMatcher
from .perception.base import PerceptionOutput
from .perception.stage import PerceptionStage, REST_PITCH  # noqa: F401
from .planning import make_planner
from .node_core import (ControlLoop, PoseEstimator, declare_follower_parameters,  # noqa: F401
                        path_len, rng, spin, wrap)


class FollowerNode(PoseEstimator, ControlLoop, Node):
    def __init__(self):
        super().__init__('follower_node')
        declare_follower_parameters(self)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.dbg_pub = self.create_publisher(Image, '/line_follower/debug', 1)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data)

        self.map = PerceptionStage.make_map(self)
        self.init_pose_state()      # pose, hist, imu, pitch_f, travel, rocking, v_cmd / w_cmd
        self.path = []              # line ahead of the robot, odom frame, ~2 cm spacing
        self.path_end = 'open'
        self.t_point = None         # finish T position (odom)
        self.init_control_state()   # done, blind, last_t
        self.last_obstacle = None   # latest BEV obstacle mask from the perception plugin (or None)
        self.last_observation = None  # latest PerceptionOutput (line, valid, obstacle) + .pitch
        self.perception = make_perception(self.get_parameter('perception').value, self)
        line_map.WIDE_TURNS = bool(self.get_parameter('trace_wide').value)
        rpath = self.get_parameter('restore_model').value
        self.restorer = restore.load(rpath) if rpath else None
        if rpath and self.restorer is None:
            self.get_logger().warning(f'restore_model not found: {rpath}')
        self.perception_stage = PerceptionStage(self)
        self.planner = make_planner(self.get_parameter('planner').value, self)
        self.controller = make_controller(self.get_parameter('controller').value, self)
        self.create_timer(0.02, self.control)
        self.get_logger().info(
            f"line follower started (perception={self.get_parameter('perception').value}, "
            f"controller={self.get_parameter('controller').value})")

    # ---- frames, state estimation: node_core.PoseEstimator ---------------
    # ---- perception + path ----------------------------------------------
    def on_image(self, msg):
        # role 1 perception: frame -> lane map; role 2 planning: lane map -> local path
        if not self.perception_stage.process(msg):
            return
        if self.get_parameter('restore_mode').value == 'map':
            self.perception_stage.complete_map()
        self.replan()
        if self.get_parameter('debug_image').value:
            self.publish_debug()

    def replan(self):
        self.planner.replan()           # dev tools (oracle, shadow logger) override this hook

    def prune(self):
        self.planner.prune()

    # ---- control: node_core.ControlLoop (stop, control) -------------------
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
    spin(node, (lambda: node.cmd_pub.publish(Twist()),
                lambda: node.perception_stage.log.flush()))
