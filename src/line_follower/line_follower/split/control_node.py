"""control_node (split mode, dummy = follower_node.control + RuleController): /lf/path + /lf/pose -> /cmd_vel.

The 50 Hz tick is node_core.ControlLoop.control(), the one follower_node runs. Passed path points
are dropped with the planner plugin's own prune() (the plugin is instantiated on this node).
"""
import rclpy
from geometry_msgs.msg import Twist
from line_follower_interfaces.msg import LfPose, LinePath
from rclpy.node import Node

from ..control import make_controller
from ..node_core import ControlLoop, Frames, spin
from ..planning import make_planner
from .codec import SLOPE_NAMES
from .common import POSE_QOS, TOPIC_PATH, TOPIC_POSE, setup


class ControlNode(ControlLoop, Frames, Node):
    def __init__(self):
        super().__init__('control_node')
        setup(self)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose = None
        self.travel = 0.0
        self.pitch_f = 0.0
        self.elev = 0.0
        self.rocking = False
        self.slope_state = 'flat'
        self.v_cmd = self.w_cmd = 0.0
        self.path = []
        self.path_end = 'open'
        self.t_point = None
        self.init_control_state()
        self.controller = make_controller(self.get_parameter('controller').value, self)
        self.planner = make_planner(self.get_parameter('planner').value, self)   # prune() only
        t = self.timing.wrap
        self.create_subscription(LinePath, TOPIC_PATH, t('path', self.on_path), POSE_QOS)
        self.create_subscription(LfPose, TOPIC_POSE, t('pose', self.on_pose), POSE_QOS)
        self.create_timer(0.02, t('control', self.control))
        self.get_logger().info(f"control node started (controller={self.get_parameter('controller').value})")

    def on_pose(self, msg):
        self.pose = (msg.x, msg.y, msg.yaw)
        self.travel = msg.travel
        self.pitch_f = msg.pitch_f
        self.elev = msg.elev
        self.rocking = msg.rocking
        self.slope_state = SLOPE_NAMES.get(msg.slope, 'flat')

    def on_path(self, msg):
        self.path = [(p.x, p.y) for p in msg.points]
        self.path_end = msg.end_state
        if msg.has_t_point and self.t_point is None:
            self.t_point = (msg.t_point.x, msg.t_point.y)

    def prune(self):
        self.planner.prune()


def main():
    rclpy.init()
    node = ControlNode()
    spin(node, (lambda: node.cmd_pub.publish(Twist()),))
