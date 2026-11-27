"""planning_node (split mode, dummy = RulePlanner): /lf/map/patch + /lf/pose -> /lf/path.

Keeps a mirror of the map node's LineMap (patches pasted in order), the pose and travel from
LfPose, and runs the planner plugin's replan() after every per-frame patch, exactly the call
follower_node makes after each camera frame. The planner reads node.map, node.pose, node.travel,
node.path / path_end / t_point, node.restorer, node.controller.confirm_finish, to_odom / to_robot.
"""
import rclpy
from line_follower_interfaces.msg import LfPose, LinePath, LineMapPatch
from rclpy.node import Node

from .. import line_map, restore
from ..control import make_controller
from ..node_core import Frames, spin
from ..perception.stage import PerceptionStage
from ..planning import make_planner
from . import codec
from .common import POSE_QOS, RELIABLE, TOPIC_PATCH, TOPIC_PATH, TOPIC_POSE, fill, patch_of, setup


class PlanningNode(Frames, Node):
    def __init__(self):
        super().__init__('planning_node')
        setup(self)
        self.map = PerceptionStage.make_map(self)       # mirror of the map node's map
        if self.get_parameter('map_mode').value == 'logodds':
            self.get_logger().warning('map_mode logodds: the mirror has no evidence layer, '
                                      'map.confidence() falls back to conf')
        self.pose = None
        self.travel = 0.0
        self.pitch_f = 0.0
        self.rocking = False
        self.path = []
        self.path_end = 'open'
        self.t_point = None
        self.last_observation = None    # (perception output stays in the perception node)
        line_map.WIDE_TURNS = bool(self.get_parameter('trace_wide').value)
        rpath = self.get_parameter('restore_model').value
        self.restorer = restore.load(rpath) if rpath else None
        if rpath and self.restorer is None:
            self.get_logger().warning(f'restore_model not found: {rpath}')
        self.controller = make_controller(self.get_parameter('controller').value, self)   # confirm_finish
        self.planner = make_planner(self.get_parameter('planner').value, self)
        self.path_pub = self.create_publisher(LinePath, TOPIC_PATH, POSE_QOS)
        t = self.timing.wrap
        self.create_subscription(LineMapPatch, TOPIC_PATCH, t('patch', self.on_patch), RELIABLE)
        self.create_subscription(LfPose, TOPIC_POSE, t('pose', self.on_pose), POSE_QOS)
        self.get_logger().info(f"planning node started (planner={self.get_parameter('planner').value})")

    def on_pose(self, msg):
        self.pose = (msg.x, msg.y, msg.yaw)
        self.travel = msg.travel
        self.pitch_f = msg.pitch_f
        self.rocking = msg.rocking

    def on_patch(self, msg):
        codec.paste_patch(self.map, patch_of(msg))
        if msg.full or self.pose is None:
            return                      # full patches only resync the mirror
        self.planner.replan()
        out = LinePath()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'odom'
        self.path_pub.publish(fill(out, codec.make_path(self.path, self.path_end, self.t_point)))


def main():
    rclpy.init()
    spin(PlanningNode())
