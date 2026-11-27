"""map_node (split mode): PerceptionFrame -> LineMap -> /lf/map/patch (+ /lf/map for rviz).

Applies each frame with the same LineMap.freeze / update calls follower_node makes and publishes
the window that changed (one patch per frame, empty when nothing changed, so planning replans
once per camera frame as in follower_node). Once a second the whole written extent goes out as a
full patch (full = true) for late joiners; /lf/map is a 4 m x 4 m, 1 cm OccupancyGrid at 2 Hz.
"""
import rclpy
from line_follower_interfaces.msg import LfPose, LineMapPatch, PerceptionFrame
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node

from ..node_core import spin
from ..perception.stage import PerceptionStage
from . import codec
from .common import (POSE_QOS, RELIABLE, TOPIC_FRAME, TOPIC_MAP, TOPIC_PATCH, TOPIC_POSE, fill,
                     frame_of, setup)


class MapNode(Node):
    def __init__(self):
        super().__init__('map_node')
        setup(self)
        self.map = PerceptionStage.make_map(self)
        self.extent = None              # union of every window written so far
        self.pose = None
        self.bad_grid = False
        self.patch_pub = self.create_publisher(LineMapPatch, TOPIC_PATCH, RELIABLE)
        self.grid_pub = self.create_publisher(OccupancyGrid, TOPIC_MAP, 1)
        t = self.timing.wrap
        self.create_subscription(PerceptionFrame, TOPIC_FRAME, t('frame', self.on_frame), RELIABLE)
        self.create_subscription(LfPose, TOPIC_POSE, self.on_pose, POSE_QOS)
        self.create_timer(1.0, t('full_patch', self.publish_full))
        self.create_timer(0.5, t('occupancy', self.publish_grid))
        self.get_logger().info(f"map node started (map_mode={self.get_parameter('map_mode').value})")

    def on_pose(self, msg):
        self.pose = (msg.x, msg.y, msg.yaw)

    def on_frame(self, msg):
        frame = frame_of(msg)
        if not codec.grid_matches(frame):
            if not self.bad_grid:
                self.get_logger().error(
                    f"frame BEV grid {frame['rows']}x{frame['cols']} (x_far {frame['x_far']:.3f}) does not "
                    'match this node (set the same LF_X_FAR on every node): frames dropped')
                self.bad_grid = True
            return
        rect = codec.apply_frame(self.map, frame)
        self.extent = codec.union(self.extent, rect)
        out = LineMapPatch()
        out.header = msg.header
        self.patch_pub.publish(fill(out, codec.make_patch(self.map, rect)))

    def publish_full(self):
        if self.extent is None:
            return
        out = LineMapPatch()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'odom'
        self.patch_pub.publish(fill(out, codec.make_patch(self.map, self.extent, full=True)))

    def publish_grid(self):
        if self.pose is None or self.grid_pub.get_subscription_count() == 0:
            return
        data, ox, oy, n = codec.occupancy(self.map, self.pose[0], self.pose[1])
        out = OccupancyGrid()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'odom'
        out.info.resolution = 0.01
        out.info.width = out.info.height = n
        out.info.origin.position.x, out.info.origin.position.y = ox, oy
        out.info.origin.orientation.w = 1.0
        out.data = data.ravel().tolist()
        self.grid_pub.publish(out)


def main():
    rclpy.init()
    spin(MapNode())
