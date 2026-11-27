"""perception_node (split mode): state estimation + PerceptionStage -> /lf/perception/frame, /lf/pose.

Runs the same PerceptionStage.process() as follower_node on every camera frame, but node.map is a
FrameSink: the map writes it asks for (freeze, update) go out as one PerceptionFrame for the map
node. The perception log (perception_log / LF_PERCEPTION_LOG) is written here.
"""
import rclpy
from geometry_msgs.msg import Twist
from line_follower_interfaces.msg import LfPose, PerceptionFrame
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu

from ..node_core import PoseEstimator, spin
from ..perception import make_perception
from ..perception.stage import PerceptionStage
from . import codec
from .common import POSE_QOS, RELIABLE, TOPIC_FRAME, TOPIC_POSE, fill, pose_payload, setup


class PerceptionNode(PoseEstimator, Node):
    def __init__(self):
        super().__init__('perception_node')
        setup(self)
        self.frame_pub = self.create_publisher(PerceptionFrame, TOPIC_FRAME, RELIABLE)
        self.pose_pub = self.create_publisher(LfPose, TOPIC_POSE, POSE_QOS)
        t = self.timing.wrap
        self.create_subscription(Image, '/camera/image_raw', t('image', self.on_image), qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', t('odom', self.on_odom), qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu', t('imu', self.on_imu), qos_profile_sensor_data)
        # the last command, for pose_at's extrapolation of images newer than the last odometry
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)

        self.map = codec.FrameSink()
        self.init_pose_state()
        self.done = False
        self.slope_state = 'flat'
        self.last_obstacle = None
        self.last_observation = None
        self.restorer = None           # restoration runs in planning ('path' mode)
        self.perception = make_perception(self.get_parameter('perception').value, self)
        self.perception_stage = PerceptionStage(self)
        self.get_logger().info(f"perception node started (perception={self.get_parameter('perception').value})")

    def on_cmd(self, msg):
        self.v_cmd, self.w_cmd = msg.linear.x, msg.angular.z

    def on_odom(self, msg):
        super().on_odom(msg)
        out = LfPose()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'odom'
        self.pose_pub.publish(fill(out, pose_payload(self)))

    def on_image(self, msg):
        self.map.reset()
        if not self.perception_stage.process(msg):
            return
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        obs = self.last_observation
        frame = codec.make_frame(self.map, self.pose_at(ts), obs.pitch if obs is not None else None,
                                 self.pitch_f, getattr(self, 'elev', 0.0), self.travel, self.slope_state)
        out = PerceptionFrame()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'odom'
        self.frame_pub.publish(fill(out, frame))


def main():
    rclpy.init()
    node = PerceptionNode()
    spin(node, (lambda: node.perception_stage.log.flush(),))
