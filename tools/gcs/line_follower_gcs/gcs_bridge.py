"""Dev-only snapshot relay and link/camera monitoring; never commands the rover."""
import time
import signal
import threading
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from line_follower_interfaces.msg import GcsSnapshot, GcsLinkStatus
from .model import LinkMonitor


class GcsBridge(Node):
    def __init__(self):
        super().__init__('gcs_bridge')
        self.monitor = LinkMonitor(self.declare_parameter('stale_sec', 1.).value,
                                   self.declare_parameter('lost_sec', 2.).value)
        self.camera_period = 1./max(.1, self.declare_parameter('camera_rate_hz', 6.).value)
        self.last_camera = 0.
        self.snapshot_pub = self.create_publisher(GcsSnapshot, '/gcs/snapshot', 10)
        self.link_pub = self.create_publisher(GcsLinkStatus, '/gcs/link', 10)
        self.camera_pub = self.create_publisher(Image, '/gcs/camera', qos_profile_sensor_data)
        self.create_subscription(GcsSnapshot, '/mission/gcs_snapshot', self.snapshot, 10)
        self.create_subscription(Image, '/camera/image_raw', self.camera, qos_profile_sensor_data)
        self.create_timer(.1, self.health, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def snapshot(self, msg):
        self.monitor.receive(time.monotonic())
        self.snapshot_pub.publish(msg)

    def camera(self, msg):
        now = time.monotonic()
        if now-self.last_camera >= self.camera_period:
            self.last_camera = now
            self.camera_pub.publish(msg)

    def health(self):
        state, age, hz = self.monitor.status(time.monotonic())
        msg = GcsLinkStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state, msg.last_received_age, msg.snapshot_rate_hz = state, age, hz
        msg.received_count = self.monitor.count
        self.link_pub.publish(msg)


def main():
    stopped = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stopped.set())
    signal.signal(signal.SIGTERM, lambda *_: stopped.set())
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = GcsBridge()
    try:
        while rclpy.ok() and not stopped.is_set():
            rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
