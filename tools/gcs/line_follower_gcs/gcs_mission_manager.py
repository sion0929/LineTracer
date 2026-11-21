"""Qt GCS: ROS spins in a background executor; only queued signals touch widgets."""
import signal
import threading
import time
import uuid
from PyQt5.QtCore import QObject, QTimer, pyqtSignal
from PyQt5.QtGui import QImage
from PyQt5.QtWidgets import QApplication
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from line_follower_interfaces.msg import GcsSnapshot, GcsLinkStatus, GcsCommandEvent, GcsCommandRequest, RecordingStatus
from .adapter import display_snapshot
from .window import GcsWindow
from .replay_worker import ReplayWorker


class Signals(QObject):
    recording = pyqtSignal(object)
    archive = pyqtSignal(object)
    replay_frame = pyqtSignal(object)
    replay_error = pyqtSignal(str)
    snapshot = pyqtSignal(object)
    camera = pyqtSignal(object)
    link = pyqtSignal(object)
    command = pyqtSignal(object)


class GcsMissionManager(Node):
    def __init__(self, signals):
        super().__init__('gcs_mission_manager')
        self.signals = signals
        self.cv = CvBridge()
        self.requests = self.create_publisher(GcsCommandRequest, '/gcs/command_requests', 20)
        self.create_subscription(RecordingStatus, '/gcs/recording_status', self.recording, 10)
        self.create_subscription(GcsSnapshot, '/gcs/snapshot', self.snapshot, 10)
        self.create_subscription(Image, '/gcs/camera', self.camera, qos_profile_sensor_data)
        self.create_subscription(GcsLinkStatus, '/gcs/link', self.link, 10)
        self.create_subscription(GcsCommandEvent, '/gcs/command_events', self.command, 20)
        self.snapshots = self.frames = 0
        self.started = time.monotonic()
        self.create_timer(5., self.report, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def recording(self, msg):
        self.signals.recording.emit({'state': msg.state, 'path': msg.bag_path,
                                    'reason': msg.reason, 'bytes': msg.bytes_written})

    def snapshot(self, msg):
        self.snapshots += 1
        self.signals.snapshot.emit(display_snapshot(msg))

    def camera(self, msg):
        try:
            rgb = self.cv.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            image = QImage(rgb.data, rgb.shape[1], rgb.shape[0], rgb.strides[0], QImage.Format_RGB888).copy()
            self.frames += 1
            self.signals.camera.emit(image)
        except Exception as exc:
            self.get_logger().warning(f'camera conversion: {exc}', throttle_duration_sec=5.)

    def link(self, msg):
        self.signals.link.emit({'state': msg.state, 'rate': msg.snapshot_rate_hz, 'age': msg.last_received_age})

    def command(self, msg):
        self.signals.command.emit({'command': msg.command, 'outcome': msg.outcome, 'reason': msg.reason,
                                   'request_id': msg.request_id, 'run_id': msg.run_id,
                                   'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9})

    def report(self):
        elapsed = time.monotonic()-self.started
        self.get_logger().info(f'UI_RECEIVE snapshots={self.snapshots} camera={self.frames} '
                               f'elapsed={elapsed:.2f}s snapshot_hz={self.snapshots/elapsed:.2f} camera_hz={self.frames/elapsed:.2f}')


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    app = QApplication([])
    app.setApplicationName('Line Follower GCS')
    signals = Signals()
    node = GcsMissionManager(signals)
    window = GcsWindow()
    def convert_image(msg):
        rgb = CvBridge().imgmsg_to_cv2(msg, desired_encoding='rgb8')
        return QImage(rgb.data, rgb.shape[1], rgb.shape[0], rgb.strides[0], QImage.Format_RGB888).copy()
    replay = ReplayWorker(signals, convert_image)
    signals.archive.connect(window.archive_loaded)
    signals.replay_frame.connect(window.replay_frame)
    signals.replay_error.connect(window.replay_error)
    window.bag_requested.connect(lambda path: replay.request('open', path))
    window.seek_requested.connect(lambda seconds: replay.request('seek', seconds))
    replay.start()
    def rendered_snapshot(data):
        window.on_snapshot(data)
        node.get_logger().info(f'UI_RENDER snapshots={window.snapshot_count} camera={window.camera_count} '
                               f'state={data["mission"]["state"]}')
    signals.snapshot.connect(rendered_snapshot)
    signals.camera.connect(window.on_camera)
    signals.link.connect(window.on_link)
    signals.command.connect(window.on_command)
    signals.recording.connect(window.on_recording)
    def send(command):
        if window.replay_mode:
            return
        msg = GcsCommandRequest()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.request_id = str(uuid.uuid4())
        msg.command = command
        msg.run_id = window.model.current['mission']['run_id'] if window.model.current else ''
        node.requests.publish(msg)
    window.command_requested.connect(send)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, name='gcs-ros', daemon=True)
    thread.start()
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    signal.signal(signal.SIGTERM, lambda *_: app.quit())
    timer = QTimer()
    timer.timeout.connect(lambda: None)
    timer.start(100)
    render_timer = QTimer()
    def render_report():
        state = window.model.current['mission']['state'] if window.model.current else -1
        node.get_logger().info(f'UI_RENDER snapshots={window.snapshot_count} camera={window.camera_count} state={state}')
    render_timer.timeout.connect(render_report)
    render_timer.start(5000)
    window.show()
    try:
        app.exec_()
    finally:
        replay.close()
        node.report()
        executor.shutdown(timeout_sec=2.)
        thread.join(timeout=2.)
        node.destroy_node()
        rclpy.shutdown()
