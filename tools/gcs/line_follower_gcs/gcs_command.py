"""Async command broker and compatible one-shot CLI (command:=kill/start).

Default command:=listen serves UI requests. KILL is a software stop, not a physical E-STOP.
BagCommand at /gcs/recorder/command controls the dev recorder; absence yields a timeout event.
"""
import time
import signal
import threading
import uuid
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.clock import Clock, ClockType
from line_follower_interfaces.msg import GcsSnapshot, GcsCommandRequest, GcsCommandEvent, MissionState
from line_follower_interfaces.srv import MissionCommand, BagCommand

COMMANDS = {'start': MissionCommand.Request.START, 'kill': MissionCommand.Request.KILL}


class GcsCommand(Node):
    def __init__(self):
        super().__init__('gcs_command')
        self.client = self.create_client(MissionCommand, '/mission/command')
        self.bag_client = self.create_client(BagCommand, '/gcs/recorder/command')
        self.run_id = ''
        self.pending = {}
        self.bag_timeout = self.declare_parameter('bag_timeout_sec', 10.).value
        self.timeout = self.declare_parameter('timeout_sec', 2.).value
        self.events = self.create_publisher(GcsCommandEvent, '/gcs/command_events', 20)
        self.create_subscription(GcsSnapshot, '/gcs/snapshot', self.snapshot, 10)
        self.create_subscription(MissionState, '/mission/state', self.mission, 10)
        self.create_subscription(GcsCommandRequest, '/gcs/command_requests', self.request, 20)
        self.create_timer(.05, self.poll, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.finished = False

    def snapshot(self, msg):
        self.run_id = msg.mission.run_id

    def mission(self, msg):
        self.run_id = msg.run_id

    def send(self, command, run_id='', request_id=''):
        msg = GcsCommandRequest()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.request_id = request_id or str(uuid.uuid4())
        msg.run_id = run_id or self.run_id
        msg.command = command
        self.request(msg)
        return msg.request_id

    def request(self, msg):
        if msg.request_id in self.pending:
            return
        # Fill missing UI IDs here as well so each dispatch has a stable audit identity.
        msg.request_id = msg.request_id or str(uuid.uuid4())
        msg.run_id = msg.run_id or self.run_id
        self.pending[msg.request_id] = [msg, time.monotonic(), None, None]

    def result(self, msg, outcome, reason):
        event = GcsCommandEvent()
        event.header.stamp = self.get_clock().now().to_msg()
        event.sent_stamp = msg.header.stamp
        event.request_id, event.run_id, event.command = msg.request_id, msg.run_id, msg.command
        event.outcome, event.reason = outcome, reason
        self.events.publish(event)
        self.get_logger().info(f'{msg.command}: {outcome} request_id={msg.request_id} run_id={msg.run_id} reason={reason}')
        self.finished = True

    def poll(self):
        for key, item in list(self.pending.items()):
            msg, started, future, client = item
            if msg.command not in (*COMMANDS, 'bag_start', 'bag_stop'):
                self.result(msg, 'rejected', 'unknown command')
                del self.pending[key]
                continue
            if future is not None and future.done():
                try:
                    res = future.result()
                    self.result(msg, 'accepted' if res.accepted else 'rejected', res.reason)
                except Exception as exc:
                    self.result(msg, 'rejected', str(exc))
                del self.pending[key]
                continue
            if time.monotonic()-started >= (self.bag_timeout if future is not None and msg.command.startswith('bag_') else self.timeout):
                if future is not None:
                    client.remove_pending_request(future)
                self.result(msg, 'timeout', 'service unavailable or response deadline exceeded')
                del self.pending[key]
                continue
            if future is None:
                is_bag = msg.command.startswith('bag_')
                client = self.bag_client if is_bag else self.client
                if client.service_is_ready():
                    req = BagCommand.Request() if is_bag else MissionCommand.Request()
                    req.request_id, req.run_id = msg.request_id, msg.run_id
                    if is_bag:
                        req.command = BagCommand.Request.START if msg.command == 'bag_start' else BagCommand.Request.STOP
                        req.sent_stamp = msg.header.stamp
                    else:
                        req.command = COMMANDS[msg.command]
                        req.sent_stamp = msg.header.stamp
                    item[2], item[3] = client.call_async(req), client


def main():
    stopped = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stopped.set())
    signal.signal(signal.SIGTERM, lambda *_: stopped.set())
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = GcsCommand()
    command = node.declare_parameter('command', 'listen').value
    # Give the broker one snapshot period to learn the current run for CLI calls.
    deadline = time.monotonic()+1.
    if command != 'listen':
        while rclpy.ok() and not stopped.is_set() and time.monotonic() < deadline and not node.run_id:
            rclpy.spin_once(node, timeout_sec=.05)
        node.send(command)
    try:
        while rclpy.ok() and not stopped.is_set() and (command == 'listen' or not node.finished):
            rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
