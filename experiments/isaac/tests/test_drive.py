import json
import math
import os
from pathlib import Path
import socket
from contextlib import contextmanager
import unittest
from concurrent.futures import ThreadPoolExecutor

from experiments.isaac.drive_core import (
    FRAME_BYTES, HZ, Command, Pose, Score, advance, drive, ground_height,
    ground_pitch, load_map, orientation, receive_command, receive_frame, send_command,
)


class KinematicsTest(unittest.TestCase):
    def test_clamped_straight_reverse_and_arc(self):
        origin = Pose(0, 0, 0, 0, 0)
        pose, cmd = advance(origin, Command(5, 0), (), 0)
        self.assertEqual(cmd, Command(0.26, 0))
        self.assertAlmostEqual(pose.x, 0.26 / HZ)
        pose, cmd = advance(origin, Command(-5, -8), (), 0)
        self.assertEqual(cmd, Command(-0.26, -1.82))
        self.assertAlmostEqual(pose.yaw, -1.82 / HZ)
        self.assertAlmostEqual(pose.x, cmd.v / cmd.w * math.sin(pose.yaw))
        self.assertAlmostEqual(pose.y, cmd.v / cmd.w * (1 - math.cos(pose.yaw)))

    def test_ramp_profiles_and_heading(self):
        ramp = dict(x0=0, y0=0, heading=math.pi / 2, height_m=0.1,
                    up=1, deck=0.5, down=1, width=0.6)
        self.assertAlmostEqual(ground_height([ramp], 0, 0.5), 0.05)
        self.assertAlmostEqual(ground_height([ramp], 0, 1.25), 0.1)
        self.assertAlmostEqual(ground_height([ramp], 0, 2), 0.05)
        self.assertEqual(ground_height([ramp], 0.4, 0.5), 0)
        pitch = math.atan(0.1)
        self.assertAlmostEqual(ground_pitch([ramp], 0, 0.5, math.pi / 2), pitch)
        self.assertAlmostEqual(ground_pitch([ramp], 0, 0.5, -math.pi / 2), -pitch)
        self.assertAlmostEqual(ground_pitch([ramp], 0, 0.5, 0), 0)
        legacy = dict(x=[0, 1, 1.5, 2.5], y=[0, 0.6], height_m=0.1)
        self.assertAlmostEqual(ground_height([legacy], 0.5, 0.3), 0.05)

    def test_imu_quaternion_recovers_nose_up_and_yaw(self):
        yaw, pitch = 1.72, 0.19
        x, y, z, w = orientation(yaw, pitch)
        self.assertAlmostEqual(x*x + y*y + z*z + w*w, 1.0)
        self.assertAlmostEqual(-math.asin(2 * (w*y - z*x)), pitch)
        self.assertAlmostEqual(math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z)), yaw)

    def test_repo_map_inputs(self):
        official = load_map('official')
        self.assertAlmostEqual(official.start.x, 0.33)
        self.assertAlmostEqual(ground_height(official.ramps, 1.32, 0.33), 0.06)
        self.assertAlmostEqual(ground_pitch(official.ramps, 0.88, 0.33, 0),
                               math.atan(0.06 / 0.44))
        root = Path(__file__).resolve().parents[2]
        path = next((root / 'worlds/suites/decoy1000').glob('part*/maps/decoy_0000.json'))
        data = json.loads(path.read_text())
        generated = load_map(str(path))
        self.assertEqual(generated.start.x, data['start_pose'][0])
        self.assertEqual(generated.arc[-1], data['arc_m'][-1])


class ScoringTest(unittest.TestCase):
    def test_finish_and_crossing(self):
        score = Score([(0, 0), (1, 0), (1, 1), (0.2, -0.1)], [0, 1, 2, 3.36], 300)
        self.assertIsNone(score.observe(0, 0.2, 0))
        self.assertAlmostEqual(score.progress, 0.2)
        self.assertAlmostEqual(score.history[-1][4], 0)
        score = Score([(0, 0), (1, 0)], [0, 1], 300)
        for i in range(1, 9):
            self.assertIsNone(score.observe(i, i / 10, 0))
        self.assertEqual(score.observe(9, 0.89, 0), 'finished')

    def test_off_line_reset_stall_and_limit(self):
        score = Score([(0, 0), (1, 0)], [0, 1], 300)
        self.assertIsNone(score.observe(0, 0, 0.13))
        self.assertIsNone(score.observe(0.5, 0, 0.12))
        self.assertIsNone(score.observe(1, 0, 0.13))
        self.assertIsNone(score.observe(1.59, 0, 0.13))
        self.assertIn('off line', score.observe(1.61, 0, 0.13))
        self.assertEqual(score.progress, 0)
        score = Score([(0, 0), (1, 0)], [0, 1], 300)
        self.assertIn('stalled', score.observe(20.01, 0, 0))
        score = Score([(0, 0), (1, 0)], [0, 1], 2)
        self.assertEqual(score.observe(2, 0, 0), 'time limit')


class PipePeer:
    """Socket-shaped byte stream for sandboxes that restrict socket syscalls."""

    def __init__(self, reader, writer):
        self.reader, self.writer = reader, writer

    def recv_into(self, buffer):
        data = os.read(self.reader, len(buffer))
        buffer[:len(data)] = data
        return len(data)

    def sendall(self, data):
        view = memoryview(data)
        while view:
            view = view[os.write(self.writer, view):]

    def close_writer(self):
        if self.writer is not None:
            os.close(self.writer)
            self.writer = None


@contextmanager
def pipe_pair():
    r1, w1 = os.pipe()
    r2, w2 = os.pipe()
    a, b = PipePeer(r1, w2), PipePeer(r2, w1)
    try:
        yield a, b
    finally:
        a.close_writer()
        b.close_writer()
        os.close(r1)
        os.close(r2)


class ProtocolTest(unittest.TestCase):
    def exercise_closed_loop(self, peer, relay_peer):
        poses = []
        frames = []
        rgb = bytes([255, 20, 0]) * (FRAME_BYTES // 3)

        def render(pose, tick):
            poses.append(pose)
            return rgb

        def relay():
            try:
                for tick in range(4):
                    frame = receive_frame(relay_peer)
                    frames.append(frame)
                    self.assertEqual(frame.tick, tick)
                    self.assertEqual(frame.rgb, rgb)
                    send_command(relay_peer, tick, Command(0.26 if tick == 0 else 0, 0))
            finally:
                if isinstance(relay_peer, PipePeer):
                    relay_peer.close_writer()
                else:
                    relay_peer.shutdown(socket.SHUT_WR)

        score = Score([(0, 0), (1, 0)], [0, 1], 3 / HZ)
        with ThreadPoolExecutor(max_workers=1) as executor:
            future = executor.submit(relay)
            try:
                self.assertEqual(drive(peer, render, Pose(0, 0, 0, 0, 0), (), score), 'time limit')
            finally:
                if isinstance(peer, PipePeer):
                    peer.close_writer()
                else:
                    peer.shutdown(socket.SHUT_WR)
            future.result(timeout=2)
        self.assertAlmostEqual(poses[1].x, 0.26 / HZ)
        self.assertEqual(poses[1:], [poses[1]] * 3)
        self.assertEqual(frames[1].stamp_ns, 33_333_333)
        self.assertEqual(frames[1].position, (poses[1].x, 0, 0))
        self.assertEqual(frames[1].twist, Command(0.26, 0))
        self.assertEqual(frames[1].angular_velocity, (0, 0, 0))
        self.assertAlmostEqual(frames[1].acceleration[0], 0.26 * HZ)
        self.assertAlmostEqual(frames[1].acceleration[2], 9.81)

    def test_fake_renderer_and_relay_over_pipes(self):
        with pipe_pair() as (peer, relay):
            self.exercise_closed_loop(peer, relay)

    def test_fake_renderer_and_relay_over_tcp(self):
        try:
            server = socket.socket()
        except PermissionError as error:
            self.skipTest(f"TCP socket unavailable: {error}")
        with server:
            try:
                server.bind(('127.0.0.1', 0))
                server.listen(1)
                relay = socket.create_connection(server.getsockname(), timeout=2)
            except PermissionError as error:
                self.skipTest(f"TCP loopback unavailable: {error}")
            server.settimeout(2)
            with relay:
                peer, _ = server.accept()
                with peer:
                    peer.settimeout(2)
                    self.exercise_closed_loop(peer, relay)

    def test_fragmentation_mismatched_tick_and_truncation(self):
        class Fragmented:
            def __init__(self, peer):
                self.peer = peer

            def recv_into(self, buffer):
                return self.peer.recv_into(buffer[:3])

        with pipe_pair() as (a, b):
            send_command(a, 4, Command(0.1, -0.2))
            self.assertEqual(receive_command(Fragmented(b), 4), Command(0.1, -0.2))
            send_command(a, 5, Command(0, 0))
            with self.assertRaisesRegex(ValueError, 'tick'):
                receive_command(b, 6)
            send_command(a, 6, Command(float('nan'), 0))
            with self.assertRaisesRegex(ValueError, 'Non-finite'):
                receive_command(b, 6)
            a.sendall(b'KL')
            a.close_writer()
            with self.assertRaisesRegex(ValueError, "Truncated"):
                receive_command(b, 6)
            with self.assertRaises(EOFError):
                receive_command(b, 6)
