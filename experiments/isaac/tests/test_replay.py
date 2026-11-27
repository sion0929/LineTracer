import io
import math
import random
import unittest

import numpy as np

from experiments.isaac import drive_core
from experiments.isaac.run_map_dump import world_xy


class ReplayTest(unittest.TestCase):
    def setUp(self):
        self.score = drive_core.Score([(0, 0), (1, 0), (1, 2)], [0, 1, 3], 100)
        ramp = dict(x=[0, 1, 1.5, 2], y=[-0.2, 0.2], height_m=0.1)
        self.replay = drive_core.Replay(self.score, [ramp], 0.02, 0.3)

    def test_arc_length_steps_and_ramp(self):
        pose, distance = self.replay.at_tick(50)
        self.assertAlmostEqual(distance, 0.5)
        self.assertAlmostEqual(pose.x, 0.5)
        self.assertAlmostEqual(pose.z, 0.07)
        self.assertAlmostEqual(pose.pitch, math.atan(0.1))

    def test_tangent_yaw_at_corner_and_after(self):
        pose, _ = self.replay.at_tick(100)
        self.assertAlmostEqual(pose.yaw, math.pi / 2)
        pose, _ = self.replay.at_tick(150)
        self.assertAlmostEqual(pose.y, 0.5)
        self.assertAlmostEqual(pose.yaw, math.pi / 2)

    def test_endpoint_is_rendered_and_clamped(self):
        pose, distance = self.replay.at_tick(299)
        self.assertLess(distance, 3)
        end, distance = self.replay.at_tick(301)
        self.assertEqual(distance, 3)
        self.assertEqual((end.x, end.y, end.yaw), (1, 2, math.pi / 2))
        self.assertEqual(self.replay.at_tick(500), (end, distance))

    def test_drive_ignores_commands_and_transports_truth_with_noisy_odom(self):
        class Stream(io.BytesIO):
            def recv_into(self, buffer):
                return self.readinto(buffer)

        commands = Stream()
        for tick in range(4):
            commands.write(drive_core.COMMAND_PACKET.pack(
                drive_core.COMMAND_MAGIC, tick, -0.26, 1.82))
        commands.seek(0)
        frames = Stream()

        class Peer:
            recv_into = commands.recv_into
            sendall = frames.write

        start = drive_core.Pose(5, -3, 0.02, 0, math.pi / 2)
        score = drive_core.Score([(5, -3), (5, -2.75)], [0, 0.25], 1)
        replay = drive_core.Replay(score, (), start.z, 3)
        rgb = bytes(drive_core.FRAME_BYTES)
        reason = drive_core.drive(
            Peer(), lambda pose, tick: rgb, start, (), score, replay=replay,
            relative_odom=True, noise=random.Random(42), position_std=0.01, yaw_std=0.02,
        )
        self.assertEqual(reason, "finished")
        self.assertTrue(score.complete)
        frames.seek(0)
        for tick, distance in enumerate((0, 0.1, 0.2, 0.25)):
            frame = drive_core.receive_frame(frames)
            self.assertEqual(frame.tick, tick)
            self.assertEqual(frame.truth, (5, -3 + distance, start.z, start.yaw))
            self.assertNotEqual(frame.position, (distance, 0, 0))
            self.assertAlmostEqual(frame.twist.v, (0, 3, 3, 1.5)[tick])
            self.assertAlmostEqual(frame.twist.w, 0)
        with self.assertRaises(EOFError):
            drive_core.receive_frame(frames)


class WorldTransformTest(unittest.TestCase):
    def test_rotated_start_transforms_and_round_trips(self):
        xy = np.array([[0, 0], [1, 0], [0, 2]])
        origin = (5, -3, math.pi / 2)
        world = world_xy(xy, origin)
        np.testing.assert_allclose(world, [[5, -3], [5, -2], [3, -3]])
        np.testing.assert_allclose(world_xy(world, origin, inverse=True), xy, atol=1e-12)
        self.assertEqual(world_xy(np.empty((0, 2)), origin).shape, (0, 2))
