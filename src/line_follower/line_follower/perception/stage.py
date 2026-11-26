"""Perception stage (team role 1): camera frame -> line map handed to planning.

Owns everything that turns pixels into the lane map: BEV projection (near range from a
plugin, optional full-resolution far band), slope handling (IMU floor / terrain profile),
pose-consistent map writing, and line restoration. Planning reads node.map only
(grid = seen line, seen = looked-at area, restored = learned guess where unseen).
State stays on the node (shared with planning / control / dev tools).
"""
import math

import cv2
import numpy as np

from .. import terrain, vision
from ..scanmatch import ScanMatcher
from .base import PerceptionOutput

REST_PITCH = math.radians(0.33)   # nose-up tilt of the robot standing on flat ground


class PerceptionStage:
    def __init__(self, node):
        self.node = node

    def process(self, msg):
        """One camera frame -> map update. Returns False when the frame was skipped."""
        node = self.node
        if node.pose is None or node.done:
            return False
        if msg.encoding in ('rgb8', 'bgr8'):
            img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
            bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else img
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        elif msg.encoding == 'mono8':
            gray = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width)
            bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        else:
            return False
        if gray.shape != (vision.H, vision.W):
            gray = cv2.resize(gray, (vision.W, vision.H))
            bgr = cv2.resize(bgr, (vision.W, vision.H))
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # while the body rocks or the robot is on the hill, only trust the near ground
        gp = node.get_parameter
        steady = (node.rock < gp('steady_rock').value
                  and abs(node.pitch_f) < math.radians(gp('steady_pitch_deg').value))
        node.rocking = (not steady) and abs(node.pitch_f) < math.radians(gp('steady_pitch_deg').value)
        x_far = vision.X_FAR if steady else min(vision.X_FAR, gp('x_far_unsteady').value)
        # raised deck: after a climb and before the matching descent the ground beyond the
        # edge lies below the plane the robot stands on, so far lines project too close
        # (3 cm lower at 0.8 m -> ~18 cm short). Trust only the near ground up there.
        if node.pitch_f > math.radians(3.0):
            node.on_deck = True
        elif node.pitch_f < -math.radians(3.0):
            node.on_deck = False
        deck = gp('deck_xfar').value
        if deck > 0.0 and (getattr(node, 'on_deck', False) or getattr(node, 'elev', 0.0) > 0.01):
            x_far = min(x_far, deck)
        add_only = None
        if (not steady and gp('far_add_unsteady').value and x_far < vision.X_FAR
                and abs(node.pitch_f) < math.radians(gp('steady_pitch_deg').value)):
            # rocking on flat ground: the far part is projected well but the pose used to
            # write it is less certain -> let it only fill line the map has not seen yet
            add_only = np.zeros((vision.BH, vision.BW), np.uint8)
            add_only[:max(0, int(round((vision.X_FAR - x_far) / vision.RES)))] = 255
            x_far = vision.X_FAR
        out = None
        on_hill = abs(node.pitch_f) >= math.radians(gp('steady_pitch_deg').value)
        if on_hill and gp('terrain_down_only').value and node.pitch_f > 0.0:
            on_hill = False                                  # climbing: keep the original projection
        bend_z = gp('terrain_bend_z').value
        if not on_hill and bend_z > 0.0:
            # standing level (ramp deck, foot of a ramp) but the ground ahead drops or rises
            prof_b, _ = terrain.estimate(gray, REST_PITCH, gp('terrain_calib').value)
            if (prof_b is not None and prof_b.b is not None
                    and abs(prof_b.ds) * max(0.0, vision.X_FAR - prof_b.b) > bend_z):
                line, valid = terrain.line_mask_profile(gray, REST_PITCH, prof_b, x_far)
                out = PerceptionOutput(line=line, valid=valid, obstacle=None)
                add_only = None
        if on_hill and gp('terrain_hill').value:
            # the ground ahead is not the plane the robot stands on (crest / foot of a ramp):
            # measure its height profile from the line width instead of assuming a plane
            prof, _ = terrain.estimate(gray, REST_PITCH, gp('terrain_calib').value)
            if prof is not None:
                node.terrain_prof, node.terrain_t = prof, ts
            elif ts - getattr(node, 'terrain_t', -9.0) < 0.4:
                prof = node.terrain_prof
            if prof is not None:
                xt = min(vision.X_FAR, gp('x_far_terrain').value)
                line, valid = terrain.line_mask_profile(gray, REST_PITCH, prof, xt)
                out = PerceptionOutput(line=line, valid=valid, obstacle=None)
                add_only = None
        elev = getattr(node, 'elev', 0.0)
        if out is None and gp('imu_floor').value and (on_hill or elev > gp('imu_floor_min_h').value):
            # the floor ahead is level in the world: in the body frame it lies at
            # z = -(h + x sin b) / cos b (h: wheel height above the floor, b: IMU body pitch).
            # The ground ahead is the higher of the plane under the wheels and that floor.
            b = node.imu_pitch_at(ts) - REST_PITCH
            cb, sb = math.cos(b), math.sin(b)

            def ground(xs, h=elev, cb=cb, sb=sb):
                return np.maximum(0.0, (-h - xs * sb) / cb)
            line, valid = terrain.line_mask_profile(gray, REST_PITCH, ground, min(x_far, vision.X_FAR))
            out = PerceptionOutput(line=line, valid=valid, obstacle=None)
        far = None
        node.far_stats = getattr(node, 'far_stats', [0, 0])
        node.far_stats[1] += 1
        if out is None and gp('far_band').value and steady and x_far > gp('far_split').value \
                and add_only is None:
            node.far_stats[0] += 1
            # dual resolution: near part from the plugin (low res), far band from full res
            split = gp('far_split').value
            if not hasattr(node, 'far_net'):
                from .. import far_band
                node.far_net = far_band.load(gp('far_model').value) if gp('far_model').value else None
            th = node.pitch_at(ts)
            limg = node.far_net.mask(bgr, th) if node.far_net is not None else None
            far = vision.far_band_mask(gray, th, split, x_far, line_img=limg, thin=gp('far_thin').value)
            x_far = split
        if out is None:
            out = node.perception.process(bgr, gray, node.pitch_at(ts), x_far)
        if far is not None:
            line = np.maximum(out.line, far[0])
            valid = np.maximum(out.valid, far[1])
            out = PerceptionOutput(line=line, valid=valid, obstacle=out.obstacle)
            add_only = far[1] if gp('far_add_only').value else None
        node.last_obstacle = out.obstacle
        out.pitch = node.pitch_at(ts)
        node.last_observation = out
        pose_ts = node.pose_at(ts)
        if gp('scan_match').value and not on_hill and pose_ts is not None:
            if not hasattr(node, 'scanmatch'):
                node.scanmatch = ScanMatcher(gain=gp('scan_gain').value)
            if node.scanmatch.correct(node, out.line, out.valid, pose_ts):
                pose_ts = node.pose_at(ts)
        if gp('freeze_passed').value and pose_ts is not None and node.travel > 0.05:
            node.map.freeze(pose_ts[0], pose_ts[1], gp('freeze_radius').value)
        skip = gp('map_skip_pitch_deg').value
        if not (skip > 0.0 and abs(node.pitch_f) > math.radians(skip)):
            # (while the body tips over a ramp edge the camera-to-ground angle is unknown:
            # such frames drew corners 13-19 cm off. Keep the map from the good frames.)
            node.map.update(out.line, out.valid, pose_ts, add_only,
                            far_mask=far[1] if far is not None else None)
        return True


    def complete_map(self):
        """Line restoration into the map ('map' mode): the learned route where the camera has not
        looked is written to node.map.restored, so planning traces through it like seen line."""
        node = self.node
        if node.restorer is None or node.pose is None or node.travel < 0.3:
            return
        gp = node.get_parameter
        now = node.get_clock().now().nanoseconds * 1e-9
        if now - getattr(self, 't_restore', -1e9) < gp('restore_period').value:
            return                                   # the net costs ~10-70 ms: run it a few times a second
        self.t_restore = now
        node.map.write_restored(node.restorer, node.pose, gp('restore_thresh').value, gp('restore_len').value,
                                min_x=gp('restore_min_x').value)
