"""Split nodes (docs/architecture/NODE_SPLIT.md) build the same map and path as follower_node.

No ROS graph: the payload helpers in line_follower/split/codec.py are the code the nodes run;
messages are imitated by turning every bytes field into array('B') as rclpy hands it over.
"""
import array
import math
from types import SimpleNamespace

import numpy as np
import pytest

cv2 = pytest.importorskip('cv2')

from line_follower import vision  # noqa: E402
from line_follower.control import make_controller  # noqa: E402
from line_follower.line_map import LineMap  # noqa: E402
from line_follower.node_core import Frames  # noqa: E402
from line_follower.planning.rule import RulePlanner  # noqa: E402
from line_follower.split import codec  # noqa: E402


def transport(payload):
    """What the receiving node reads back from the message."""
    return {k: array.array('B', v) if isinstance(v, (bytes, bytearray)) else v for k, v in payload.items()}


# ---- (a) packbits payload ---------------------------------------------------------
@pytest.mark.parametrize('shape', [(vision.BH, vision.BW), (7, 13), (1, 1)])
def test_pack_unpack_roundtrip_binary(shape):
    rng = np.random.default_rng(0)
    m = (rng.random(shape) > 0.7).astype(np.uint8) * 255
    data = codec.pack_mask(m)
    assert len(data) == (m.size + 7) // 8                   # packed, not raw
    back = codec.unpack_mask(array.array('B', data), *shape)
    assert back.dtype == np.uint8 and np.array_equal(back, m)


def test_pack_unpack_soft_mask_is_raw_and_exact():
    rng = np.random.default_rng(1)
    m = rng.integers(0, 256, (vision.BH, vision.BW), dtype=np.uint8)
    data = codec.pack_mask(m)
    assert len(data) == m.size
    assert np.array_equal(codec.unpack_mask(data, *m.shape), m)


def test_pack_absent_and_empty():
    assert codec.pack_mask(None) == b''
    assert codec.unpack_mask(array.array('B'), 4, 4) is None
    zeros = np.zeros((5, 9), np.uint8)
    assert np.array_equal(codec.unpack_mask(codec.pack_mask(zeros), 5, 9), zeros)
    with pytest.raises(ValueError):
        codec.unpack_mask(b'\x00\x01\x02', 5, 9)


# ---- (b) equivalence: one LineMap vs frame -> map node -> patch -> planning mirror ----
TRACK = np.array([(-0.3, 0.0), (1.4, 0.0), (1.4, 1.6)])     # straight, then a 90 deg left turn


def dist_to_track(X, Y):
    d = np.full(X.shape, np.inf)
    for (ax, ay), (bx, by) in zip(TRACK[:-1], TRACK[1:]):
        ux, uy = bx - ax, by - ay
        t = np.clip(((X - ax) * ux + (Y - ay) * uy) / (ux * ux + uy * uy), 0.0, 1.0)
        d = np.minimum(d, np.hypot(X - ax - t * ux, Y - ay - t * uy))
    return d


def poses(n=60, step=0.045):
    """Moving along the track, heading rounding the corner over ~0.2 m."""
    seg = np.hypot(*np.diff(TRACK, axis=0).T)
    out = []
    for k in range(n):
        s = 0.3 + k * step
        if s < seg[0]:
            x, y = TRACK[0][0] + s, 0.0
        else:
            x, y = TRACK[1][0], s - seg[0]
        corner = s - seg[0]
        yaw = 0.5 * math.pi * min(1.0, max(0.0, (corner + 0.1) / 0.2))
        out.append((x, y, yaw, s))
    return out


def bev_frame(pose, k):
    """BEV masks of the track seen from pose (+ deterministic noise and the optional masks)."""
    rows, cols = np.arange(vision.BH), np.arange(vision.BW)
    xs = vision.X_FAR - (rows[:, None] + 0.5) * vision.RES + 0.0 * cols[None, :]
    ys = vision.Y_HALF - (cols[None, :] + 0.5) * vision.RES + 0.0 * rows[:, None]
    px, py, th = pose
    X = px + xs * math.cos(th) - ys * math.sin(th)
    Y = py + xs * math.sin(th) + ys * math.cos(th)
    line = (dist_to_track(X, Y) < 0.01).astype(np.uint8) * 255
    valid = (np.abs(ys) < 0.25 + 0.6 * (xs - vision.X_NEAR)).astype(np.uint8) * 255
    rng = np.random.default_rng(k)
    noise = rng.random(line.shape) > 0.995                          # specks the map must keep / erase
    line[noise & (valid > 0)] = 255
    line &= valid
    if k % 5 == 0:                                                  # soft mask (learned plugin + bilinear warp)
        line = cv2.GaussianBlur(line, (3, 3), 0)
    add_only = far = None
    if k % 4 == 1:                                                  # far band, add-only
        far = np.zeros_like(valid)
        far[:20] = valid[:20]
        add_only = far
    elif k % 7 == 3:                                                # rocking: far rows add-only
        add_only = np.zeros_like(valid)
        add_only[:30] = 255
    return line, valid, add_only, far


class FakeNode(Frames):
    """What the planner plugin reads on follower_node / planning_node."""
    DEFAULTS = {'restore_replace': False, 'restore_mode': 'path', 'relink_radius': 0.0,
                'finish_min_dist': 10.0, 'controller': 'rule'}

    def __init__(self, line_map):
        self.map = line_map
        self.pose, self.travel = None, 0.0
        self.path, self.path_end, self.t_point = [], 'open', None
        self.restorer, self.last_observation = None, None
        self.pitch_f, self.rocking = 0.0, False
        self.controller = make_controller('rule', self)

    def get_parameter(self, name):
        return SimpleNamespace(value=self.DEFAULTS[name])

    def get_logger(self):
        return SimpleNamespace(info=lambda *_: None, warning=lambda *_: None)


@pytest.mark.parametrize('map_mode', ['legacy', 'logodds'])
def test_split_map_and_path_equal_single_node(map_mode):
    direct = LineMap(map_mode=map_mode)              # follower_node
    authority = LineMap(map_mode=map_mode)           # map_node
    mirror = LineMap(map_mode=map_mode)              # planning_node
    single, split = FakeNode(direct), FakeNode(mirror)
    p_single, p_split = RulePlanner(single), RulePlanner(split)
    extent = None
    paths = 0
    for k, (x, y, yaw, travel) in enumerate(poses()):
        pose = (x, y, yaw)
        line, valid, add_only, far = bev_frame(pose, k)
        freeze = travel > 0.05 and k % 3 == 0
        used = k % 11 != 6
        # follower_node: PerceptionStage writes the LineMap directly
        if freeze:
            direct.freeze(x, y, 0.15)
        if used:
            direct.update(line, valid, pose, add_only, far_mask=far)
        # perception_node: the same calls land in the sink and go out as one frame
        sink = codec.FrameSink()
        if freeze:
            sink.freeze(x, y, 0.15)
        if used:
            sink.update(line, valid, pose, add_only, far_mask=far)
        frame = transport(codec.make_frame(sink, pose, 0.01, 0.0, 0.0, travel, 'flat'))
        assert codec.grid_matches(frame)
        # map_node
        rect = codec.apply_frame(authority, frame)
        extent = codec.union(extent, rect)
        codec.paste_patch(mirror, transport(codec.make_patch(authority, rect)))
        if k % 20 == 19:                                            # late-joiner resync is harmless
            codec.paste_patch(mirror, transport(codec.make_patch(authority, extent, full=True)))
        for layer in ('grid', 'seen', 'conf'):
            assert np.array_equal(getattr(mirror, layer), getattr(direct, layer)), (k, layer)
        assert (mirror.frozen is None) == (direct.frozen is None)
        if direct.frozen is not None:
            assert np.array_equal(mirror.frozen, direct.frozen), k
        # planning on both maps, same pose / travel
        for node in (single, split):
            node.pose, node.travel = pose, travel
        p_single.replan()
        p_split.replan()
        assert split.path == single.path and split.path_end == single.path_end, k
        assert split.t_point == single.t_point
        paths += len(single.path) >= 2
    assert direct.grid.any() and direct.frozen is not None
    assert paths > 40                                               # the planner did follow the line


def test_patch_rect_covers_frozen_skip():
    """update() narrows map.last's seen in place (frozen / add-only): the patch uses last_rect."""
    a, b = LineMap(), LineMap()
    pose = (0.0, 0.0, 0.0)
    line, valid, _, _ = bev_frame(pose, 1)
    for m in (a, b):
        m.freeze(0.31, 0.0, 0.4)                 # freezes the near part of the view
    frame = transport(codec.make_frame(_sink(line, valid, pose), pose))
    rect = codec.apply_frame(a, frame)
    codec.paste_patch(b, transport(codec.make_patch(a, rect)))
    for layer in ('grid', 'seen', 'conf', 'frozen'):
        assert np.array_equal(getattr(a, layer), getattr(b, layer)), layer


def _sink(line, valid, pose):
    s = codec.FrameSink()
    s.update(line, valid, pose)
    return s


def test_occupancy_grid_values():
    m = LineMap()
    pose = (0.2, 0.0, 0.0)
    line, valid, _, _ = bev_frame(pose, 2)
    m.update(line, valid, pose)
    data, ox, oy, n = codec.occupancy(m, *pose[:2])
    assert data.shape == (400, 400) and n == 400
    assert set(np.unique(data)) <= {-1, 0, 100} and (data == 100).any() and (data == 0).any()
    assert abs(ox - (0.2 - 2.0)) < 0.01 and abs(oy + 2.0) < 0.01


def test_params_follower_section_reaches_every_node():
    from line_follower.split.params import to_all_nodes
    data = {'/**': {'ros__parameters': {'use_sim_time': True, 'v_max': 0.1}},
            'follower_node': {'ros__parameters': {'v_max': 0.25, 'perception': 'rule'}},
            'other': {'ros__parameters': {'a': 1}}}
    out = to_all_nodes(data)
    assert out['/**']['ros__parameters'] == {'use_sim_time': True, 'v_max': 0.25, 'perception': 'rule'}
    assert out['other'] == {'ros__parameters': {'a': 1}} and 'follower_node' not in out
