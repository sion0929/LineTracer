"""Runtime situation features (판단 입력): a small vector from what the node itself knows.

Inputs, all from this run only:
  * the node's odom-frame LineMap grid, cut out as a robot-frame patch (1 cm cells,
    x -0.10..0.90 m ahead of the axle, y -0.45..0.45 m)
  * the traced path ahead (node.path, odom frame) and how it ends (node.path_end)
  * the filtered IMU pitch and the start pose of this run (odometry)

Groups (36 values, names in FEATURE_NAMES):
  occ_*      line fraction (x10) in 5 x-bands x 3 y-bands of the robot-frame patch
  path_*     traced path: length, end type, end point, net / sharpest / wiggle turn
  junc_*     side branches next to the path (ring of radius 7 cm around path points)
  end_*      the path end (the T candidate): bar arm lengths, line beyond it, how far
             beyond it the camera currently sees, distance from the robot
  ctx_*      IMU pitch, closeness to and facing of the start pose
"""
import math

import cv2
import numpy as np

from .. import vision
from ..line_map import RES, C0

PX = 0.01                      # patch cell [m]
X_MIN, X_MAX = -0.10, 0.90     # patch extent ahead of the axle [m]
Y_HALF = 0.45
PH = int(round((X_MAX - X_MIN) / PX))      # rows, row 0 = X_MAX
PW = int(round(2 * Y_HALF / PX))           # cols, col 0 = +Y_HALF (left)

X_BANDS = (0.90, 0.70, 0.50, 0.30, 0.10, -0.10)
Y_BANDS = (0.45, 0.15, -0.15, -0.45)
RING_R = 0.07
RING_A = np.deg2rad(np.arange(-180, 180, 10))
_FWD = np.abs(RING_A) < math.radians(40)
_LEFT = (RING_A >= math.radians(40)) & (RING_A <= math.radians(140))
_RIGHT = (RING_A <= -math.radians(40)) & (RING_A >= -math.radians(140))
TAN_HALF_FOV = math.tan(vision.HFOV / 2.0)

FEATURE_NAMES = tuple(
    [f'occ_{xi}{yn}' for xi in range(5) for yn in ('l', 'c', 'r')]
    + ['path_len', 'path_open', 'path_t', 'path_lenlim', 'path_end_x', 'path_end_y',
       'path_turn', 'path_sharp', 'path_wiggle']
    + ['junc_left', 'junc_right', 'junc_both', 'junc_first']
    + ['end_arm_l', 'end_arm_r', 'end_beyond_hit', 'end_beyond_seen', 'end_dist']
    + ['ctx_pitch', 'ctx_near_start', 'ctx_start_facing'])
F = {name: i for i, name in enumerate(FEATURE_NAMES)}


def robot_patch(grid, pose):
    """Robot-frame binary patch (PH x PW, bool) of the odom grid around pose (x, y, yaw)."""
    px, py, th = pose
    c, s = math.cos(th), math.sin(th)
    k = PX / RES
    tx = px / RES + C0 + (X_MAX * c - Y_HALF * s) / RES
    ty = py / RES + C0 + (X_MAX * s + Y_HALF * c) / RES
    m = np.float32([[k * s, -k * c, tx], [-k * c, -k * s, ty]])
    out = cv2.warpAffine(grid, m, (PW, PH), flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP,
                         borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    return out > 40


def _lookup(patch, x, y):
    """patch value at robot-frame points (arrays); outside the patch = False."""
    i = np.rint((X_MAX - np.asarray(x, float)) / PX).astype(int)
    j = np.rint((Y_HALF - np.asarray(y, float)) / PX).astype(int)
    ok = (i >= 0) & (i < PH) & (j >= 0) & (j < PW)
    out = np.zeros(i.shape, bool)
    out[ok] = patch[i[ok], j[ok]]
    return out


def _in_view(x, y, x_far):
    """Robot-frame points inside the camera's ground view (eroded a little)."""
    x = np.asarray(x, float)
    y = np.asarray(y, float)
    return (x >= vision.X_NEAR + 0.02) & (x <= x_far - 0.02) & \
        (np.abs(y) <= (x - vision.CAM_X) * TAN_HALF_FOV - 0.03)


def _to_robot(points, pose):
    px, py, th = pose
    c, s = math.cos(th), math.sin(th)
    p = np.asarray(points, float).reshape(-1, 2)
    dx, dy = p[:, 0] - px, p[:, 1] - py
    return np.column_stack((dx * c + dy * s, -dx * s + dy * c))


def _arm(patch, ex, ey, ux, uy, side):
    """Length of the bar beside the path end E along the normal (side +1 left, -1 right)."""
    nx, ny = -uy * side, ux * side
    d = np.arange(0.02, 0.305, 0.01)
    hit = np.zeros(d.shape, bool)
    for f in (0.0, 0.02, 0.04):              # the bar centre is 2-3 cm past the traced end
        hit |= _lookup(patch, ex + f * ux + d * nx, ey + f * uy + d * ny)
    length, miss = 0.0, 0
    for k, h in enumerate(hit):
        if h:
            length, miss = d[k], 0
        else:
            miss += 1
            if miss > 1:
                break
    return length


def extract(grid, pose, path, path_end, pitch=0.0, start=None, x_far=vision.X_FAR):
    """Feature vector (float32, len(FEATURE_NAMES)).

    grid: LineMap.grid (odom frame); pose: (x, y, yaw) odom; path: odom points ahead
    (nearest first); path_end: 'open' | 't' | 'len'; pitch: nose-up IMU pitch [rad];
    start: (x, y) odom position the run started at (or None); x_far: forward limit of
    the current camera view [m] (the node shortens it while the body rocks / on hills).
    """
    rp = _to_robot(path, pose) if path is not None and len(path) else None
    st = _to_robot([start], pose)[0] if start is not None else None
    return extract_local(robot_patch(grid, pose), rp, path_end, pitch, st, x_far)


def extract_local(patch, rp, path_end, pitch=0.0, start=None, x_far=vision.X_FAR):
    """Same as extract() with everything already in the robot frame.

    patch: robot_patch() output; rp: (N, 2) path points or None; start: (x, y) of the
    start position in the robot frame or None.
    """
    f = np.zeros(len(FEATURE_NAMES), np.float32)
    # -- occupancy bands
    for xi in range(5):
        r0, r1 = int(round((X_MAX - X_BANDS[xi]) / PX)), int(round((X_MAX - X_BANDS[xi + 1]) / PX))
        for yi, yn in enumerate('lcr'):
            c0, c1 = int(round((Y_HALF - Y_BANDS[yi]) / PX)), int(round((Y_HALF - Y_BANDS[yi + 1]) / PX))
            f[F[f'occ_{xi}{yn}']] = 10.0 * patch[r0:r1, c0:c1].mean()
    # -- context
    f[F['ctx_pitch']] = math.degrees(pitch)
    f[F['ctx_near_start']] = 1.5
    if start is not None:
        dist = math.hypot(start[0], start[1])
        f[F['ctx_near_start']] = min(dist, 1.5)
        if 0.05 < dist < 1.5:
            f[F['ctx_start_facing']] = start[0] / dist
    if rp is None or len(rp) < 2:
        return f
    # -- traced path
    rp = np.asarray(rp, float)
    seg = np.diff(rp, axis=0)
    sl = np.hypot(seg[:, 0], seg[:, 1])
    keep = sl > 1e-6
    if keep.sum() < 1:
        return f
    seg, sl = seg[keep], sl[keep]
    pts = np.vstack((rp[:1], rp[1:][keep]))
    s = np.r_[0.0, np.cumsum(sl)]
    h = np.unwrap(np.arctan2(seg[:, 1], seg[:, 0]))
    hp = np.r_[h[0], 0.5 * (h[:-1] + h[1:]), h[-1]] if len(h) > 1 else np.r_[h, h]
    f[F['path_len']] = min(s[-1], 1.2)
    f[F['path_open']] = path_end == 'open'
    f[F['path_t']] = path_end == 't'
    f[F['path_lenlim']] = path_end == 'len'
    f[F['path_end_x']], f[F['path_end_y']] = pts[-1]
    n3 = min(3, len(h))
    f[F['path_turn']] = abs(np.mean(h[-n3:]) - np.mean(h[:n3]))
    j = np.minimum(np.searchsorted(s, s + 0.15), len(s) - 1)
    f[F['path_sharp']] = float(np.max(np.abs(hp[j] - hp))) if len(s) > 1 else 0.0
    f[F['path_wiggle']] = max(0.0, float(np.sum(np.abs(np.diff(h)))) - f[F['path_turn']]) if len(h) > 1 else 0.0
    # -- side branches along the path (ring around points every 4 cm)
    sk = np.arange(0.06, s[-1] - 0.02, 0.04)
    first = 1.2
    if len(sk):
        x = np.interp(sk, s, pts[:, 0])[:, None]
        y = np.interp(sk, s, pts[:, 1])[:, None]
        ang = np.interp(sk, s, hp)[:, None] + RING_A[None, :]
        hits = _lookup(patch, x + RING_R * np.cos(ang), y + RING_R * np.sin(ang))
        fwd = (hits & _FWD).any(axis=1)
        left = (hits & _LEFT).any(axis=1) & fwd
        right = (hits & _RIGHT).any(axis=1) & fwd
        f[F['junc_left']], f[F['junc_right']] = left.sum(), right.sum()
        f[F['junc_both']] = (left & right).sum()
        if (left | right).any():
            first = float(sk[np.argmax(left | right)])
    f[F['junc_first']] = first
    # -- the path end (T candidate)
    ex, ey = pts[-1]
    he = float(np.mean(h[-n3:]))
    ux, uy = math.cos(he), math.sin(he)
    f[F['end_arm_l']] = _arm(patch, ex, ey, ux, uy, +1)
    f[F['end_arm_r']] = _arm(patch, ex, ey, ux, uy, -1)
    t = np.arange(0.06, 0.37, 0.02)
    lat = np.array([-0.04, 0.0, 0.04])
    bx = ex + t[:, None] * ux - lat[None, :] * uy
    by = ey + t[:, None] * uy + lat[None, :] * ux
    f[F['end_beyond_hit']] = _lookup(patch, bx, by).mean()
    tv = np.arange(0.04, 0.41, 0.02)
    f[F['end_beyond_seen']] = 0.02 * _in_view(ex + tv * ux, ey + tv * uy, x_far).sum()
    f[F['end_dist']] = math.hypot(ex, ey)
    return f


def node_x_far(node):
    """Forward view limit the node used for the last frame (see FollowerNode.on_image)."""
    steady = node.rock < 0.08 and abs(node.pitch_f) < math.radians(3.0)
    return vision.X_FAR if steady else 0.50


def from_node(node, path=None, path_end=None, start=None):
    """extract() with the node's current state (path / path_end default to the node's)."""
    return extract(node.map.grid, node.pose, node.path if path is None else path,
                   node.path_end if path_end is None else path_end, node.pitch_f, start, node_x_far(node))
