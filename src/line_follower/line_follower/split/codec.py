"""Payloads of the split nodes (docs/architecture/NODE_SPLIT.md), without ROS.

The nodes convert these dicts to / from the line_follower_interfaces messages field by field
(same names), so the unit tests run the exact code path of the nodes without a ROS graph:

    perception_node  FrameSink (as node.map) -> make_frame()            -> PerceptionFrame
    map_node         apply_frame(map, frame) -> make_patch(map, rect)   -> LineMapPatch
    planning_node    paste_patch(mirror, patch); planner.replan()       -> LinePath
"""
import numpy as np

from .. import vision
from ..line_map import C0, RES, SIZE
from ..perception.log import SLOPE_CODES

SLOPE_NAMES = {v: k for k, v in SLOPE_CODES.items()}
F32 = ('pitch', 'imu_pitch', 'elev', 'travel', 'freeze_radius', 'res', 'x_near', 'x_far', 'y_half')
MASKS = ('line', 'valid', 'far', 'add_only')
LAYERS = ('grid', 'seen', 'conf', 'frozen', 'restored')


def f32(v):
    """The value a float32 message field hands back."""
    return float(np.float32(v))


# ---- BEV masks ---------------------------------------------------------------
def pack_mask(mask):
    """BEV mask -> bytes. 0/255 masks (rule, terrain, valid, far, add-only) as np.packbits;
    anything else (soft masks, e.g. a bilinear warp of a learned mask) as raw uint8 so the map
    gets exactly the same input. None -> b''. The length tells the two apart (unpack_mask)."""
    if mask is None:
        return b''
    m = np.ascontiguousarray(mask, np.uint8)
    if np.count_nonzero((m != 0) & (m != 255)):
        return m.tobytes()
    return np.packbits(m > 0).tobytes()


def unpack_mask(data, rows, cols):
    """Inverse of pack_mask -> uint8 (rows, cols) array, or None for an empty field."""
    if data is None or len(data) == 0:
        return None
    n = rows * cols
    buf = np.frombuffer(data, np.uint8)        # bytes or the message's array('B')
    if buf.size == n:
        return buf.reshape(rows, cols).copy()
    if buf.size != (n + 7) // 8:
        raise ValueError(f'mask of {buf.size} bytes does not fit a {rows}x{cols} grid')
    return (np.unpackbits(buf)[:n].reshape(rows, cols) * np.uint8(255)).astype(np.uint8)


# ---- perception -> map -----------------------------------------------------------
class FrameSink:
    """Stands in for node.map on the perception node: PerceptionStage.process() calls
    freeze() / update() with the same arguments it would give a LineMap; they are recorded
    and sent to the map node instead."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.freeze_at = None        # (x, y, radius)
        self.update_args = None      # (line, valid, pose, add_only, far_mask)

    def freeze(self, x, y, radius):
        self.freeze_at = (x, y, radius)

    def update(self, bev_mask, bev_valid, pose, add_only=None, far_mask=None):
        self.update_args = (bev_mask, bev_valid, pose, add_only, far_mask)


def make_frame(sink, pose, pitch=0.0, imu_pitch=0.0, elev=0.0, travel=0.0, slope='flat'):
    """PerceptionFrame fields from what one PerceptionStage.process() call wrote into the sink.
    pose: the capture-time pose (used when the frame did not update the map)."""
    upd = sink.update_args
    if upd is not None:
        pose = upd[2]
    line, valid, add_only, far = (upd[0], upd[1], upd[3], upd[4]) if upd is not None else (None,) * 4
    fz = sink.freeze_at
    frame = dict(
        has_pose=pose is not None,
        x=float(pose[0]) if pose is not None else 0.0,
        y=float(pose[1]) if pose is not None else 0.0,
        yaw=float(pose[2]) if pose is not None else 0.0,
        pitch=pitch if pitch is not None else float('nan'), imu_pitch=imu_pitch, elev=elev, travel=travel,
        slope=SLOPE_CODES.get(slope, 0),
        use_for_map=upd is not None, freeze=fz is not None, freeze_radius=fz[2] if fz is not None else 0.0,
        rows=vision.BH, cols=vision.BW, res=vision.RES, x_near=vision.X_NEAR, x_far=vision.X_FAR,
        y_half=vision.Y_HALF,
        line=pack_mask(line), valid=pack_mask(valid), far=pack_mask(far), add_only=pack_mask(add_only))
    if fz is not None and (fz[0], fz[1]) != (frame['x'], frame['y']):
        raise ValueError('freeze position differs from the frame pose')
    for k in F32:
        frame[k] = f32(frame[k])
    return frame


def grid_matches(frame):
    """Does the frame's BEV grid match this process's vision grid (same LF_X_FAR)?"""
    return (frame['rows'], frame['cols']) == (vision.BH, vision.BW) and \
        abs(frame['x_far'] - vision.X_FAR) < 1e-4 and abs(frame['res'] - vision.RES) < 1e-6


def union(a, b):
    """Union of two (i0, j0, rows, cols) rectangles; None / empty ones are ignored."""
    if a is None or a[2] <= 0 or a[3] <= 0:
        return b if b is not None and b[2] > 0 and b[3] > 0 else None
    if b is None or b[2] <= 0 or b[3] <= 0:
        return a
    i0, j0 = min(a[0], b[0]), min(a[1], b[1])
    i1, j1 = max(a[0] + a[2], b[0] + b[2]), max(a[1] + a[3], b[1] + b[3])
    return (i0, j0, i1 - i0, j1 - j0)


def freeze_rect(x, y, radius):
    """Map window cv2.circle touches in LineMap.freeze()."""
    cj, ci, r = int(round(x / RES)) + C0, int(round(y / RES)) + C0, int(round(radius / RES))
    i0, j0 = max(0, ci - r), max(0, cj - r)
    i1, j1 = min(SIZE, ci + r + 1), min(SIZE, cj + r + 1)
    return (i0, j0, i1 - i0, j1 - j0) if i1 > i0 and j1 > j0 else None


def apply_frame(line_map, frame):
    """map node: the LineMap calls the perception node's stage made, in the same order
    (freeze, then update). Returns the map window that changed, (i0, j0, rows, cols) or None."""
    rect = None
    if frame['freeze'] and frame['has_pose']:
        line_map.freeze(frame['x'], frame['y'], frame['freeze_radius'])
        rect = union(rect, freeze_rect(frame['x'], frame['y'], frame['freeze_radius']))
    if frame['use_for_map'] and frame['has_pose']:
        r, c = frame['rows'], frame['cols']
        written = line_map.update(unpack_mask(frame['line'], r, c), unpack_mask(frame['valid'], r, c),
                                  (frame['x'], frame['y'], frame['yaw']), unpack_mask(frame['add_only'], r, c),
                                  far_mask=unpack_mask(frame['far'], r, c))
        rect = union(rect, written)
    return rect


# ---- map -> planning ------------------------------------------------------------------
def make_patch(line_map, rect, full=False):
    """LineMapPatch fields: the map layers inside rect (None = empty patch). frozen / restored
    are sent only when the map has them."""
    i0, j0, rows, cols = rect if rect is not None else (0, 0, 0, 0)
    patch = dict(i0=int(i0), j0=int(j0), rows=int(rows), cols=int(cols), full=bool(full))
    win = np.s_[i0:i0 + rows, j0:j0 + cols]
    for k in LAYERS:
        layer = getattr(line_map, k)
        patch[k] = np.ascontiguousarray(layer[win]).tobytes() if layer is not None and rows and cols else b''
    return patch


def paste_patch(line_map, patch):
    """planning node: copy a patch into the mirror map (allocating frozen / restored on first use)."""
    rows, cols = patch['rows'], patch['cols']
    if not rows or not cols:
        return
    win = np.s_[patch['i0']:patch['i0'] + rows, patch['j0']:patch['j0'] + cols]
    for k in LAYERS:
        data = patch[k]
        if data is None or len(data) == 0:
            continue
        if getattr(line_map, k) is None:
            setattr(line_map, k, np.zeros((SIZE, SIZE), np.uint8))
        getattr(line_map, k)[win] = np.frombuffer(data, np.uint8).reshape(rows, cols)


def occupancy(line_map, x, y, half=2.0, cell=0.01):
    """rviz view of the map around (x, y): (int8 data rows x cols, origin x, origin y, n).
    100 = line, 0 = seen without line, -1 = never seen. Rows run along +y, columns along +x."""
    k = max(1, int(round(cell / RES)))
    n = int(round(2 * half / cell))
    span = n * k
    i0 = int(round(y / RES)) + C0 - span // 2
    j0 = int(round(x / RES)) + C0 - span // 2
    grid = np.zeros((span, span), np.uint8)
    seen = np.zeros((span, span), np.uint8)
    a0, b0 = max(0, i0), max(0, j0)
    a1, b1 = min(SIZE, i0 + span), min(SIZE, j0 + span)
    if a1 > a0 and b1 > b0:
        grid[a0 - i0:a1 - i0, b0 - j0:b1 - j0] = line_map.grid[a0:a1, b0:b1]
        seen[a0 - i0:a1 - i0, b0 - j0:b1 - j0] = line_map.seen[a0:a1, b0:b1]
    g = grid.reshape(n, k, n, k).max(axis=(1, 3)) > 0
    s = seen.reshape(n, k, n, k).max(axis=(1, 3)) > 0
    data = np.full((n, n), -1, np.int8)
    data[s] = 0
    data[g] = 100
    return data, (j0 - C0 - 0.5) * RES, (i0 - C0 - 0.5) * RES, n


# ---- planning -> control ---------------------------------------------------------------
def make_path(path, path_end, t_point):
    return dict(points=[(float(p[0]), float(p[1])) for p in path], end_state=str(path_end),
                has_t_point=t_point is not None,
                t_point=(float(t_point[0]), float(t_point[1])) if t_point is not None else (0.0, 0.0))
