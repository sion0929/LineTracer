"""Line map in the odom frame + line tracer.

Every camera frame is projected to the ground and written into a grid fixed to the
odom frame.  The camera cannot see the first 0.3 m in front of the robot, so the map
is what lets the robot follow a line that is already below / beside it.
"""
import math
from functools import lru_cache
import numpy as np
import cv2

from . import vision

RES = vision.RES
SIZE = 2400                  # 12 m x 12 m
C0 = SIZE // 2
# window written per frame (covers the whole camera view; grows when LF_X_FAR extends the BEV)
PATCH = max(520, 2 * int(math.ceil(math.hypot(vision.X_FAR, vision.Y_HALF) / RES)) + 20)

_ANGLES = np.deg2rad(np.arange(-100, 101, 4))
_CONE = np.deg2rad(np.arange(-32, 33, 4))
# sharper than 100 deg (acute corners): searched only when nothing is found within +-100 deg
_WIDE = np.deg2rad(np.r_[np.arange(-150, -99, 4), np.arange(102, 151, 4)])
WIDE_TURNS = False                     # set by the node (parameter trace_wide)
ADD_GAP = 0.06               # add-only observations stay this far from known line [m]
_ADD_RADIUS = round(ADD_GAP / RES)
_ADD_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * _ADD_RADIUS + 1,) * 2)


@lru_cache(maxsize=1)
def _bev_conf_tables(x_far, height, width, scale):
    x = x_far - (np.arange(height) + 0.5) * RES
    row = np.clip(1.0 - (x - 0.45) / (1.6 - 0.45) * 0.65, 0.35, 1.0) * 255
    near = np.repeat(row.astype(np.uint8)[:, None], width, axis=1)
    # Scale in float before quantizing, as in the original confidence calculation.
    far = (row * scale).astype(np.uint8)[:, None]
    return near, far


class LineMap:
    logodds = None  # Only logodds maps allocate an evidence layer.

    def __init__(self, *, map_mode='legacy', map_l_hit=0.85, map_l_miss=-0.4,
                 map_l_min=-2.0, map_l_max=3.5, map_l_thresh=0.6):
        if map_mode not in ('legacy', 'logodds'):
            raise ValueError(f'Unknown map_mode {map_mode!r}')
        self.grid = np.zeros((SIZE, SIZE), np.uint8)
        self.seen = np.zeros((SIZE, SIZE), np.uint8)     # cells the camera has looked at (restore.py)
        self.frozen = None          # cells the robot has already driven over: never rewritten (freeze())
        self.restored = None        # learned route where the camera has not looked (perception restores)
        # Legacy confidence of every line cell (0-255, see confidence()):
        # near observations high, falling with distance; far-band cells lower; restored cells low
        self.conf = np.zeros((SIZE, SIZE), np.uint8)
        self.last = None
        if map_mode == 'logodds':
            if not map_l_min <= 0.0 <= map_l_thresh <= map_l_max:
                raise ValueError('Expected map_l_min <= 0 <= map_l_thresh <= map_l_max')
            self.logodds = np.zeros(self.grid.shape, np.float32)
            self._l_min = float(map_l_min)
            self._l_max = float(map_l_max)
            self._l_thresh = float(map_l_thresh)
            weights = np.arange(256, dtype=np.float32) / 255
            self._hit_delta = weights * map_l_hit
            self._miss_delta = weights * map_l_miss

    def freeze(self, x, y, radius):
        """Mark the ground within radius [m] of the robot (odom) as final: later frames skip it."""
        if self.frozen is None:
            self.frozen = np.zeros((SIZE, SIZE), np.uint8)
        cv2.circle(self.frozen, (int(round(x / RES)) + C0, int(round(y / RES)) + C0),
                   int(round(radius / RES)), 255, -1)

    def write_restored(self, restorer, pose, thresh=0.6, length=0.4, radius=1.3, min_x=0.2):
        """Line restoration handed to planning: from every place where the seen line runs into
        never-seen ground ("frontier"), follow the learned route (restore.Restorer) for at most
        `length` m and draw that thin route into self.restored. Refreshed within radius of the
        robot on every call; cells the camera has seen are never restored."""
        from .restore import PRES

        thresh = max(thresh, 0.90)
        length = min(length, 0.12)
        if self.restored is None:
            self.restored = np.zeros((SIZE, SIZE), np.uint8)
        px, py, th = pose
        ci, cj = int(round(py / RES)) + C0, int(round(px / RES)) + C0
        r = int(radius / RES)
        self.restored[:] = 0          # only the current guess: old guesses would come back as ghost lines on revisits
        line, seen = restorer.patch(self, pose)
        unseen_near = cv2.dilate((seen == 0).astype(np.uint8), np.ones((5, 5), np.uint8)) > 0
        front = (line > 0) & unseen_near
        if not front.any():
            return
        prob = restorer.route(self, pose)
        n, lab = cv2.connectedComponents(front.astype(np.uint8), connectivity=8)
        ly, lx = np.nonzero(line > 0)
        lxr, lyr = restorer.to_robot(lx, ly)
        interior = (line > 0) & (seen > 0) & ~unseen_near
        interior_rc = np.column_stack(np.nonzero(interior))
        c, s = math.cos(th), math.sin(th)
        for k in range(1, n):
            rows, cols = np.nonzero(lab == k)
            xs, ys = restorer.to_robot(cols, rows)
            eligible = xs > min_x          # min_x near the far edge: only restore past the view's end, not where a curve leaves it sideways
            if not eligible.any():
                continue
            xs, ys = xs[eligible], ys[eligible]
            pixels = np.column_stack((rows[eligible], cols[eligible]))
            center = pixels.mean(axis=0)
            offsets = center - interior_rc
            local = offsets[np.sum(offsets ** 2, axis=1) < (0.12 / PRES) ** 2]
            pick = int(np.argmax(xs))
            if len(local):
                outward = local.mean(axis=0)
                if np.linalg.norm(outward) > 0:
                    pick = int(np.argmax((pixels - center) @ outward))
            start = (float(xs[pick]), float(ys[pick]))
            near = np.hypot(lxr - start[0], lyr - start[1]) < 0.12
            order = np.argsort(-np.hypot(lxr[near] - start[0], lyr[near] - start[1]))
            known = list(zip(lxr[near][order], lyr[near][order]))       # farthest first, start last
            chain = restorer.extend_ridge(prob, start, known, length, thresh, min_mean=0.98)
            for x, y in chain:
                X = px + x * c - y * s
                Y = py + x * s + y * c
                i, j = int(round(Y / RES)) + C0, int(round(X / RES)) + C0
                if not (1 <= i < SIZE - 1 and 1 <= j < SIZE - 1):
                    continue
                stamp = self.seen[i - 1:i + 2, j - 1:j + 2] == 0          # never over seen ground
                self.restored[i - 1:i + 2, j - 1:j + 2][stamp] = 255

    RESTORED_CONF = 80          # confidence reported for a restored (never seen) cell
    FAR_BAND_SCALE = 0.6        # far band (full-res strip near the horizon) vs near range

    def confidence(self, X, Y):
        """Line confidence in 0..1, with a sigmoid posterior on seen cells in logodds mode."""
        i = int(round(Y / RES)) + C0
        j = int(round(X / RES)) + C0
        if not (0 <= i < SIZE and 0 <= j < SIZE):
            return 0.0
        if self.logodds is not None and self.seen[i, j]:
            evidence = float(self.logodds[i, j])
            tail = math.exp(-abs(evidence))
            return (1.0 if evidence >= 0.0 else tail) / (1.0 + tail)
        if self.grid[i, j] > 0:
            return self.conf[i, j] / 255.0
        if self.restored is not None and self.restored[i, j] > 0:
            return self.RESTORED_CONF / 255.0
        return 0.0

    @staticmethod
    def _bev_conf(far_mask=None):
        """Per-BEV-pixel confidence: 1.0 up to 0.45 m ahead, falling linearly to 0.35 at 1.6 m;
        far-band pixels scaled by FAR_BAND_SCALE."""
        near, far = _bev_conf_tables(vision.X_FAR, vision.BH, vision.BW, LineMap.FAR_BAND_SCALE)
        if far_mask is None:
            return near.copy()
        return np.where(far_mask > 0, far, near)

    def update(self, bev_mask, bev_valid, pose, add_only=None, far_mask=None):
        """Update the part of the map currently seen by the camera.

        add_only (BEV mask, optional): there the observation is less trusted. It never
        erases, and adds line only where the map has no line within ADD_GAP (fills what
        was never seen, e.g. a corner arc, without drawing a second, shifted copy of a
        known line).
        Returns the map window written, (i0, j0, rows, cols), or None when off the map.
        """
        px, py, th = pose
        ox = int(round(px / RES)) + C0 - PATCH // 2
        oy = int(round(py / RES)) + C0 - PATCH // 2
        if ox < 0 or oy < 0 or ox + PATCH > SIZE or oy + PATCH > SIZE:
            return
        c, s = math.cos(th), math.sin(th)
        # BEV pixel (col,row) -> robot (x,y) -> odom -> patch pixel
        ax, ay = vision.X_FAR, vision.Y_HALF
        tx = px / RES + C0 - ox
        ty = py / RES + C0 - oy
        m = np.float32([
            [s, -c, (c * ax - s * ay) / RES + tx],
            [-c, -s, (s * ax + c * ay) / RES + ty],
        ])
        warp_size = (PATCH, PATCH)
        obs = cv2.warpAffine(bev_mask, m, warp_size, flags=cv2.INTER_LINEAR) > 100
        val = cv2.warpAffine(bev_valid, m, warp_size, flags=cv2.INTER_NEAREST)
        seen = val > 0
        debug = (ox, oy, obs, seen)  # keep the full patch for the debug image

        # Crop after warping to preserve OpenCV's sampling at the original patch origin.
        cobs = cv2.warpAffine(self._bev_conf(far_mask), m, warp_size, flags=cv2.INTER_NEAREST)
        x, y, width, height = cv2.boundingRect(seen.view(np.uint8))
        region = np.s_[y:y + height, x:x + width]
        world = np.s_[oy + y:oy + y + height, ox + x:ox + x + width]
        obs = obs[region]
        seen = seen[region]
        self.seen[world][seen] = 255
        if self.restored is not None:
            self.restored[world][seen] = 0
        if self.frozen is not None:
            seen &= self.frozen[world] == 0

        win = self.grid[world]
        cwin = self.conf[world]
        cobs = cobs[region]
        if add_only is not None:
            add = val if add_only is bev_valid else cv2.warpAffine(
                add_only, m, warp_size, flags=cv2.INTER_NEAREST)
            add = add[region] > 0
            candidates = add & seen & obs
            seen &= ~add

            bx, by, bw, bh = cv2.boundingRect(candidates.view(np.uint8))
            if bw:
                # Read the original grid, including neighbors outside the valid footprint.
                # Clip the halo to the old patch to retain its dilation border behavior.
                left = max(0, x + bx - _ADD_RADIUS)
                top = max(0, y + by - _ADD_RADIUS)
                right = min(PATCH, x + bx + bw + _ADD_RADIUS)
                bottom = min(PATCH, y + by + bh + _ADD_RADIUS)
                known = cv2.dilate(self.grid[oy + top:oy + bottom, ox + left:ox + right],
                                   _ADD_KERNEL)
                kx, ky = x + bx - left, y + by - top
                box = np.s_[by:by + bh, bx:bx + bw]
                new = candidates[box] & (known[ky:ky + bh, kx:kx + bw] == 0)
                if self.logodds is None:
                    win[box][new] = 255
                    cwin[box][new] = cobs[box][new]
                else:
                    # Eligible add-only hits accumulate until they cross the line threshold.
                    seen[box] |= new

        if width:
            seen_u8 = seen.view(np.uint8)
            if self.logodds is None:
                line = cv2.compare(obs.view(np.uint8), 0, cv2.CMP_GT)
                cv2.copyTo(line, seen_u8, win)
                cv2.copyTo(cv2.bitwise_and(cobs, line), seen_u8, cwin)
            else:
                evidence = self.logodds[world]
                delta = cv2.LUT(cobs, self._miss_delta)
                cv2.copyTo(cv2.LUT(cobs, self._hit_delta), obs.view(np.uint8), delta)
                delta += evidence
                np.clip(delta, self._l_min, self._l_max, out=delta)
                cv2.copyTo(delta, seen_u8, evidence)
                active = (delta > self._l_thresh).astype(np.uint8) * 255
                cv2.copyTo(active, seen_u8, win)
        self.last = debug
        # every layer this call wrote lies in this window (map rows / cols): the split map node
        # sends it to planning (map.last's seen is narrowed in place above, so it can be smaller)
        return (oy + y, ox + x, height, width)

    def hit(self, X, Y):
        i = int(round(Y / RES)) + C0
        j = int(round(X / RES)) + C0
        if 0 <= i < SIZE and 0 <= j < SIZE:
            return self.grid[i, j] > 0 or (self.restored is not None and self.restored[i, j] > 0)
        return False

    def snap(self, X, Y, radius=0.06):
        """Centre of the line nearest to (X, Y), or None."""
        r = int(radius / RES)
        i0 = int(round(Y / RES)) + C0
        j0 = int(round(X / RES)) + C0
        if i0 - r < 0 or j0 - r < 0 or i0 + r >= SIZE or j0 + r >= SIZE:
            return None
        win = self.grid[i0 - r:i0 + r + 1, j0 - r:j0 + r + 1]
        if self.restored is not None:
            win = np.maximum(win, self.restored[i0 - r:i0 + r + 1, j0 - r:j0 + r + 1])
        ii, jj = np.nonzero(win)
        if ii.size == 0:
            return None
        d2 = (ii - r) ** 2 + (jj - r) ** 2
        k = int(np.argmin(d2))
        if d2[k] > r * r:
            return None
        near = (np.abs(ii - ii[k]) <= 3) & (np.abs(jj - jj[k]) <= 3)
        return ((j0 - r + jj[near].mean() - C0) * RES, (i0 - r + ii[near].mean() - C0) * RES)

    def _clusters(self, X, Y, h, radius, span=_ANGLES):
        """Directions (relative to h) in which the line continues at `radius`."""
        flags = [self.hit(X + radius * math.cos(h + a), Y + radius * math.sin(h + a)) for a in span]
        out, s = [], None
        for i, f in enumerate(flags):
            if f and s is None:
                s = i
            if not f and s is not None:
                out.append(0.5 * (span[s] + span[i - 1]))
                s = None
        if s is not None:
            out.append(0.5 * (span[s] + span[-1]))
        return out

    def _bridge(self, X, Y, h):
        """The line stops: look a little further ahead for its continuation (map gaps)."""
        for r in (0.045, 0.06, 0.075, 0.09):
            c = self._clusters(X, Y, h, r, _CONE)
            if c:
                a = min(c, key=abs)
                return X + r * math.cos(h + a), Y + r * math.sin(h + a), h + 0.5 * a
        return None

    def trace(self, X, Y, h, length=0.8, step=0.02):
        """Follow the line from (X, Y) with heading h.

        Returns (points, end) where end is 'open' (map ends / line not seen yet),
        't' (line ends in a T bar: the finish) or 'len' (requested length reached).
        At a crossing the straightest branch is taken.
        """
        pts = [(X, Y)]
        end = 'len'
        for _ in range(int(length / step)):
            cands = self._clusters(X, Y, h, 0.03)
            if not cands and WIDE_TURNS and self._bridge(X, Y, h) is None:
                # an acute corner: the line continues back to one side (120-150 deg)
                wide = self._clusters(X, Y, h, 0.03, _WIDE)
                if len(wide) == 1:
                    cands = wide
            if not cands:
                nxt = self._bridge(X, Y, h)
                if nxt is None:
                    end = 'open'
                    break
                X, Y, h = nxt
                pts.append((X, Y))
                continue
            fwd = [a for a in cands if abs(a) < math.radians(50)]
            side_l = [a for a in cands if a >= math.radians(50)]
            side_r = [a for a in cands if a <= -math.radians(50)]
            if not fwd and side_l and side_r:
                # bar to both sides and nothing ahead: a T if the bar is long on both sides
                # (a blurred sharp corner is not) and nothing continues further on
                far = self._clusters(X, Y, h, 0.06)
                if (any(a >= math.radians(50) for a in far) and any(a <= -math.radians(50) for a in far)
                        and self._bridge(X, Y, h) is None):
                    end = 't'
                    break
            a = min(cands, key=abs)
            h += 0.8 * a
            X += step * math.cos(h)
            Y += step * math.sin(h)
            c = self.snap(X, Y, 0.015)
            if c is not None:
                X, Y = 0.5 * (X + c[0]), 0.5 * (Y + c[1])
            pts.append((X, Y))
        return pts, end
