"""Camera-only box footprints and a local, odometry-frame bypass plan."""

from dataclasses import dataclass
import math

import cv2
import numpy as np

from experiments.dl.geometry import CAMERA_OFFSET, CENTER, FOCAL

DESIGN_CORRIDOR = 0.10
MAX_BYPASS = 0.35
HALF_WIDTH = 0.16
CLEARANCE = HALF_WIDTH + 0.025
MAX_BOX_DEPTH = 0.20
CAMERA_HEIGHT = 0.103


@dataclass(frozen=True)
class Box:
    near: float
    far: float
    right: float
    left: float


def camera_boxes(mask, pitch, roll):
    """Use each upright component's lower silhouette as a ground-contact estimate."""
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), 8)
    boxes = []
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    for i in range(1, count):
        u, v, w, h, area = stats[i]
        if w < 12 or h < 20 or area < 250:
            continue
        where = labels[v:v + h, u:u + w] == i
        columns = np.flatnonzero(where.any(axis=0))
        # The top of a course box is above the camera; ramp shadows lie on the ground.
        top = v + where[:, columns].argmax(axis=0)
        lower = v + h - 1 - where[::-1, columns].argmax(axis=0)
        y_ray = -(u + columns - CENTER[0]) / FOCAL
        z_top = -(top - CENTER[1]) / FOCAL
        if np.quantile(-sp + cp * (sr * y_ray + cr * z_top), 0.70) < -0.017:
            continue
        z_ray = -(lower - CENTER[1]) / FOCAL
        rotated_z = sr * y_ray + cr * z_ray
        z = -sp + cp * rotated_z
        valid = z < -0.065
        if valid.sum() < 8:
            continue
        distance = -CAMERA_HEIGHT / z[valid]
        x = CAMERA_OFFSET[0] + distance * (cp + sp * rotated_z[valid])
        y = distance * (cr * y_ray[valid] - sr * z_ray[valid])
        near = float(np.quantile(x, 0.17))
        if not 0.18 <= near <= 1.2:
            continue
        # Very distant columns from the side of a box need not enlarge its width estimate.
        front = x < near + MAX_BOX_DEPTH
        if front.sum() < 8:
            continue
        right, left = np.quantile(y[front], [0.04, 0.96])
        boxes.append(Box(near, near + MAX_BOX_DEPTH, float(right), float(left)))
    return boxes


def choose_detour(boxes, start=(0.0, 0.0), *, trigger=0.56, padding=0.025):
    """Plan a route in a local straight-line frame. Positive lateral is left."""
    x, y = start
    blocking = [b for b in boxes if x + 0.18 <= b.near <= x + trigger
                and b.right <= DESIGN_CORRIDOR and b.left >= -DESIGN_CORRIDOR]
    if not blocking:
        return None
    blocker = min(blocking, key=lambda b: b.near)
    relevant = [b for b in boxes if b.far >= blocker.near - 0.14 and b.near <= blocker.far + 0.19]
    candidates = []
    for side in (-1, 1):
        edge = blocker.left if side > 0 else -blocker.right
        clearance = HALF_WIDTH + padding
        lateral = side * max(0.23, edge + clearance)
        if abs(lateral) > MAX_BYPASS:
            continue
        gaps = [max(b.right - lateral, lateral - b.left) for b in relevant]
        if min(gaps) < clearance - 1e-6:
            continue
        waypoint = np.array([
            [max(x + 0.035, blocker.near - 0.16), lateral],
            [blocker.far + 0.12, lateral],
            [blocker.far + 0.255, 0.0],
        ])
        margin = min((g for b, g in zip(relevant, gaps) if b is not blocker), default=0.65)
        candidates.append((margin - abs(lateral - y), waypoint))
    return max(candidates, key=lambda item: item[0])[1] if candidates else None


def transform_boxes(boxes, origin, direction):
    """Transform robot-frame footprints into the local line frame."""
    output = []
    for box in boxes:
        corners = np.array([[box.near, box.right], [box.near, box.left],
                            [box.far, box.right], [box.far, box.left]])
        along, lateral = (origin + corners @ direction.T).T
        output.append(Box(float(along.min()), float(along.max()), float(lateral.min()), float(lateral.max())))
    return output
