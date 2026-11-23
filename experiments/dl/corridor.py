"""Short-horizon corner speed and wheel-clearance checks in the local line map."""

import math

import numpy as np

WHEEL_OFFSET = 0.1435
CONTACT_RADIUS = 0.019


def path_arc(points):
    return np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]


def near_bend(points, horizon=0.23):
    """Detect a turn supported by two chords, without differentiating noisy cells."""
    arc = path_arc(points)
    if arc[-1] < 0.10:
        return False
    distance = np.arange(0, min(arc[-1], horizon) + 1e-8, 0.045)
    samples = np.column_stack([np.interp(distance, arc, xy) for xy in points.T])
    delta = np.diff(samples, axis=0)
    headings = np.unwrap(np.arctan2(delta[:, 1], delta[:, 0]))
    return bool(np.ptp(headings) > math.radians(38))


def guarded_speed(points, speed, target_angle, bend=None):
    if bend is None:
        bend = near_bend(points)
    if not bend:
        return speed
    if abs(target_angle) > 0.72:
        return 0.0
    return min(speed, 0.055)


def recovery_motion_supported(mapped, position, heading):
    """Check the next 10 cm of the heading against locally mapped line evidence."""
    if len(mapped) < 10:
        return True
    forward = np.array((math.cos(heading), math.sin(heading)))
    target = np.asarray(position) + 0.10 * forward
    distance_sq = np.sum((mapped - target) ** 2, axis=1)
    return bool(distance_sq.min() <= 0.073 ** 2)


def wheel_risk(points, centerline):
    """Sum predicted contact penetration at the two wheels along the candidate."""
    previous = np.maximum(np.arange(len(points)) - 2, 0)
    following = np.minimum(np.arange(len(points)) + 2, len(points) - 1)
    direction = points[following] - points[previous]
    normals = np.column_stack((-direction[:, 1], direction[:, 0]))
    normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-9)
    left = points + WHEEL_OFFSET * normals
    right = points - WHEEL_OFFSET * normals
    samples = np.vstack((left, right))
    bases = centerline[:-1]
    segments = np.diff(centerline, axis=0)
    relative = samples[:, None] - bases
    lengths_sq = np.sum(segments ** 2, axis=1)
    fraction = np.clip((relative * segments).sum(axis=2) / np.maximum(lengths_sq, 1e-9), 0, 1)
    distance_sq = ((relative - fraction[:, :, None] * segments) ** 2).sum(axis=2)
    distance = np.sqrt(distance_sq.min(axis=1))
    return float(np.maximum(0.0, CONTACT_RADIUS - distance).sum())


def wheel_aware_path(centerline, corridor, *, bend=None):
    """Compare small, tapered offsets within the mapped corridor at a bend."""
    arc = path_arc(centerline)
    if not (len(centerline) >= 8 and arc[-1] >= 0.19 and corridor > 0):
        return centerline
    if bend is None:
        bend = near_bend(centerline)
    if not bend:
        return centerline
    end = max(8, int(np.searchsorted(arc, 0.33)))
    prefix = centerline[:end]
    direction = np.gradient(prefix, axis=0)
    normals = np.column_stack((-direction[:, 1], direction[:, 0]))
    normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-9)
    span = arc[end - 1]
    weights = np.sin(np.pi * arc[:end] / span) ** 2
    best, best_score = prefix, wheel_risk(prefix, prefix)
    # A narrow search keeps a large reserve for map-estimation and tracking error.
    for offset in (-0.025, 0.025, -0.040, 0.040):
        if abs(offset) > corridor:
            continue
        candidate = prefix + (offset * weights)[:, None] * normals
        score = wheel_risk(candidate, prefix) + abs(offset) * 0.15
        if score < best_score:
            best, best_score = candidate, score
    return np.vstack((best, centerline[end:])) if best is not prefix else centerline
