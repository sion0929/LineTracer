"""Constrained racing-line planning for the experimental map follower."""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class RacingPlan:
    points: np.ndarray
    arc: np.ndarray
    speed: np.ndarray
    travel_time: float


def speed_ceiling(points, *, vmax, lateral_accel, accel, braking, wmax, initial_speed,
                  corner_vmax=None, reaction_time=0.0):
    """Return the pointwise speed ceiling and a time estimate from the current speed."""
    delta = np.diff(points, axis=0)
    segments = np.linalg.norm(delta, axis=1)
    arc = np.r_[0.0, np.cumsum(segments)]
    if np.any(segments < 1e-5):
        return RacingPlan(points, arc, np.zeros_like(arc), float("inf"))

    before, after = delta[:-1], delta[1:]
    cross = before[:, 0] * after[:, 1] - before[:, 1] * after[:, 0]
    dot = np.einsum("ij,ij->i", before, after)
    heading_change = np.abs(np.arctan2(cross, dot))
    curvature = np.r_[0.0, heading_change / ((segments[:-1] + segments[1:]) / 2), 0.0]
    # Each segment must respect the stricter curvature at its neighboring vertices.
    edge_curvature = np.maximum(curvature[1:], curvature[:-1])
    curvature[1:] = np.maximum(curvature[1:], edge_curvature)
    curvature[:-1] = np.maximum(curvature[:-1], edge_curvature)
    curvature = np.maximum(curvature, 1e-6)
    squared = np.minimum(vmax ** 2, np.minimum(lateral_accel / curvature, (wmax / curvature) ** 2))
    if corner_vmax is not None:
        squared = np.where(curvature >= 3.0, np.minimum(squared, corner_vmax ** 2), squared)
    delay_speed = braking * reaction_time
    stopping_speed = np.sqrt(delay_speed ** 2 + 2 * braking * (arc[-1] - arc)) - delay_speed
    squared = np.minimum(squared, stopping_speed ** 2)
    squared[-1] = 0.0
    for i in range(len(segments) - 1, -1, -1):
        squared[i] = min(squared[i], squared[i + 1] + 2 * braking * segments[i])

    predicted = squared.copy()
    predicted[0] = min(predicted[0], initial_speed ** 2)
    for i, distance in enumerate(segments):
        predicted[i + 1] = min(predicted[i + 1], predicted[i] + 2 * accel * distance)
    prediction = np.sqrt(predicted)
    pairs = prediction[:-1] + prediction[1:]
    if len(segments) == 1 and pairs[0] == 0.0:
        inverse_acceleration = 1 / accel + 1 / braking
        peak = min(vmax, np.sqrt(2 * segments[0] / inverse_acceleration))
        cruise_distance = max(0.0, segments[0] - peak ** 2 * inverse_acceleration / 2)
        time = peak * inverse_acceleration + cruise_distance / peak
    else:
        time = float(np.sum(2 * segments / pairs)) if np.all(pairs > 0) else float("inf")
    return RacingPlan(points, arc, np.sqrt(squared), time)


def plan_path(centerline, *, corridor=0.10, **speed_limits):
    """Compare elastic-band shapes by traversal time inside the given corridor.

    Each vertex is constrained to a disc around its corresponding centerline
    vertex. Linear interpolation preserves that bound between vertices.
    """
    best = speed_ceiling(centerline, **speed_limits)
    if len(centerline) < 5 or corridor <= 0.0:
        return best
    indices = np.arange(1, len(centerline) - 1)
    left = np.maximum(indices - 4, 0)
    right = np.minimum(indices + 4, len(centerline) - 1)
    fraction = ((indices - left) / (right - left))[:, None]

    for allowance in (0.45 * corridor, 0.75 * corridor, corridor):
        band = centerline.copy()
        for _ in range(18):
            neighbor = (1.0 - fraction) * band[left] + fraction * band[right]
            offset = (band[indices] + neighbor) * 0.5 - centerline[indices]
            distance = np.linalg.norm(offset, axis=1)
            scale = np.minimum(1.0, allowance / np.maximum(distance, 1e-9))
            band[indices] = centerline[indices] + scale[:, None] * offset
        candidate = speed_ceiling(band, **speed_limits)
        if candidate.travel_time < best.travel_time:
            best = candidate
    return best


def nearest_station(plan, position, prefix=0.25):
    """Project onto the near end of the ordered plan, before any distant loop."""
    end = max(2, int(np.searchsorted(plan.arc, prefix, side="right")))
    points = plan.points[:end]
    delta = np.diff(points, axis=0)
    relative = np.asarray(position) - points[:-1]
    norm_sq = np.einsum("ij,ij->i", delta, delta)
    ratio = np.clip(np.einsum("ij,ij->i", relative, delta) / np.maximum(norm_sq, 1e-10), 0, 1)
    distance_sq = np.sum((relative - ratio[:, None] * delta) ** 2, axis=1)
    i = int(np.argmin(distance_sq))
    return float(plan.arc[i] + ratio[i] * np.sqrt(norm_sq[i]))
