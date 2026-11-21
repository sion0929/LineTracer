"""Pure measurement predicates shared by the dev-only Gazebo acceptance checks."""
import math


def pose_delta(start, end):
    dx, dy = end[0] - start[0], end[1] - start[1]
    c, s = math.cos(start[2]), math.sin(start[2])
    yaw = math.atan2(math.sin(end[2] - start[2]), math.cos(end[2] - start[2]))
    return c * dx + s * dy, -s * dx + c * dy, yaw


def motion_ok(v, w, duration, delta):
    x, y, yaw = delta
    if v and x * math.copysign(1, v) < abs(v) * duration * .35:
        return False
    if w and yaw * math.copysign(1, w) < abs(w) * duration * .35:
        return False
    if not v and math.hypot(x, y) > .02:
        return False
    if not w and (abs(yaw) > .15 or abs(y) > .02):
        return False
    return True


def sample_rate(times):
    return (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 and times[-1] > times[0] else 0
