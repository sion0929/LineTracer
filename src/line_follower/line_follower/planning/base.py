"""Interface every planning plugin implements (team role 2)."""
import math


def path_len(pts):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))


class Planner:
    """Turn the lane map into the local path, once per camera frame (after the map update).

    Input: node.map (LineMap: .grid seen line, .seen, .restored, hit/snap/trace), node.pose,
    node.travel. Output: node.path (list of odom (x, y), ~2 cm apart, nearest first),
    node.path_end ('open' | 't' | 'len'), node.t_point (finish T, odom) once accepted.
    prune() drops passed points; control calls it every tick.
    """

    def __init__(self, node):
        self.node = node

    def replan(self):
        raise NotImplementedError

    def prune(self):
        raise NotImplementedError
