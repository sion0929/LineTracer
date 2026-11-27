"""Interface every control plugin implements."""


class Controller:
    """Compute one velocity command per control tick (50 Hz).

    The node calls compute(rp, dt) only when a path of >= 2 points exists and the
    finish has not been reached; search/creep and stopping stay in the shared node.

    rp: list of (x, y) path points in the ROBOT frame (x forward, y left), ~2 cm apart,
        nearest first. Useful node state: node.v_cmd, node.w_cmd (last command),
        node.path_end ('open' | 't' | 'len'), node.t_point, node.travel,
        node.pitch_f (filtered pitch), node.to_robot(X, Y).
    Returns (v [m/s], w [rad/s]); the node publishes them on /cmd_vel.
    """

    def __init__(self, node):
        self.node = node

    def compute(self, rp, dt):
        raise NotImplementedError

    def reset(self):
        """Called while no path is known (search / creep): drop any tracking state."""

    def confirm_finish(self, path, observation):
        """Optional finish judge, called when the traced path ends in a T bar.

        path: the new path (odom frame) whose last point is the candidate finish.
        observation: latest PerceptionOutput (line / valid BEV masks, .pitch) or None.
        Return True (accept), False (reject) or None (use the node's legacy rule:
        driven distance > finish_min_dist)."""
        return None
