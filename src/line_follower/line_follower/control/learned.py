"""Learned control (owner: deep-learning teammate).

Suggested design (see experiments/rl/): a residual policy on top of the rule
controller, v = v_rule + dv, w = w_rule + dw with |dv|, |dw| bounded, so a zero
policy reproduces the rule controller exactly. Observation: the path points `rp`
(robot frame), node.v_cmd / node.w_cmd, node.path_end.

Until a policy is configured this plugin refuses to start.
"""
import os

from .base import Controller
from .rule import RuleController


class LearnedController(Controller):
    def __init__(self, node):
        super().__init__(node)
        node.declare_parameter('learned.policy_path', '')
        path = node.get_parameter('learned.policy_path').value
        if not path or not os.path.isfile(path):
            raise RuntimeError("controller 'learned' needs parameter learned.policy_path")
        self.base = RuleController(node)
        self.policy = self.load(path)

    def load(self, path):
        """TODO(dl teammate): load the policy weights (e.g. numpy npz MLP)."""
        raise NotImplementedError('LearnedController.load: load the policy here')

    def residual(self, rp, v_rule, w_rule):
        """TODO(dl teammate): return (dv, dw) from the policy."""
        raise NotImplementedError('LearnedController.residual: run the policy here')

    def compute(self, rp, dt):
        v, w = self.base.compute(rp, dt)
        dv, dw = self.residual(rp, v, w)
        return v + dv, w + dw
