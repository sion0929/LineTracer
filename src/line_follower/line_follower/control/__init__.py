"""Control plugins: path ahead (from the shared line map) -> (v, w) command.

    rule     - pure pursuit + curvature speed limit (original user algorithm, rule teammate)
    learned  - learned policy, e.g. residual RL on top of 'rule' (deep-learning teammate)
"""
from .base import Controller


def make_controller(name, node):
    if name == 'rule':
        from .rule import RuleController
        return RuleController(node)
    if name == 'learned':
        from .learned import LearnedController
        return LearnedController(node)
    raise ValueError(f"unknown controller plugin '{name}' (use 'rule' or 'learned')")


__all__ = ['Controller', 'make_controller']
