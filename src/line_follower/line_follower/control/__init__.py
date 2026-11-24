"""Control plugins: path ahead (from the shared line map) -> (v, w) command.

    rule     - pure pursuit + curvature speed limit (original user algorithm, rule teammate)
    learned  - learned policy, e.g. residual RL on top of 'rule' (deep-learning teammate)
    playbook - 'rule' + learned situation classifier: per-situation actions and the finish judge
"""
from .base import Controller


def make_controller(name, node):
    if name == 'rule':
        from .rule import RuleController
        return RuleController(node)
    if name == 'learned':
        from .learned import LearnedController
        return LearnedController(node)
    if name == 'playbook':
        from .playbook import PlaybookController
        return PlaybookController(node)
    raise ValueError(f"unknown controller plugin '{name}' (use 'rule', 'learned' or 'playbook')")


__all__ = ['Controller', 'make_controller']
