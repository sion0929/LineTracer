"""Planning plugins: lane map (from perception) -> local path (for control).

    rule  - map trace with path memory, gap bridging, finish T (original user algorithm)
"""
from .base import Planner, path_len


def make_planner(name, node):
    if name == 'rule':
        from .rule import RulePlanner
        return RulePlanner(node)
    raise ValueError(f"unknown planner plugin '{name}' (use 'rule')")


__all__ = ['Planner', 'make_planner', 'path_len']
