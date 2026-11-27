"""Perception plugins: camera image -> bird's-eye-view (BEV) masks.

Every plugin returns masks in the SAME BEV grid as `vision.py` (rows = forward
distance X_FAR..X_NEAR, cols = left..right, RES metres per cell), so the shared
line map (`line_map.py`) can use any of them unchanged.

    rule  - OpenCV dark-pixel threshold (owner: rule-based teammate)
    rule_hill - rule + hill correction from the 2 cm line width (lead experiment)
    dl    - learned segmentation (owner: deep-learning teammate)
    depth - learned line mask placed on the ground with learned metric depth (no flat-floor assumption)
    hybrid - rule mask with the depth mask as a gate ('and' / 'rule_gated')
"""
from .base import Perception, PerceptionOutput


def make_perception(name, node):
    if name == 'rule':
        from .rule import RulePerception
        return RulePerception(node)
    if name == 'rule_hill':
        from .rule_hill import RuleHillPerception
        return RuleHillPerception(node)
    if name == 'rule_plane':
        from .rule_plane import RulePlanePerception
        return RulePlanePerception(node)
    if name == 'depth':
        from .depth import DepthPerception
        return DepthPerception(node)
    if name == 'hybrid':
        from .hybrid import HybridPerception
        return HybridPerception(node)
    if name == 'dl':
        from .dl import DlPerception
        return DlPerception(node)
    raise ValueError(f"unknown perception plugin '{name}' (use 'rule', 'hybrid', 'depth' or 'dl')")


__all__ = ['Perception', 'PerceptionOutput', 'make_perception']
