"""Hybrid perception: the rule mask cleaned/gated by the learned depth mask.

Both masks are computed on every frame; the depth plugin keeps using the depth.*
parameters and the rule plugin keeps using near_scale / rule.max_width_m.

    hybrid.mode = 'and'        keep only cells where both plugins see line
                = 'rule_gated' (default) keep whole rule-line components that come
                               within hybrid.gate_px cells of the depth line

'rule_gated' trusts the rule mask's geometry (sharper, direction-independent) but
needs the learned mask as evidence that a component really is line, so isolated
shadows are dropped without thinning the line or erasing a junction. 'and' is the
conservative intersection. valid is always the rule plugin's valid footprint.
"""
import cv2
import numpy as np

from .base import Perception, PerceptionOutput
from .depth import DepthPerception
from .rule import RulePerception


def _param(node, name, default):
    try:
        if node.has_parameter(name):
            return node.get_parameter(name).value
    except (AttributeError, KeyError):
        pass
    return default


class HybridPerception(Perception):
    def __init__(self, node):
        super().__init__(node)
        self.mode = _param(node, 'hybrid.mode', 'rule_gated')
        if self.mode not in ('and', 'rule_gated'):
            raise ValueError(f"hybrid.mode must be 'and' or 'rule_gated': {self.mode!r}")
        self.gate_px = int(round(float(_param(node, 'hybrid.gate_px', 2))))
        self.rule = RulePerception(node)
        self.depth = DepthPerception(node)

    def combine(self, rule_line, depth_line):
        rule = rule_line > 0
        depth = depth_line > 0
        if self.mode == 'and':
            return np.where(rule & depth, 255, 0).astype(np.uint8)
        n, labels = cv2.connectedComponents(rule.astype(np.uint8), connectivity=8)[:2]
        if n <= 1:
            return np.zeros_like(rule_line)
        if self.gate_px > 0:
            k = 2 * self.gate_px + 1
            gate = cv2.dilate(depth.astype(np.uint8), np.ones((k, k), np.uint8))
        else:
            gate = depth.astype(np.uint8)
        out = np.zeros_like(rule_line)
        for label in range(1, n):
            component = labels == label
            if np.any(gate[component]):
                out[component] = rule_line[component]
        return out

    def process(self, bgr, gray, pitch, x_max):
        rule = self.rule.process(bgr, gray, pitch, x_max)
        depth = self.depth.process(bgr, gray, pitch, x_max)
        line = self.combine(rule.line, depth.line)
        obstacle = depth.obstacle if depth.obstacle is not None else rule.obstacle
        return PerceptionOutput(line=line, valid=rule.valid, obstacle=obstacle)
