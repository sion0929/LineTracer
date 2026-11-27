"""Rule-based perception: OpenCV threshold on the BEV image (original user algorithm).

Owner: rule-based teammate. Change freely; keep the PerceptionOutput contract.
"""
from .. import vision
from .base import Perception, PerceptionOutput


class RulePerception(Perception):
    def process(self, bgr, gray, pitch, x_max):
        # near_scale (node parameter, dual resolution): 0.5 = threshold the 320x240 image
        scale = self.node.get_parameter('near_scale').value if self.node.has_parameter('near_scale') else 1.0
        line, valid = vision.line_mask(gray, pitch, x_max, scale=scale)
        # rule.max_width_m: drop broad shadow/dark blobs (0 = off, legacy bit-identical)
        max_width = (float(self.node.get_parameter('rule.max_width_m').value)
                     if self.node.has_parameter('rule.max_width_m') else 0.0)
        if max_width > 0.0:
            line = vision.remove_wide(line, max_width)
        return PerceptionOutput(line=line, valid=valid, obstacle=None)
