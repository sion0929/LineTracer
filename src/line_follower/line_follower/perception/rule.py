"""Rule-based perception: OpenCV threshold on the BEV image (original user algorithm).

Owner: rule-based teammate. Change freely; keep the PerceptionOutput contract.
"""
from .. import vision
from .base import Perception, PerceptionOutput


class RulePerception(Perception):
    def process(self, bgr, gray, pitch, x_max):
        line, valid = vision.line_mask(gray, pitch, x_max)
        return PerceptionOutput(line=line, valid=valid, obstacle=None)
