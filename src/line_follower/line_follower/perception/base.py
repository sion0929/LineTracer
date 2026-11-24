"""Interface every perception plugin implements."""
from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class PerceptionOutput:
    line: np.ndarray                     # BEV line mask, uint8 0/255, shape (vision.BH, vision.BW)
    valid: np.ndarray                    # BEV validity mask (ground actually seen), uint8 0/255
    obstacle: Optional[np.ndarray] = None  # BEV obstacle mask (same shape) or None


class Perception:
    """Turn one camera frame into BEV masks.

    `node` is the FollowerNode; plugins may declare their own ROS parameters on it
    (prefix them, e.g. 'dl.model_path') and read shared state such as node.pose.
    """

    def __init__(self, node):
        self.node = node

    def process(self, bgr, gray, pitch, x_max):
        """bgr: HxWx3 uint8, gray: HxW uint8, pitch: camera nose-up pitch [rad],
        x_max: only report ground closer than this [m] (far cells are pitch-sensitive).
        Returns PerceptionOutput."""
        raise NotImplementedError
