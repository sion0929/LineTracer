"""Trained situation classifier: feature vector -> probabilities over SITUATIONS.

The model is a cv2.FileStorage YAML (config/situation_model.yaml) written by
experiments/situation/train.py. Two kinds are supported:
  mlp     tiny numpy MLP: x -> (x - mean) / std -> relu(W1 x + b1) [-> relu(W2 . + b2)] -> softmax
  rtrees  cv2.ml.RTrees, probabilities = share of tree votes
Both store 'features' and 'classes' (comma lists) that must match this package.
"""
import os

import cv2
import numpy as np

from . import SITUATIONS
from .features import FEATURE_NAMES


def default_path():
    """config/situation_model.yaml in the source tree, else in the installed share dir."""
    here = os.path.dirname(os.path.abspath(__file__))
    src = os.path.join(here, '..', '..', 'config', 'situation_model.yaml')
    if os.path.isfile(src):
        return os.path.normpath(src)
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('line_follower'), 'config', 'situation_model.yaml')
    except Exception:
        return src


class SituationModel:
    def __init__(self, path=None):
        path = path or default_path()
        if not os.path.isfile(path):
            raise FileNotFoundError(f'situation model not found: {path}')
        fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
        try:
            self.kind = fs.getNode('model_type').string()
            feats = tuple(fs.getNode('features').string().split(','))
            classes = tuple(fs.getNode('classes').string().split(','))
            if feats != FEATURE_NAMES:
                raise ValueError(f'{path}: model features do not match features.FEATURE_NAMES')
            if classes != SITUATIONS:
                raise ValueError(f'{path}: model classes do not match SITUATIONS')
            if self.kind == 'mlp':
                self.mean = fs.getNode('mean').mat().ravel().astype(np.float32)
                self.std = fs.getNode('std').mat().ravel().astype(np.float32)
                self.layers = []
                k = 1
                while not fs.getNode(f'W{k}').empty():
                    self.layers.append((fs.getNode(f'W{k}').mat().astype(np.float32),
                                        fs.getNode(f'b{k}').mat().ravel().astype(np.float32)))
                    k += 1
                if not self.layers:
                    raise ValueError(f'{path}: mlp without layers')
            elif self.kind == 'rtrees':
                self.rtrees = cv2.ml.RTrees_create()
                self.rtrees.read(fs.getNode('rtrees'))
            else:
                raise ValueError(f'{path}: unknown model_type {self.kind!r}')
            self.version = fs.getNode('version').string()
        finally:
            fs.release()
        self.path = path

    def proba(self, x):
        """Class probabilities (float64, len(SITUATIONS)) for one feature vector."""
        x = np.asarray(x, np.float32).reshape(1, -1)
        if self.kind == 'mlp':
            h = (x - self.mean) / self.std
            for k, (w, b) in enumerate(self.layers):
                h = h @ w + b
                if k + 1 < len(self.layers):
                    h = np.maximum(h, 0.0)
            z = h[0].astype(np.float64)
            z -= z.max()
            p = np.exp(z)
            return p / p.sum()
        votes = self.rtrees.getVotes(x, 0)
        p = np.zeros(len(SITUATIONS))
        for label, count in zip(votes[0], votes[1]):
            if 0 <= int(label) < len(p):
                p[int(label)] = count
        total = p.sum()
        return p / total if total > 0 else np.full(len(p), 1.0 / len(p))
