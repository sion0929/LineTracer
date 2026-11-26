import numpy as np

from line_follower import vision
from line_follower.perception.log import CHUNK, PerceptionLog, load


class _P:
    def __init__(self, v):
        self.value = v


class Node:
    def __init__(self, p):
        self.p = p

    def has_parameter(self, k):
        return k in self.p

    def get_parameter(self, k):
        return _P(self.p[k])


def test_log_roundtrip(tmp_path):
    log = PerceptionLog(Node({'perception': 'rule'}), str(tmp_path))
    rng = np.random.default_rng(0)
    frames = []
    for i in range(CHUNK + 7):
        line = rng.random((vision.BH, vision.BW)) > 0.97
        valid = np.ones_like(line)
        log.record(i / 30, (i * 0.01, 0.0, 0.1), 0.002, i * 0.01, i % 5 != 0,
                   line.astype(np.uint8) * 255, valid.astype(np.uint8) * 255)
        frames.append(line)
    log.flush()
    meta, d = load(tmp_path)
    assert meta['settings']['perception'] == 'rule'
    assert len(d['t']) == CHUNK + 7
    assert np.array_equal(d['line'], np.stack(frames))
    assert d['valid'].all() and not d['far'].any()
    assert d['used'].sum() == sum(i % 5 != 0 for i in range(CHUNK + 7))
    assert np.allclose(d['pose'][3], [0.03, 0.0, 0.1])


def test_log_off_without_path(monkeypatch):
    monkeypatch.delenv('LF_PERCEPTION_LOG', raising=False)
    log = PerceptionLog(Node({}), '')
    log.record(0.0, None, None, 0.0, False, np.zeros((vision.BH, vision.BW), np.uint8),
               np.zeros((vision.BH, vision.BW), np.uint8))
    log.flush()
    assert not log.ok


def test_log_hill_fields_and_format1(tmp_path):
    log = PerceptionLog(Node({}), str(tmp_path))
    z = np.zeros((vision.BH, vision.BW), np.uint8)
    for i, s in enumerate(['flat', 'climb', 'deck', 'descent', None]):
        log.record(i / 30, (0.0, 0.0, 0.0), 0.0, i * 0.01, True, z, z, imu_pitch=0.1 * i, elev=0.01 * i, slope=s)
    log.flush()
    _, d = load(tmp_path)
    assert list(d['slope']) == [0, 1, 2, 3, -1]
    assert np.allclose(d['imu_pitch'], [0.0, 0.1, 0.2, 0.3, 0.4]) and np.allclose(d['elev'][2], 0.02)
    # a format-1 chunk (no hill fields) still loads, with the 'unknown' values
    c = dict(np.load(tmp_path / 'chunk_0000.npz'))
    for k in ('imu_pitch', 'elev', 'slope'):
        del c[k]
    np.savez_compressed(tmp_path / 'chunk_0000.npz', **c)
    _, d = load(tmp_path)
    assert np.isnan(d['imu_pitch']).all() and (d['slope'] == -1).all() and len(d['t']) == 5
