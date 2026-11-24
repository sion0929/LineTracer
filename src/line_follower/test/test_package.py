"""Sanity tests for the line_follower package (colcon test; no ROS graph, no Gazebo).

Without any test file `colcon test` reports "NO TESTS RAN" (pytest exit code 5) and CI fails.
"""
import hashlib
from pathlib import Path

import pytest

PKG = Path(__file__).resolve().parents[1]
BRINGUP = PKG.parent / 'line_follower_bringup'


def _sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_copied_assets_match_provided_originals():
    """The package's worlds/ and models/ copies stay byte-identical to the provided originals."""
    expected = {}
    for line in (BRINGUP / 'SHA256SUMS').read_text().splitlines():
        if line.strip():
            digest, rel = line.split(None, 1)
            expected[rel.strip().lstrip('*')] = digest
    assert expected, 'SHA256SUMS is empty'
    copied = {str(p.relative_to(PKG)) for top in ('models', 'worlds')
              for p in (PKG / top).rglob('*') if p.is_file()}
    assert copied == set(expected), f'file sets differ: {sorted(copied ^ set(expected))}'
    mismatched = [rel for rel, digest in expected.items() if _sha256(PKG / rel) != digest]
    assert not mismatched, f'modified copies: {mismatched}'


def test_rule_plugins_import_and_unknown_names_fail():
    pytest.importorskip('numpy')
    pytest.importorskip('cv2')
    from line_follower.control import make_controller
    from line_follower.control.rule import RuleController  # noqa: F401
    from line_follower.perception import make_perception
    from line_follower.perception.rule import RulePerception  # noqa: F401

    with pytest.raises(ValueError):
        make_perception('no_such_plugin', None)
    with pytest.raises(ValueError):
        make_controller('no_such_plugin', None)
