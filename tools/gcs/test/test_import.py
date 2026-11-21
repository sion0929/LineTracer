"""The dev-only GCS node modules import and expose main()."""
import importlib

import pytest


@pytest.mark.parametrize('name', ['gcs_bridge', 'gcs_command', 'gcs_mission_manager'])
def test_module_has_main(name):
    module = importlib.import_module(f'line_follower_gcs.{name}')
    assert callable(module.main)
