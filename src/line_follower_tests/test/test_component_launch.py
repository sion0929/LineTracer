"""The composed and baseline profiles must never enable both camera bridges."""
import importlib.util
from pathlib import Path
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
import pytest


@pytest.mark.parametrize('profile', ['competition', 'dev_sim'])
@pytest.mark.parametrize('compose', ['true', 'True', 'false', 'False'])
def test_external_camera_is_only_for_baseline(profile, compose):
    path = Path(__file__).resolve().parents[2]/'line_follower_bringup/launch'/f'{profile}.launch.py'
    spec = importlib.util.spec_from_file_location('profile_under_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    sim = next(action for action in description.entities if isinstance(action, IncludeLaunchDescription))
    context = LaunchContext()
    context.launch_configurations.update({'compose':compose,'headless':'true'})
    expression = dict(sim.launch_arguments)['camera_bridge']
    assert expression.perform(context).lower() == str(compose.lower() != 'true').lower()


def test_competition_disables_extension():
    path = Path(__file__).resolve().parents[2]/'line_follower_bringup/launch/competition.launch.py'
    spec = importlib.util.spec_from_file_location('competition_policy', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    includes = [a for a in module.generate_launch_description().entities if isinstance(a, IncludeLaunchDescription)]
    assert dict(includes[1].launch_arguments).get('obstacles') == 'false'

@pytest.mark.parametrize('camera', ['true', 'false'])
def test_gazebo_bridge_stays_loaded_through_shutdown(monkeypatch,camera):
    path=Path(__file__).resolve().parents[2]/'line_follower_bringup/launch/nodes.launch.py'
    spec=importlib.util.spec_from_file_location('container_lifetime',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    monkeypatch.setattr(module,'ComposableNodeContainer',lambda **kwargs:kwargs)
    monkeypatch.setenv('LD_PRELOAD','existing_library.so')
    context=LaunchContext()
    context.launch_configurations.update({'params_file':'unused.yaml','use_sim_time':'true',
        'compose':'true','obstacles':'false','camera_bridge':camera})
    container=module._perception(context)[0]
    env=container.get('additional_env',{})
    if camera=='true':
        assert 'libros_gz_bridge.so' in env.get('LD_PRELOAD','')
        assert 'existing_library.so' in env['LD_PRELOAD']
    else: assert 'LD_PRELOAD' not in env
