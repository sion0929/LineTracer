"""Replay uses a positive sensor allowlist; command-bearing bag topics cannot play."""
from pathlib import Path
import ast


def test_replay_sensor_allowlist_and_final_remap():
    path = Path(__file__).resolve().parents[2]/'line_follower_bringup/launch/replay.launch.py'
    source = path.read_text()
    tree = ast.parse(source)
    cmd = next(node for node in ast.walk(tree) if isinstance(node, ast.keyword) and node.arg=='cmd'
               and isinstance(node.value, ast.List))
    strings = [x.value for x in cmd.value.elts if isinstance(x, ast.Constant)]
    topics = strings[strings.index('--topics')+1:]
    assert topics == ['/camera/image_raw','/imu','/odom']
    assert "'cmd_vel_topic': '/replay/cmd_vel_blocked'" in source
    assert 'sim.launch.py' not in source
    assert "'camera_bridge': 'false'" in source
    assert "'obstacles': 'false'" in source
