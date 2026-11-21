"""T-05: no /world/ topics/services or set_pose anywhere in runtime code, launch or config."""
from pathlib import Path
import re

WS = Path(__file__).resolve().parents[3]
SCAN_DIRS = [WS / 'src', WS / 'tools']
# Only this offline capture harness may teleport an owned stationary simulator.
DEV_CAPTURE = WS / 'tools/check_d05.py'
SKIP_DIRS = {'line_follower_tests', 'worlds', 'models'}  # tests + provided originals
SUFFIXES = {'.cpp', '.hpp', '.h', '.py', '.yaml', '.yml', '.xml', '.txt', '.msg', '.srv'}
FORBIDDEN = re.compile(r'/world/|set_pose')


def _files():
    for root in SCAN_DIRS:
        for path in root.rglob('*'):
            if path != DEV_CAPTURE and path.is_file() and path.suffix in SUFFIXES and \
                    not SKIP_DIRS.intersection(path.relative_to(WS).parts):
                yield path


def test_scan_finds_sources():
    assert any(p.suffix == '.cpp' for p in _files())


def test_no_forbidden_gazebo_interfaces():
    hits = [
        f'{p.relative_to(WS)}:{i}: {line.strip()}'
        for p in _files()
        for i, line in enumerate(p.read_text(errors='ignore').splitlines(), 1)
        if FORBIDDEN.search(line)
    ]
    assert not hits, 'forbidden Gazebo interfaces used:\n' + '\n'.join(hits)


def test_only_safety_guard_publishes_cmd_vel():
    publishers = [
        p.relative_to(WS) for p in _files()
        if p.suffix == '.cpp' and re.search(r'create_publisher<[^>]*Twist>\(\s*"/cmd_vel"',
                                            p.read_text())
    ]
    assert [p.name for p in publishers] == ['safety_guard_node.cpp']


def test_capture_harness_stays_out_of_runtime():
    for path in (WS/'src').rglob('*'):
        if path.is_file() and path.suffix in {'.py','.cpp','.hpp','.xml','.yaml'} and \
                not SKIP_DIRS.intersection(path.relative_to(WS).parts):
            assert 'check_d05' not in path.read_text(errors='ignore')
            assert 'd05_groundtruth' not in path.read_text(errors='ignore')
