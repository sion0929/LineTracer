import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ros_args import node_scoped  # noqa: E402


def test_plain_override_is_scoped_to_node():
    assert node_scoped(['-p', 'perception:=depth']) == ['-p', 'follower_node:perception:=depth']


def test_dotted_name_and_other_args_kept():
    args = ['--params-file', 'x.yaml', '--param', 'depth.model_path:=/a:b.onnx', '-r', 'a:=b']
    assert node_scoped(args) == ['--params-file', 'x.yaml', '--param',
                                 'follower_node:depth.model_path:=/a:b.onnx', '-r', 'a:=b']


def test_already_scoped_left_alone():
    assert node_scoped(['-p', 'other:k:=1']) == ['-p', 'other:k:=1']
