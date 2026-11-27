"""Apply a follower_node params file to every split node.

The yaml files (config/rule.yaml, experiments/isaac/params/*.yaml) put the settings under
`follower_node:`. The split nodes have other names, so a copy is written with that section
merged into `/**` (all nodes; the follower_node values win over existing `/**` keys).
"""
import os
import tempfile

import yaml

SECTIONS = ('follower_node', '/follower_node')
SPLIT_NODES = ('perception_node', 'map_node', 'planning_node', 'control_node')


def to_all_nodes(data):
    """Params dict -> the same with the follower_node section moved into /**."""
    data = dict(data or {})
    merged = {}
    for key in ('/**',) + SECTIONS:
        section = data.pop(key, None) or {}
        merged.update(section.get('ros__parameters', {}) or {})
    out = {'/**': {'ros__parameters': merged}}
    out.update(data)
    return out


def write_all_nodes(path, out_dir=None):
    """Write the rewritten copy of the params file `path`; returns the new path."""
    with open(path) as f:
        data = yaml.safe_load(f)
    fd, out = tempfile.mkstemp(prefix='lf_split_', suffix='.yaml', dir=out_dir)
    with os.fdopen(fd, 'w') as f:
        yaml.safe_dump(to_all_nodes(data), f, default_flow_style=None, sort_keys=False)
    return out
