"""A source or parameter edit must change the no-git recording fingerprint."""
from line_follower_gcs.gcs_recorder import fingerprint


def test_fingerprint_tracks_sources_and_parameters(tmp_path):
    source = tmp_path/'src/example'
    source.mkdir(parents=True)
    code = source/'node.cpp'
    params = tmp_path/'params.yaml'
    code.write_text('int main() {return 0;}')
    params.write_text('speed: 0.1')
    initial = fingerprint(tmp_path, params)
    assert initial == fingerprint(tmp_path, params)
    code.write_text('int main() {return 1;}')
    changed_code = fingerprint(tmp_path, params)
    assert initial != changed_code
    params.write_text('speed: 0.2')
    assert fingerprint(tmp_path, params) != changed_code
