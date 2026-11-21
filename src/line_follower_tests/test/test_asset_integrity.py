"""T-02: provided world and models are byte-identical to the recorded manifest."""
import hashlib
from pathlib import Path

ASSETS = Path(__file__).resolve().parents[2] / 'line_follower_bringup'


def _manifest():
    entries = {}
    for line in (ASSETS / 'SHA256SUMS').read_text().splitlines():
        digest, name = line.split(maxsplit=1)
        entries[name.strip()] = digest
    return entries


def test_manifest_covers_world():
    assert 'worlds/line_track.sdf' in _manifest()


def test_assets_match_manifest():
    manifest = _manifest()
    on_disk = {
        str(p.relative_to(ASSETS)) for d in ('worlds', 'models') for p in (ASSETS / d).rglob('*')
        if p.is_file()
    }
    assert on_disk == set(manifest), 'asset files added or removed'
    for name, digest in manifest.items():
        actual = hashlib.sha256((ASSETS / name).read_bytes()).hexdigest()
        assert actual == digest, f'{name} was modified'
