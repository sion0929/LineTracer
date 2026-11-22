#!/usr/bin/env python3
"""Dependency boundary checks for CI (no ROS needed).

1. Every workspace package's package.xml parses and names a unique package.
2. The competition set (line_follower_bringup and everything it depends on inside the
   workspace) never depends on dev-only packages (GCS, integration tests, tools/).
3. The Dockerfile builds exactly that set from src/ (COPY src, --packages-up-to bringup).
Exit code 1 with a readable report on any violation.
"""
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
DEV_ONLY = {'line_follower_gcs', 'line_follower_tests'}
DEP_TAGS = ('depend', 'build_depend', 'build_export_depend', 'exec_depend', 'buildtool_depend')


def packages():
    found = {}
    for xml in sorted(ROOT.glob('src/*/package.xml')) + sorted(ROOT.glob('tools/*/package.xml')):
        root = ET.parse(xml).getroot()
        name = root.findtext('name').strip()
        if name in found:
            sys.exit(f'duplicate package name {name}: {found[name]["path"]} and {xml}')
        deps = {e.text.strip() for tag in DEP_TAGS for e in root.findall(tag) if e.text}
        found[name] = {'path': xml.relative_to(ROOT), 'deps': deps}
    return found


def closure(pkgs, start):
    seen, todo = set(), [start]
    while todo:
        name = todo.pop()
        if name in seen or name not in pkgs:
            continue
        seen.add(name)
        todo += [d for d in pkgs[name]['deps'] if d in pkgs]
    return seen


def main():
    pkgs = packages()
    errors = []
    competition = closure(pkgs, 'line_follower_bringup')
    print('workspace packages:', ', '.join(sorted(pkgs)))
    print('competition set   :', ', '.join(sorted(competition)))
    for name in sorted(competition):
        if str(pkgs[name]['path']).startswith('tools/'):
            errors.append(f'{name} is in the competition set but lives under tools/')
        bad = pkgs[name]['deps'] & DEV_ONLY
        if bad:
            errors.append(f'{name} ({pkgs[name]["path"]}) depends on dev-only {sorted(bad)}')
    dockerfile = (ROOT / 'Dockerfile').read_text()
    if not re.search(r'^COPY\s+src\s', dockerfile, re.M):
        errors.append('Dockerfile must COPY src (competition sources only)')
    if re.search(r'^COPY\s+tools', dockerfile, re.M):
        errors.append('Dockerfile must not COPY tools/ (dev-only)')
    if '--packages-up-to line_follower_bringup' not in dockerfile:
        errors.append('Dockerfile must build --packages-up-to line_follower_bringup')
    for e in errors:
        print('ERROR:', e)
    print('dependency boundary:', 'FAIL' if errors else 'PASS')
    return 1 if errors else 0


if __name__ == '__main__':
    sys.exit(main())
