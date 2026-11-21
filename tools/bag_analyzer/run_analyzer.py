#!/usr/bin/env python3
"""Extract a time-ordered event timeline and topic counts from a rosbag2 directory."""
import argparse
import json
from pathlib import Path
from line_follower_gcs.bag_archive import BagArchive


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag')
    parser.add_argument('--json', action='store_true', help='print JSON instead of text')
    parser.add_argument('--output', type=Path, help='also write the JSON report')
    args = parser.parse_args()
    archive = BagArchive(args.bag)
    report = {'bag': archive.path, 'duration_sec': archive.duration,
              'counts': archive.counts, 'events': archive.events}
    formatted = json.dumps(report, indent=2, ensure_ascii=False)
    if args.output:
        args.output.write_text(formatted+'\n')
    if args.json:
        print(formatted)
    else:
        for topic, count in sorted(archive.counts.items()):
            print(f'{count:8d}  {topic}')
        for event in archive.events:
            print(f'{event["time"]:9.3f}s  {event["kind"]:11}  {event["text"]}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
