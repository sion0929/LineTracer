"""Convert ROS snapshots to a plain display model, retaining geometry provenance."""
import math


def display_snapshot(msg):
    q, pos = msg.robot_pose.orientation, msg.robot_pose.position
    pose = (pos.x, pos.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
    stale = set(getattr(msg, 'stale_sources', []))
    graph = msg.observed_graph
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    graph_stamp = graph.header.stamp.sec + graph.header.stamp.nanosec * 1e-9
    valid_graph = 'observed_graph' not in stale and graph.header.frame_id == 'odom' and 0 <= stamp-graph_stamp < 1.
    observed = [[(p.x, p.y) for p in seg.points if p.source == p.SOURCE_OBSERVED]
                for seg in graph.segments] if valid_graph else []
    memory, planned = [], []
    if not {'selected_path','path_status','pose'}.intersection(stale) and msg.path_valid and msg.selected_path.header.frame_id == 'odom':
        # Split at invalid/unknown points rather than drawing through gaps.
        for target, predicate in [(planned, lambda p: p.source != p.SOURCE_UNKNOWN),
                                  (memory, lambda p: p.source == p.SOURCE_MEMORY_UPDATED)]:
            run = []
            for point in msg.selected_path.points:
                if predicate(point):
                    run.append((point.x, point.y))
                elif run:
                    target.append(run)
                    run = []
            if run:
                target.append(run)
    candidates, quality = [], []
    if valid_graph and 'candidates' not in stale:
        for candidate in msg.candidates.candidates:
            if not candidate.geometry_feasible:
                continue
            lines = []
            for seg in graph.segments:
                if seg.id in candidate.segment_ids:
                    line = [(p.x, p.y) for p in seg.points]
                    candidates.append(line)
                    lines.append(line)
            quality.append({'id': getattr(candidate, 'candidate_id', 0), 'lines': lines,
                            'rule_score': getattr(candidate, 'rule_score', 0.),
                            'mlp_score': getattr(candidate, 'mlp_score', float('nan')),
                            'selected': getattr(candidate, 'selected', False)})
    s = msg.mission
    events = []
    for kind, messages in [('mission', getattr(msg, 'mission_events', [])),
                           ('mode', getattr(msg, 'mode_events', [])),
                           ('safety', getattr(msg, 'safety_events', []))]:
        for event in messages:
            value = event.state if kind == 'mission' else event.mode.value if kind == 'mode' else event.stop_reason
            events.append({'kind': kind, 'value': value,
                           'stamp': event.header.stamp.sec + event.header.stamp.nanosec*1e-9})
    return {'events': sorted(events, key=lambda e: e['stamp']), 'stamp': stamp, 'mission': {'state': s.state, 'drive_enabled': s.drive_enabled, 'reason': s.reason,
                        'run_id': s.run_id, 'elapsed': s.elapsed_since_start},
            'pose': pose if msg.pose_valid and 'pose' not in stale else None, 'observed': observed, 'memory': memory,
            'planned': planned, 'candidates': candidates, 'candidate_quality': quality, 'stale_sources': sorted(stale),
            'mode': msg.path_status.mode.value if 'path_status' not in stale else None,
            'stop_reason': msg.safety_status.stop_reason if 'safety_status' not in stale else 'STALE: safety_status'}
