"""Geometry validity and source colours must survive the ROS-to-view boundary."""
from types import SimpleNamespace as N
from line_follower_gcs.adapter import display_snapshot


def fixture():
    stamp = N(sec=2, nanosec=0)
    header = N(stamp=stamp, frame_id='odom')
    def point(x, source):
        return N(x=x, y=0., source=source, SOURCE_OBSERVED=0, SOURCE_MEMORY_UPDATED=1, SOURCE_UNKNOWN=2)
    points = [point(0, 0), point(.1, 1), point(.2, 2), point(.3, 1)]
    return N(header=header, pose_valid=True, robot_pose=N(position=N(x=0.,y=0.),
             orientation=N(x=0.,y=0.,z=0.,w=1.)), mission=N(state=3, drive_enabled=True,
             reason='run', run_id='r', elapsed_since_start=21.),
             observed_graph=N(header=header, segments=[N(id=1, points=[points[0]])]),
             path_valid=True, selected_path=N(header=header, points=points),
             candidates=N(candidates=[N(geometry_feasible=True, segment_ids=[1])]),
             path_status=N(mode=N(value=0)), safety_status=N(stop_reason=''))


def test_unknown_points_do_not_join_planned_lines():
    data = display_snapshot(fixture())
    assert data['planned'] == [[(0., 0.), (.1, 0.)], [(.3, 0.)]]
    assert data['memory'] == [[(.1, 0.)], [(.3, 0.)]]
    assert data['candidates'] == [[(0., 0.)]]


def test_stale_or_invalid_geometry_is_hidden():
    msg = fixture()
    msg.path_valid = False
    msg.observed_graph.header = N(stamp=N(sec=0, nanosec=0), frame_id='odom')
    data = display_snapshot(msg)
    assert not any(data[layer] for layer in ('observed', 'memory', 'planned', 'candidates'))


def test_stale_sources_hide_layers_and_report_status():
    msg = fixture()
    msg.stale_sources = ['observed_graph', 'candidates', 'selected_path', 'safety_status']
    data = display_snapshot(msg)
    assert not data['observed'] and not data['candidates'] and not data['planned']
    assert 'safety_status' in data['stale_sources']


def test_candidate_scores_are_retained_and_nan_safe():
    import math
    msg = fixture()
    candidate = msg.candidates.candidates[0]
    candidate.rule_score, candidate.mlp_score = .8, float('nan')
    candidate.selected, candidate.candidate_id = True, 7
    quality = display_snapshot(msg)['candidate_quality'][0]
    assert quality['rule_score'] == .8 and math.isnan(quality['mlp_score'])
    assert quality['selected'] and quality['id'] == 7
