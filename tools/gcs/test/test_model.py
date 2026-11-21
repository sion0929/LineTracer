"""Display bookkeeping stays usable without ROS or Qt."""
from line_follower_gcs.model import DisplayModel, LinkMonitor


def test_link_and_rate():
    link = LinkMonitor(stale_sec=1, lost_sec=2)
    assert link.status(0)[0] == 'LOST'
    for t in [0, .5, 1, 1.5]:
        link.receive(t)
    assert link.status(1.6) == ('OK', .10000000000000009, 2.0)
    assert link.status(2.6)[0] == 'STALE'
    assert link.status(3.6)[0] == 'LOST'


def test_events_and_layers():
    model = DisplayModel()
    snapshot = {'mission': {'state': 2, 'run_id': 'r', 'reason': 'ready'},
                'pose': (1, 2, 0), 'observed': [[(1, 2), (2, 2)]],
                'memory': [[(2, 2), (3, 2)]], 'planned': [[(1, 2), (3, 2)]],
                'candidates': [], 'mode': 3, 'stop_reason': 'drive not enabled'}
    model.snapshot(snapshot, 1)
    model.snapshot(snapshot, 2)
    assert len(model.events) == 3
    assert len(model.layers['memory']) == 1
    snapshot['mission']['state'] = 3
    model.snapshot(snapshot, 3)
    assert len(model.events) == 4
    model.command({'command': 'kill', 'outcome': 'accepted', 'reason': 'software stop'}, 4)
    assert model.events[-1]['kind'] == 'command'
    assert model.last_command['command'] == 'kill'
    for n in range(1000):
        model.command({'command': str(n)}, n)
    assert len(model.events) <= 500


def test_short_transitions_are_not_lost_between_snapshots():
    model = DisplayModel()
    model.snapshot({'mission': {'state': 8, 'run_id': 'run'},
                    'events': [{'kind': 'mission', 'value': 7, 'stamp': 20.1},
                               {'kind': 'mission', 'value': 8, 'stamp': 20.2}]}, 21)
    transitions = [e for e in model.events if e['kind'] == 'mission']
    assert [e['text'] for e in transitions] == ['E_STOP', 'STOPPED']
    assert [e['time'] for e in transitions] == [20.1, 20.2]



def _snap(run, pose, observed=(), planned=()):
    return {'mission': {'state': 3, 'run_id': run, 'reason': ''}, 'pose': pose,
            'observed': [list(observed)] if observed else [], 'memory': [],
            'planned': [list(planned)] if planned else [], 'candidates': [], 'mode': 0, 'stop_reason': ''}


def test_only_passed_path_is_confirmed():
    model = DisplayModel()
    ahead = [(0.3 + 0.01*i, 0.) for i in range(21)]          # seen 0.3-0.5 m ahead of the robot
    model.snapshot(_snap('r1', (0., 0., 0.), ahead, [(0.3, 0.)]), 1)
    assert model.history_points('detected') == []           # not passed yet -> tentative only
    model.snapshot(_snap('r1', (0.42, 0., 0.)), 2)          # robot drove over 0.30-0.42
    confirmed = model.history_points('detected')
    assert confirmed and all(x <= 0.42 + 1e-9 for x, _ in confirmed)
    assert (0.3, 0.) in model.history_points('planned')


def test_passed_places_are_frozen_and_untaken_branches_dropped():
    model = DisplayModel(tentative_sec=5.)
    model.snapshot(_snap('r1', (0., 0., 0.), [(0.3, 0.), (0.31, 0.)]), 1)
    model.snapshot(_snap('r1', (0.4, 0., 0.)), 2)
    frozen = sorted(model.history_points('detected'))
    # a later, slightly different observation of an already passed place is not added again
    model.snapshot(_snap('r1', (0.4, 0., 0.), [(0.305, 0.004)]), 3)
    model.snapshot(_snap('r1', (0.5, 0., 0.)), 4)
    assert sorted(model.history_points('detected')) == frozen
    # a branch seen beside the route but never driven over expires instead of being confirmed
    model.snapshot(_snap('r1', (0.5, 0., 0.), [(0.8, 0.5)]), 5)
    model.snapshot(_snap('r1', (0.6, 0., 0.)), 20)
    assert (0.8, 0.5) not in model.history_points('detected')


def test_new_run_and_clear_reset_the_map():
    model = DisplayModel()
    model.snapshot(_snap('r1', (0., 0., 0.), [(0.3, 0.)]), 1)
    model.snapshot(_snap('r1', (0.4, 0., 0.)), 2)
    assert model.history_points('detected')
    model.snapshot(_snap('r2', (0., 0., 0.)), 3)
    assert model.history_points('detected') == []
    model.snapshot(_snap('r2', (0., 0., 0.), [(0.3, 0.)]), 4)
    model.snapshot(_snap('r2', (0.4, 0., 0.)), 5)
    model.clear_history()
    assert model.history_points('detected') == [] and len(model.trail) > 0   # button keeps the driven trail
    model.clear_history(trail=True)
    assert len(model.trail) == 0
