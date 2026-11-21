"""Recorder request idempotency and event extraction use plain Python data."""
from line_follower_gcs.bag_logic import RequestCache, EventExtractor


def test_bounded_cache_and_original_response():
    cache = RequestCache(2)
    cache.save('a', (True, 'started'))
    cache.save('b', (False, 'wrong run'))
    assert cache.get('a') == (True, 'started')
    cache.save('c', (True, 'stopped'))
    assert cache.get('a') is None
    assert cache.get('b') == (False, 'wrong run')


def test_events_sorted_relative_and_change_only():
    e = EventExtractor(start_ns=1000000000)
    e.feed('/mission/state', {'state': 0, 'run_id': 'r'}, 1000000000)
    e.feed('/mission/state', {'state': 0, 'run_id': 'r'}, 1100000000)
    e.feed('/mission/state', {'state': 3, 'run_id': 'r'}, 2000000000)
    e.feed('/safety/status', {'stop_reason': 'command candidate stale'}, 2500000000)
    e.feed('/path/status', {'mode': {'value': 5}}, 2400000000)
    e.feed('/perception/status', {'healthy': False, 'fallback_active': True, 'detail': 'fallback'}, 2600000000)
    e.feed('/gcs/command_events', {'command': 'kill', 'outcome': 'accepted', 'request_id': 'id', 'run_id': 'r', 'reason': 'stop'}, 2700000000)
    events = e.result()
    assert [x['time'] for x in events] == sorted(x['time'] for x in events)
    assert [x['text'] for x in events if x['kind']=='mission'] == ['STARTUP_WAIT', 'RUNNING']
    assert events[-1]['request_id'] == 'id' and events[-1]['time'] == 1.7
    assert any(x['kind']=='perception' and 'fallback' in x['text'] for x in events)
