"""Plain recording-policy and event primitives shared by analysis and UI."""
from collections import OrderedDict
from .model import STATES, MODES


class RequestCache:
    def __init__(self, capacity=256):
        self.capacity = capacity
        self.replies = OrderedDict()

    def get(self, key):
        return self.replies.get(key)

    def save(self, key, reply):
        self.replies[key] = reply
        while len(self.replies)>self.capacity:
            self.replies.popitem(last=False)


class EventExtractor:
    def __init__(self, start_ns):
        self.start_ns = start_ns
        self.previous = {}
        self.events = []
        self.run_id = ''

    def feed(self, topic, data, stamp_ns):
        kind, value, text = None, None, None
        if topic == '/mission/state':
            self.run_id = data.get('run_id', self.run_id)
            kind, value = 'mission', data['state']
            text = STATES[value]
        elif topic == '/safety/status':
            kind, value = 'safety', data['stop_reason']
            text = value or 'command approved'
        elif topic == '/path/status':
            kind, value = 'mode', data['mode']['value']
            text = MODES[value]
        elif topic == '/perception/status':
            kind, value = 'perception', (data['healthy'], data['fallback_active'])
            text = f'healthy={value[0]} fallback={value[1]} {data.get("detail", "")}'
        elif topic == '/gcs/command_events':
            self.events.append({'time': (stamp_ns-self.start_ns)/1000000000, 'stamp_ns': stamp_ns,
                'kind': 'command', 'text': f'{data["command"]}: {data["outcome"]} {data["reason"]}',
                'command': data['command'], 'outcome': data['outcome'],
                'request_id': data['request_id'], 'run_id': data.get('run_id', self.run_id)})
            return
        if kind and self.previous.get(kind) != value:
            self.previous[kind] = value
            self.events.append({'time': (stamp_ns-self.start_ns)/1000000000, 'stamp_ns': stamp_ns,
                                'kind': kind, 'text': text, 'run_id': self.run_id})

    def result(self):
        return sorted(self.events, key=lambda item: item['stamp_ns'])
