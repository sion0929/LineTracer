"""ROS/Qt-independent display and link bookkeeping; no rover decisions live here."""
from collections import deque
import math

STATES = ['STARTUP_WAIT', 'CHECKING', 'READY', 'RUNNING', 'STOPPING', 'FINISHED',
          'FAULT', 'E_STOP', 'STOPPED']
MODES = ['VISION_TRACKING', 'MEMORY_TRACKING', 'REACQUIRING', 'SLOWDOWN',
         'ROTATE_IN_PLACE', 'STOPPING', 'STOPPED', 'IMAGE_TRACKING']


class LinkMonitor:
    def __init__(self, stale_sec=1., lost_sec=2.):
        self.stale_sec, self.lost_sec = stale_sec, lost_sec
        self.times = deque(maxlen=20)
        self.count = 0

    def receive(self, now):
        self.times.append(now)
        self.count += 1

    def status(self, now):
        age = now - self.times[-1] if self.times else math.inf
        rate = (len(self.times)-1)/(self.times[-1]-self.times[0]) if len(self.times) > 1 else 0.
        return ('LOST' if age > self.lost_sec else 'STALE' if age > self.stale_sec else 'OK',
                age, rate)


class DisplayModel:
    """Live layers + a run map of CONFIRMED path (display only, never sent to the rover — §10.2).

    Observations ahead of the robot are tentative; a point is confirmed once the robot has driven
    over it (just behind the axle, within a narrow lateral band). Confirmed cells and places the robot
    already passed are frozen: later observations there are not added again. Tentative points that
    are never driven over (e.g. a branch not taken) expire.
    """
    HISTORY_LAYERS = {'detected': ('observed', 'memory'), 'planned': ('planned',)}

    def __init__(self, history_cap=60000, history_cell=0.01, tentative_sec=30.,
                 pass_band=0.08, pass_depth=0.25, passed_cell=0.05):
        self.current = None
        self.layers = {}
        self.events = deque(maxlen=500)
        self.trail = deque(maxlen=20000)
        self.last_command = None
        self._changes = {}
        self.history_cap, self.history_cell = history_cap, history_cell
        self.tentative_sec, self.pass_band, self.pass_depth = tentative_sec, pass_band, pass_depth
        self.passed_cell = passed_cell
        self._history = {name: {} for name in self.HISTORY_LAYERS}
        self._tentative = {name: {} for name in self.HISTORY_LAYERS}
        self._passed = set()
        self._last_pose = None
        self._history_run = None

    def clear_history(self, trail=False):
        # The user's clear button only resets the confirmed map; a new run also resets the trail.
        for layer in (*self._history.values(), *self._tentative.values()):
            layer.clear()
        self._passed.clear()
        self._last_pose = None
        if trail:
            self.trail.clear()

    def history_points(self, name):
        return list(self._history[name].values())

    def _cell(self, x, y, size):
        return round(x/size), round(y/size)

    def _accumulate(self, data, now):
        run_id = data['mission'].get('run_id')
        if run_id != self._history_run:
            self.clear_history(trail=True)
            self._history_run = run_id
        pose = data.get('pose')
        if pose:
            x0, y0, yaw = pose[0], pose[1], pose[2] if len(pose) > 2 else 0.
            c, s = math.cos(yaw), math.sin(yaw)
            for name, tentative in self._tentative.items():
                confirmed = self._history[name]
                for cell, (x, y, seen) in list(tentative.items()):
                    along, lateral = c*(x-x0) + s*(y-y0), -s*(x-x0) + c*(y-y0)
                    if -self.pass_depth <= along <= 0. and abs(lateral) <= self.pass_band:
                        if len(confirmed) < self.history_cap:
                            confirmed.setdefault(cell, (x, y))
                        del tentative[cell]
                    elif now - seen > self.tentative_sec:
                        del tentative[cell]
        for name, sources in self.HISTORY_LAYERS.items():
            confirmed, tentative = self._history[name], self._tentative[name]
            for source in sources:
                for line in data.get(source, []):
                    for x, y in line:
                        cell = self._cell(x, y, self.history_cell)
                        if cell in confirmed or self._cell(x, y, self.passed_cell) in self._passed:
                            continue
                        tentative[cell] = (x, y, now)
        if pose:
            # mark the driven corridor as passed (interpolated between snapshots)
            x1, y1 = pose[0], pose[1]
            x_prev, y_prev = self._last_pose or (x1, y1)
            steps = max(1, int(math.hypot(x1-x_prev, y1-y_prev)/(self.passed_cell/2)))
            for k in range(steps + 1):
                f = k/steps
                self._passed.add(self._cell(x_prev + f*(x1-x_prev), y_prev + f*(y1-y_prev), self.passed_cell))
            self._last_pose = (x1, y1)

    def snapshot(self, data, now):
        self.current = data
        self.layers = {name: data.get(name, []) for name in ('observed', 'memory', 'planned', 'candidates')}
        self._accumulate(data, now)
        for event in data.get('events', []):
            kind, value = event['kind'], event['value']
            if value != self._changes.get(kind):
                labels = STATES if kind == 'mission' else MODES if kind == 'mode' else []
                text = labels[value] if isinstance(value, int) and 0 <= value < len(labels) else str(value)
                self.event(kind, text, event['stamp'])
                self._changes[kind] = value
        for kind, value in [('mission', data['mission']['state']), ('mode', data.get('mode')),
                            ('safety', data.get('stop_reason', ''))]:
            if value != self._changes.get(kind):
                labels = STATES if kind == 'mission' else MODES if kind == 'mode' else []
                text = labels[value] if isinstance(value, int) and 0 <= value < len(labels) else str(value)
                self.event(kind, text, data.get('stamp', now))
                self._changes[kind] = value
        if data.get('pose'):
            self.trail.append(data['pose'][:2])

    def event(self, kind, text, now):
        run_id = self.current['mission']['run_id'] if self.current else ''
        self.events.append({'kind': kind, 'text': text, 'time': now, 'run_id': run_id})

    def command(self, data, now):
        self.last_command = data
        self.event('command', f'{data.get("command")}: {data.get("outcome")} {data.get("reason", "")}', data.get('stamp', now))
