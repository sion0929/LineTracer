"""Playbook control (판단): learned situation classifier + per-situation actions on top of 'rule'.

Every ~0.1 s the situation of the line ahead is classified from the node's own line map,
traced path and IMU pitch (line_follower.situation). The class probabilities are smoothed and
the confident situation selects a row of config/playbook.yaml:

  v_cap           speed cap [m/s] in this situation (pure pursuit curvature is kept: w scales with v)
  v_max, a_lat, a_dec, omega_max, lookahead_min, lookahead_gain
                  replace the node parameter of the same name while in this situation

Finish judgement (confirm_finish, replaces the driven-distance gate finish_min_dist):
a T-bar path end is accepted only when the classifier says FINISH_T with probability
>= finish.p_min on finish.frames consecutive candidates at the same place, the camera has
already seen at least finish.beyond_seen_min metres past the bar and the map holds no line
there (finish.beyond_hit_max). Crossing bars at the edge of the view, stubs, loop crossings
and the start T are rejected. While such a candidate is pending the speed is kept low enough
to stop at it (finish.approach_decel).

The rule controller (control/rule.py) is used unchanged through a parameter proxy.
"""
import math
import os

from .base import Controller
from .rule import RuleController
from ..situation import INDEX, SITUATIONS
from ..situation import features as FE
from ..situation.model import SituationModel

DEFAULTS = {
    'update_period': 0.1,
    'smoothing': 0.5,
    'min_confidence': 0.45,
    'finish': {'p_min': 0.6, 'frames': 3, 'beyond_seen_min': 0.10, 'beyond_hit_max': 0.10,
               'same_place': 0.15, 'approach_decel': 0.6},
    'situations': {},
}
OVERRIDABLE = ('v_max', 'a_lat', 'a_dec', 'omega_max', 'lookahead_min', 'lookahead_gain')


def default_config():
    here = os.path.dirname(os.path.abspath(__file__))
    src = os.path.normpath(os.path.join(here, '..', '..', 'config', 'playbook.yaml'))
    if os.path.isfile(src):
        return src
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('line_follower'), 'config', 'playbook.yaml')
    except Exception:
        return src


def load_config(path):
    import yaml
    with open(path) as fh:
        raw = yaml.safe_load(fh) or {}
    cfg = dict(DEFAULTS, **{k: v for k, v in raw.items() if k != 'finish'})
    cfg['finish'] = dict(DEFAULTS['finish'], **(raw.get('finish') or {}))
    cfg['situations'] = {k: dict(v or {}) for k, v in (raw.get('situations') or {}).items()}
    unknown = set(cfg['situations']) - set(SITUATIONS)
    if unknown:
        raise ValueError(f'{path}: unknown situations {sorted(unknown)}')
    return cfg


class _Value:
    __slots__ = ('value',)

    def __init__(self, value):
        self.value = value


class _ParamProxy:
    """The node as RuleController sees it, with some parameters replaced."""

    def __init__(self, node):
        self._node = node
        self.overrides = {}

    def get_parameter(self, name):
        if name in self.overrides:
            return _Value(self.overrides[name])
        return self._node.get_parameter(name)

    def __getattr__(self, name):
        return getattr(self._node, name)


class PlaybookController(Controller):
    def __init__(self, node, config=None, model=None):
        super().__init__(node)
        for name in ('playbook.config', 'playbook.model'):
            if hasattr(node, 'has_parameter') and not node.has_parameter(name):
                node.declare_parameter(name, '')
        cfg_path = config or self._param('playbook.config') or default_config()
        self.cfg = load_config(cfg_path) if isinstance(cfg_path, str) else cfg_path
        self.model = model if model is not None else SituationModel(self._param('playbook.model') or None)
        self.proxy = _ParamProxy(node)
        self.rule = RuleController(self.proxy)
        self.probs = None
        self.situation = None           # confident situation name or None
        self.last_update = None
        self.start = None
        self.finish_votes = 0
        self.finish_last = None         # (x, y) odom of the last voted candidate
        self.last_finish_p = 0.0

    def _param(self, name):
        try:
            return self.node.get_parameter(name).value
        except Exception:
            return ''

    def _now(self):
        try:
            return self.node.get_clock().now().nanoseconds * 1e-9
        except Exception:
            return None

    # ---- situation ------------------------------------------------------
    def classify(self, path=None, path_end=None):
        node = self.node
        if self.start is None and node.pose is not None:
            self.start = node.pose[:2]
        f = FE.from_node(node, path, path_end, self.start)
        return f, self.model.proba(f)

    def update_situation(self):
        now = self._now()
        if now is not None and self.last_update is not None and now - self.last_update < self.cfg['update_period']:
            return
        self.last_update = now
        _, p = self.classify()
        a = self.cfg['smoothing']
        self.probs = p if self.probs is None else a * p + (1.0 - a) * self.probs
        k = int(self.probs.argmax())
        self.situation = SITUATIONS[k] if self.probs[k] >= self.cfg['min_confidence'] else None

    def actions(self):
        return self.cfg['situations'].get(self.situation, {}) if self.situation else {}

    # ---- control --------------------------------------------------------
    def compute(self, rp, dt):
        node = self.node
        self.update_situation()
        act = self.actions()
        self.proxy.overrides = {k: act[k] for k in OVERRIDABLE if k in act}
        v, w = self.rule.compute(rp, dt)
        cap = act.get('v_cap')
        if node.t_point is None and node.path_end == 't' and len(rp) >= 2:
            # unconfirmed T ahead: stay able to stop at it
            d = math.hypot(*rp[-1])
            dec = self.cfg['finish']['approach_decel']
            v_stop = math.sqrt(2.0 * dec * max(0.0, d - 0.05)) + node.get_parameter('v_min').value
            cap = v_stop if cap is None else min(cap, v_stop)
        if cap is not None and v > cap:
            w = w * cap / v if v > 1e-6 else w
            v = cap
        return v, w

    # ---- finish ---------------------------------------------------------
    def confirm_finish(self, path, observation):
        fin = self.cfg['finish']
        f, p = self.classify(path, 't')
        pf = float(p[INDEX['FINISH_T']])
        self.last_finish_p = pf
        cand = path[-1]
        same = self.finish_last is not None and math.hypot(cand[0] - self.finish_last[0],
                                                          cand[1] - self.finish_last[1]) < fin['same_place']
        ok = (pf >= fin['p_min'] and f[FE.F['end_beyond_seen']] >= fin['beyond_seen_min']
              and f[FE.F['end_beyond_hit']] <= fin['beyond_hit_max'])
        self.finish_votes = (self.finish_votes + 1 if same else 1) if ok else 0
        self.finish_last = cand if ok else None
        accept = self.finish_votes >= fin['frames']
        if accept and hasattr(self.node, 'get_logger'):
            self.node.get_logger().info(
                f'playbook: finish T accepted (p={pf:.2f}, beyond seen {f[FE.F["end_beyond_seen"]]:.2f} m, '
                f'{self.finish_votes} frames)')
        return accept
