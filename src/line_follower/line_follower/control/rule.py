"""Rule-based control: pure pursuit + curvature speed limit (original user algorithm).

Owner: rule-based teammate. Moved verbatim from FollowerNode.control(); behaviour is
unchanged. Tune via the node parameters (v_max, a_lat, lookahead_* ...).
"""
import math

from .base import Controller


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class RuleController(Controller):
    def compute(self, rp, dt):
        node = self.node
        gp = node.get_parameter
        v_max, v_min = gp('v_max').value, gp('v_min').value
        a_lat, a_dec, a_acc = gp('a_lat').value, gp('a_dec').value, gp('a_acc').value
        w_max = gp('omega_max').value

        # top speed here: v_max, or the track-specific plan [s_from, s_to, v, ...]
        # (s = distance driven), braking early for slower windows ahead
        v_top, v_brake = v_max, 1e9
        prof = list(gp('speed_profile').value)
        for k in range(0, len(prof) - 2, 3):
            s0, s1, vw = prof[k], prof[k + 1], prof[k + 2]
            if s0 <= node.travel < s1:
                v_top = vw
            elif s0 > node.travel:
                v_brake = min(v_brake, math.sqrt(vw * vw + 2.0 * a_dec * (s0 - node.travel)))
        v_max = min(v_top, v_brake) if v_top > v_brake else v_top
        v_max = max(v_max, v_min)

        # speed limit from the curvature of the path ahead, with braking distance
        v_lim = v_max
        s, a, prev_h = math.hypot(*rp[0]), rp[0], None
        for b in rp[1:]:
            seg = math.hypot(b[0] - a[0], b[1] - a[1])
            if seg < 0.05:
                continue
            h = math.atan2(b[1] - a[1], b[0] - a[0])
            if prev_h is not None:
                kappa = abs(wrap(h - prev_h)) / seg
                v_c = max(v_min, min(v_max, math.sqrt(a_lat / max(kappa, 1e-3))))
                v_lim = min(v_lim, math.sqrt(v_c * v_c + 2.0 * a_dec * max(0.0, s - 0.10)))
            prev_h, a = h, b
            s += seg
        if abs(node.pitch_f) > math.radians(3.0):
            v_lim = min(v_lim, gp('v_hill').value)
        # do not outrun what has been seen (unless the path ends at the finish T)
        if node.t_point is not None:
            d = math.hypot(*node.to_robot(*node.t_point))
            v_lim = min(v_lim, math.sqrt(2.0 * a_dec * max(0.0, d - 0.01)) + 0.04)
        elif node.path_end == 'open':
            v_lim = min(v_lim, math.sqrt(v_min * v_min + 2.0 * a_dec * max(0.0, s - 0.10)))
        v = max(min(v_min, v_lim), min(v_lim, node.v_cmd + a_acc * dt))
        v = max(v, node.v_cmd - 1.0 * dt)

        # pure pursuit
        ld = gp('lookahead_min').value + gp('lookahead_gain').value * v
        tgt = rp[-1]
        for p in rp:
            if math.hypot(p[0], p[1]) >= ld:
                tgt = p
                break
        d2 = max(1e-4, tgt[0] ** 2 + tgt[1] ** 2)
        alpha = math.atan2(tgt[1], tgt[0])
        if abs(alpha) > math.radians(60):
            v = min(v, v_min)               # sharp corner: turn almost on the spot
            w = 2.5 * alpha
        else:
            w = 2.0 * v * tgt[1] / d2
        w = max(-w_max, min(w_max, w))
        return v, w
