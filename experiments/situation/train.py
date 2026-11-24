#!/usr/bin/env python3
"""DEV ONLY: train the situation classifier, report held-out metrics, export the package model.

Data (collect.py): data/synth.npz (synthetic replay on training maps), data/gz_train.npz (Gazebo,
30 training maps), data/gz_val.npz (Gazebo, 12 held-out maps; never used for fitting or selection).
Model selection uses an internal split by map of the training data (20 % of the synthetic maps and
of the gz_train maps).

  python3 experiments/situation/train.py                 # train mlp + rtrees, pick, export, report
  python3 experiments/situation/train.py --model mlp     # force a model kind

Writes
  src/line_follower/config/situation_model.yaml     cv2.FileStorage model (package runtime)
  src/line_follower/test/data/situation_fixture.npz real Gazebo patches from gz_val (test fixture)
  experiments/results/situation/model/metrics.{json,md} per-class precision / recall, confusion,
                                                    finish-judge replay, size, latency
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT / "src/line_follower"))
from line_follower.situation import INDEX, SITUATIONS  # noqa: E402
from line_follower.situation import features as FE  # noqa: E402

DATA = HERE / "data"
MODEL_OUT = ROOT / "src/line_follower/config/situation_model.yaml"
FIXTURE = ROOT / "src/line_follower/test/data/situation_fixture.npz"
REPORT = ROOT / "experiments/results/situation/model"
NC = len(SITUATIONS)
FIN = INDEX["FINISH_T"]
P_GRID = (0.4, 0.5, 0.6, 0.7, 0.8, 0.9)
MAX_BYTES = 1_000_000


def load(name):
    z = np.load(DATA / f"{name}.npz", allow_pickle=False)
    assert tuple(z["features"]) == FE.FEATURE_NAMES, f"{name}: feature set changed, rebuild the data"
    return {k: z[k] for k in ("X", "y", "tcand", "s", "map", "run", "cand_finish")}


def cat(*parts):
    return {k: np.concatenate([p[k] for p in parts]) for k in parts[0]}


def subset(d, mask):
    return {k: v[mask] for k, v in d.items()}


def split_by_map(d, frac, seed):
    maps = np.unique(d["map"])
    rng = np.random.default_rng(seed)
    held = set(rng.choice(maps, max(1, int(round(frac * len(maps)))), replace=False))
    m = np.array([x in held for x in d["map"]])
    return subset(d, ~m), subset(d, m)


# ---------------------------------------------------------------- models
class MLP:
    kind = "mlp"

    def __init__(self, hidden=(64, 32), epochs=40, lr=3e-3, batch=512, l2=1e-4, seed=0):
        self.hidden, self.epochs, self.lr, self.batch, self.l2, self.seed = hidden, epochs, lr, batch, l2, seed

    def fit(self, X, y, w):
        rng = np.random.default_rng(self.seed)
        self.mean = X.mean(0).astype(np.float32)
        self.std = np.maximum(X.std(0), 1e-3).astype(np.float32)
        Xn = ((X - self.mean) / self.std).astype(np.float32)
        sizes = [X.shape[1], *self.hidden, NC]
        self.W = [(rng.normal(0, math.sqrt(2.0 / a), (a, b))).astype(np.float32) for a, b in zip(sizes[:-1], sizes[1:])]
        self.b = [np.zeros(b, np.float32) for b in sizes[1:]]
        params = self.W + self.b
        m = [np.zeros_like(p) for p in params]
        v = [np.zeros_like(p) for p in params]
        t = 0
        Y = np.eye(NC, dtype=np.float32)[y]
        w = (w / w.mean()).astype(np.float32)
        for ep in range(self.epochs):
            lr = self.lr * (0.5 * (1 + math.cos(math.pi * ep / self.epochs)))
            order = rng.permutation(len(Xn))
            for i0 in range(0, len(order), self.batch):
                idx = order[i0:i0 + self.batch]
                xb, yb, wb = Xn[idx], Y[idx], w[idx]
                acts = [xb]
                h = xb
                for k, (W, b) in enumerate(zip(self.W, self.b)):
                    h = h @ W + b
                    if k + 1 < len(self.W):
                        h = np.maximum(h, 0)
                    acts.append(h)
                z = acts[-1] - acts[-1].max(1, keepdims=True)
                p = np.exp(z)
                p /= p.sum(1, keepdims=True)
                g = (p - yb) * wb[:, None] / len(idx)
                gW, gb = [None] * len(self.W), [None] * len(self.W)
                for k in range(len(self.W) - 1, -1, -1):
                    gW[k] = acts[k].T @ g + self.l2 * self.W[k]
                    gb[k] = g.sum(0)
                    if k:
                        g = (g @ self.W[k].T) * (acts[k] > 0)
                t += 1
                for j, (p_, g_) in enumerate(zip(params, gW + gb)):
                    m[j] = 0.9 * m[j] + 0.1 * g_
                    v[j] = 0.999 * v[j] + 0.001 * g_ * g_
                    p_ -= lr * (m[j] / (1 - 0.9 ** t)) / (np.sqrt(v[j] / (1 - 0.999 ** t)) + 1e-8)
        return self

    def proba(self, X):
        h = (X - self.mean) / self.std
        for k, (W, b) in enumerate(zip(self.W, self.b)):
            h = h @ W + b
            if k + 1 < len(self.W):
                h = np.maximum(h, 0)
        z = h - h.max(1, keepdims=True)
        p = np.exp(z)
        return p / p.sum(1, keepdims=True)

    def write(self, fs):
        fs.write("mean", self.mean.reshape(1, -1))
        fs.write("std", self.std.reshape(1, -1))
        for k, (W, b) in enumerate(zip(self.W, self.b), 1):
            fs.write(f"W{k}", W)
            fs.write(f"b{k}", b.reshape(1, -1))


class Forest:
    kind = "rtrees"

    def __init__(self, trees=25, depth=8, min_count=10):
        self.trees, self.depth, self.min_count = trees, depth, min_count

    def fit(self, X, y, w):
        # cv2 RTrees has no sample weights: resample by weight instead
        rng = np.random.default_rng(1)
        p = w / w.sum()
        idx = rng.choice(len(X), len(X), p=p)
        self.m = cv2.ml.RTrees_create()
        self.m.setMaxDepth(self.depth)
        self.m.setMinSampleCount(self.min_count)
        self.m.setActiveVarCount(int(math.sqrt(X.shape[1])) + 1)
        self.m.setTermCriteria((cv2.TERM_CRITERIA_MAX_ITER, self.trees, 0))
        self.m.train(X[idx].astype(np.float32), cv2.ml.ROW_SAMPLE, y[idx].astype(np.int32))
        return self

    def proba(self, X):
        out = np.zeros((len(X), NC))
        for i, x in enumerate(X.astype(np.float32)):
            votes = self.m.getVotes(x.reshape(1, -1), 0)
            for lab, c in zip(votes[0], votes[1]):
                out[i, int(lab)] = c
        return out / np.maximum(out.sum(1, keepdims=True), 1)

    def write(self, fs):
        self.m.write(fs, "rtrees")


def weights(d, gz_factor):
    counts = np.bincount(d["y"], minlength=NC).astype(float)
    cw = 1.0 / np.sqrt(np.maximum(counts, 1.0))
    cw /= cw[d["y"]].mean()
    w = cw[d["y"]] * np.where(d["tcand"] > 0, 2.0, 1.0)
    w = w * np.where(np.char.startswith(d["src"], "gz"), gz_factor, 1.0)
    return w


# ---------------------------------------------------------------- metrics
def per_class(y, pred):
    out = {}
    for k, name in enumerate(SITUATIONS):
        tp = int(((pred == k) & (y == k)).sum())
        fp = int(((pred == k) & (y != k)).sum())
        fn = int(((pred != k) & (y == k)).sum())
        out[name] = dict(n=int((y == k).sum()), precision=round(tp / (tp + fp), 3) if tp + fp else None,
                         recall=round(tp / (tp + fn), 3) if tp + fn else None)
    return out


def confusion(y, pred):
    c = np.zeros((NC, NC), int)
    np.add.at(c, (y, pred), 1)
    return c


def macro_f1(y, pred):
    f = []
    for k in range(NC):
        tp = ((pred == k) & (y == k)).sum()
        fp = ((pred == k) & (y != k)).sum()
        fn = ((pred != k) & (y == k)).sum()
        if (y == k).sum():
            f.append(2 * tp / max(2 * tp + fp + fn, 1))
    return float(np.mean(f))


def finish_replay(d, P, p_min=0.6, frames=3, seen_min=0.10, hit_max=0.10):
    """confirm_finish replay on the T candidates of each run (time order within a run is the file order).

    Counts false accepts (an accept whose candidate is not the finish) and correct accepts. The
    same-place rule is approximated by consecutive candidates of one label."""
    X = d["X"]
    gate = (X[:, FE.F["end_beyond_seen"]] >= seen_min) & (X[:, FE.F["end_beyond_hit"]] <= hit_max)
    ok = (P[:, FIN] >= p_min) & gate & (d["tcand"] > 0)
    false_acc = good_acc = 0
    runs = {}
    for i in np.nonzero(d["tcand"] > 0)[0]:
        runs.setdefault((d["map"][i], d["run"][i]), []).append(i)
    for idx in runs.values():
        votes, prev, accepted = 0, None, False
        for i in idx:
            same = prev is not None and d["y"][i] == d["y"][prev]
            votes = (votes + 1 if same else 1) if ok[i] else 0
            prev = i if ok[i] else None
            if votes >= frames and not accepted:
                accepted = True
                if d["y"][i] == FIN:
                    good_acc += 1
                else:
                    false_acc += 1
    tc = d["tcand"] > 0
    fin_runs = len({(d["map"][i], d["run"][i]) for i in np.nonzero(tc & (d["y"] == FIN))[0]})
    return dict(runs_with_t=len(runs), runs_with_finish_candidates=fin_runs, false_accepts=false_acc,
                correct_accepts=good_acc,
                tcand_precision=round(float((ok & (d["y"] == FIN)).sum() / max(ok.sum(), 1)), 3),
                tcand_recall=round(float((ok & (d["y"] == FIN)).sum() / max((tc & (d["y"] == FIN)).sum(), 1)), 3))


def write_model(model, path, meta):
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_WRITE)
    fs.write("model_type", model.kind)
    fs.write("version", meta)
    fs.write("features", ",".join(FE.FEATURE_NAMES))
    fs.write("classes", ",".join(SITUATIONS))
    model.write(fs)
    fs.release()


def latency(path):
    from line_follower.situation.model import SituationModel
    m = SituationModel(str(path))
    z = np.load(FIXTURE) if FIXTURE.is_file() else None
    x = np.zeros(len(FE.FEATURE_NAMES), np.float32)
    t = time.perf_counter()
    for _ in range(2000):
        m.proba(x)
    t_pred = (time.perf_counter() - t) / 2000 * 1e3
    t_feat = None
    if z is not None:
        patches = [np.unpackbits(p)[:FE.PH * FE.PW].reshape(FE.PH, FE.PW).astype(bool) for p in z["patch"][:200]]
        t = time.perf_counter()
        for i, patch in enumerate(patches):
            n = int(z["n"][i])
            FE.extract_local(patch, z["rp"][i][:n] if n >= 2 else None, {0: "open", 1: "t", 2: "len"}.get(int(z["end"][i]), "x"),
                             float(z["pitch"][i]), z["start"][i], float(z["x_far"][i]))
        t_feat = (time.perf_counter() - t) / len(patches) * 1e3
    return round(t_pred, 4), None if t_feat is None else round(t_feat, 3)


def make_fixture(per_class=25, seed=3):
    """Real Gazebo samples (patch + path + context + label) from the held-out gz_val runs."""
    import collect
    rows = []
    for base in sorted((DATA / "gz").glob("gz_val*")):
        for run in sorted(base.iterdir()):
            if (run / "situation.npz").is_file():
                rows += collect.run_samples(run, keep_patches=True)
    rng = np.random.default_rng(seed)
    pick = []
    for k in range(NC):
        idx = [i for i, r in enumerate(rows) if r["y"] == k]
        tc = [i for i in idx if rows[i]["tcand"]]
        rest = [i for i in idx if not rows[i]["tcand"]]
        take = list(rng.permutation(tc)[:per_class // 2]) if tc else []
        take += list(rng.permutation(rest)[:per_class - len(take)])
        pick += take
    pick = sorted(pick)
    FIXTURE.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        FIXTURE, patch=np.stack([rows[i]["patch"] for i in pick]), rp=np.stack([rows[i]["rp"] for i in pick]).astype(np.float32),
        n=np.array([rows[i]["n"] for i in pick], np.int16), end=np.array([rows[i]["end"] for i in pick], np.int8),
        pitch=np.array([rows[i]["pitch"] for i in pick], np.float32), x_far=np.array([rows[i]["x_far"] for i in pick], np.float32),
        start=np.stack([rows[i]["start"] for i in pick]).astype(np.float32), y=np.array([rows[i]["y"] for i in pick], np.int8),
        tcand=np.array([rows[i]["tcand"] for i in pick], np.int8), classes=np.array(SITUATIONS))
    print("fixture", len(pick), "samples", FIXTURE.stat().st_size, "bytes")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", choices=("auto", "mlp", "rtrees"), default="auto")
    ap.add_argument("--gz-factor", type=float, default=3.0, help="weight of Gazebo samples vs synthetic")
    ap.add_argument("--no-synth", action="store_true")
    ap.add_argument("--no-export", action="store_true")
    a = ap.parse_args()

    parts = []
    if not a.no_synth:
        s = cat(*[load(n) for n in ("synth", "synth_markings") if (DATA / f"{n}.npz").is_file()])
        s["src"] = np.full(len(s["y"]), "synth")
        parts.append(s)
    g = load("gz_train")
    g["src"] = np.full(len(g["y"]), "gz")
    parts.append(g)
    val = load("gz_val")
    train_all = cat(*parts)
    fit, sel = split_by_map(train_all, 0.2, seed=7)
    print("train", len(fit["y"]), "select", len(sel["y"]), "gz_val", len(val["y"]))
    cands = {"mlp": lambda: MLP(), "mlp_wide": lambda: MLP(hidden=(96, 48)), "rtrees": lambda: Forest()}
    kinds = list(cands) if a.model == "auto" else [a.model]
    scores = {}
    for kind in kinds:
        m = cands[kind]().fit(fit["X"], fit["y"], weights(fit, a.gz_factor))
        P = m.proba(sel["X"])
        gzsel = np.char.startswith(sel["src"], "gz")
        tmp = REPORT / f"_size_{kind}.yaml"
        REPORT.mkdir(parents=True, exist_ok=True)
        write_model(m, tmp, "size probe")
        size = tmp.stat().st_size
        tmp.unlink()
        # lowest P(FINISH_T) threshold without a false finish on the selection split
        p_min = next((pm for pm in P_GRID if finish_replay(sel, P, p_min=pm)["false_accepts"] == 0), None)
        scores[kind] = dict(macro_f1=round(macro_f1(sel["y"], P.argmax(1)), 4),
                            macro_f1_gz=round(macro_f1(sel["y"][gzsel], P[gzsel].argmax(1)), 4),
                            bytes=size, p_min=p_min,
                            finish=finish_replay(sel, P, p_min=p_min or P_GRID[-1]))
        print(kind, scores[kind], flush=True)
    ok = [k for k in kinds if scores[k]["bytes"] < MAX_BYTES and scores[k]["p_min"] is not None] or kinds
    best = max(ok, key=lambda k: scores[k]["macro_f1_gz"])
    p_min = scores[best]["p_min"] or P_GRID[-1]
    model = cands[best]().fit(train_all["X"], train_all["y"], weights(train_all, a.gz_factor))
    P = model.proba(val["X"])
    pred = P.argmax(1)
    tc = val["tcand"] > 0
    report = dict(
        model=best, p_min=p_min, selection=scores, created=time.strftime("%Y-%m-%d %H:%M"),
        data={k: int(len(d["y"])) for k, d in (("synth", parts[0]) if not a.no_synth else ("synth", {"y": []}),
                                                   ("gz_train", g), ("gz_val", val))},
        maps={"gz_train": int(len(np.unique(g["map"]))), "gz_val": int(len(np.unique(val["map"]))),
              "synth": int(len(np.unique(parts[0]["map"]))) if not a.no_synth else 0},
        class_counts={k: {name: int((d["y"] == i).sum()) for i, name in enumerate(SITUATIONS)}
                      for k, d in (("train", train_all), ("gz_val", val))},
        gz_val_per_class=per_class(val["y"], pred), gz_val_macro_f1=round(macro_f1(val["y"], pred), 4),
        gz_val_accuracy=round(float((pred == val["y"]).mean()), 4),
        gz_val_tcand_per_class=per_class(val["y"][tc], pred[tc]),
        gz_val_confusion=confusion(val["y"], pred).tolist(),
        gz_val_finish_replay={str(pm): finish_replay(val, P, p_min=pm) for pm in P_GRID},
    )
    if not a.no_export:
        write_model(model, MODEL_OUT, f"{best} {time.strftime('%Y-%m-%d')} synth+gz_train")
        make_fixture()
        report["model_bytes"] = MODEL_OUT.stat().st_size
        report["latency_ms"] = dict(zip(("predict", "features"), latency(MODEL_OUT)))
    REPORT.mkdir(parents=True, exist_ok=True)
    (REPORT / "metrics.json").write_text(json.dumps(report, indent=1) + "\n")
    write_md(report)
    print(json.dumps({k: report[k] for k in ("model", "p_min", "gz_val_macro_f1", "gz_val_accuracy")}, indent=1))


def write_md(r):
    L = ["# 상황 분류기 지표 (보류 Gazebo 맵 gz_val)", "",
         f"- 모델: **{r['model']}** (선택된 P(FINISH_T) 문턱 {r['p_min']}) · 생성 {r['created']} · 크기 {r.get('model_bytes', '-')} B · "
         f"지연 {r.get('latency_ms', {})} ms",
         f"- 데이터: {r['data']} · 맵 {r['maps']}",
         f"- gz_val 정확도 {r['gz_val_accuracy']} · macro F1 {r['gz_val_macro_f1']}", "",
         "| 상황 | 표본 | 정밀도 | 재현율 | T 후보 표본 | T 후보 정밀도 | T 후보 재현율 |", "|---|---|---|---|---|---|---|"]
    for name in SITUATIONS:
        a, b = r["gz_val_per_class"][name], r["gz_val_tcand_per_class"][name]
        L.append(f"| {name} | {a['n']} | {a['precision']} | {a['recall']} | {b['n']} | {b['precision']} | {b['recall']} |")
    L += ["", "## 종점 판정 재생 (confirm_finish 규칙을 gz_val T 후보 순서에 적용, 3프레임·시야/너머 조건 포함)", "",
          "회차 = Gazebo 수집 1회(맵 하나의 한 구간). 문턱은 학습 쪽 선택 분할에서 잘못된 확정이 0인 가장 낮은 값을 고른다.", "",
          "| p_min | T 후보가 있던 회차 | 실제 종점 후보가 있던 회차 | 잘못된 확정 | 올바른 확정 | 후보 정밀도 | 후보 재현율 |",
          "|---|---|---|---|---|---|---|"]
    for pm, f in r["gz_val_finish_replay"].items():
        L.append(f"| {pm} | {f['runs_with_t']} | {f['runs_with_finish_candidates']} | {f['false_accepts']} | "
                 f"{f['correct_accepts']} | {f['tcand_precision']} | {f['tcand_recall']} |")
    L += ["", "## 혼동 행렬 (행 = 정답, 열 = 예측)", "", "| | " + " | ".join(s[:6] for s in SITUATIONS) + " |",
          "|---|" + "---|" * NC]
    for name, row in zip(SITUATIONS, r["gz_val_confusion"]):
        L.append(f"| {name} | " + " | ".join(str(v) for v in row) + " |")
    L += ["", "## 모델 선택 (학습 데이터 안의 맵 단위 20 % 분할; 1 MB 미만 중 Gazebo 표본 macro F1 최대)", "",
          "| 후보 | macro F1(전체) | macro F1(Gazebo) | 크기 B | 문턱 | 잘못된 확정 | 올바른 확정 |", "|---|---|---|---|---|---|---|"]
    for k, v in r["selection"].items():
        L.append(f"| {k} | {v['macro_f1']} | {v['macro_f1_gz']} | {v['bytes']} | {v['p_min']} | "
                 f"{v['finish']['false_accepts']} | {v['finish']['correct_accepts']} |")
    (REPORT / "metrics.md").write_text("\n".join(L) + "\n")


if __name__ == "__main__":
    main()
