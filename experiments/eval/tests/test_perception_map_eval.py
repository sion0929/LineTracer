import json
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import perception_map_eval as evaluator


@pytest.fixture
def metadata():
    return {
        "centerline": [[0, 0], [1, 0]],
        "painted": [[[0, 0], [0.4, 0]], [[0.6, 0], [1, 0]]],
        "markings": [[[1, -0.05, 0], [1, 0.05, 0]], [[1, 0.7, 0], [0, 0.7, 0]]],
        "case": {"decoys": [{"kind": "parallel", "xy": [[0, 0.7], [1, 0.7]]}]},
        "line_width_m": 0.02,
    }


def test_union_deduplicates_decoy_and_keeps_unpainted_gap(metadata):
    lines, samples, decoy = evaluator.ground_truth(metadata)
    assert len(lines) == 4
    assert decoy.sum() == 201
    assert np.sum(np.isclose(samples[:, 0], 1) & ~decoy) == 21
    assert np.min(np.linalg.norm(samples - [0.5, 0], axis=1)) == pytest.approx(0.1)
    metadata["painted"].extend([[[0, 0.7], [0.5, 0.7]], [[0.5, 0.7], [1, 0.7]]])
    _, samples_with_duplicate, decoy_with_duplicate = evaluator.ground_truth(metadata)
    np.testing.assert_allclose(samples_with_duplicate, samples)
    np.testing.assert_array_equal(decoy_with_duplicate, decoy)


def test_score_exact_precision_visibility_and_distance_bins(metadata):
    z = {
        "xy": np.array([[0.3, 0.02], [0.5, 0], [0.5, 0.7], [0.8, 0.021]]),
        "dist": np.array([0.49, 0.5, 0.9, 1.3]),
        "track": np.array([[0, 0, 0], [2, 1, 0]]),
        "conf": np.array([0.95, 0.95, 0.3, 0.1]),
    }
    scores, (_, _, _, good, missed) = evaluator.score_map(z, metadata)
    np.testing.assert_array_equal(good, [True, False, True, False])
    assert scores["precision"] == {"hits": 2, "total": 4, "rate": 0.5}
    assert scores["recall"]["1.0"]["decoy"]["total"] == 201
    assert scores["recall"]["1.0"]["decoy"]["hits"] > 0  # between sparse trajectory samples
    assert scores["recall"]["0.5"]["decoy"]["rate"] is None
    assert scores["recall"]["0.5"]["main"]["hits"] > 0
    assert [band["precision"]["hits"] for band in scores["distance"].values()] == [1, 0, 1, 0]
    assert scores["distance"]["0.9-1.3"]["recall"]["1.0"]["main"]["hits"] == 0
    assert scores["calibration"]["0.8-1.0"]["rate"] == 0.5
    assert len(missed) > 0


def test_signed_offsets_follow_travel_direction():
    forward, _ = evaluator.signed_offsets([[0.5, 0.01]], [[0, 0], [1, 0]])
    backward, _ = evaluator.signed_offsets([[0.5, 0.01]], [[1, 0], [0, 0]])
    assert forward[0] == pytest.approx(1.0)
    assert backward[0] == pytest.approx(-1.0)


def test_score_offset_metrics_and_sections():
    meta = {"centerline": [[0, 0], [1, 0]], "painted": [[[0, 0], [1, 0]]], "line_width_m": 0.02}
    z = {
        "xy": np.array([[0.2, 0.01], [0.5, -0.02], [0.8, 0.0]]),
        "dist": np.array([0.2, 0.5, 0.8]),
        "track": np.array([[0, 0, 0], [1, 1, 0]]),
    }
    scores, _ = evaluator.score_map(z, meta, section_ends=[0.5, 1.0])
    assert scores["offset_cm"]["mean"] == pytest.approx(-1 / 3, abs=1e-9)
    assert scores["offset_cm"]["abs_mean"] == pytest.approx(1 / 3, abs=1e-9)
    assert scores["offset_cm"]["p95_abs"] == pytest.approx(1.9)
    assert scores["offset_cm"]["cells"] == 3
    assert scores["offset_by_section"]["1"]["cells"] == 1
    assert scores["offset_by_section"]["1"]["mean"] == pytest.approx(1.0)
    assert scores["offset_by_section"]["2"]["cells"] == 2
    assert scores["offset_by_section"]["2"]["mean"] == pytest.approx(-1.0)


def test_pool_recomputes_offset_from_cells():
    meta = {"centerline": [[0, 0], [1, 0]], "painted": [[[0, 0], [1, 0]]], "line_width_m": 0.02}
    z = {
        "xy": np.array([[0.2, 0.01], [0.5, -0.02]]),
        "dist": np.array([0.2, 0.5]),
        "track": np.array([[0, 0, 0], [1, 1, 0]]),
    }
    scores, _ = evaluator.score_map(z, meta)
    merged = evaluator.pool([scores, scores])
    assert merged["offset_cm"]["cells"] == 4
    assert merged["offset_cm"]["mean"] == pytest.approx(-0.5)
    assert merged["offset_cm"]["p95_abs"] == pytest.approx(2.0)


def test_shared_route_point_is_counted_once():
    meta = {"centerline": [[0, 0], [1, 0]], "case": {"decoys": [{"xy": [[0.5, 0], [0.5, 1]]}]}}
    _, points, decoy = evaluator.ground_truth(meta)
    shared = np.where(np.all(np.isclose(points, [0.5, 0]), axis=1))[0]
    assert len(shared) == 1
    assert not decoy[shared[0]]


def test_reports_pool_partial_and_empty_maps_skip_error(tmp_path, metadata):
    runs = tmp_path / "runs"
    world = tmp_path / "track.sdf"
    world.with_suffix(".json").write_text(json.dumps(metadata))
    for name, reason in [("complete", "finish"), ("empty", "stall"), ("error", "error: follower")]:
        run = runs / name
        run.mkdir(parents=True)
        (run / "job.json").write_text(json.dumps({"map": str(world), "profile": "rule"}))
        (run / "result.json").write_text(json.dumps({"reason": reason, "complete": name == "complete"}))
    np.savez(runs / "complete/map_world.npz", xy=[[0.2, 0], [0.5, 0]], dist=[0.5, 1.3],
             track=[[0, 0, 0], [1, 1, 0]], conf=[0.8, 1.0])
    np.savez(runs / "empty/map_world.npz", xy=np.empty((0, 2)), dist=[], track=[[0, 0.5, 0]])
    out = tmp_path / "out"
    report = evaluator.evaluate(runs, out)
    assert report == json.loads((out / "perception_map_eval.json").read_text())
    assert [row["run"] for row in report["runs"]] == ["complete", "empty"]
    assert report["skipped"] == [{"run": "error", "reason": "error: follower"}]
    assert report["aggregate"] == report["profiles"]["rule"]
    assert report["aggregate"]["precision"]["rate"] == 0.5
    run_recall = [row["scores"]["recall"]["1.0"]["all"] for row in report["runs"]]
    assert run_recall[1]["hits"] == 0
    assert report["aggregate"]["recall"]["1.0"]["all"]["total"] == sum(r["total"] for r in run_recall)
    assert report["aggregate"]["calibration"]["0.8-1.0"]["mean_confidence"] == pytest.approx(0.9)
    assert {p.name for p in out.glob("*.png")} == {"complete.png", "empty.png"}
    assert (out / "complete.png").read_bytes().startswith(b"\x89PNG")
    assert report["runs"][0]["scores"]["offset_cm"]["cells"] == 1
    assert report["runs"][0]["scores"]["offset_cm"]["mean"] == pytest.approx(0.0)
    assert report["aggregate"]["offset_cm"] == report["profiles"]["rule"]["offset_cm"]
    table = (out / "perception_map_eval.md").read_text()
    assert "Recall 0.5 m" in table and "Empirical precision" in table
    assert "Offset mean (cm)" in table and "Offset p95 abs (cm)" in table
