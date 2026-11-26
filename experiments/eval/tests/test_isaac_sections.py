import json
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import isaac_sections


def test_profile_name_strips_last_rep():
    assert isaac_sections.profile_name("V0_01") == "V0"
    assert isaac_sections.profile_name("V0-logodds_03") == "V0-logodds"
    assert isaac_sections.profile_name("DLV4close-logodds_02") == "DLV4close-logodds"
    assert isaac_sections.profile_name("no-suffix") == "no-suffix"


def test_section_means_and_reach():
    traj = np.array([
        [0.0, 0.0, 0.0, 0.1, 0.01],
        [0.0, 0.0, 0.0, 0.5, 0.03],
        [0.0, 0.0, 0.0, 1.2, -0.02],
        [0.0, 0.0, 0.0, 1.8, 0.04],
    ])
    stats = isaac_sections.section_stats(traj, np.array([1.0, 2.0]))
    assert stats[0]["mean_cm"] == pytest.approx(2.0)
    assert stats[1]["mean_cm"] == pytest.approx(3.0)
    assert [item["reached"] for item in stats] == [True, True]
    assert isaac_sections.run_pass_count(stats) == 1

    short_traj = np.array([[0.0, 0.0, 0.0, 0.1, 0.01], [0.0, 0.0, 0.0, 0.8, 0.03]])
    short = isaac_sections.section_stats(short_traj, np.array([1.0, 2.0]))
    assert [item["reached"] for item in short] == [True, False]
    assert np.isnan(short[1]["mean_cm"])
    assert isaac_sections.section_stats(traj[:2], np.array([1.0]))[0]["reached"] is False


def test_ignores_nonfinite_lateral_error():
    traj = np.array([[0.0, 0.0, 0.0, 0.2, 0.01], [0.0, 0.0, 0.0, 0.4, np.nan],
                     [0.0, 0.0, 0.0, 0.8, -0.03]])
    stats = isaac_sections.section_stats(traj, np.array([1.0]))
    assert stats[0]["mean_cm"] == pytest.approx(2.0)
    assert stats[0]["samples"] == 2


def test_fisher_one_sided_completion():
    assert isaac_sections.fisher_one_sided(2, 2, 0, 2) == pytest.approx(1 / 6)
    assert isaac_sections.fisher_one_sided(0, 2, 2, 2) == pytest.approx(1.0)
    assert isaac_sections.fisher_one_sided(5, 10, 5, 10) == pytest.approx(0.5, abs=0.2)


def make_run(root, name, traj, complete, reason="finished", lap=60.0, progress=100.0):
    run = root / name
    run.mkdir(parents=True)
    (run / "result.json").write_text(json.dumps({
        "complete": complete, "reason": reason, "progress_pct": progress, "lap_sec": lap}))
    np.save(run / "trajectory.npy", np.asarray(traj, float))


def test_report_groups_profiles_and_renders(tmp_path):
    ends = isaac_sections.load_ends()
    s = np.linspace(0.0, float(ends[-1]) - 1e-3, 300)
    traj = np.column_stack((s, np.zeros_like(s), np.zeros_like(s), s, np.full_like(s, 0.01)))
    make_run(tmp_path, "V0_01", traj, True, lap=60.0)
    make_run(tmp_path, "V0_02", traj, True, lap=62.0)
    make_run(tmp_path, "V0-logodds_01", traj[:100], False, reason="stalled at 1.0m", lap=None,
             progress=20.0)
    (tmp_path / "empty").mkdir()
    report = isaac_sections.build_report([tmp_path])
    assert set(report["profiles"]) == {"V0", "V0-logodds"}
    assert report["profiles"]["V0"]["complete"] == 2
    assert report["profiles"]["V0"]["mean_lap_sec"] == pytest.approx(61.0)
    assert report["profiles"]["V0"]["mean_pass_sections"] == pytest.approx(14.0)
    assert report["profiles"]["V0-logodds"]["complete"] == 0
    text = isaac_sections.render(report)
    assert "**S4**" in text and "**S11**" in text and "**S14**" in text
    assert "Fisher exact" in text
    assert "V0-logodds_01" in text


def test_missing_results_are_skipped(tmp_path):
    report = isaac_sections.build_report([tmp_path / "absent", tmp_path])
    assert report["runs"] == []
    assert report["profiles"] == {}
