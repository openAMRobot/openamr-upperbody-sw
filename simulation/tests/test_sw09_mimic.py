# SPDX-License-Identifier: MIT
"""SW-09 of the single-writer map: openarm_*_finger_joint2 has no physics constraint with Gazebo's default engine.

Each backend here constrains it explicitly: MuJoCo with an equality constraint (checked structurally by the parity tests and
dynamically here), Isaac Sim with the importer's mimic joint (checked in the job here and on the USD by check_usd.py).
"""
import mujoco
import pytest

from openamr_sim import isaac
from openamr_sim import physics as ph

TARGETS = {"left": 0.7854, "right": -0.7854}  # SRDF `open` of each hand


@pytest.mark.parametrize("name", ["bp000", "bp050"])
def test_mujoco_follower_tracks_leader(models, name, overrides, evidence):
    m = models[name]
    r = ph.check_mimic_follow(mujoco, m, mujoco.MjData(m), "L1175_home", overrides, TARGETS)
    evidence(f"sw09_mujoco_{name}", r)
    for side in TARGETS:
        assert abs(r["finger1_final"][side]) > 0.5, "the leader did not move, so the test says nothing"
        assert r["max_follow_error_rad"][side] < 1e-3


def test_mujoco_without_the_constraint_the_follower_does_not_track(models, overrides):
    """Control: the same motion with the equality switched off. The follower stays behind, so the test above can fail."""
    m = models["bp000"]
    r = ph.check_mimic_follow(mujoco, m, mujoco.MjData(m), "L1175_home", overrides, TARGETS, equality_active=False)
    for side in TARGETS:
        assert r["max_follow_error_rad"][side] > 0.3


def test_mujoco_follower_has_no_actuator(models, overrides):
    for m in models.values():
        actuated = {m.joint(m.actuator_trnid[a, 0]).name for a in range(m.nu)}
        assert not set(overrides["mimic"]["joints"]) & actuated


def test_isaac_job_imports_the_mimic(expansions, tmp_path, overrides):
    job = isaac.build_job(expansions["bp000"], tmp_path)
    assert job["importer"]["parse_mimic"] is True
    assert sorted(job["mimic"]) == sorted(overrides["mimic"]["joints"])
    for name, spec in job["mimic"].items():
        assert spec["joint"] == name.replace("finger_joint2", "finger_joint1")
        assert spec["multiplier"] == 1.0 and spec["offset"] == 0.0
    assert not set(job["mimic"]) & set(job["drives"])
