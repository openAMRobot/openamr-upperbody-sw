# SPDX-License-Identifier: MIT
"""The expansion each backend is generated from is the locked one (digital-twin lock, mirrored in simulation/expansion_lock.yaml)."""
import copy
import subprocess

import pytest

from openamr_sim import expand as ex


@pytest.mark.parametrize("name", sorted(ex.CONFIGURATIONS))
def test_expansion_matches_lock(expansions, name, evidence):
    e = expansions[name]
    assert e.lock_status in ("matches lock", "hash in lock, inputs changed (description digest not locked; re-lock pending)")
    entry = ex.lock_lookup(e.inputs)
    if entry is None:  # inputs changed but the expansion is a locked one: compare with that entry
        entry = next(x for x in ex.load_lock()["entries"] if x["sha256"] == e.sha256
                     and x["inputs"]["base_plate"] == e.inputs["base_plate"] and x["inputs"]["hardware_profile"] == "gazebo")
    assert entry["sha256"] == e.sha256
    assert entry["bytes"] == len(e.text.encode())
    assert entry["links"] == e.text.count("<link ") and entry["joints"] == e.text.count("<joint ")
    evidence(f"expansion_{name}", {"sha256": e.sha256, "inputs": e.inputs, "lock_entry": entry["source"], "status": e.lock_status})


def test_gazebo_profile_and_named_configurations():
    """The backends use the gazebo expansion, the one the Gazebo simulation uses. Lift position is not an argument."""
    assert sorted(ex.CONFIGURATIONS) == ["bp000", "bp050"]
    for args in ex.CONFIGURATIONS.values():
        assert args["hardware_profile"] == "gazebo"
        assert set(args) == {"base_plate", "hardware_profile"}


def test_lock_mismatch_is_an_error(platform_sw, openarm_description, monkeypatch):
    current = ex.expand("bp000", platform_sw, openarm_description, check_lock=False)
    lock = {"entries": [{"source": "test", "inputs": dict(current.inputs), "sha256": "0" * 64}]}
    monkeypatch.setattr(ex, "load_lock", lambda: lock)
    with pytest.raises(ex.LockMismatch):
        ex.expand("bp000", platform_sw, openarm_description)


def test_unlocked_inputs_are_not_a_pass(platform_sw, openarm_description, monkeypatch):
    lock = copy.deepcopy(ex.load_lock())
    for entry in lock["entries"]:
        entry["inputs"]["description_digest"] = "f" * 64
        entry["sha256"] = "1" * 64  # neither the inputs nor the expansion are locked
    monkeypatch.setattr(ex, "load_lock", lambda: lock)
    with pytest.raises(ex.NotLocked):
        ex.expand("bp000", platform_sw, openarm_description)


def test_engine_overrides_are_not_an_expansion_input(platform_sw, openarm_description):
    """The override file is read by the generators only. xacro never reads it, so it cannot change the locked expansion."""
    import os
    import pathlib
    import tempfile
    with tempfile.TemporaryDirectory() as tmp:
        prefix = pathlib.Path(tmp) / "p"
        ex._make_prefix(prefix, [ex.REPO / "ros2", platform_sw, openarm_description])
        env = dict(os.environ, AMENT_PREFIX_PATH=f"{prefix}:{os.environ.get('AMENT_PREFIX_PATH', '')}")
        deps = subprocess.run(["xacro", "--deps", str(ex.COMBINED), "hardware_profile:=gazebo"], env=env, capture_output=True,
                              text=True, check=True).stdout
    assert "engine_overrides" not in deps
    assert "openarm_v2.yaml" in deps



def test_changed_inputs_with_a_locked_expansion_are_labelled(platform_sw, openarm_description, monkeypatch):
    """Inputs not in the lock, expansion hash in the lock: allowed, but never labelled 'matches lock'."""
    lock = copy.deepcopy(ex.load_lock())
    for entry in lock["entries"]:
        entry["inputs"]["description_digest"] = "f" * 64
    monkeypatch.setattr(ex, "load_lock", lambda: lock)
    e = ex.expand("bp000", platform_sw, openarm_description)
    assert e.lock_status.startswith("hash in lock, inputs changed")
