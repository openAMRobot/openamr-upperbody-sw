# SPDX-License-Identifier: MIT
"""The expansion each backend is generated from is the locked one (digital-twin lock, mirrored in simulation/expansion_lock.yaml)."""
import copy
import os
import subprocess
import sys

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


def test_installed_xacro_is_the_locked_build():
    """The xacro build is a lock input. The installed one must be the build the lock names; a superseded build is named as such."""
    locked = {e["inputs"]["xacro_package_version"] for e in ex.load_lock()["entries"]}
    assert len(locked) == 1, locked
    installed = ex.xacro_version()
    if installed == "unknown":
        pytest.fail("BLOCKED: no xacro on PATH whose version can be read; the installed xacro cannot be compared with the lock",
                    pytrace=False)
    superseded = {h["version"]: h["reason"] for h in ex.load_lock().get("xacro_superseded", [])}
    assert installed not in superseded, f"xacro {installed} is superseded in the lock: {superseded[installed]}"
    assert installed == locked.pop()


def _requirement_lines():
    lines = (ex.REPO / "simulation" / "requirements-xacro.txt").read_text().splitlines()
    return [line.split("#")[0].strip() for line in lines if line.split("#")[0].strip()]


def test_xacro_install_file_is_the_locked_wheel():
    """CI installs xacro from requirements-xacro.txt with --require-hashes. That file must name exactly the wheel the lock names."""
    pin = ex.load_lock()["xacro"]
    assert {e["inputs"]["xacro_package_version"] for e in ex.load_lock()["entries"]} == {pin["version"]}
    assert pin["version"] == "pypi:" + pin["requirement"]
    assert _requirement_lines() == [f"{pin['requirement']} --hash=sha256:{pin['wheel_sha256']}"]


def test_installed_xacro_has_the_pinned_module_files(evidence):
    """The module files the xacro on PATH imports must be those of the pinned wheel (sources_sha256 of the lock). This fails for
    another release, a locally changed module, or a ROS deb or other copy that shadows the pinned wheel with different files."""
    pin = ex.load_lock()["xacro"]
    found = ex.xacro_identity()
    if found["sources_sha256"] is None:
        pytest.fail("BLOCKED: no xacro on PATH whose module can be read", pytrace=False)
    evidence("xacro_installed", {**found, "pinned": pin})
    assert found["sources_sha256"] == pin["sources_sha256"], (
        f"xacro module files at {found['module']} ({found['version']}) differ from the pinned wheel {pin['file']}")


def test_xacro_identity_tells_apart_same_version_other_files(tmp_path, monkeypatch):
    """A copy that reports version 2.1.1 but carries other module files is detected by the module digest."""
    site = tmp_path / "site"
    (site / "xacro").mkdir(parents=True)
    (site / "xacro" / "__init__.py").write_text("def main():\n    return 0\n")
    (site / "xacro-2.1.1.dist-info").mkdir()
    (site / "xacro-2.1.1.dist-info" / "METADATA").write_text("Metadata-Version: 2.1\nName: xacro\nVersion: 2.1.1\n")
    (tmp_path / "bin").mkdir()
    script = tmp_path / "bin" / "xacro"
    script.write_text(f"#!{sys.executable}\nfrom xacro import main\nmain()\n")
    script.chmod(0o755)
    monkeypatch.setenv("PATH", f"{tmp_path / 'bin'}:{os.environ['PATH']}")
    monkeypatch.setenv("PYTHONPATH", str(site))
    found = ex.xacro_identity()
    assert found["module"] == str((site / "xacro").resolve())
    assert found["version"] == "pypi:xacro==2.1.1"
    assert found["sources_sha256"] != ex.load_lock()["xacro"]["sources_sha256"]


@pytest.mark.parametrize("base_plate", ["bp000", "bp050"])
@pytest.mark.parametrize("profile", ["mock", "real"])
def test_other_profiles_match_lock_with_the_installed_build(platform_sw, openarm_description, monkeypatch, base_plate, profile, evidence):
    """The lock also holds the mock and real expansions. With the installed xacro build they must still be the locked text."""
    monkeypatch.setitem(ex.CONFIGURATIONS, "profile_check", {"base_plate": base_plate, "hardware_profile": profile})
    e = ex.expand("profile_check", platform_sw, openarm_description)
    assert e.lock_status in ("matches lock", "hash in lock, inputs changed (description digest not locked; re-lock pending)")
    evidence(f"expansion_{base_plate}_{profile}", {"sha256": e.sha256, "inputs": e.inputs, "status": e.lock_status})
