# SPDX-License-Identifier: MIT
"""The import-step corrections of tools/isaac_generate_usd.py, on a small USD written with usd-core (no Isaac Sim, no GPU).

Findings of the first GPU run of the kit (#107, d7945c7, Isaac Sim 5.1.0): the importer wrote no physics scene (the step then
stopped before saving: 57 of 60 check_usd failures), the version check read the Kit build 107.3.3, and the mimic finger limits came
out 9 degrees wider than the URDF.
"""
import ast
import copy
import math
import pathlib

import pytest

from openamr_sim import isaac, usd_fixups
from tests.test_isaac import MINI, MINI_JOB, write_mini_usd

pxr = pytest.importorskip("pxr", reason="usd-core is a test requirement of simulation/ (requirements); tracked in #107")
GENERATOR = pathlib.Path(__file__).resolve().parents[1] / "tools" / "isaac_generate_usd.py"
JOB = dict(copy.deepcopy(MINI_JOB), physics={"time_steps_per_second": 1500, "gravity": [0.0, 0.0, -9.81]})


def _as_importer_wrote_it(path, widen_deg=9.0):
    """The mini USD as Isaac Sim 5.1 wrote the robot: no physics scene, mimic follower limits wider than the URDF."""
    from pxr import Usd, UsdPhysics
    stage = Usd.Stage.Open(str(write_mini_usd(path)))
    stage.RemovePrim("/physicsScene")
    f2 = UsdPhysics.RevoluteJoint(stage.GetPrimAtPath("/mini/joints/f2"))
    f2.GetLowerLimitAttr().Set(f2.GetLowerLimitAttr().Get() - widen_deg)
    f2.GetUpperLimitAttr().Set(f2.GetUpperLimitAttr().Get() + widen_deg)
    return stage


def test_the_importer_output_fails_the_check_without_the_fixups(tmp_path):
    stage = _as_importer_wrote_it(tmp_path / "raw.usda")
    stage.Save()
    failed = {f["parameter"] for f in isaac.check_usd(tmp_path / "raw.usda", MINI, JOB).failures}
    assert {"usd.physics_scene", "usd.joint_limit"} <= failed


def test_the_fixups_make_the_importer_output_pass(tmp_path):
    from pxr import Sdf, UsdPhysics
    stage = _as_importer_wrote_it(tmp_path / "fixed.usda")
    prim, created = usd_fixups.ensure_physics_scene(stage, JOB)
    assert created and str(prim.GetPath()) == "/physicsScene"
    prim.CreateAttribute("physxScene:timeStepsPerSecond", Sdf.ValueTypeNames.Int).Set(1500)   # PhysxSceneAPI in the generator
    scene = UsdPhysics.Scene(prim)
    assert math.isclose(scene.GetGravityMagnitudeAttr().Get(), 9.81, rel_tol=1e-6)
    assert tuple(scene.GetGravityDirectionAttr().Get()) == (0.0, 0.0, -1.0)
    changes = usd_fixups.mimic_limits_from_urdf(stage, MINI, JOB)
    assert [c["joint"] for c in changes] == ["f2"]
    assert changes[0]["before"] == pytest.approx([-9.0, math.degrees(0.7) + 9.0], abs=1e-4)
    assert changes[0]["after"] == pytest.approx([0.0, math.degrees(0.7)], abs=1e-9)
    stage.Save()
    report = isaac.check_usd(tmp_path / "fixed.usda", MINI, JOB)
    assert not report.failures, report.failures


def test_an_existing_scene_is_kept_and_two_are_refused(tmp_path):
    from pxr import Usd, UsdPhysics
    stage = Usd.Stage.Open(str(write_mini_usd(tmp_path / "one.usda")))
    prim, created = usd_fixups.ensure_physics_scene(stage, JOB)
    assert not created and str(prim.GetPath()) == "/physicsScene"
    UsdPhysics.Scene.Define(stage, "/second")
    with pytest.raises(RuntimeError, match="at most one physics scene"):
        usd_fixups.ensure_physics_scene(stage, JOB)


def test_the_version_check_reads_the_isaac_sim_release_not_the_kit_build():
    assert usd_fixups.isaac_sim_version_matches("5.1.0", "5.1.0")
    assert usd_fixups.isaac_sim_version_matches("5.1.0", "5.1.0-rc.19+release.26219.9c81211b.gl")
    assert not usd_fixups.isaac_sim_version_matches("5.1.0", "107.3.3")     # the Kit build of Isaac Sim 5.1.0 (#107 GPU run)
    assert not usd_fixups.isaac_sim_version_matches("5.1.0", "5.1.01")
    assert not usd_fixups.isaac_sim_version_matches("5.1.0", None)


def test_the_generator_applies_the_fixups_before_it_saves():
    """Static: the generator no longer requires a physics scene from the importer, and runs both fixups before stage.Save()."""
    text = GENERATOR.read_text()
    ast.parse(text)
    assert "expected one physics scene" not in text
    save = text.index("stage.Save()")
    assert text.index("usd_fixups.ensure_physics_scene(") < save
    assert text.index("usd_fixups.mimic_limits_from_urdf(") < save
    assert "get_build_version()" in text and 'log["kit_build"]' in text
    assert "usd_fixups.isaac_sim_version_matches(" in text


def test_usd_fixups_imports_no_yaml():
    """Isaac Sim's python.sh loads this module; it must not need PyYAML (isaac.py and mjcf.py do)."""
    tree = ast.parse((pathlib.Path(usd_fixups.__file__)).read_text())
    imported = {a.name.split(".")[0] for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names}
    imported |= {n.module.split(".")[0] for n in ast.walk(tree) if isinstance(n, ast.ImportFrom) and n.module and n.level == 0}
    assert "yaml" not in imported
    model = (pathlib.Path(usd_fixups.__file__).parent / "urdf_model.py").read_text()
    assert "import yaml" not in model
