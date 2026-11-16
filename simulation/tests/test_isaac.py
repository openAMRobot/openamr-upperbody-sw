# SPDX-License-Identifier: MIT
"""Isaac Sim: the import job from the pinned expansion (no GPU), and the USD checker on a small known USD.

The USD of the combined model needs Isaac Sim on a GPU. Without it the check is BLOCKED (exit 3), which is not a pass.
"""
import json
import math
import pathlib
import subprocess
import sys

import pytest

from openamr_sim import isaac

TOOLS = pathlib.Path(__file__).resolve().parents[1] / "tools"


@pytest.fixture(scope="module")
def job(expansions, tmp_path_factory):
    out = tmp_path_factory.mktemp("isaac")
    return out, isaac.build_job(expansions["bp000"], out)


def test_job_settings(job, overrides):
    out, j = job
    assert j["isaac_sim_version"] == "5.1.0" and j["container_image"].endswith(":5.1.0")
    imp = j["importer"]
    assert imp["merge_fixed_joints"] is False and imp["fix_base"] is False and imp["import_inertia_tensor"] is True
    assert imp["self_collision"] is False and imp["parse_mimic"] is True
    assert j["physics"]["time_steps_per_second"] == 1500


def test_job_urdf_is_the_expansion_with_absolute_mesh_paths(job, expansions):
    out, j = job
    text = (out / j["urdf"]).read_text()
    assert "package://" not in text
    import re
    normalised = re.sub(r'filename="[^"]*?/(meshes|assets)/', r'filename="X/\1/', text)
    reference = re.sub(r'filename="package://[^"]*?/(meshes|assets)/', r'filename="X/\1/', expansions["bp000"].text)
    assert normalised == reference
    for f in re.findall(r'<mesh filename="([^"]+)"', text):
        assert pathlib.Path(f).is_file(), f


def test_job_drives_in_usd_units(job, overrides):
    out, j = job
    d = j["drives"]["openarm_left_joint1"]
    kp = overrides["actuators"]["arm_position_gains"]["per_joint"]["joint1"]["kp"]
    assert d["type"] == "angular" and d["mode"] == "position"
    assert d["stiffness"] == pytest.approx(kp * math.pi / 180, rel=1e-12)  # USD angular drives are per degree
    assert d["max_force"] == 40.0
    assert j["drives"]["lift_joint"]["type"] == "linear"
    assert j["drives"]["left_wheel_joint"]["mode"] == "velocity" and j["drives"]["left_wheel_joint"]["stiffness"] == 0.0
    assert len(j["drives"]) == 19 and len(j["armature"]) == 18


def test_without_usd_the_check_is_blocked(job, platform_sw, openarm_description):
    out, j = job
    run = subprocess.run([sys.executable, str(TOOLS / "check_usd.py"), "--job", str(out / "isaac_job.json"),
                          "--platform-sw", str(platform_sw), "--openarm-description", str(openarm_description)],
                         capture_output=True, text=True)
    assert run.returncode == 3, run.stderr
    assert "BLOCKED" in run.stderr


# ------------------------------------------------------------------------------------------------ checker self-test
MINI = """<robot name="mini">
  <link name="base"><inertial><origin xyz="0 0 0.1"/><mass value="2.0"/><inertia ixx="0.02" ixy="0" ixz="0" iyy="0.03" iyz="0" izz="0.04"/></inertial></link>
  <link name="arm"><inertial><origin xyz="0.1 0 0"/><mass value="0.5"/><inertia ixx="0.001" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.003"/></inertial></link>
  <link name="fa"><inertial><mass value="0.1"/><inertia ixx="1e-5" ixy="0" ixz="0" iyy="1e-5" iyz="0" izz="1e-5"/></inertial></link>
  <link name="fb"><inertial><mass value="0.1"/><inertia ixx="1e-5" ixy="0" ixz="0" iyy="1e-5" iyz="0" izz="1e-5"/></inertial></link>
  <joint name="j1" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="10" velocity="1"/></joint>
  <joint name="f1" type="revolute"><parent link="arm"/><child link="fa"/><origin xyz="0.2 0 0"/><axis xyz="1 0 0"/><limit lower="0" upper="0.7" effort="2" velocity="1"/></joint>
  <joint name="f2" type="revolute"><parent link="arm"/><child link="fb"/><origin xyz="0.2 0.05 0"/><axis xyz="1 0 0"/><limit lower="0" upper="0.7" effort="2" velocity="1"/><mimic joint="f1" multiplier="1" offset="0"/></joint>
</robot>"""

MINI_JOB = {"drives": {"j1": {"type": "angular", "stiffness": 70 * math.pi / 180, "damping": 2.75 * math.pi / 180, "max_force": 10.0},
                       "f1": {"type": "angular", "stiffness": 5 * math.pi / 180, "damping": 0.1 * math.pi / 180, "max_force": 2.0}},
            "armature": {"j1": 0.016, "f1": 0.002, "f2": 0.002},
            "mimic": {"f2": {"joint": "f1", "multiplier": 1.0, "offset": 0.0}},
            "physics": {"time_steps_per_second": 1500}}


def write_mini_usd(path, with_mimic=True, stiffness_scale=1.0, mass_scale=1.0):
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
    stage = Usd.Stage.CreateNew(str(path))
    root = UsdGeom.Xform.Define(stage, "/mini")
    stage.SetDefaultPrim(root.GetPrim())
    scene = UsdPhysics.Scene.Define(stage, "/physicsScene")
    scene.GetPrim().CreateAttribute("physxScene:timeStepsPerSecond", Sdf.ValueTypeNames.Int).Set(1500)
    poses = {"base": (0, 0, 0), "arm": (0, 0, 0.5), "fa": (0.2, 0, 0.5), "fb": (0.2, 0.05, 0.5)}
    masses = {"base": (2.0, (0, 0, 0.1), (0.02, 0.03, 0.04)), "arm": (0.5, (0.1, 0, 0), (0.001, 0.002, 0.003)),
              "fa": (0.1, (0, 0, 0), (1e-5, 1e-5, 1e-5)), "fb": (0.1, (0, 0, 0), (1e-5, 1e-5, 1e-5))}
    for name, xyz in poses.items():
        x = UsdGeom.Xform.Define(stage, f"/mini/{name}")
        x.AddTranslateOp().Set(Gf.Vec3d(*xyz))
        UsdPhysics.RigidBodyAPI.Apply(x.GetPrim())
        m = UsdPhysics.MassAPI.Apply(x.GetPrim())
        mass, com, diag = masses[name]
        m.CreateMassAttr(mass * mass_scale)
        m.CreateCenterOfMassAttr(Gf.Vec3f(*com))
        m.CreateDiagonalInertiaAttr(Gf.Vec3f(*diag))
        m.CreatePrincipalAxesAttr(Gf.Quatf(1, 0, 0, 0))
    for name, parent, child, lo, hi in (("j1", "base", "arm", -1, 1), ("f1", "arm", "fa", 0, 0.7), ("f2", "arm", "fb", 0, 0.7)):
        j = UsdPhysics.RevoluteJoint.Define(stage, f"/mini/joints/{name}")
        j.CreateBody0Rel().SetTargets([f"/mini/{parent}"])
        j.CreateBody1Rel().SetTargets([f"/mini/{child}"])
        j.CreateLowerLimitAttr(math.degrees(lo))
        j.CreateUpperLimitAttr(math.degrees(hi))
        j.GetPrim().CreateAttribute("physxJoint:armature", Sdf.ValueTypeNames.Float).Set(MINI_JOB["armature"][name])
        if name in MINI_JOB["drives"]:
            spec = MINI_JOB["drives"][name]
            dr = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
            dr.CreateStiffnessAttr(spec["stiffness"] * stiffness_scale)
            dr.CreateDampingAttr(spec["damping"])
            dr.CreateMaxForceAttr(spec["max_force"])
        if name == "f2" and with_mimic:
            p = j.GetPrim()
            p.AddAppliedSchema("PhysxMimicJointAPI:rotX")
            p.CreateRelationship("physxMimicJoint:rotX:referenceJoint").SetTargets(["/mini/joints/f1"])
            p.CreateAttribute("physxMimicJoint:rotX:gearing", Sdf.ValueTypeNames.Float).Set(-1.0)
            p.CreateAttribute("physxMimicJoint:rotX:offset", Sdf.ValueTypeNames.Float).Set(0.0)
    stage.GetRootLayer().Save()
    return path


def test_usd_checker_passes_a_correct_usd(tmp_path):
    rep = isaac.check_usd(write_mini_usd(tmp_path / "ok.usda"), MINI, MINI_JOB)
    assert rep.checks > 30
    assert not rep.failures, rep.failures


@pytest.mark.parametrize("kwargs,parameter", [({"with_mimic": False}, "usd.mimic_api"),
                                              ({"stiffness_scale": 1.01}, "usd.drive_stiffness"),
                                              ({"mass_scale": 1.001}, "usd.mass")])
def test_usd_checker_finds_errors(tmp_path, kwargs, parameter):
    rep = isaac.check_usd(write_mini_usd(tmp_path / "bad.usda", **kwargs), MINI, MINI_JOB)
    assert any(f["parameter"] == parameter for f in rep.failures), rep.failures
