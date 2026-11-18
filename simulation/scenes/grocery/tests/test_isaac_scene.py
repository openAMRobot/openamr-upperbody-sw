# SPDX-License-Identifier: MIT
"""The Isaac Sim grocery stage, checked on the CPU with usd-core: schema, prim structure, units, articulation count and sensor frames.

The robot USD of the #80 importer needs Isaac Sim on a GPU and does not exist here. The sensor-placement tests therefore use a
KINEMATIC STAND-IN written by this test from the importer URDF of the job (one Xform per link at its URDF pose, flat under the robot
root, one articulation root, one physics scene). It is a test fixture: it checks where compose_isaac.py puts the sensors relative to
the link prims, not what Isaac Sim imports. The Isaac import and the render are BLOCKED without a GPU.
"""
import hashlib
import json
import math
import subprocess
import sys

import numpy as np
import pytest

import compose_isaac
import store as st


def _stage(path):
    from pxr import Usd
    return Usd.Stage.Open(str(path))


def test_store_stage_without_the_robot_is_written_and_reported_blocked(tmp_path):
    from pxr import UsdGeom, UsdPhysics
    out = tmp_path / "store.usda"
    run = subprocess.run([sys.executable, str(compose_isaac.HERE / "compose_isaac.py"), "--robot-usd", str(tmp_path / "none.usd"),
                          "--out", str(out)], capture_output=True, text=True)
    assert run.returncode == 3 and "BLOCKED" in run.stderr, run.stderr
    stage = _stage(out)
    assert UsdGeom.GetStageUpAxis(stage) == UsdGeom.Tokens.z
    assert UsdGeom.GetStageMetersPerUnit(stage) == 1.0 and UsdPhysics.GetStageKilogramsPerUnit(stage) == 1.0
    assert stage.GetDefaultPrim().GetPath() == "/World"
    lay = st.layout()
    colliders = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)]
    expected = 1 + len(lay["shelves"]) + len(lay["fillers"]) + len(lay["packing"]) + 5 + 1   # floor, store, tote parts, product
    assert len(colliders) == expected
    markings = stage.GetPrimAtPath(f"{compose_isaac.STORE_PATH}/markings")
    assert not any(c.HasAPI(UsdPhysics.CollisionAPI) for c in markings.GetChildren())
    bodies = {p.GetName(): p for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)}
    assert set(bodies) == {"tote", "target_product"}
    assert UsdPhysics.MassAPI(bodies["target_product"]).GetMassAttr().Get() == pytest.approx(0.15)
    assert UsdPhysics.MassAPI(bodies["tote"]).GetMassAttr().Get() == pytest.approx(0.6)
    product = stage.GetPrimAtPath(f"{compose_isaac.STORE_PATH}/target_product/geom")
    scale = product.GetAttribute("xformOp:scale").Get()
    assert tuple(scale) == pytest.approx((0.035, 0.035, 0.14))
    assert [str(p.GetPath()) for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)] == ["/World/physicsScene"]
    assert not [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]   # robot unresolved: none


# ------------------------------------------------------------------------------------------------ with the job's URDF
@pytest.fixture(scope="module")
def job(isaac_job):
    return json.loads(isaac_job.read_text())


@pytest.fixture(scope="module")
def urdf(isaac_job, job):
    from openamr_sim.urdf_model import URDF
    return URDF((isaac_job.parent / job["urdf"]).read_text())


def write_stand_in(path, urdf, root_name="openamrobot_bp000"):
    """Kinematic stand-in of the importer's robot USD (test fixture, not Isaac output)."""
    from pxr import Gf, Usd, UsdGeom, UsdPhysics
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, f"/{root_name}")
    stage.SetDefaultPrim(root.GetPrim())
    poses = urdf.fk({})
    for name in urdf.links:
        x = UsdGeom.Xform.Define(stage, f"/{root_name}/{name}")
        x.AddTransformOp().Set(Gf.Matrix4d(*np.asarray(poses[name]).T.flatten().tolist()))
        UsdPhysics.RigidBodyAPI.Apply(x.GetPrim())
    UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(f"/{root_name}/{urdf.root_link}"))
    UsdPhysics.Scene.Define(stage, "/physicsScene")       # outside the default prim, as a referenced robot does not bring it
    stage.GetRootLayer().Save()
    return path


@pytest.fixture(scope="module")
def composed_stage(urdf, isaac_job, tmp_path_factory):
    d = tmp_path_factory.mktemp("isaac")
    robot = write_stand_in(d / "openamrobot_bp000.usda", urdf)
    before = hashlib.sha256(robot.read_bytes()).hexdigest()
    summary = compose_isaac.compose(robot, d / "grocery_store.usda", job=isaac_job)
    return {"dir": d, "robot": robot, "robot_sha_before": before, "summary": summary, "stage": _stage(d / "grocery_store.usda")}


def test_robot_usd_is_referenced_unchanged(composed_stage):
    c = composed_stage
    assert hashlib.sha256(c["robot"].read_bytes()).hexdigest() == c["robot_sha_before"]
    stage = c["stage"]
    refs = stage.GetPrimAtPath(compose_isaac.ROBOT_PATH).GetMetadata("references")
    assert [str(r.assetPath) for r in refs.prependedItems] == [str(c["robot"].resolve())]
    # The stage's own layer has no property opinion on any robot link prim: below the robot, only the new sensor prims carry values.
    sensor_names = {s[1] for s in compose_isaac.SENSORS}
    below = [p for p in _paths(c["stage"].GetRootLayer()) if p.startswith(compose_isaac.ROBOT_PATH + "/") and "." in p]
    assert below
    for path in below:
        prim_path = path.split(".")[0]
        assert any(part in sensor_names for part in prim_path.split("/")), path


def _paths(layer):
    out = []
    layer.Traverse("/", lambda p: out.append(str(p)))
    return out


def test_one_articulation_one_physics_scene(composed_stage, job):
    from pxr import UsdPhysics
    stage = composed_stage["stage"]
    roots = [str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    assert roots == [f"{compose_isaac.ROBOT_PATH}/{job['articulation_root']}"]
    scenes = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
    assert len(scenes) == 1
    assert scenes[0].GetAttribute("physxScene:timeStepsPerSecond").Get() == job["physics"]["time_steps_per_second"] == 1500
    assert UsdPhysics.Scene(scenes[0]).GetGravityMagnitudeAttr().Get() == pytest.approx(9.81)


def _world(stage, prim):
    from pxr import Usd, UsdGeom
    return np.array(UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(prim)).T


def test_sensors_sit_on_the_authoritative_frames(composed_stage, urdf, job):
    """Each camera: position at the link origin, USD -Z along the link x axis, +Y along link z. LiDAR mount: the lidar_link frame."""
    from pxr import UsdGeom
    stage = composed_stage["stage"]
    data = st.load()
    x, y, yaw = data["robot"]["pick_station"]["pose"]
    robot_T = np.eye(4)
    robot_T[:3, :3] = [[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]]
    robot_T[:3, 3] = (x, y, 0)
    fk = urdf.fk({})
    found = {s["link"]: s for s in composed_stage["summary"]["sensors"]}
    assert set(found) == {"base_camera_link", "head_camera_link", "left_wrist_camera_link", "right_wrist_camera_link", "lidar_link"}
    for link, prim_name, kind, hfov, _ in compose_isaac.SENSORS:
        prim = stage.GetPrimAtPath(found[link]["prim"])
        assert prim.IsValid() and prim.GetParent().GetName() == link and prim.GetName() == prim_name
        link_world = robot_T @ np.asarray(fk[link])
        T = _world(stage, prim)
        np.testing.assert_allclose(T[:3, 3], link_world[:3, 3], atol=1e-9)
        R = T[:3, :3]
        if kind == "camera":
            assert prim.IsA(UsdGeom.Camera)
            np.testing.assert_allclose(-R[:, 2], link_world[:3, 0], atol=1e-9)     # looks along the link's x
            np.testing.assert_allclose(R[:, 1], link_world[:3, 2], atol=1e-9)      # up is the link's z
            cam = UsdGeom.Camera(prim)
            got = 2 * math.atan(cam.GetHorizontalApertureAttr().Get() / (2 * cam.GetFocalLengthAttr().Get()))
            assert got == pytest.approx(hfov, rel=1e-6)
        else:
            np.testing.assert_allclose(R, link_world[:3, :3], atol=1e-9)


def test_base_camera_is_the_gemini_frame_not_the_legacy_camera_link(composed_stage, urdf):
    """audits PR 89 P12: the generated MJCF binds its base camera to the legacy camera_link. The Isaac stage uses base_camera_link:
    (0.387, 0, 0.046) from base_link, pitched 10 degrees up (decisions.yaml CAMERAS)."""
    base = next(s for s in composed_stage["summary"]["sensors"] if s["link"] == "base_camera_link")
    assert "/camera_link/" not in base["prim"]
    fk = urdf.fk({})
    rel = np.linalg.inv(fk["base_link"]) @ fk["base_camera_link"]
    np.testing.assert_allclose(rel[:3, 3], (0.387, 0.0, 0.046), atol=1e-9)
    pitch = math.atan2(-rel[2, 0], rel[0, 0])
    assert pitch == pytest.approx(-0.1745, abs=1e-6)                            # URDF pitch -0.1745 rad = 10 deg up


def test_stage_records_the_expansion(composed_stage, job):
    world = composed_stage["stage"].GetPrimAtPath("/World")
    assert world.GetCustomDataByKey("openamrobot_expansion_sha256") == job["expansion_sha256"]
