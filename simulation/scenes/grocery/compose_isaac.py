#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Compose the Isaac Sim grocery stage: the store from store.yaml and the robot USD of the #80 importer, by reference, unchanged.

Runs with usd-core (no Isaac Sim, no GPU):

  python3 simulation/scenes/grocery/compose_isaac.py --robot-usd build/sim/bp000/isaac/openamrobot_bp000.usd \
      --job build/sim/bp000/isaac/isaac_job.json --out build/grocery/isaac/grocery_store.usda

The robot USD is the output of simulation/tools/isaac_generate_usd.py (Isaac Sim 5.1 on a GPU). It is referenced, never copied or
edited; the composition only adds prims under the robot's link prims (overs in grocery_store.usda):
  * RTX camera prims (UsdGeom.Camera) under base_camera_link (Orbbec Gemini 336L, decisions.yaml CAMERAS), head_camera_link and the
    two wrist camera links, oriented so the USD camera axis (-Z forward, +Y up) is the link's x forward, z up;
  * an RTX LiDAR mount Xform under lidar_link (RPLIDAR S3). The OmniLidar prim itself is created on the GPU machine by
    isaac_render.py with IsaacSensorCreateRtxLidar, because its schema is not part of usd-core.
The store: static colliders (UsdPhysics.CollisionAPI) for shelves, fillers, packing table and tote; the tote on the pick shelf and
the target product are rigid bodies with mass. Units: metres (metersPerUnit 1), kilograms (kilogramsPerUnit 1), Z up.
The render step (isaac_render.py) needs Isaac Sim 5.1 on an NVIDIA GPU: BLOCKED without one. Not ROS or firmware integrated.
"""
import argparse
import json
import math
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import store as st  # noqa: E402

ROBOT_PATH = "/World/openamrobot"
STORE_PATH = "/World/store"

# The sensors placed on the robot: (link name, prim name, kind, horizontal FOV rad or None, note). The FOVs are those of the generated
# model's cameras (Gazebo values), not supplier datasheet values: see docs/demo/grocery_scene.md.
SENSORS = [
    ("base_camera_link", "base_camera_rtx", "camera", 1.047, "Orbbec Gemini 336L colour; FOV 1.047 rad is the Gazebo camera value"),
    ("head_camera_link", "head_camera_rtx", "camera", math.radians(45.0), "Stereolabs ZED Mini; FOV 45 deg as in the generated MJCF"),
    ("left_wrist_camera_link", "left_wrist_camera_rtx", "camera", math.radians(45.0), "wrist RGB; FOV 45 deg as in the generated MJCF"),
    ("right_wrist_camera_link", "right_wrist_camera_rtx", "camera", math.radians(45.0), "wrist RGB; FOV 45 deg as in the generated MJCF"),
    ("lidar_link", "rtx_lidar_mount", "lidar", None, "RPLIDAR S3; OmniLidar created by isaac_render.py at this mount"),
]
# Link frame (x forward, y left, z up) to the USD camera frame (-Z forward, +Y up, +X right): camera x = -link y, camera y = link z,
# camera z = -link x. As a quaternion (w, x, y, z) of the camera in the link frame:
CAMERA_IN_LINK = (0.5, 0.5, -0.5, -0.5)
APERTURE_MM = 20.955   # USD default horizontal aperture; the focal length sets the FOV


def focal_length(hfov):
    return APERTURE_MM / (2.0 * math.tan(hfov / 2.0))


def _box(stage, path, size, pos, yaw, rgba, collider=True):
    from pxr import Gf, UsdGeom, UsdPhysics
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xf = UsdGeom.XformCommonAPI(cube)
    xf.SetTranslate(Gf.Vec3d(*pos))
    xf.SetRotate(Gf.Vec3f(0.0, 0.0, math.degrees(yaw)))
    xf.SetScale(Gf.Vec3f(*size))
    cube.CreateDisplayColorAttr([Gf.Vec3f(*rgba[:3])])
    if collider:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return cube


def _cylinder(stage, path, radius, height, pos, rgba):
    from pxr import Gf, UsdGeom, UsdPhysics
    cyl = UsdGeom.Cylinder.Define(stage, path)
    cyl.CreateRadiusAttr(radius)
    cyl.CreateHeightAttr(height)
    cyl.CreateAxisAttr("Z")
    UsdGeom.XformCommonAPI(cyl).SetTranslate(Gf.Vec3d(*pos))
    cyl.CreateDisplayColorAttr([Gf.Vec3f(*rgba[:3])])
    UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())
    return cyl


def _element(stage, parent, e, collider=True):
    name = e["name"].replace("-", "_")
    if e["kind"] == "box":
        return _box(stage, f"{parent}/{name}", e["size"], e["pos"], e.get("yaw", 0.0), e["rgba"], collider)
    return _cylinder(stage, f"{parent}/{name}", e["radius"], e["height"], e["pos"], e["rgba"])


def _rigid(stage, path, pos, mass):
    from pxr import Gf, UsdGeom, UsdPhysics
    x = UsdGeom.Xform.Define(stage, path)
    UsdGeom.XformCommonAPI(x).SetTranslate(Gf.Vec3d(*pos))
    UsdPhysics.RigidBodyAPI.Apply(x.GetPrim())
    UsdPhysics.MassAPI.Apply(x.GetPrim()).CreateMassAttr(mass)
    return x


def find_link(stage, root, name):
    """The prim of robot link `name` below `root`: exactly one prim with that name, else an error."""
    hits = [p for p in stage.Traverse() if p.GetName() == name and str(p.GetPath()).startswith(root + "/")]
    if len(hits) != 1:
        raise LookupError(f"robot link {name}: {len(hits)} prims below {root}")
    return hits[0]


def compose(robot_usd, out_path, job=None, store_path=st.STORE, robot_pose=None):
    """Write the stage. Returns a summary dict. robot_usd may not exist yet (GPU step not run): the stage is then written with the
    unresolved reference and the sensor overs are skipped, and the summary says so."""
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics
    out_path = pathlib.Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    data = st.load(store_path)
    lay = st.layout(data)
    stage = Usd.Stage.CreateNew(str(out_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    summary = {"stage": str(out_path), "robot_usd": str(robot_usd), "sensors": [], "robot_resolved": False}

    # Physics scene: the robot USD of the importer carries one (isaac_generate_usd.py checks it). If the robot brings it, the stage
    # must not get a second one; that is checked after the reference is resolved.
    store = UsdGeom.Xform.Define(stage, STORE_PATH)
    _box(stage, f"{STORE_PATH}/floor", (14.0, 14.0, 0.02), (1.4, 0.0, -0.01), 0.0, (0.45, 0.46, 0.47), True)
    for group in ("shelves", "fillers", "packing"):
        UsdGeom.Scope.Define(stage, f"{STORE_PATH}/{group}")
        for e in lay[group]:
            _element(stage, f"{STORE_PATH}/{group}", e)
    UsdGeom.Scope.Define(stage, f"{STORE_PATH}/markings")
    for e in lay["markings"]:
        _element(stage, f"{STORE_PATH}/markings", e, collider=False)
    tote, product = lay["tote"], lay["product"]
    _rigid(stage, f"{STORE_PATH}/tote", tote["pos"], tote["mass"])
    for name, size, centre in st.tote_parts(tote["size"], tote["wall"]):
        _box(stage, f"{STORE_PATH}/tote/{name}", size, centre, 0.0, (0.12, 0.38, 0.72, 1))
    _rigid(stage, f"{STORE_PATH}/{product['name']}", product["pos"], product["mass"])
    _box(stage, f"{STORE_PATH}/{product['name']}/geom", product["size"], (0, 0, 0), 0.0, product["rgba"])
    for k, light in enumerate(lay["lights"]):
        lt = UsdLux.SphereLight.Define(stage, f"{STORE_PATH}/light{k}")
        lt.CreateIntensityAttr(30000.0)
        lt.CreateRadiusAttr(0.3)
        UsdGeom.XformCommonAPI(lt).SetTranslate(Gf.Vec3d(*light["pos"]))
    UsdLux.DomeLight.Define(stage, f"{STORE_PATH}/dome").CreateIntensityAttr(400.0)

    # The robot, by reference, at the pick station.
    x, y, yaw = robot_pose or data["robot"]["pick_station"]["pose"]
    robot = UsdGeom.Xform.Define(stage, ROBOT_PATH)
    robot.GetPrim().GetReferences().AddReference(str(pathlib.Path(robot_usd).resolve()) if pathlib.Path(robot_usd).exists()
                                                 else str(robot_usd))
    common = UsdGeom.XformCommonAPI(robot)
    common.SetTranslate(Gf.Vec3d(x, y, 0.0))
    common.SetRotate(Gf.Vec3f(0.0, 0.0, math.degrees(yaw)))
    if job is not None:
        job_data = json.loads(pathlib.Path(job).read_text())
        world.GetPrim().SetCustomDataByKey("openamrobot_expansion_sha256", job_data["expansion_sha256"])
    world.GetPrim().SetCustomDataByKey("composed_by", "simulation/scenes/grocery/compose_isaac.py")

    if pathlib.Path(robot_usd).exists():
        summary["robot_resolved"] = True
        for link, prim_name, kind, hfov, note in SENSORS:
            parent = find_link(stage, ROBOT_PATH, link)
            path = parent.GetPath().AppendChild(prim_name)
            if kind == "camera":
                cam = UsdGeom.Camera.Define(stage, path)
                cam.CreateHorizontalApertureAttr(APERTURE_MM)
                cam.CreateVerticalApertureAttr(APERTURE_MM * 9.0 / 16.0)
                cam.CreateFocalLengthAttr(focal_length(hfov))
                cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 50.0))
                prim = cam.GetPrim()
            else:
                prim = UsdGeom.Xform.Define(stage, path).GetPrim()
            xf = UsdGeom.Xformable(prim)
            xf.ClearXformOpOrder()
            xf.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0))
            if kind == "camera":
                xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(*CAMERA_IN_LINK))
            prim.SetCustomDataByKey("openamrobot_sensor_note", note)
            prim.SetCustomDataByKey("openamrobot_frame", link)
            summary["sensors"].append({"link": link, "prim": str(path), "kind": kind, "hfov_rad": hfov})
    stage.GetRootLayer().Save()
    summary["prims"] = sum(1 for _ in stage.Traverse())
    summary["physics_scenes"] = [str(p.GetPath()) for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
    summary["articulation_roots"] = [str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    if not summary["physics_scenes"]:
        # The robot reference brought none (the importer keeps it outside the default prim, or the robot is not resolved yet):
        # one scene here, with the job's step and gravity so the store and the robot step as isaac_generate_usd.py configured.
        scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
        if job is not None:
            physics = job_data["physics"]
            g = physics["gravity"]
            norm = math.sqrt(sum(v * v for v in g))
            scene.CreateGravityDirectionAttr(Gf.Vec3f(*(v / norm for v in g)))
            scene.CreateGravityMagnitudeAttr(norm)
            scene.GetPrim().AddAppliedSchema("PhysxSceneAPI")
            scene.GetPrim().CreateAttribute("physxScene:timeStepsPerSecond", Sdf.ValueTypeNames.UInt).Set(
                int(physics["time_steps_per_second"]))
        stage.GetRootLayer().Save()
        summary["physics_scenes"] = ["/World/physicsScene"]
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--robot-usd", required=True, help="isaac_generate_usd.py output (the #80 importer)")
    parser.add_argument("--job", help="isaac_job.json, for the expansion hash")
    parser.add_argument("--out", default="build/grocery/isaac/grocery_store.usda")
    opts = parser.parse_args()
    summary = compose(opts.robot_usd, opts.out, job=opts.job)
    print(json.dumps(summary))
    if not summary["robot_resolved"]:
        print(f"BLOCKED: robot USD {opts.robot_usd} does not exist (Isaac Sim import needs a GPU); store written, sensors not placed",
              file=sys.stderr)
        return 3
    return 0


if __name__ == "__main__":
    sys.exit(main())
