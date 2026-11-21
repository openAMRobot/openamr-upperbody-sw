# SPDX-License-Identifier: MIT
"""Isaac Sim representation: the import job, built from the same expansion as MuJoCo, and the USD parity check.

Three steps, two of which run anywhere:
  1. build_job (here, no GPU): the URDF for the importer (the pinned expansion with package:// mesh paths made absolute, nothing else
     changed) and isaac_job.json (importer settings, drives, armature, physics scene, mimic), all from engine_overrides.yaml.
  2. tools/isaac_generate_usd.py (inside Isaac Sim, needs a GPU): imports the URDF with isaacsim.asset.importer.urdf and the job's
     settings, applies the drives, writes the USD.
  3. check_usd (here, with usd-core): reads the USD and compares it with the expansion. Without a USD the result is BLOCKED.
"""
import json
import math
import pathlib
import re

import numpy as np

from .benchmark import diffdrive_params
from .mjcf import OVERRIDES, PROFILE, load_yaml, subtree_mass
from .parity import Report, rotation_angle, tolerances
from .urdf_model import URDF

USD_ANGULAR_PER_DEGREE = math.pi / 180.0

# Isaac Sim's URDF converter drops a root link that has no inertial and no geometry, and the bodies under it then come out as
# separate articulations fixed to the world (found by the OpenRAL import of this model, OpenRAL PR 497, robots/openamrobot_v2).
# In this model base_link and lift_column_base_link are both fixed children of base_footprint, so the split is the chassis in one
# articulation and the lift and arms in another. The source model keeps its massless REP 105 root and its height datum; only the
# importer's copy gets this declared placeholder (0.1 % of the robot's mass), recorded in the job.
ROOT_PLACEHOLDER = {"mass": 0.1, "inertia": 1e-4,
                    "source": "import-step deviation: Isaac Sim URDF converter drops an empty massless root (OpenRAL PR 497 finding)"}


def articulation_root(u):
    """The articulation root body: the URDF root. The upper body hangs from base_footprint beside base_link (heights from the floor,
    height datum decision), so only the root links the chassis and the lift and arms into one tree."""
    return u.root_link


def _root_placeholder(text, root):
    """The URDF text with a placeholder inertial in the massless root link. Raises if the root already has one."""
    m = re.search(rf'<link name="{re.escape(root)}"\s*/>|<link name="{re.escape(root)}"\s*>', text)
    if m is None:
        raise ValueError(f"root link {root} not found")
    i = ROOT_PLACEHOLDER["inertia"]
    inertial = (f'<link name="{root}"><inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="{ROOT_PLACEHOLDER["mass"]}"/>'
                f'<inertia ixx="{i}" ixy="0" ixz="0" iyy="{i}" iyz="0" izz="{i}"/></inertial>')
    if m.group(0).endswith("/>"):
        inertial += "</link>"
    return text[:m.start()] + inertial + text[m.end():]


def _absolute_meshes(text, package_dirs):
    def repl(match):
        pkg, rel = match.group(1), match.group(2)
        return f'filename="{pathlib.Path(package_dirs[pkg]) / rel}"'
    return re.sub(r'filename="package://([^/]+)/([^"]+)"', repl, text)


def build_job(expansion, out_dir, overrides=OVERRIDES, profile=PROFILE):
    out_dir = pathlib.Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    ov = load_yaml(overrides)
    prof = load_yaml(profile)
    u = URDF(expansion.text)
    urdf_path = out_dir / f"openamrobot_{expansion.name}_isaac.urdf"
    text = _absolute_meshes(expansion.text, expansion.package_dirs)
    deviations = []
    if u.links[u.root_link].inertial is None:
        text = _root_placeholder(text, u.root_link)
        deviations.append({"link": u.root_link, "added_inertial_kg": ROOT_PLACEHOLDER["mass"], "reason": ROOT_PLACEHOLDER["source"]})
    urdf_path.write_text(text)
    act = ov["actuators"]
    drives = {}
    for side in ("left", "right"):
        for k in range(1, 8):
            g = act["arm_position_gains"]["per_joint"][f"joint{k}"]
            drives[f"openarm_{side}_joint{k}"] = ("angular", "position", g["kp"], g["kd"])
        g = act["gripper_position_gains"]
        drives[f"openarm_{side}_finger_joint1"] = ("angular", "position", g["kp"], g["kd"])
    lift = u.joints["lift_joint"]
    m = subtree_mass(u, lift.child)
    par = act["lift_position_servo"]["parameters"]
    kp = m * 9.81 / par["sag"]["value"]
    drives["lift_joint"] = ("linear", "position", kp, 2 * par["zeta"]["value"] * math.sqrt(kp * m))
    M = u.total_mass()
    for jn in act["wheel_velocity_servo"]["applies_to"]:
        r = u.links[u.joints[jn].child].collisions[0].params["radius"]
        drives[jn] = ("angular", "velocity", 0.0, (M / 2) * r * r / act["wheel_velocity_servo"]["parameters"]["tau"]["value"])
    job_drives = {}
    for jn, (kind, mode, kp, kd) in drives.items():
        j = u.joints[jn]
        scale = USD_ANGULAR_PER_DEGREE if kind == "angular" else 1.0
        job_drives[jn] = {"type": kind, "mode": mode, "stiffness": kp * scale, "damping": kd * scale,
                          "max_force": j.effort if j.effort is not None else None, "si": {"kp": kp, "kd": kd}}
    armature = {jn: prof["joints"][jn]["armature"]["value"] for jn in prof["joints"]}
    mimic = {jn: u.joints[jn].mimic for jn in ov["mimic"]["joints"]}
    job = {
        "schema_version": 1,
        "configuration": expansion.name, "expansion_sha256": expansion.sha256, "lock_status": expansion.lock_status,
        "urdf": urdf_path.name, "usd": f"openamrobot_{expansion.name}.usd",
        "isaac_sim_version": ov["isaac"]["version"]["value"], "container_image": ov["isaac"]["container_image"],
        "importer": ov["isaac"]["importer"],
        "physics": {"time_steps_per_second": round(1.0 / ov["common"]["physics_step"]["value"]),
                    "gravity": ov["common"]["gravity"]["value"]},
        "articulation_root": articulation_root(u), "importer_deviations": deviations,
        "drives": job_drives, "armature": armature, "mimic": mimic, "castors": ov["isaac"]["drives"]["castors"],
        # for the benchmark runner (simulation/isaac/isaac_scenarios.py): the DiffDrive values Gazebo runs with, from the expansion
        "diffdrive": diffdrive_params(expansion.text), "lift_velocity_limit": lift.velocity,
        "unsupported": [
            {"item": "joint friction", "handling": "PhysX joint friction is a unitless coefficient; URDF friction torque not applied"},
            {"item": "passive castors", "handling": "castor joints stay fixed as in the URDF"},
            {"item": "gazebo plugins and sensors", "handling": "not imported"},
            {"item": "collision meshes", "handling": "one convex hull per mesh (convex_decomp false)"},
            {"item": "wheel torque limit", "handling": "unknown; no maxForce on the wheel drives"},
        ],
    }
    (out_dir / "isaac_job.json").write_text(json.dumps(job, indent=1) + "\n")
    return job


# ---------------------------------------------------------------------------------------------------------------- USD check
def applied_schemas(prim):
    """Applied API schemas, including those whose plugin is not registered (usd-core does not register PhysxSchema)."""
    names = list(prim.GetAppliedSchemas())
    listop = prim.GetMetadata("apiSchemas")
    if listop is not None:
        for item in list(listop.GetAddedOrExplicitItems()) + list(listop.prependedItems) + list(listop.appendedItems):
            if item not in names:
                names.append(item)
    return names


def _find(stage, name, predicate):
    hits = [p for p in stage.Traverse() if p.GetName() == name and predicate(p)]
    return hits[0] if len(hits) == 1 else (None if not hits else hits)


def check_usd(usd_path, urdf_text, job):
    """Structural parity of the imported USD with the expansion. Returns a Report."""
    from pxr import Usd, UsdGeom, UsdPhysics

    tol = tolerances()
    r = Report()
    u = URDF(urdf_text)
    stage = Usd.Stage.Open(str(usd_path))
    is_body = lambda p: p.HasAPI(UsdPhysics.RigidBodyAPI) or p.HasAPI(UsdPhysics.MassAPI) or p.IsA(UsdGeom.Xform)
    is_joint = lambda p: p.IsA(UsdPhysics.Joint)
    xcache = UsdGeom.XformCache(Usd.TimeCode.Default())
    poses = u.fk({})
    root = _find(stage, u.root_link, is_body)
    root_world = np.array(xcache.GetLocalToWorldTransform(root)).T if root is not None and not isinstance(root, list) else np.eye(4)
    for name, link in u.links.items():
        prim = _find(stage, name, is_body)
        r.check("usd.link", name, prim is not None and not isinstance(prim, list), name, None if prim is None else str(prim))
        if prim is None or isinstance(prim, list):
            continue
        # Gf matrices are row-major with row vectors; the transpose is the usual column-vector form.
        M = np.linalg.inv(root_world) @ np.array(xcache.GetLocalToWorldTransform(prim)).T
        r.near("usd.frame_position", name, poses[name][:3, 3], M[:3, 3], 1e-5)
        Rm = M[:3, :3] / np.linalg.norm(M[:3, :3], axis=0)
        err = rotation_angle(poses[name][:3, :3], Rm)
        r.check("usd.frame_rotation", name, err <= 1e-5, None, None, err)
        if link.inertial is not None:
            mass_api = UsdPhysics.MassAPI(prim)
            mass = mass_api.GetMassAttr().Get() if prim.HasAPI(UsdPhysics.MassAPI) else None
            r.near("usd.mass", name, link.inertial.mass, mass if mass is not None else float("nan"), max(tol["mass"]["abs"], 1e-6 * link.inertial.mass))
            com = mass_api.GetCenterOfMassAttr().Get() if prim.HasAPI(UsdPhysics.MassAPI) else None
            if com is not None:
                r.near("usd.com", name, link.inertial.origin[:3, 3], list(com), 1e-6)
            diag = mass_api.GetDiagonalInertiaAttr().Get() if prim.HasAPI(UsdPhysics.MassAPI) else None
            axes = mass_api.GetPrincipalAxesAttr().Get() if prim.HasAPI(UsdPhysics.MassAPI) else None
            if diag is not None and axes is not None:
                q = axes
                w, (x, y, z) = q.GetReal(), q.GetImaginary()
                Rq = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                               [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                               [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
                Igot = Rq @ np.diag(list(diag)) @ Rq.T
                R = link.inertial.origin[:3, :3]
                Iref = R @ link.inertial.inertia @ R.T
                rel = float(np.linalg.norm(Igot - Iref) / max(np.linalg.norm(Iref), 1e-300))
                r.check("usd.inertia", name, rel <= 1e-5, None, None, rel)
    kinds = {"revolute": UsdPhysics.RevoluteJoint, "continuous": UsdPhysics.RevoluteJoint, "prismatic": UsdPhysics.PrismaticJoint,
             "fixed": UsdPhysics.FixedJoint}
    for j in u.joints.values():
        prim = _find(stage, j.name, is_joint)
        r.check("usd.joint", j.name, prim is not None and not isinstance(prim, list), j.name, None if prim is None else str(prim))
        if prim is None or isinstance(prim, list):
            continue
        r.check("usd.joint_type", j.name, prim.IsA(kinds[j.type]), j.type, str(prim.GetTypeName()))
        jo = UsdPhysics.Joint(prim)
        b0 = [t.name for t in jo.GetBody0Rel().GetTargets()]
        b1 = [t.name for t in jo.GetBody1Rel().GetTargets()]
        r.check("usd.joint_bodies", j.name, b0 == [j.parent] and b1 == [j.child], [j.parent, j.child], [b0, b1])
        if j.type in ("revolute", "prismatic"):
            api = UsdPhysics.RevoluteJoint(prim) if j.type == "revolute" else UsdPhysics.PrismaticJoint(prim)
            scale = 180.0 / math.pi if j.type == "revolute" else 1.0
            lo, hi = api.GetLowerLimitAttr().Get(), api.GetUpperLimitAttr().Get()
            r.near("usd.joint_limit", j.name, [j.lower * scale, j.upper * scale], [lo, hi], 1e-4 * scale)
        if j.name in job["drives"]:
            spec = job["drives"][j.name]
            drive = UsdPhysics.DriveAPI(prim, spec["type"])
            ok = prim.HasAPI(UsdPhysics.DriveAPI, spec["type"])
            r.check("usd.drive_present", j.name, ok, spec["type"], ok)
            if ok:
                r.near("usd.drive_stiffness", j.name, spec["stiffness"], drive.GetStiffnessAttr().Get(), 1e-5 * max(1.0, spec["stiffness"]))
                r.near("usd.drive_damping", j.name, spec["damping"], drive.GetDampingAttr().Get(), 1e-5 * max(1.0, spec["damping"]))
                if spec["max_force"] is not None:
                    r.near("usd.drive_max_force", j.name, spec["max_force"], drive.GetMaxForceAttr().Get(), 1e-4)
        if j.name in job["armature"]:
            attr = prim.GetAttribute("physxJoint:armature")
            r.near("usd.armature", j.name, job["armature"][j.name], attr.Get() if attr and attr.HasValue() else float("nan"), 1e-6)
        if j.name in job["mimic"]:
            schemas = applied_schemas(prim)
            has = any("PhysxMimicJointAPI" in s for s in schemas)
            r.check("usd.mimic_api", j.name, has, "PhysxMimicJointAPI", schemas)
            if has:
                inst = [s.split(":", 1)[1] for s in schemas if s.startswith("PhysxMimicJointAPI:")]
                if inst:
                    ref = prim.GetRelationship(f"physxMimicJoint:{inst[0]}:referenceJoint").GetTargets()
                    r.check("usd.mimic_reference", j.name, [t.name for t in ref] == [job["mimic"][j.name]["joint"]],
                            job["mimic"][j.name]["joint"], [t.name for t in ref])
                    gearing = prim.GetAttribute(f"physxMimicJoint:{inst[0]}:gearing").Get()
                    r.near("usd.mimic_gearing", j.name, -job["mimic"][j.name]["multiplier"], gearing if gearing is not None else float("nan"), 1e-6)
    roots = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    r.check("usd.articulation_count", "robot", len(roots) == 1, 1, [str(p.GetPath()) for p in roots])
    if len(roots) == 1:
        # Every moving joint must connect bodies reachable from the articulation root through joints, never through the world.
        bodies = {p.GetName() for p in stage.Traverse() if is_body(p)}
        edges = {}
        for p in stage.Traverse():
            if is_joint(p):
                jo = UsdPhysics.Joint(p)
                b0 = [t.name for t in jo.GetBody0Rel().GetTargets()]
                b1 = [t.name for t in jo.GetBody1Rel().GetTargets()]
                if b0 and b1:
                    edges.setdefault(b0[0], set()).add(b1[0])
                    edges.setdefault(b1[0], set()).add(b0[0])
        start = roots[0].GetName()
        if start not in bodies:  # root API on an Xform above the bodies: start from the job's articulation body
            start = job.get("articulation_root", start)
        seen, stack = {start}, [start]
        while stack:
            for nxt in edges.get(stack.pop(), ()):
                if nxt not in seen:
                    seen.add(nxt)
                    stack.append(nxt)
        outside = sorted(j.name for j in u.joints.values() if j.type != "fixed" and not {j.parent, j.child} <= seen)
        r.check("usd.articulation_contains_moving_joints", "robot", not outside, [], outside)
    scenes = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
    r.check("usd.physics_scene", "scene", len(scenes) == 1, 1, len(scenes))
    if len(scenes) == 1:
        steps = scenes[0].GetAttribute("physxScene:timeStepsPerSecond").Get()
        r.check("usd.time_steps", "scene", steps == job["physics"]["time_steps_per_second"], job["physics"]["time_steps_per_second"], steps)
    return r
