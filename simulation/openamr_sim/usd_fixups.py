# SPDX-License-Identifier: MIT
"""Corrections the Isaac Sim import step applies to the imported USD, from the job and the URDF only.

Imported by tools/isaac_generate_usd.py inside Isaac Sim's python.sh, so it needs only the standard library, numpy (urdf_model)
and pxr; no PyYAML. Tested with usd-core in tests/test_usd_fixups.py (no GPU).
"""
import math

from .urdf_model import URDF


def isaac_sim_version_matches(pinned, found):
    """True when the Isaac Sim version found (for example "5.1.0" or "5.1.0-rc.10+...") is the pinned one. The Kit build
    (107.3.3 in Isaac Sim 5.1.0) is a different number and never matches."""
    if not found:
        return False
    return found == pinned or found.startswith(pinned + "-") or found.startswith(pinned + "+")


def ensure_physics_scene(stage, job, path="/physicsScene"):
    """The one UsdPhysics.Scene of the stage, gravity from the job. Isaac Sim 5.1's URDF importer writes none (GPU run of
    d7945c7, #107), so one is defined at `path`. Returns (prim, created). Two or more scenes is an error."""
    from pxr import Gf, UsdPhysics

    scenes = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
    if len(scenes) > 1:
        raise RuntimeError(f"expected at most one physics scene, found {[str(p.GetPath()) for p in scenes]}")
    created = not scenes
    scene = UsdPhysics.Scene.Define(stage, path) if created else UsdPhysics.Scene(scenes[0])
    g = [float(x) for x in job["physics"]["gravity"]]
    magnitude = math.sqrt(sum(x * x for x in g))
    scene.CreateGravityMagnitudeAttr(magnitude)
    if magnitude > 0.0:
        scene.CreateGravityDirectionAttr(Gf.Vec3f(*(x / magnitude for x in g)))
    return scene.GetPrim(), created


def mimic_limits_from_urdf(stage, urdf_text, job):
    """Set the joint limits of every mimic follower in job["mimic"] to the URDF's (degrees for revolute, metres for prismatic).
    Isaac Sim 5.1 wrote the finger follower limits 9 degrees wider than the URDF (GPU run of d7945c7, #107). Returns the
    changes, [{"joint", "before", "after"}], for the import log."""
    from pxr import UsdPhysics

    u = URDF(urdf_text)
    changes = []
    for name in job["mimic"]:
        j = u.joints[name]
        hits = [p for p in stage.Traverse() if p.GetName() == name and p.IsA(UsdPhysics.Joint)]
        if len(hits) != 1:
            raise RuntimeError(f"mimic follower {name}: expected one joint prim, found {[str(p.GetPath()) for p in hits]}")
        prim = hits[0]
        if j.type == "revolute":
            api, scale = UsdPhysics.RevoluteJoint(prim), 180.0 / math.pi
        elif j.type == "prismatic":
            api, scale = UsdPhysics.PrismaticJoint(prim), 1.0
        else:
            raise RuntimeError(f"mimic follower {name} is {j.type}; only revolute and prismatic have limits")
        before = [api.GetLowerLimitAttr().Get(), api.GetUpperLimitAttr().Get()]
        after = [j.lower * scale, j.upper * scale]
        api.CreateLowerLimitAttr(after[0])
        api.CreateUpperLimitAttr(after[1])
        changes.append({"joint": name, "before": before, "after": after})
    return changes
