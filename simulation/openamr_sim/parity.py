# SPDX-License-Identifier: MIT
"""Structural parity of a backend model with the pinned expansion.

The reference is always the URDF text the lock hashes (urdf_model.URDF). Each check compares one parameter with the tolerance of
parity_tolerances.yaml and records a result. A Report counts checks and keeps every failure with its values, so a test can assert
zero failures and the evidence can quote counts.

Declared differences, the only ones allowed:
  - MuJoCo has a free joint at the root body (base_free_joint); the URDF root has no joint.
  - The castor joints of engine_overrides.yaml passive_castors are fixed in the URDF and hinges in MuJoCo.
Everything else must match.
"""
import dataclasses
import math
import pathlib

import numpy as np
import yaml

from .mjcf import ARM_POSES, FREE_JOINT, LIFT_STOPS, arm_pose, lift_stop_positions, load_yaml, srdf_states
from .urdf_model import URDF, rpy_matrix

REPO = pathlib.Path(__file__).resolve().parents[2]
TOLERANCES = REPO / "simulation" / "parity_tolerances.yaml"
OPTICAL_TO_MUJOCO_CAMERA = rpy_matrix((math.pi, 0.0, 0.0))


def tolerances():
    return yaml.safe_load(TOLERANCES.read_text())["parameters"]


def rotation_angle(Ra, Rb):
    c = (np.trace(Ra.T @ Rb) - 1) / 2
    return math.acos(max(-1.0, min(1.0, c)))


def quat_to_mat(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


@dataclasses.dataclass
class Report:
    checks: int = 0
    failures: list = dataclasses.field(default_factory=list)
    by_parameter: dict = dataclasses.field(default_factory=dict)

    def check(self, parameter, where, ok, reference=None, backend=None, error=None):
        self.checks += 1
        stats = self.by_parameter.setdefault(parameter, {"checks": 0, "failures": 0, "max_error": 0.0})
        stats["checks"] += 1
        if error is not None and np.isfinite(error):
            stats["max_error"] = max(stats["max_error"], float(error))
        if not ok:
            stats["failures"] += 1
            self.failures.append({"parameter": parameter, "where": where, "reference": reference, "backend": backend, "error": error})

    def near(self, parameter, where, reference, backend, abs_tol, rel_tol=0.0):
        ref, got = np.asarray(reference, float), np.asarray(backend, float)
        err = float(np.max(np.abs(ref - got))) if ref.size else 0.0
        ok = ref.shape == got.shape and err <= abs_tol + rel_tol * float(np.max(np.abs(ref)) if ref.size else 0.0)
        self.check(parameter, where, ok, ref.tolist(), got.tolist(), err)

    def summary(self):
        return {"checks": self.checks, "failures": len(self.failures), "by_parameter": self.by_parameter}


def named_configurations(urdf):
    """Joint positions for every lift stop and SRDF arm pose, grippers at the SRDF open state."""
    states = srdf_states()
    stops = lift_stop_positions()
    grip = {}
    for side in ("left", "right"):
        grip.update(states[("open", f"{side}_gripper")])
    out = {}
    for stop in LIFT_STOPS:
        for pose in ARM_POSES:
            out[f"{stop}_{pose}"] = urdf.resolve({"lift_joint": stops[stop], **arm_pose(states, pose), **grip})
    return out


class MujocoParity:
    """Compare a compiled MuJoCo model with the expansion it was generated from."""

    def __init__(self, model, urdf_text, overrides_path=None, profile_path=None):
        import mujoco

        from .mjcf import OVERRIDES, PROFILE

        self.mj = mujoco
        self.m = model
        self.d = mujoco.MjData(model)
        self.u = URDF(urdf_text)
        self.tol = tolerances()
        self.ov = load_yaml(overrides_path or OVERRIDES)
        self.profile = load_yaml(profile_path or PROFILE)
        pc = self.ov["passive_castors"]
        self.castors = {}
        for p in pc["prefixes"]:
            self.castors[pc["swivel"]["joint"].format(prefix=p)] = pc["swivel"]
            self.castors[pc["roll"]["joint"].format(prefix=p)] = pc["roll"]
        self.report = Report()

    # --- helpers
    def set_q(self, q):
        mj, m, d = self.mj, self.m, self.d
        d.qpos[:] = m.qpos0
        adr = m.jnt_qposadr[m.joint(FREE_JOINT).id]
        d.qpos[adr:adr + 7] = [0, 0, 0, 1, 0, 0, 0]
        for name, v in q.items():
            j = self.u.joints[name]
            if j.type != "fixed":
                d.qpos[m.jnt_qposadr[m.joint(name).id]] = v
        mj.mj_kinematics(m, d)
        mj.mj_comPos(m, d)
        mj.mj_camlight(m, d)

    def _body_world(self, name):
        b = self.d.body(name)
        T = np.eye(4)
        T[:3, :3] = b.xmat.reshape(3, 3)
        T[:3, 3] = b.xpos
        return T

    # --- checks
    def topology(self):
        r, m, u = self.report, self.m, self.u
        bodies = {m.body(i).name for i in range(1, m.nbody)}
        r.check("topology.links", "body set", bodies == set(u.links), sorted(set(u.links) - bodies), sorted(bodies - set(u.links)))
        for name in u.links:
            if name not in bodies:
                continue
            parent = m.body(int(m.body_parentid[m.body(name).id])).name
            ref = u.child_joint[name].parent if name in u.child_joint else "world"
            r.check("topology.parent", name, parent == ref, ref, parent)
        expected = {j.name for j in u.movable()} | set(self.castors) | {FREE_JOINT}
        joints = {m.joint(i).name for i in range(m.njnt)}
        r.check("topology.joints", "joint set", joints == expected, sorted(expected - joints), sorted(joints - expected))
        mjtype = {0: "free", 1: "ball", 2: "slide", 3: "hinge"}
        for name in expected & joints:
            jid = m.joint(name).id
            got = mjtype[int(m.jnt_type[jid])]
            if name == FREE_JOINT:
                ref = "free"
            elif name in self.castors:
                ref = "hinge"
            else:
                ref = {"revolute": "hinge", "continuous": "hinge", "prismatic": "slide"}[u.joints[name].type]
            r.check("topology.joint_type", name, got == ref, ref, got)
            if name != FREE_JOINT:
                body = m.body(m.jnt_bodyid[jid]).name
                r.check("topology.joint_child", name, body == u.joints[name].child, u.joints[name].child, body)
        for name in self.castors:
            r.check("topology.declared_difference", name, u.joints[name].type == "fixed", "fixed in URDF", u.joints[name].type)

    def limits(self):
        r, m, u, t = self.report, self.m, self.u, self.tol
        for j in u.movable():
            jid = m.joint(j.name).id
            if j.type == "continuous":
                r.check("limits.position", j.name, not bool(m.jnt_limited[jid]), "unlimited", bool(m.jnt_limited[jid]))
            else:
                r.check("limits.position", j.name + " limited", bool(m.jnt_limited[jid]), True, bool(m.jnt_limited[jid]))
                r.near("limits.position", j.name, [j.lower, j.upper], m.jnt_range[jid], t["joint_limit_position"]["abs"])
        for a in range(m.nu):
            name = m.actuator(a).name
            jn = m.joint(m.actuator_trnid[a, 0]).name
            j = u.joints[jn]
            if name.endswith("_position"):
                r.near("limits.effort", name, [-j.effort, j.effort], m.actuator_forcerange[a], t["joint_limit_effort"]["abs"])
                r.check("limits.effort", name + " forcelimited", bool(m.actuator_forcelimited[a]), True, bool(m.actuator_forcelimited[a]))
                r.near("limits.position", name + " ctrlrange", [j.lower, j.upper], m.actuator_ctrlrange[a], t["joint_limit_position"]["abs"])

    def actuators(self):
        """One actuator per command interface the controllers claim; gains from engine_overrides.yaml."""
        r, m, ov, t = self.report, self.m, self.ov["actuators"], self.tol
        expected = {}
        for side in ("left", "right"):
            for k in range(1, 8):
                g = ov["arm_position_gains"]["per_joint"][f"joint{k}"]
                expected[f"openarm_{side}_joint{k}_position"] = ("position", f"openarm_{side}_joint{k}", g["kp"], g["kd"])
            g = ov["gripper_position_gains"]
            expected[f"openarm_{side}_finger_joint1_position"] = ("position", f"openarm_{side}_finger_joint1", g["kp"], g["kd"])
        expected["lift_joint_position"] = ("position", "lift_joint", None, None)
        for jn in ov["wheel_velocity_servo"]["applies_to"]:
            expected[f"{jn}_velocity"] = ("velocity", jn, None, None)
        names = {m.actuator(a).name for a in range(m.nu)}
        r.check("actuators.set", "actuator set", names == set(expected), sorted(set(expected) - names), sorted(names - set(expected)))
        for name, (kind, jn, kp, kd) in expected.items():
            if name not in names:
                continue
            a = m.actuator(name).id
            r.check("actuators.joint", name, m.joint(m.actuator_trnid[a, 0]).name == jn, jn, m.joint(m.actuator_trnid[a, 0]).name)
            if kp is not None:
                rel = t["actuator_gain"]["rel"]
                r.near("actuators.kp", name, kp, m.actuator_gainprm[a, 0], rel * kp)
                r.near("actuators.kp", name + " bias", -kp, m.actuator_biasprm[a, 1], rel * kp)
                r.near("actuators.kd", name, -kd, m.actuator_biasprm[a, 2], rel * max(kd, 1e-12))
        # No mimic follower and no passive joint is actuated.
        actuated = {m.joint(m.actuator_trnid[a, 0]).name for a in range(m.nu)}
        for jn in set(self.ov["mimic"]["joints"]) | set(self.castors):
            r.check("actuators.passive", jn, jn not in actuated, "no actuator", "actuated" if jn in actuated else "no actuator")

    def dynamics(self):
        r, m, u, t = self.report, self.m, self.u, self.tol
        for j in u.movable():
            dof = m.jnt_dofadr[m.joint(j.name).id]
            r.near("dynamics.damping", j.name, j.damping or 0.0, m.dof_damping[dof], t["joint_dynamics"]["abs"])
            r.near("dynamics.frictionloss", j.name, j.friction or 0.0, m.dof_frictionloss[dof], t["joint_dynamics"]["abs"])
            prof = self.profile["joints"].get(j.name)
            r.near("dynamics.armature", j.name, prof["armature"]["value"] if prof else 0.0, m.dof_armature[dof], t["joint_dynamics"]["abs"])
        for name, spec in self.castors.items():
            dof = m.jnt_dofadr[m.joint(name).id]
            r.near("dynamics.damping", name, spec["damping"]["value"], m.dof_damping[dof], t["joint_dynamics"]["abs"])
            r.near("dynamics.frictionloss", name, spec["frictionloss"]["value"], m.dof_frictionloss[dof], t["joint_dynamics"]["abs"])

    def mimic(self):
        """SW-09: every URDF mimic follower is held by an equality constraint with the URDF's multiplier and offset."""
        r, m, u, t = self.report, self.m, self.u, self.tol
        followers = [j for j in u.joints.values() if j.mimic is not None]
        r.check("mimic.count", "followers", len(followers) == m.neq, len(followers), int(m.neq))
        for j in followers:
            name = f"{j.name}_mimic"
            try:
                e = m.equality(name).id
            except KeyError:
                r.check("mimic.present", j.name, False, name, None)
                continue
            r.check("mimic.type", j.name, int(m.eq_type[e]) == int(self.mj.mjtEq.mjEQ_JOINT), "joint", int(m.eq_type[e]))
            r.check("mimic.follower", j.name, m.joint(m.eq_obj1id[e]).name == j.name, j.name, m.joint(m.eq_obj1id[e]).name)
            r.check("mimic.leader", j.name, m.joint(m.eq_obj2id[e]).name == j.mimic["joint"], j.mimic["joint"], m.joint(m.eq_obj2id[e]).name)
            r.near("mimic.polycoef", j.name, [j.mimic["offset"], j.mimic["multiplier"], 0, 0, 0], m.eq_data[e, :5], t["mimic_polycoef"]["abs"])
            r.check("mimic.active", j.name, bool(m.eq_active0[e]), True, bool(m.eq_active0[e]))

    def inertials(self):
        r, m, u, t = self.report, self.m, self.u, self.tol
        self.set_q({})
        poses = u.fk({})
        for name, link in u.links.items():
            b = m.body(name).id
            if link.inertial is None:
                r.near("inertial.mass", name, 0.0, m.body_mass[b], t["mass"]["abs"])
                continue
            r.near("inertial.mass", name, link.inertial.mass, m.body_mass[b], t["mass"]["abs"])
            Tw = poses[name] @ link.inertial.origin
            r.near("inertial.com", name, Tw[:3, 3], self.d.xipos[b], t["com"]["abs"])
            Iref = Tw[:3, :3] @ link.inertial.inertia @ Tw[:3, :3].T
            R = self.d.ximat[b].reshape(3, 3)
            Igot = R @ np.diag(m.body_inertia[b]) @ R.T
            err = float(np.linalg.norm(Igot - Iref) / max(np.linalg.norm(Iref), 1e-300))
            ok = np.linalg.norm(Igot - Iref) <= t["inertia"]["abs"] + t["inertia"]["rel"] * np.linalg.norm(Iref)
            r.check("inertial.inertia", name, bool(ok), Iref.tolist(), Igot.tolist(), err)
        r.near("inertial.total_mass", "robot", u.total_mass(), float(sum(m.body_mass[1:])), t["mass"]["abs"] * len(u.links))

    def frames(self, configurations):
        r, u, t = self.report, self.u, self.tol
        for cname, q in configurations.items():
            self.set_q(q)
            ref = u.fk(q)
            for name in u.links:
                got = self._body_world(name)
                r.near("frames.position", f"{cname}:{name}", ref[name][:3, 3], got[:3, 3], t["frame_position"]["abs"])
                err = rotation_angle(ref[name][:3, :3], got[:3, :3])
                r.check("frames.rotation", f"{cname}:{name}", err <= t["frame_rotation"]["abs"], None, None, err)
            jf = u.joint_frames(q)
            for j in u.movable():
                jid = self.m.joint(j.name).id
                axis_ref = jf[j.name][:3, :3] @ (j.axis / np.linalg.norm(j.axis))
                r.near("frames.joint_axis", f"{cname}:{j.name}", axis_ref, self.d.xaxis[jid], t["joint_axis"]["abs"])
                r.near("frames.joint_anchor", f"{cname}:{j.name}", jf[j.name][:3, 3], self.d.xanchor[jid], t["frame_position"]["abs"])
            for cam, spec in self.ov.get("cameras", {}).items():
                c = self.d.cam(cam)
                Rref = ref[spec["frame"]][:3, :3] @ OPTICAL_TO_MUJOCO_CAMERA
                r.near("frames.camera_position", f"{cname}:{cam}", ref[spec["frame"]][:3, 3], c.xpos, t["frame_position"]["abs"])
                err = rotation_angle(Rref, c.xmat.reshape(3, 3))
                r.check("frames.camera_rotation", f"{cname}:{cam}", err <= t["frame_rotation"]["abs"], None, None, err)

    def geometry(self, mesh_loader):
        """Visual and collision geometry per link: count, type, size, world pose; meshes by world bounding box."""
        r, m, u, t = self.report, self.m, self.u, self.tol
        self.set_q({})
        poses = u.fk({})
        mjtype = {2: "sphere", 3: "capsule", 5: "cylinder", 6: "box", 7: "mesh"}
        for name, link in u.links.items():
            for g in link.visuals + link.collisions:
                gname = f"{name}__{g.use}_{g.index}"
                try:
                    gid = m.geom(gname).id
                except KeyError:
                    r.check("geometry.present", gname, False, g.kind, None)
                    continue
                r.check("geometry.body", gname, m.body(m.geom_bodyid[gid]).name == name, name, m.body(m.geom_bodyid[gid]).name)
                got = mjtype.get(int(m.geom_type[gid]), str(m.geom_type[gid]))
                r.check("geometry.type", gname, got == g.kind, g.kind, got)
                group = 2 if g.use == "visual" else 3
                r.check("geometry.group", gname, int(m.geom_group[gid]) == group, group, int(m.geom_group[gid]))
                collides = bool(m.geom_contype[gid] or m.geom_conaffinity[gid])
                r.check("geometry.collides", gname, collides == (g.use == "collision"), g.use == "collision", collides)
                Tw = poses[name] @ g.origin
                if g.kind != "mesh":
                    size = {"box": [s / 2 for s in g.params.get("size", ())], "cylinder": [g.params.get("radius"), g.params.get("length", 0) / 2],
                            "sphere": [g.params.get("radius")]}[g.kind]
                    r.near("geometry.size", gname, size, m.geom_size[gid][:len(size)], t["geometry_primitive"]["abs"])
                    r.near("geometry.position", gname, Tw[:3, 3], self.d.geom_xpos[gid], t["frame_position"]["abs"])
                    err = rotation_angle(Tw[:3, :3], self.d.geom_xmat[gid].reshape(3, 3))
                    if g.kind in ("sphere",):
                        err = 0.0  # a sphere has no orientation
                    elif g.kind == "cylinder":
                        err = math.acos(max(-1.0, min(1.0, float(Tw[:3, 2] @ self.d.geom_xmat[gid].reshape(3, 3)[:, 2]))))
                    r.check("geometry.rotation", gname, err <= t["frame_rotation"]["abs"], None, None, err)
                else:
                    verts = mesh_loader(g)  # reference vertices in the geometry frame, scale applied
                    ref = (Tw[:3, :3] @ verts.T).T + Tw[:3, 3]
                    mid = m.geom_dataid[gid]
                    adr, n = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
                    v = m.mesh_vert[adr:adr + n]
                    got = (self.d.geom_xmat[gid].reshape(3, 3) @ v.T).T + self.d.geom_xpos[gid]
                    r.near("geometry.mesh_bounds", gname, np.concatenate([ref.min(0), ref.max(0)]),
                           np.concatenate([got.min(0), got.max(0)]), t["geometry_mesh_bounds"]["abs"])
        n_ref = sum(len(l.visuals) + len(l.collisions) for l in u.links.values())
        n_got = m.ngeom - 1  # the floor
        r.check("geometry.count", "all geoms", n_ref == n_got, n_ref, int(n_got))

    def run(self, configurations, mesh_loader):
        self.topology()
        self.limits()
        self.actuators()
        self.dynamics()
        self.mimic()
        self.inertials()
        self.frames(configurations)
        self.geometry(mesh_loader)
        return self.report
