# SPDX-License-Identifier: MIT
"""Compare the arms of the generated MuJoCo model with Enactic's openarm_mujoco (v2/openarm_bimanual.xml at its pin).

Both models are compiled by MuJoCo and compared item by item, per arm. Bodies are matched through their joints (the finger bodies
have different names). Kinematics and inertials are compared in each arm's base frame, so the mount pose on the lift does not count.
A difference is reported when it exceeds the comparison tolerance, which is the printing precision of the upstream file (it writes
five or six significant digits). This is a comparison of two models' parameters, not of their physics.
"""
import math

import numpy as np

SIDES = ("left", "right")
ARM_JOINTS = [f"joint{k}" for k in range(1, 8)] + ["finger_joint1", "finger_joint2"]
POS_TOL = 1e-5      # m; upstream positions are printed to 1e-5 m or finer
ROT_TOL = 1e-4      # rad
REL_TOL = 1e-4      # relative; masses and inertias printed to five or six significant digits
ABS_TOL = 1e-9


def _pose(d, body):
    T = np.eye(4)
    T[:3, :3] = d.xmat[body].reshape(3, 3)
    T[:3, 3] = d.xpos[body]
    return T


def _angle(Ra, Rb):
    return math.acos(max(-1.0, min(1.0, (np.trace(Ra.T @ Rb) - 1) / 2)))


class Side:
    def __init__(self, mj, m, side):
        self.mj, self.m, self.side = mj, m, side
        self.d = mj.MjData(m)
        self.base = m.body(f"openarm_{side}_base_link").id
        self.joints = {j: m.joint(f"openarm_{side}_{j}").id for j in ARM_JOINTS}
        self.bodies = {"base_link": self.base}
        self.bodies.update({j: int(m.jnt_bodyid[jid]) for j, jid in self.joints.items()})

    def set(self, q):
        self.d.qpos[:] = self.m.qpos0
        for j, v in q.items():
            self.d.qpos[self.m.jnt_qposadr[self.joints[j]]] = v
        self.mj.mj_kinematics(self.m, self.d)
        self.mj.mj_comPos(self.m, self.d)

    def rel(self, body):
        return np.linalg.inv(_pose(self.d, self.base)) @ _pose(self.d, body)

    def actuator(self, joint):
        m = self.m
        for a in range(m.nu):
            if int(m.actuator_trnid[a, 0]) == self.joints[joint]:
                return a
        return None


def compare(mj, ours_model, upstream_model, configurations=8, seed=0):
    rows = []

    def row(item, where, ours, theirs, same):
        rows.append({"item": item, "where": where, "ours": ours, "enactic": theirs, "same": bool(same)})

    def num(x):
        return float(np.round(x, 6))

    for side in SIDES:
        a, b = Side(mj, ours_model, side), Side(mj, upstream_model, side)
        # Joint parameters
        for j in ARM_JOINTS:
            ja, jb = a.joints[j], b.joints[j]
            name = f"openarm_{side}_{j}"
            ra, rb = ours_model.jnt_range[ja], upstream_model.jnt_range[jb]
            row("joint range", name, ra.round(6).tolist(), rb.round(6).tolist(), np.allclose(ra, rb, atol=1e-6))
            for attr, label in (("dof_damping", "damping"), ("dof_frictionloss", "frictionloss"), ("dof_armature", "armature")):
                va = getattr(ours_model, attr)[ours_model.jnt_dofadr[ja]]
                vb = getattr(upstream_model, attr)[upstream_model.jnt_dofadr[jb]]
                row(label, name, num(va), num(vb), abs(va - vb) <= ABS_TOL)
            aa, ab = a.actuator(j), b.actuator(j)
            if aa is None and ab is None:
                row("actuator", name, "none", "none", True)
                continue
            if (aa is None) != (ab is None):
                row("actuator", name, "none" if aa is None else ours_model.actuator(aa).name,
                    "none" if ab is None else upstream_model.actuator(ab).name, False)
                continue
            for label, fa, fb in (
                ("actuator kp", ours_model.actuator_gainprm[aa, 0], upstream_model.actuator_gainprm[ab, 0]),
                ("actuator kv", -ours_model.actuator_biasprm[aa, 2], -upstream_model.actuator_biasprm[ab, 2]),
            ):
                row(label, name, num(fa), num(fb), abs(fa - fb) <= REL_TOL * max(abs(fb), 1e-9))
            fa, fb = ours_model.actuator_forcerange[aa], upstream_model.actuator_forcerange[ab]
            row("actuator force range", name, fa.round(6).tolist(), fb.round(6).tolist(), np.allclose(fa, fb, atol=1e-9))
            ca, cb = ours_model.actuator_ctrlrange[aa], upstream_model.actuator_ctrlrange[ab]
            row("actuator control range", name, ca.round(6).tolist(), cb.round(6).tolist(), np.allclose(ca, cb, atol=1e-5))
        # Kinematics and inertials in the arm base frame, at the zero pose and at seeded poses within the limits.
        rng = np.random.default_rng(seed)
        qs = [{}]
        for _ in range(configurations):
            q = {}
            for j in ARM_JOINTS[:-1]:
                lo, hi = upstream_model.jnt_range[b.joints[j]]
                q[j] = float(rng.uniform(lo, hi))
            q["finger_joint2"] = q["finger_joint1"]
            qs.append(q)
        worst = {}
        for q in qs:
            a.set(q)
            b.set(q)
            for key in a.bodies:
                Ta, Tb = a.rel(a.bodies[key]), b.rel(b.bodies[key])
                w = worst.setdefault(key, [0.0, 0.0])
                w[0] = max(w[0], float(np.linalg.norm(Ta[:3, 3] - Tb[:3, 3])))
                w[1] = max(w[1], _angle(Ta[:3, :3], Tb[:3, :3]))
        for key, (dp, dr) in worst.items():
            where = f"openarm_{side}_{key if key == 'base_link' else ARM_JOINTS[ARM_JOINTS.index(key)]} child body"
            row("body frame in arm base frame (max over poses), position m", where, f"{dp:.2e}", "reference", dp <= POS_TOL)
            row("body frame in arm base frame (max over poses), rotation rad", where, f"{dr:.2e}", "reference", dr <= ROT_TOL)
        a.set({})
        b.set({})
        for key in a.bodies:
            ba, bb = a.bodies[key], b.bodies[key]
            ma, mb = ours_model.body_mass[ba], upstream_model.body_mass[bb]
            where = f"{ours_model.body(ba).name} / {upstream_model.body(bb).name}"
            row("mass kg", where, num(ma), num(mb), abs(ma - mb) <= REL_TOL * mb)
            Tb_a = np.linalg.inv(_pose(a.d, a.base))
            Tb_b = np.linalg.inv(_pose(b.d, b.base))
            ca = (Tb_a @ np.append(a.d.xipos[ba], 1))[:3]
            cb = (Tb_b @ np.append(b.d.xipos[bb], 1))[:3]
            row("centre of mass in arm base frame, m", where, ca.round(6).tolist(), cb.round(6).tolist(), np.linalg.norm(ca - cb) <= POS_TOL)
            Ra = Tb_a[:3, :3] @ a.d.ximat[ba].reshape(3, 3)
            Rb = Tb_b[:3, :3] @ b.d.ximat[bb].reshape(3, 3)
            Ia = Ra @ np.diag(ours_model.body_inertia[ba]) @ Ra.T
            Ib = Rb @ np.diag(upstream_model.body_inertia[bb]) @ Rb.T
            rel = float(np.linalg.norm(Ia - Ib) / np.linalg.norm(Ib))
            row("inertia tensor in arm base frame (relative Frobenius difference)", where, f"{rel:.2e}", "reference", rel <= REL_TOL * 10)
        # Collision geometry count per body
        for key in a.bodies:
            ba, bb = a.bodies[key], b.bodies[key]
            na = int(sum(1 for g in range(ours_model.ngeom) if ours_model.geom_bodyid[g] == ba and ours_model.geom_group[g] == 3))
            nb = int(sum(1 for g in range(upstream_model.ngeom) if upstream_model.geom_bodyid[g] == bb and upstream_model.geom_group[g] == 3))
            row("collision geoms", f"{ours_model.body(ba).name}", na, nb, na == nb)
        # Mimic
        f2a, f2b = a.joints["finger_joint2"], b.joints["finger_joint2"]
        def eq_for(model, jid):
            for e in range(model.neq):
                if int(model.eq_type[e]) == int(mj.mjtEq.mjEQ_JOINT) and jid in (int(model.eq_obj1id[e]), int(model.eq_obj2id[e])):
                    return e
            return None
        ea, eb = eq_for(ours_model, f2a), eq_for(upstream_model, f2b)
        row("finger_joint2 constraint", f"openarm_{side}_finger_joint2", "equality" if ea is not None else "none",
            "equality" if eb is not None else "none", (ea is None) == (eb is None))
        if ea is not None and eb is not None:
            for label, va, vb in (("equality polycoef", ours_model.eq_data[ea, :5], upstream_model.eq_data[eb, :5]),
                                  ("equality solref", ours_model.eq_solref[ea], upstream_model.eq_solref[eb]),
                                  ("equality solimp", ours_model.eq_solimp[ea, :3], upstream_model.eq_solimp[eb, :3])):
                row(label, f"openarm_{side}_finger_joint2", np.round(va, 6).tolist(), np.round(vb, 6).tolist(), np.allclose(va, vb, atol=1e-9))
    # Options
    for label, va, vb in (("timestep s", ours_model.opt.timestep, upstream_model.opt.timestep),
                          ("integrator", int(ours_model.opt.integrator), int(upstream_model.opt.integrator)),
                          ("friction cone", int(ours_model.opt.cone), int(upstream_model.opt.cone)),
                          ("impratio", ours_model.opt.impratio, upstream_model.opt.impratio)):
        row(label, "option", num(va), num(vb), abs(va - vb) <= 1e-12)
    return rows


def markdown(rows, only_differences=True):
    lines = ["| Item | Where | This model | openarm_mujoco |", "|---|---|---|---|"]
    for r in rows:
        if only_differences and r["same"]:
            continue
        lines.append(f"| {r['item']} | {r['where']} | {r['ours']} | {r['enactic']} |")
    return "\n".join(lines) + "\n"
