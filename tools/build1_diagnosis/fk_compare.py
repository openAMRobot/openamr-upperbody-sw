# SPDX-License-Identifier: MIT
"""D9 step 1: FK of the same joint vectors in the xacro expansion (pinocchio) and in the generated MJCF (MuJoCo).
Base and lift frozen at 0. Poses in base_footprint."""
import sys, json, numpy as np, pinocchio as pin, mujoco
urdf, mjcf = sys.argv[1], sys.argv[2]
pm = pin.buildModelFromUrdf(urdf); pd = pm.createData()
mm = mujoco.MjModel.from_xml_path(mjcf); md = mujoco.MjData(mm)
ARM = [f"openarm_{s}_joint{i}" for s in ("left", "right") for i in range(1, 8)]
FRAMES = [f"openarm_{s}_{f}" for s in ("left", "right") for f in ("ee_base_link", "grasp_frame")]
lim = {n: (pm.lowerPositionLimit[pm.joints[pm.getJointId(n)].idx_q], pm.upperPositionLimit[pm.joints[pm.getJointId(n)].idx_q]) for n in ARM}

def pin_fk(q):
    qq = pin.neutral(pm)
    for n, v in q.items(): qq[pm.joints[pm.getJointId(n)].idx_q] = v
    pin.framesForwardKinematics(pm, pd, qq)
    root = pd.oMf[pm.getFrameId("base_footprint")]
    return {f: root.actInv(pd.oMf[pm.getFrameId(f)]) for f in FRAMES}

def mj_fk(q):
    md.qpos[:] = mm.qpos0
    for n, v in q.items(): md.qpos[mm.jnt_qposadr[mm.joint(n).id]] = v
    mujoco.mj_kinematics(mm, md)
    def se3(b):
        R = md.xmat[mm.body(b).id].reshape(3, 3).copy(); p = md.xpos[mm.body(b).id].copy()
        return pin.SE3(R, p)
    root = se3("base_footprint")
    return {f: root.actInv(se3(f)) for f in FRAMES}

def vec(values): return dict(zip(ARM, values))
cases = {"activation_zero": vec([0.0] * 14),
         "park": vec([0, 0, 0, 0.1, 0, 0, 0] * 2),
         "right_joint1_at_lower_limit": vec([0, 0, 0, 0.1, 0, 0, 0] + [-1.39639, 0, 0, 0.1, 0, 0, 0])}
for k in range(mm.nkey):
    q = {n: float(mm.key_qpos[k][mm.jnt_qposadr[mm.joint(n).id]]) for n in ARM}
    cases["key_" + mm.key(k).name] = q
extra = json.load(open(sys.argv[3])) if len(sys.argv) > 3 else {}
cases.update(extra)
rng = np.random.default_rng(7)
for i in range(200):
    cases[f"random_{i:03d}"] = {n: float(rng.uniform(*lim[n])) for n in ARM}
rows, worst = [], {f: (0.0, 0.0) for f in FRAMES}
for name, q in cases.items():
    a, b = pin_fk(q), mj_fk(q)
    for f in FRAMES:
        d = a[f].actInv(b[f])
        dp = float(np.linalg.norm(a[f].translation - b[f].translation)) * 1000.0
        da = float(np.degrees(np.linalg.norm(pin.log3(d.rotation))))
        worst[f] = (max(worst[f][0], dp), max(worst[f][1], da))
        if not name.startswith("random"):
            rows.append((name, f, a[f].translation.round(4).tolist(), dp, da))
for r in rows: print(f"{r[0]:34s} {r[1]:28s} pin {r[2]}  dpos {r[3]:.4f} mm  drot {r[4]:.4f} deg")
print("worst over all", len(cases), "vectors:", json.dumps({f: {"mm": float("%.3e" % v[0]), "deg": float("%.3e" % v[1])} for f, v in worst.items()}))
