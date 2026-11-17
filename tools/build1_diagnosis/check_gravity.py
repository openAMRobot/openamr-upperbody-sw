# SPDX-License-Identifier: MIT
"""Compare gravity.py with pinocchio's generalized gravity on the xacro expansion, and hold the grasp pose in MuJoCo
with the driver's gains plus that torque as the MIT tau term."""
import sys, re, os, json, numpy as np, pinocchio as pin, mujoco
sys.path.insert(0, sys.argv[3])
from openamrobot_build1_scene.gravity import GravityModel, ARM_JOINTS
urdf_text = open(sys.argv[1]).read()
gm = GravityModel(urdf_text)
pm = pin.buildModelFromUrdf(sys.argv[1]); pd = pm.createData()
pm.gravity.linear = np.array([0, 0, -9.80665])
names = ARM_JOINTS["left"] + ARM_JOINTS["right"]
rng = np.random.default_rng(3); worst = 0.0; refs = {}
def lim(n): i = pm.joints[pm.getJointId(n)].idx_q; return pm.lowerPositionLimit[i], pm.upperPositionLimit[i]
cases = {"zero": {n: 0.0 for n in names}, "park": {n: (0.1 if n.endswith("4") else 0.0) for n in names},
         "grasp": dict(zip(ARM_JOINTS["left"], [-1.057, -0.0153, -0.004, 0.6676, -0.0041, -0.1539, -0.0147]))}
for i in range(300): cases[f"r{i}"] = {n: float(rng.uniform(*lim(n))) for n in names}
for name, q in cases.items():
    qq = pin.neutral(pm)
    for n, v in q.items(): qq[pm.joints[pm.getJointId(n)].idx_q] = v
    g = pin.computeGeneralizedGravity(pm, pd, qq)
    ref = [float(g[pm.joints[pm.getJointId(n)].idx_v]) for n in names]
    mine = gm.torques(q, names)
    worst = max(worst, max(abs(a - b) for a, b in zip(ref, mine)))
    if not name.startswith("r") or name in ("r0", "r1"):
        refs[name] = {"q": {n: round(v, 6) for n, v in q.items()}, "tau": [round(v, 6) for v in ref]}
print("max |gravity.py - pinocchio| over", len(cases), "vectors:", worst, "N m")
json.dump(refs, open(sys.argv[4], "w"), indent=1)
# MuJoCo: the driver law (MJCF actuators carry kp, kd of the driver) plus tau = g(q measured)
xml = open(sys.argv[2]).read(); xml = re.sub(r'<freejoint[^>]*/>', '', xml); xml = re.sub(r'<keyframe>.*?</keyframe>', '', xml, flags=re.S)
tmp = os.path.join(os.path.dirname(sys.argv[2]), "_w10_fixed.xml"); open(tmp, "w").write(xml)
mm = mujoco.MjModel.from_xml_path(tmp); os.remove(tmp); md = mujoco.MjData(mm)
q = cases["grasp"]
for ff in (False, True):
    mujoco.mj_resetData(mm, md)
    for n in names:
        v = q.get(n, 0.1 if n.endswith("4") else 0.0)
        md.qpos[mm.jnt_qposadr[mm.joint(n).id]] = v; md.ctrl[mm.actuator(n + "_position").id] = v
    for step in range(int(6.0 / mm.opt.timestep)):
        if ff:
            meas = {n: float(md.qpos[mm.jnt_qposadr[mm.joint(n).id]]) for n in names}
            meas.update({f"openarm_{s}_finger_joint1": float(md.qpos[mm.jnt_qposadr[mm.joint(f'openarm_{s}_finger_joint1').id]]) for s in ("left", "right")})
            for n, t in zip(names, gm.torques(meas, names)): md.qfrc_applied[mm.jnt_dofadr[mm.joint(n).id]] = t
        mujoco.mj_step(mm, md)
    err = [round(float(md.qpos[mm.jnt_qposadr[mm.joint(n).id]]) - q[n], 4) for n in ARM_JOINTS["left"]]
    b = mm.body("openarm_left_ee_base_link").id
    print("gravity feed-forward" if ff else "driver gains only   ", "joint error rad", err, "hand base z", round(float(md.xpos[b][2]), 4))
