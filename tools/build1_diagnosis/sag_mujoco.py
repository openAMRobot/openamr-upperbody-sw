# SPDX-License-Identifier: MIT
"""D9 step 3 off-line: the commanded joint vector for a hand pose (pinocchio IK on the xacro expansion), held by the
driver's MIT gains (the MJCF actuators are kp, kd of openarm_ros2 4e837e1, no gravity feed-forward), base and lift
frozen. Reports where the hand settles against where it was commanded. Gazebo and the emulator are not in this loop."""
import sys, json, re, tempfile, numpy as np, pinocchio as pin, mujoco
urdf, mjcf = sys.argv[1], sys.argv[2]
pm = pin.buildModelFromUrdf(urdf); pd = pm.createData()
xml = open(mjcf).read()
xml = re.sub(r'<freejoint[^>]*/>', '', xml)            # base welded to the world: base frozen
xml = re.sub(r'<keyframe>.*?</keyframe>', '', xml, flags=re.S)
tmp = mjcf.rsplit('/', 1)[0] + '/_fixed_base.xml'; open(tmp, 'w').write(xml)
mm = mujoco.MjModel.from_xml_path(tmp); md = mujoco.MjData(mm)
import os; os.remove(tmp)
def qidx(n): return pm.joints[pm.getJointId(n)].idx_q
def ik(side, frame, p, R, q0):
    q = q0.copy(); fid = pm.getFrameId(frame); root = pm.getFrameId("base_footprint")
    joints = [f"openarm_{side}_joint{i}" for i in range(1, 8)]
    cols = [pm.joints[pm.getJointId(n)].idx_v for n in joints]
    target = pin.SE3(R, np.array(p))
    for _ in range(500):
        pin.framesForwardKinematics(pm, pd, q)
        cur = pd.oMf[root].actInv(pd.oMf[fid])
        err = pin.log6(cur.actInv(target)).vector
        if np.linalg.norm(err) < 1e-9: break
        J = pin.computeFrameJacobian(pm, pd, q, fid, pin.LOCAL)[:, cols]
        dq = J.T @ np.linalg.solve(J @ J.T + 1e-6 * np.eye(6), err)
        for k, n in enumerate(joints):
            i = qidx(n); lo, hi = pm.lowerPositionLimit[i], pm.upperPositionLimit[i]
            q[i] = np.clip(q[i] + 0.5 * dq[k], lo + 1e-4, hi - 1e-4)
    return q, float(np.linalg.norm(err))
def settle(side, qcmd, seconds=6.0):
    mujoco.mj_resetData(mm, md)
    for n in [f"openarm_{s}_joint{i}" for s in ("left", "right") for i in range(1, 8)]:
        md.qpos[mm.jnt_qposadr[mm.joint(n).id]] = qcmd[qidx(n)]
        md.ctrl[mm.actuator(n + "_position").id] = qcmd[qidx(n)]
    for _ in range(int(seconds / mm.opt.timestep)): mujoco.mj_step(mm, md)
    return md
def pose_mj(body):
    b = mm.body(body).id; r = mm.body("base_footprint").id
    X = pin.SE3(md.xmat[r].reshape(3, 3).copy(), md.xpos[r].copy()).actInv(pin.SE3(md.xmat[b].reshape(3, 3).copy(), md.xpos[b].copy()))
    return X
out = {}
q0 = pin.neutral(pm)
for s in ("left", "right"): q0[qidx(f"openarm_{s}_joint4")] = 0.1
Ry = pin.utils.rpyToMatrix(0, -np.pi / 2, 0)
cases = {"grasp_reach_hand_base": ("left", "openarm_left_ee_base_link", (0.645, 0.16, 0.925), Ry),
         "grasp_reach_tcp_at_item": ("left", "openarm_left_grasp_frame", (0.75, 0.16, 0.925), np.eye(3)),
         "shelf_L0_left_hand_base": ("left", "openarm_left_ee_base_link", (0.75 - 0.05 + 0.0, 0.16, 0.98), None),
         "shelf_L0_right_hand_base": ("right", "openarm_right_ee_base_link", (0.70, -0.16, 0.98), None)}
for name, (side, frame, p, R) in cases.items():
    if R is None:   # position-only goal: keep the orientation the IK reaches from park with a hand pointing forward
        R = Ry
    q, e = ik(side, frame, p, R, q0)
    settle(side, q)
    ach = pose_mj(frame)
    pin.framesForwardKinematics(pm, pd, q)
    cmd = pd.oMf[pm.getFrameId("base_footprint")].actInv(pd.oMf[pm.getFrameId(frame)])
    joints = [f"openarm_{side}_joint{i}" for i in range(1, 8)]
    qact = [float(md.qpos[mm.jnt_qposadr[mm.joint(n).id]]) for n in joints]
    out[name] = {"frame": frame, "ik_residual": e, "q_cmd": [round(float(q[qidx(n)]), 4) for n in joints],
                 "q_err_rad": [round(a - float(q[qidx(n)]), 4) for a, n in zip(qact, joints)],
                 "cmd_xyz": cmd.translation.round(4).tolist(), "settled_xyz": ach.translation.round(4).tolist(),
                 "pos_err_mm": round(float(np.linalg.norm(ach.translation - cmd.translation)) * 1000, 1),
                 "z_err_mm": round(float(ach.translation[2] - cmd.translation[2]) * 1000, 1),
                 "rot_err_deg": round(float(np.degrees(np.linalg.norm(pin.log3(cmd.rotation.T @ ach.rotation)))), 2)}
print(json.dumps(out, indent=1))
