# SPDX-License-Identifier: MIT
"""Arm reach of the Build 1 work-shelf grasp against base pose errors (docs/integration/station_approach.md).

Damped least-squares IK of openarm_left_joint1..7 (pinocchio) for the scn04 approach, grasp and lift hand poses
(SIDE_GRASP, MoveIt 0.01 m / 0.3 rad), with the base displaced from stations.work_shelf. Collisions are not checked.
    reach_probe.py URDF axes
    reach_probe.py URDF circle RADIUS_M YAW_RAD DIRECTIONS
"""
import sys, math, numpy as np, pinocchio as pin
urdf = sys.argv[1]
m = pin.buildModelFromUrdf(urdf); d = m.createData()
arm = [f"openarm_left_joint{i}" for i in range(1, 8)]
idx = [m.joints[m.getJointId(n)].idx_q for n in arm]
lo = np.array([m.lowerPositionLimit[i] for i in idx]); hi = np.array([m.upperPositionLimit[i] for i in idx])
print("limits", np.round(lo, 3), np.round(hi, 3))
fb, fh, ft = (m.getFrameId(n) for n in ("base_footprint", "openarm_left_ee_base_link", "openarm_left_grasp_frame"))
q0 = pin.neutral(m)
def fk(q):
    pin.framesForwardKinematics(m, d, q); return d.oMf[fb].inverse() * d.oMf[fh]
pin.framesForwardKinematics(m, d, q0)
tcp = (d.oMf[fh].inverse() * d.oMf[ft]).translation
print("tcp in hand", np.round(tcp, 4))
R_side = pin.Quaternion(math.sqrt(0.5), 0.0, -math.sqrt(0.5), 0.0).toRotationMatrix()  # w, x, y, z
def hand_target(p):
    dx, dy, dz = -tcp[2], tcp[1], tcp[0]
    return np.array([p[0] - dx, p[1] - dy, p[2] - dz])
def ik(target, seeds, ang_tol=0.3, pos_tol=0.01):
    best = None
    for s in seeds:
        q = q0.copy(); q[idx] = s
        for it in range(300):
            T = fk(q)
            ep = target - T.translation
            eo = pin.log3(R_side @ T.rotation.T)  # rotation error in base frame
            if np.linalg.norm(ep) < pos_tol * 0.3 and np.linalg.norm(eo) < 0.02: break
            J6 = pin.computeFrameJacobian(m, d, q, fh, pin.LOCAL_WORLD_ALIGNED)
            Jb = J6[:, [m.joints[m.getJointId(n)].idx_v for n in arm]]
            Rb = d.oMf[fb].rotation.T
            J = np.vstack([Rb @ Jb[:3], 0.3 * (Rb @ Jb[3:])])
            e = np.concatenate([ep, 0.3 * eo])
            dq = J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), e)
            qa = np.clip(q[idx] + 0.5 * dq, lo + 0.02, hi - 0.02); q[idx] = qa
        T = fk(q); pe = np.linalg.norm(target - T.translation); ae = np.linalg.norm(pin.log3(R_side @ T.rotation.T))
        if best is None or pe + 0.05 * ae < best[0] + 0.05 * best[1]: best = (pe, ae, q[idx].copy())
    ok = best[0] < pos_tol and best[1] < ang_tol
    return ok, best
rng = np.random.default_rng(1)
seeds = [np.array([0, 0, 0, 0.1, 0, 0, 0]), np.array([-1.057, -0.0153, -0.004, 0.6676, -0.0041, -0.1539, -0.0147])] + [rng.uniform(lo, hi) for _ in range(6)]
item_world = np.array([0.16, -2.835, 0.895]); station = (0.0, -2.085, -math.pi / 2)
def in_base(base):
    x, y, yaw = base; dx, dy = item_world[0] - x, item_world[1] - y; c, s = math.cos(-yaw), math.sin(-yaw)
    return np.array([c * dx - s * dy, s * dx + c * dy, item_world[2]])
def feasible(ef, el, eyaw):
    x0, y0, yaw0 = station
    # ef: forward error along the station heading (+ is towards the shelf), el: to the left of the heading
    base = (x0 + ef * math.cos(yaw0) - el * math.sin(yaw0), y0 + ef * math.sin(yaw0) + el * math.cos(yaw0), yaw0 + eyaw)
    item = in_base(base); grasp = hand_target((item[0], item[1], item[2] + 0.03)); approach = grasp - np.array([0.10, 0, 0])
    a = ik(grasp, seeds); b = ik(approach, seeds)
    lift = ik(grasp + np.array([0, 0, 0.08]), seeds, pos_tol=0.015, ang_tol=0.1 + 0.2)
    return a[0] and b[0] and lift[0], (a[1][:2], b[1][:2], lift[1][:2]), item
ok, info, item = feasible(0, 0, 0); print("nominal item in base", np.round(item, 3), "feasible", ok, [tuple(np.round(i, 3)) for i in info])
mode = sys.argv[2] if len(sys.argv) > 2 else "axes"
if mode == "axes":
    for name, axis in (("forward", 0), ("lateral", 1)):
        for e in [-0.30, -0.25, -0.20, -0.15, -0.12, -0.10, -0.08, -0.06, -0.04, -0.02, 0.02, 0.04, 0.06, 0.08, 0.10, 0.12, 0.15, 0.20]:
            args = [0.0, 0.0, 0.0]; args[axis] = e
            ok, info, _ = feasible(*args); print(name, e, ok, [tuple(np.round(i, 3)) for i in info], flush=True)
    for e in [-0.30, -0.2, -0.15, -0.1, -0.05, 0.05, 0.1, 0.15, 0.2, 0.3]:
        ok, info, _ = feasible(0, 0, e); print("yaw", e, ok, [tuple(np.round(i, 3)) for i in info], flush=True)
else:
    r = float(sys.argv[3]); ya = float(sys.argv[4]); bad = 0; n = 0
    for k in range(int(sys.argv[5])):
        th = 2 * math.pi * k / int(sys.argv[5])
        for yy in (-ya, 0.0, ya):
            ok, info, _ = feasible(r * math.cos(th), r * math.sin(th), yy); n += 1; bad += (not ok)
            if not ok: print("infeasible", round(r * math.cos(th), 3), round(r * math.sin(th), 3), yy, [tuple(np.round(i, 3)) for i in info])
    print(f"circle r={r} yaw=+-{ya}: {n - bad}/{n} feasible")
