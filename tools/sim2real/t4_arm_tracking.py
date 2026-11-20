#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""T4 arm tracking: per arm, SRDF home -> hands_up -> park at velocity scaling 0.1. docs/sim_to_real_protocol.md, T4.

One FollowJointTrajectory goal per segment on /<side>_arm_controller (arm_controllers.yaml), its duration set by the slowest joint at
0.1 of its URDF velocity limit. Recorded: the controller's own reference and feedback (~/controller_state) and /joint_states.
Per segment: RMS and max |reference - feedback| per joint, settle time after the trajectory end to within 1e-3 rad of the goal on every
joint. At each held pose: mean /joint_states effort per joint (driver-reported on the robot, simulator-reported in sim) next to the
static gravity torque of the URDF model at the measured joint positions (the A12 input for #73's gravity feed-forward). At park: the
M07 rest margin of every arm joint, against the datum joint_rest_margin_rad.
"""
import math

import numpy as np

import common

SETTLE_TOL = 1e-3


def segment_stats(ref_rows, names, t0, t1):
    """ref_rows: (t, ref[7], fb[7]). RMS / max |ref - fb| per joint over [t0, t1]."""
    seg = [(r, f) for t, r, f in ref_rows if t0 <= t <= t1]
    if not seg:
        return {n: (math.nan, math.nan) for n in names}
    e = np.array([[a - b for a, b in zip(r, f)] for r, f in seg])
    return {n: (float(np.sqrt(np.mean(e[:, i] ** 2))), float(np.max(np.abs(e[:, i])))) for i, n in enumerate(names)}


def settle_time(js_rows, names, goal, t_end, tol=SETTLE_TOL):
    """Seconds after t_end until every joint is within tol of goal and stays there for the rest of the record; nan if never."""
    after = [(t, d) for t, d in js_rows if t >= t_end and all(n in d for n in names)]
    last_out = None
    for t, d in after:
        if any(abs(d[n][0] - g) > tol for n, g in zip(names, goal)):
            last_out = t
    if not after:
        return math.nan
    if last_out is None:
        return 0.0
    nxt = next((t for t, _ in after if t > last_out), None)
    return (nxt - t_end) if nxt is not None else math.nan


def main(argv=None):
    p = common.base_parser(__doc__)
    p.add_argument("--sides", default="left,right")
    p.add_argument("--scaling", type=float, default=common.ARM_MAX_VELOCITY_SCALING)
    p.add_argument("--hold-s", type=float, default=3.0)
    a = common.parse(p, argv)
    s = common.Session("t4_arm_tracking", a)
    guard = common.StopGuard()
    import rclpy  # noqa: F401
    from control_msgs.msg import JointTrajectoryControllerState
    import rosio
    io = rosio.Io(s, guard, "sim2real_t4")
    scaling = common.cap_velocity_scaling(a.scaling, s.caps)
    refs = {}
    for side in ("left", "right"):
        refs[side] = []
        io.node.create_subscription(JointTrajectoryControllerState, common.ARM_CONTROLLER[side] + "/controller_state",
                                    lambda m, side=side: refs[side].append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                                                                            list(m.reference.positions), list(m.feedback.positions))), 100)
    metrics, segs, holds = [], [], []
    try:
        names_all = rosio.arm_joints("left") + rosio.arm_joints("right")
        io.wait_for_joint_states(names_all, 60)
        urdf = io.wait_for_urdf()
        if urdf is None:
            raise SystemExit("no /robot_description: cannot read velocity limits")
        joints, links = common.load_urdf_joints(urdf)
        margin = common.datum("joint_rest_margin_rad")
        for side in a.sides.split(","):
            names = rosio.arm_joints(side)
            vel = [joints[n]["velocity"] for n in names]
            for pose_name, target in (("home", rosio.HOME), ("hands_up", rosio.HANDS_UP), ("park", rosio.PARK)):
                q0 = io.pos(names) if not a.dry_run or all(n in io.js_last for n in names) else [0.0] * 7
                (acc, code, msg, t_sent, t_done), dur = rosio.move_arm(io, side, target, scaling, vel, f"{side} arm to {pose_name} at scaling {scaling}")
                if a.dry_run:
                    continue
                t_end = t_sent + dur if t_sent is not None else None
                io.spin_sim(a.hold_s)
                st = segment_stats(refs[side], names, t_sent, t_end) if t_end else {}
                settle = settle_time(io.js_rows, names, target, t_end) if t_end else math.nan
                q = io.pos(names)
                window = [d for t, d in io.js_rows if t_end and t >= t_end + max(0.0, a.hold_s - 2.0)]
                eff = {n: float(np.nanmean([d[n][2] for d in window if n in d])) if window else math.nan for n in names}
                lift = io.js_last.get("lift_joint", (0.0,))[0]
                qfull = {n: v[0] for n, v in io.js_last.items()}
                qfull["lift_joint"] = lift
                model = common.gravity_torques(joints, links, qfull, "base_footprint", names)
                segs.append([side, pose_name, dur, acc, code, msg, t_sent, t_done, settle] + [st.get(n, (math.nan,))[0] for n in names] + [st.get(n, (0, math.nan))[1] for n in names])
                for n, qq, g in zip(names, q, target):
                    holds.append([side, pose_name, n, g, qq, qq - g, eff[n], model[n]])
                    metrics.append(("T4", f"{n}_{pose_name}_rms_rad", st.get(n, (math.nan,))[0], "rad", "controller_state reference - feedback"))
                    metrics.append(("T4", f"{n}_{pose_name}_max_rad", st.get(n, (0, math.nan))[1], "rad", "controller_state reference - feedback"))
                    metrics.append(("T4", f"{n}_{pose_name}_effort", eff[n], "N*m", "joint_states effort, mean of the last 2 s of the hold"))
                    metrics.append(("T4", f"{n}_{pose_name}_model_gravity_torque", model[n], "N*m", "URDF masses and COMs at the measured q"))
                metrics.append(("T4", f"{side}_{pose_name}_settle_s", settle, "s", f"joint_states, all 7 joints within {SETTLE_TOL} rad"))
                if pose_name == "park":
                    for n, qq in zip(names, q):
                        m = min(qq - joints[n]["lower"], joints[n]["upper"] - qq)
                        metrics.append(("T4", f"{n}_park_m07_margin_rad", m, "rad", f"min distance to a URDF limit; datum joint_rest_margin_rad {margin}"))
        common.write_csv(s.path("t4_segments.csv"), ["side", "pose", "duration_s", "accepted", "error_code", "error_string", "t_sent_s", "t_done_s", "settle_s"]
                         + [f"rms_j{i}" for i in range(1, 8)] + [f"max_j{i}" for i in range(1, 8)], segs)
        common.write_csv(s.path("t4_holds.csv"), ["side", "pose", "joint", "goal_rad", "measured_rad", "error_rad", "effort_reported", "model_gravity_torque"], holds)
        for side in ("left", "right"):
            common.write_csv(s.path(f"t4_controller_state_{side}.csv"), ["stamp_s"] + [f"ref_j{i}" for i in range(1, 8)] + [f"fb_j{i}" for i in range(1, 8)],
                             [[t] + r + f for t, r, f in refs[side]])
        io.dump_joint_states(s.path("joint_states.csv"), names_all)
        common.write_metrics(s, metrics)
    finally:
        guard.stop()
        s.write_meta(finished=True)
        io.close()
    print(s.dir)


if __name__ == "__main__":
    main()
