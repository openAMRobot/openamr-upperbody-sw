#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""T5 gripper: both hands, open / half / closed (or any --fractions), max_effort <= 5. docs/sim_to_real_protocol.md, T5.

Units (#79). finger_joint1 is revolute in the model: left [0, 0.7854] rad, right [-0.7854, 0] rad; SRDF `open` is the far end of each hand's
range and `closed` is 0. Decision of 17 Nov 2026 on #79: radians everywhere. The real profile passes ee_type pinch_gripper, with which the
pinned driver maps finger radians to motor radians 1:1 (openarm_ros2 4e837e1 openarm_simple_hardware.cpp 375-392); its 0 to 0.044 m mapping
is the parallel_link branch only. Still open on #79: the motor-angle against finger-angle table measured on the real gripper (5 or more
points, the linkage may not be 1:1): run with --fractions 1,0.75,0.5,0.25,0 for it. The open value per hand is read from the URDF limits
on /robot_description; --left-open / --right-open override it, and the raw value sent is recorded either way. The pinned driver reports
gripper velocity and effort as 0 (openarm_simple_hardware.cpp 277-278): effort is not a gripper measurement on the robot.

Recorded per command: the raw command value, the action result, /joint_states finger_joint1/2, time to settle (within 1e-3 of its
final value), the model jaw opening at the measured angle (common.fingertip_gap, the M05 method, sim), and in sim the Gazebo roll of
openarm_<side>_ee_link1. On the bench: caliper jaw opening (templates/t5_gripper_manual.csv).
"""
import math

import common

SETTLE_TOL = 1e-3


def settle_after(js_rows, joint, t0):
    seq = [(t, d[joint][0]) for t, d in js_rows if t >= t0 and joint in d]
    if not seq:
        return math.nan, math.nan
    final = seq[-1][1]
    last_out = max((t for t, v in seq if abs(v - final) > SETTLE_TOL), default=None)
    if last_out is None:
        return 0.0, final
    nxt = next((t for t, _ in seq if t > last_out), None)
    return ((nxt - t0) if nxt is not None else math.nan), final


def main(argv=None):
    p = common.base_parser(__doc__)
    p.add_argument("--left-open", type=float, default=None, help="override: raw GripperCommand position, left hand fully open (default: URDF limit)")
    p.add_argument("--right-open", type=float, default=None, help="override: raw GripperCommand position, right hand fully open (default: URDF limit)")
    p.add_argument("--fractions", default="1,0.5,0", help="fractions of the open value to command, in order (1 open, 0.5 half, 0 closed)")
    p.add_argument("--max-effort", type=float, default=common.GRIPPER_MAX_EFFORT)
    p.add_argument("--hold-s", type=float, default=3.0)
    p.add_argument("--sides", default="left,right")
    p.add_argument("--physics", default="", help="sim: the physics engine of the running simulation, recorded in session.yaml")
    a = common.parse(p, argv)
    s = common.Session("t5_gripper", a)
    s.write_meta(physics=a.physics, unit_note="#79: model revolute rad vs pinned driver prismatic m; raw values recorded as sent")
    guard = common.StopGuard()
    import rosio
    io = rosio.Io(s, guard, "sim2real_t5")
    rows, metrics = [], []
    try:
        io.wait_for_joint_states([f"openarm_{x}_finger_joint1" for x in ("left", "right")], 60)
        urdf = io.wait_for_urdf()
        joints, _ = common.load_urdf_joints(urdf) if urdf else ({}, {})
        for side in a.sides.split(","):
            j1 = f"openarm_{side}_finger_joint1"
            override = a.left_open if side == "left" else a.right_open
            if override is not None:
                open_v = override
            else:
                lo, hi = joints[j1]["lower"], joints[j1]["upper"]
                open_v = hi if abs(hi) > abs(lo) else lo        # the far end of this hand's range (SRDF `open`)
            names = {1.0: "open", 0.5: "half", 0.0: "closed"}
            for frac in [float(x) for x in a.fractions.split(",")]:
                state, cmd = names.get(frac, f"f{frac:g}"), open_v * frac
                r = io.gripper(side, cmd, a.max_effort, f"{side} gripper {state}: raw position {cmd!r}")
                if not r.get("sent"):
                    rows.append([side, state, cmd, None] + [math.nan] * 9)
                    continue
                io.spin_sim(a.hold_s)
                st, final = settle_after(io.js_rows, j1, r.get("t_sent", 0.0))
                f2 = io.js_last.get(f"openarm_{side}_finger_joint2", (math.nan,))[0]
                gap = common.fingertip_gap(urdf, side, final) if (urdf and math.isfinite(final) and a.target == "sim") else math.nan
                roll = common.gz_pose(f"openarm_{side}_ee_link1")["rpy"][0] if a.target == "sim" else math.nan
                res = r.get("result") or {}
                rows.append([side, state, cmd, r.get("accepted"), res.get("position", math.nan), res.get("stalled"), res.get("reached_goal"),
                             final, f2, final - cmd, st, gap, roll])
                tag = f"{side}_{state}"
                metrics += [("T5", f"{tag}_final_rad", final, "rad", "joint_states finger_joint1"),
                            ("T5", f"{tag}_error", final - cmd, "rad", "joint_states - raw command"),
                            ("T5", f"{tag}_settle_s", st, "s", f"joint_states, within {SETTLE_TOL} of final"),
                            ("T5", f"{tag}_sign_ok", float(cmd == 0.0 or (final * cmd) > 0), "1", "final has the sign of the command")]
                if math.isfinite(gap):
                    metrics.append(("T5", f"{tag}_model_jaw_gap_m", gap, "m", "fingertip_gap at the measured angle (M05 method)"))
        common.write_csv(s.path("t5_gripper.csv"), ["side", "state", "command_raw", "accepted", "result_position", "stalled", "reached_goal",
                         "finger_joint1", "finger_joint2", "error", "settle_s", "model_jaw_gap_m", "gt_ee_link1_roll_rad"], rows)
        io.dump_joint_states(s.path("joint_states.csv"), [f"openarm_{x}_finger_joint{k}" for x in ("left", "right") for k in (1, 2)])
        common.write_metrics(s, metrics)
    finally:
        guard.stop()
        s.write_meta(finished=True)
        io.close()
    print(s.dir)


if __name__ == "__main__":
    main()
