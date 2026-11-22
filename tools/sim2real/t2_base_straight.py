#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""T2 base straight: 1.0 m forward and 1.0 m back at 0.2 m/s, both arms in park (AD-30). docs/sim_to_real_protocol.md, T2.

Commanded: a capped Twist on the base's velocity topic for distance / speed seconds of the node's clock, then zero.
Recorded: odometry (never the reference), and in sim Gazebo ground truth of the model (`gz model -m openamrobot -p`) before and after
each leg. On the bench the reference is tape between floor marks at the front face centre of base_link's collision box
(templates/t2_base_straight_manual.csv).
"""
import math

import common


def leg_metrics(prefix, gt0, gt1, commanded_m, odom0, odom1):
    """Distance along the initial heading, lateral drift, and odometry over ground truth, from two poses."""
    yaw0 = gt0["rpy"][2]
    dx, dy = gt1["xyz"][0] - gt0["xyz"][0], gt1["xyz"][1] - gt0["xyz"][1]
    along = dx * math.cos(yaw0) + dy * math.sin(yaw0)
    lateral = -dx * math.sin(yaw0) + dy * math.cos(yaw0)
    out = [("T2", f"{prefix}_gt_along_m", along, "m", "gz model -p"),
           ("T2", f"{prefix}_gt_lateral_m", lateral, "m", "gz model -p"),
           ("T2", f"{prefix}_sim_over_commanded", abs(along) / commanded_m, "1", "gz model -p / commanded")]
    if odom0 and odom1:
        od = math.hypot(odom1[1] - odom0[1], odom1[2] - odom0[2])
        out.append(("T2", f"{prefix}_odom_over_gt", od / math.hypot(dx, dy) if math.hypot(dx, dy) > 0 else math.nan, "1", "odometry / gz model -p"))
    return out


def main(argv=None):
    p = common.base_parser(__doc__, robot_required=[("--cmd-vel-topic", "geometry_msgs/Twist topic of the base."),
                                                   ("--odom-topic", "nav_msgs/Odometry topic of the base (recorded, not a reference).")])
    p.add_argument("--distance", type=float, default=1.0)
    p.add_argument("--speed", type=float, default=0.2)
    p.add_argument("--pause-s", type=float, default=2.0)
    a = common.parse(p, argv)
    s = common.Session("t2_base_straight", a)
    guard = common.StopGuard()
    import rosio
    io = rosio.Io(s, guard, "sim2real_t2")
    cmd_topic = a.cmd_vel_topic or common.SIM_CMD_VEL
    odom_topic = a.odom_topic or common.SIM_ODOM
    io.record_odom(odom_topic)
    pub = io.twist_publisher(cmd_topic)
    metrics, legs = [], []
    try:
        io.wait_for_joint_states(rosio.arm_joints("left") + rosio.arm_joints("right"), 60)
        ok, wj, err = rosio.park_check(io)
        s.write_meta(park_check={"ok": ok, "worst_joint": wj, "error_rad": err}, cmd_vel_topic=cmd_topic, odom_topic=odom_topic)
        if not ok and not a.dry_run:
            raise SystemExit(f"AD-30: arms not in park ({wj} off by {err:.4f} rad); base motion refused (run t4 to park first)")
        speed = common.clamp_abs(a.speed, common.BASE_MAX_LINEAR_MPS, "base linear.x m/s", s.caps)
        dur = a.distance / abs(speed)
        io.spin(1.0)
        for name, sign in (("forward", 1.0), ("back", -1.0)):
            gt0 = common.gz_pose() if a.target == "sim" and not a.dry_run else None
            o0 = io.odom_rows[-1] if io.odom_rows else None
            t = io.drive(pub, cmd_topic, sign * speed, 0.0, dur, f"base {name} {a.distance:.2f} m at {speed:.2f} m/s")
            io.spin_sim(a.pause_s)
            gt1 = common.gz_pose() if a.target == "sim" and not a.dry_run else None
            o1 = io.odom_rows[-1] if io.odom_rows else None
            commanded = abs(speed) * ((t[1] - t[0]) if t else dur)
            legs.append([name, sign * speed, dur, t[0] if t else None, t[1] if t else None, commanded,
                         *(gt0["xyz"][:2] + [gt0["rpy"][2]] if gt0 else [math.nan] * 3), *(gt1["xyz"][:2] + [gt1["rpy"][2]] if gt1 else [math.nan] * 3),
                         *(o0[1:] if o0 else [math.nan] * 3), *(o1[1:] if o1 else [math.nan] * 3)])
            metrics.append(("T2", f"{name}_commanded_m", commanded, "m", "speed x published duration (node clock)"))
            if gt0 and gt1:
                metrics += leg_metrics(name, gt0, gt1, commanded, o0, o1)
        common.write_csv(s.path("t2_base_straight.csv"), ["leg", "linear_x_mps", "planned_s", "t_start_s", "t_end_s", "commanded_m",
                         "gt0_x", "gt0_y", "gt0_yaw", "gt1_x", "gt1_y", "gt1_yaw", "odom0_x", "odom0_y", "odom0_yaw", "odom1_x", "odom1_y", "odom1_yaw"], legs)
        io.dump_odom(s.path("odom.csv"))
        common.write_metrics(s, metrics)
    finally:
        guard.stop()
        s.write_meta(finished=True)
        io.close()
    print(s.dir)


if __name__ == "__main__":
    main()
