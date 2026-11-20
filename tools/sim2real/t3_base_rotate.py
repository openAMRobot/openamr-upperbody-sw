#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""T3 base rotation: +90 deg and +180 deg in place at 0.5 rad/s, both arms in park (AD-30). docs/sim_to_real_protocol.md, T3.

Commanded: a capped Twist (angular.z only) for angle / rate seconds of the node's clock, then zero. Recorded: odometry yaw (never the
reference), and in sim the Gazebo ground-truth yaw before and after each turn. On the bench the reference is floor marks under the
two front corners of base_link's collision box, the angle by common.floor_turn_angle (templates/t3_base_rotate_manual.csv).
"""
import math

import common


def main(argv=None):
    p = common.base_parser(__doc__, robot_required=[("--cmd-vel-topic", "geometry_msgs/Twist topic of the base."),
                                                   ("--odom-topic", "nav_msgs/Odometry topic of the base (recorded, not a reference).")])
    p.add_argument("--rate", type=float, default=0.5)
    p.add_argument("--angles-deg", default="90,180")
    p.add_argument("--pause-s", type=float, default=2.0)
    a = common.parse(p, argv)
    s = common.Session("t3_base_rotate", a)
    guard = common.StopGuard()
    import rosio
    io = rosio.Io(s, guard, "sim2real_t3")
    cmd_topic = a.cmd_vel_topic or common.SIM_CMD_VEL
    odom_topic = a.odom_topic or common.SIM_ODOM
    io.record_odom(odom_topic)
    pub = io.twist_publisher(cmd_topic)
    metrics, turns = [], []
    try:
        io.wait_for_joint_states(rosio.arm_joints("left") + rosio.arm_joints("right"), 60)
        ok, wj, err = rosio.park_check(io)
        s.write_meta(park_check={"ok": ok, "worst_joint": wj, "error_rad": err}, cmd_vel_topic=cmd_topic, odom_topic=odom_topic)
        if not ok and not a.dry_run:
            raise SystemExit(f"AD-30: arms not in park ({wj} off by {err:.4f} rad); base motion refused")
        rate = common.clamp_abs(a.rate, common.BASE_MAX_ANGULAR_RADPS, "base angular.z rad/s", s.caps)
        io.spin(1.0)
        for deg in [float(x) for x in a.angles_deg.split(",")]:
            ang = math.radians(deg)
            dur = ang / abs(rate)
            gt0 = common.gz_pose() if a.target == "sim" and not a.dry_run else None
            i0 = len(io.odom_rows)
            t = io.drive(pub, cmd_topic, 0.0, rate, dur, f"base turn {deg:.0f} deg at {rate:.2f} rad/s")
            io.spin_sim(a.pause_s)
            gt1 = common.gz_pose() if a.target == "sim" and not a.dry_run else None
            commanded = abs(rate) * ((t[1] - t[0]) if t else dur)
            od = common.unwrap_sum([r[3] for r in io.odom_rows[max(0, i0 - 1):]]) if len(io.odom_rows) > i0 else math.nan
            # ground truth: wrap(after - before); for 180 deg the sign is taken from the command (a 180 deg turn wraps to -pi)
            gtd = common.wrap(gt1["rpy"][2] - gt0["rpy"][2]) if gt0 and gt1 else math.nan
            if math.isfinite(gtd) and deg >= 179 and gtd < 0:
                gtd += 2 * math.pi
            turns.append([deg, rate, dur, t[0] if t else None, t[1] if t else None, commanded, gt0["rpy"][2] if gt0 else math.nan,
                          gt1["rpy"][2] if gt1 else math.nan, gtd, od])
            tag = f"turn{deg:.0f}"
            metrics.append(("T3", f"{tag}_commanded_rad", commanded, "rad", "rate x published duration (node clock)"))
            if math.isfinite(gtd):
                metrics.append(("T3", f"{tag}_sim_over_commanded", gtd / commanded, "1", "gz model -p yaw / commanded"))
                metrics.append(("T3", f"{tag}_odom_over_gt", od / gtd if gtd else math.nan, "1", "odometry yaw / gz model -p yaw"))
                metrics.append(("T3", f"{tag}_gt_position_drift_m", math.hypot(gt1["xyz"][0] - gt0["xyz"][0], gt1["xyz"][1] - gt0["xyz"][1]), "m", "gz model -p"))
        common.write_csv(s.path("t3_base_rotate.csv"), ["angle_deg", "angular_z_radps", "planned_s", "t_start_s", "t_end_s", "commanded_rad",
                         "gt_yaw0", "gt_yaw1", "gt_turn_rad", "odom_turn_rad"], turns)
        io.dump_odom(s.path("odom.csv"))
        common.write_metrics(s, metrics)
    finally:
        guard.stop()
        s.write_meta(finished=True)
        io.close()
    print(s.dir)


if __name__ == "__main__":
    main()
