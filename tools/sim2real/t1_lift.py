#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""T1 lift: L1000 -> L1175 -> L1350 -> L1000 with both arms in park (docs/sim_to_real_protocol.md, T1).

sim    the script commands lift_controller (FollowJointTrajectory, mock and gazebo profiles only) at the lift speed datum
       lift_max_velocity_dold_v4_350, and records /joint_states lift_joint and Gazebo ground truth of lift_carriage_link and
       adapter_plate_link at each stop.
robot  the script does NOT command the lift: the real profile has no lift interface (ros2_control.xacro, openamr_lift_ros2_control:
       "real nothing"; the lift node firmware owns it, CTL-017; interlock platform-fw #20). The operator moves the lift through the
       authoritative controller; the script asks for Enter at the start and the end of each move (wall-clock travel time) and records
       --lift-state-topic if one exists ('none' if it does not). Tape heights go into templates/t1_lift_manual.csv.
"""
import math
import time

import common

STOPS = [("L1000", 0.0), ("L1175", 0.175), ("L1350", 0.350), ("L1000", 0.0)]   # SRDF group_state L1000/L1175/L1350 (lift_joint m)


def main(argv=None):
    p = common.base_parser(__doc__, robot_required=[("--lift-state-topic", "sensor_msgs/JointState topic with lift_joint, or 'none'.")])
    p.add_argument("--park-first", action="store_true", help="sim: command both arms to park (scaling 0.1) if they are not there")
    p.add_argument("--hold-s", type=float, default=3.0)
    a = common.parse(p, argv)
    s = common.Session("t1_lift", a)
    guard = common.StopGuard()
    import rosio
    io = rosio.Io(s, guard, "sim2real_t1_lift")
    rows, metrics, events = [], [], []
    try:
        io.wait_for_joint_states(rosio.arm_joints("left") + rosio.arm_joints("right"), 60)
        urdf = io.wait_for_urdf()
        joints, _ = common.load_urdf_joints(urdf) if urdf else ({}, {})
        ok, wj, err = rosio.park_check(io)
        if not ok and a.park_first and a.target == "sim":
            vel = [joints[n]["velocity"] for n in rosio.arm_joints("left")]
            for side in ("left", "right"):
                events.append(("park", side, rosio.move_arm(io, side, rosio.PARK, common.ARM_MAX_VELOCITY_SCALING, vel, f"{side} arm to park")))
            io.spin_sim(1.0)
            ok, wj, err = rosio.park_check(io)
        s.write_meta(park_check={"ok": ok, "worst_joint": wj, "error_rad": err})
        if not ok and not a.dry_run:
            raise SystemExit(f"AD-30: arms not in park ({wj} off by {err:.4f} rad); lift motion refused")
        speed = common.datum("lift_max_velocity_dold_v4_350")
        # The bench reference face is the TOP FACE of adapter_plate_link (the face arm_profile_link stands on). adapter_plate_link is on a
        # fixed joint, so Gazebo lumps it into lift_carriage_link and has no pose of its own: its top face is lift_carriage_link's z plus
        # adapter_plate_joint's z plus half the plate's box height, both read from /robot_description.
        plate_offset = math.nan
        if joints and "adapter_plate_joint" in joints:
            import xml.etree.ElementTree as ET
            box = ET.fromstring(urdf).find("link[@name='adapter_plate_link']/collision/geometry/box")
            plate_offset = joints["adapter_plate_joint"]["xyz"][2] + float(box.get("size").split()[2]) / 2.0
        s.write_meta(plate_top_offset_from_carriage_m=plate_offset)
        for name, z in STOPS:
            q0 = io.js_last.get("lift_joint", (math.nan,))[0]
            dur = max(1.0, abs(z - q0) / speed) if math.isfinite(q0) else abs(0.175) / speed
            what = f"lift to {name} ({z:.3f} m) in {dur:.2f} s"
            if a.target == "sim":
                r = io.follow_joint_trajectory(common.LIFT_ACTION_SIM, ["lift_joint"], [([z], dur)], what, result_wall_s=4 * dur + 60)
                t_sent = r[3]
                # arrival: first /joint_states sample within 1 mm of the stop after the goal was sent
                arr = next((t for t, d in io.js_rows if t_sent is not None and t >= t_sent and "lift_joint" in d and abs(d["lift_joint"][0] - z) <= 1e-3), None)
                io.spin_sim(a.hold_s)
                gt = {}
                if not a.dry_run:
                    try:
                        gt["lift_carriage_link"] = common.gz_pose("lift_carriage_link")
                    except (ValueError, OSError) as e:
                        events.append(("gz_pose failed", name, str(e)))
                q = io.js_last.get("lift_joint", (math.nan,))[0]
                plate_top_gt = gt["lift_carriage_link"]["xyz"][2] + plate_offset if gt else math.nan
                rows.append([name, z, dur, r[0], r[1], r[2], t_sent, arr, (arr - t_sent) if (arr and t_sent) else math.nan, q,
                             gt.get("lift_carriage_link", {}).get("xyz", [math.nan] * 3)[2], plate_top_gt])
            else:
                if a.dry_run:
                    s.log_sent(what, {"operator_moves_lift": True})
                    continue
                common.confirm(a, what + " THROUGH THE AUTHORITATIVE LIFT CONTROLLER (this script sends nothing)")
                input("press Enter when the lift STARTS moving: ")
                t0 = time.time()
                input("press Enter when the lift STOPS: ")
                t1 = time.time()
                io.spin(a.hold_s)
                q = io.js_last.get("lift_joint", (math.nan,))[0]
                rows.append([name, z, dur, None, None, "operator", t0, t1, t1 - t0, q, math.nan, math.nan])
        common.write_csv(s.path("t1_lift.csv"), ["stop", "target_m", "planned_duration_s", "accepted", "error_code", "error_string", "t_sent_s",
                                                  "t_arrived_s", "travel_time_s", "lift_joint_m", "gt_lift_carriage_z_m", "gt_plate_top_z_m"], rows)
        io.dump_joint_states(s.path("joint_states.csv"), ["lift_joint"] + rosio.arm_joints("left") + rosio.arm_joints("right"))
        prev = 0.0
        for r in rows:
            stop, z, tt = r[0], r[1], r[8]
            if math.isfinite(tt or math.nan) and abs(z - prev) > 0:
                metrics.append(("T1", f"speed_ratio_{stop}_from_{prev:.3f}", abs(z - prev) / tt / speed, "1", "joint_states arrival / datum lift_max_velocity_dold_v4_350"))
            metrics.append(("T1", f"lift_joint_error_{stop}_{z:.3f}", r[9] - z, "m", "joint_states lift_joint - stop"))
            if math.isfinite(r[11]):
                metrics.append(("T1", f"gt_plate_top_z_{stop}_{z:.3f}", r[11], "m", "gz lift_carriage_link z + adapter_plate_joint z + plate half height (floor = world z 0)"))
            prev = z
        common.write_metrics(s, metrics)
    finally:
        guard.stop()
        s.write_meta(events=[str(e) for e in events], finished=True)
        io.close()
    print(s.dir)


if __name__ == "__main__":
    main()
