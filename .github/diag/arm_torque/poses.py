# SPDX-License-Identifier: MIT
"""Scratch (not for merge): the ground truth base pose, lift and item at the start of arm_approach of a recorded
Baseline 1 mission, plus the mission's own numbers for the reach. Usage: poses.py <artifact dir> <label> <j1 end>
Writes <label>_pose=x,y,yaw to $GITHUB_OUTPUT when set."""
import glob
import math
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from common import at, bag_messages, quantities, yaw  # noqa: E402

root, label, j1_end = sys.argv[1], sys.argv[2], sys.argv[3]
mission = sorted(glob.glob(root + "/**/baseline1_mission.json", recursive=True))[0]
q, data = quantities(mission)
phase = {p["phase"]: p for p in data["phases"]}
start = phase["arm_approach"]["sim_start"]
print(label, "mission", mission, "result", data.get("result"), "arm_approach", start, phase["arm_approach"].get("sim_end"))
for name in ("nav_station_error_along_heading", "nav_station_error_left", "station_reach_yaw_error", "nav_ground_truth_yaw_error",
             "lift_stop_0_gazebo_position", "grasp_face_yaw", "tcp_depth_along_fingers", "grasp_approach_servo_torque_max",
             "grasp_reach_cartesian_fraction", "grasp_reach_tracking_error_max", "grasp_reach_servo_torque_max",
             "item_pushed_before_closing", "finger_position_on_item"):
    if name in q:
        print(label, "Q", name, q[name]["value"], q[name]["unit"], q[name].get("note", "")[:80])
for e in data.get("events", []):
    if any(k in e["text"] for k in ("grasp_reach", "seed", "planning", "pose source")):
        print(label, "EVENT", e["text"][:400])
rows = bag_messages(root, ["/build1/ground_truth/odom", "/sil/gz/joint_states", "/build1/ground_truth/item/pose",
                           "/build1/rtf_stats"])
odom = at(rows["/build1/ground_truth/odom"], start)[1].pose.pose
end_odom = rows["/build1/ground_truth/odom"][-1]
x, y, th = odom.position.x, odom.position.y, yaw(odom.orientation)
print(label, "BASE at arm_approach start x %.5f y %.5f z %.5f yaw %.5f" % (x, y, odom.position.z, th))
later = [at(rows["/build1/ground_truth/odom"], start + dt) for dt in (5.0, 20.0, 40.0)]
for dt, r in zip((5.0, 20.0, 40.0), later):
    if r:
        p = r[1].pose.pose
        print(label, "BASE at +%.0f s x %.5f y %.5f yaw %.5f" % (dt, p.position.x, p.position.y, yaw(p.orientation)))
lift = at(rows["/sil/gz/joint_states"], start)
if lift:
    m = lift[1]
    print(label, "LIFT", dict(zip(m.name, [round(v, 5) for v in m.position])).get("lift_joint"))
item = at(rows["/build1/ground_truth/item/pose"], start)
if item:
    p, o = item[1].pose.position, item[1].pose.orientation
    print(label, "ITEM", round(p.x, 5), round(p.y, 5), round(p.z, 5), "yaw", round(yaw(o), 5))
rtf = [m for t, m in rows["/build1/rtf_stats"] if start <= t <= start + 60]
if rtf:
    print(label, "RTF sample", str(rtf[len(rtf) // 2])[:300])
out = os.environ.get("GITHUB_OUTPUT")
if out:
    with open(out, "a") as f:
        f.write(f"{label}_pose={x:.5f},{y:.5f},{th:.5f}\n")
cans = sorted(glob.glob(root + "/**/record_candump_vcan_arm_l*.log", recursive=True))
print(label, "candump", cans)
if cans:
    os.system(f"python3 -P {os.path.join(os.path.dirname(__file__), 'can_hold.py')} {cans[0]} {j1_end}")
