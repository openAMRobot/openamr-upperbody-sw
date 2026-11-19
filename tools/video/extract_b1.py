# SPDX-License-Identifier: MIT
"""Extract the recorded series used by Video C from the Baseline 1 bag into one npz. Bag log time is Gazebo time."""
import sys, json, math, collections
from pathlib import Path
import numpy as np
from rosbags.highlevel import AnyReader

bag, out = Path(sys.argv[1]), sys.argv[2]
S = collections.defaultdict(list)
plans = []
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
LJ = [f"openarm_left_joint{i}" for i in range(1, 8)]
topics = {"/build1/ground_truth/odom", "/odom", "/amcl_pose", "/plan", "/cmd_vel", "/sil/lift/status", "/sil/gz/joint_states",
          "/sil/lift/command", "/sil/lift/permit", "/arm/joint_states", "/left_arm_gravity_controller/commands",
          "/build1/ground_truth/finger_joint_states", "/build1/ground_truth/item/pose", "/build1/ground_truth/tote/pose",
          "/build1/rtf_stats", "/sil/drive/status", "/sil/bms/status", "/clock"}
with AnyReader([bag]) as r:
    conns = [c for c in r.connections if c.topic in topics]
    for c, t, raw in r.messages(connections=conns):
        t = t / 1e9; tp = c.topic
        if tp == "/clock":
            continue
        m = r.deserialize(raw, c.msgtype)
        if tp in ("/build1/ground_truth/odom", "/odom"):
            p = m.pose.pose; S[tp].append((t, p.position.x, p.position.y, yaw(p.orientation), m.twist.twist.linear.x))
        elif tp == "/amcl_pose":
            p = m.pose.pose; S[tp].append((t, p.position.x, p.position.y, yaw(p.orientation)))
        elif tp == "/plan":
            plans.append((t, m.header.frame_id, np.array([(q.pose.position.x, q.pose.position.y) for q in m.poses])))
        elif tp == "/cmd_vel":
            S[tp].append((t, m.linear.x, m.angular.z))
        elif tp == "/sil/lift/status":
            d = json.loads(m.data); S[tp].append((t, d["position_mm"], d["velocity_mm_s"], d["state"], int(bool(d["permit"])), d["motor_current_ma"]))
        elif tp == "/sil/gz/joint_states":
            S[tp].append((t, m.position[list(m.name).index("lift_joint")]))
        elif tp == "/sil/lift/command":
            S[tp].append((t, m.data))
        elif tp == "/sil/lift/permit":
            S[tp].append((t, int(m.data)))
        elif tp == "/arm/joint_states":
            n = list(m.name); idx = [n.index(j) for j in LJ]
            S[tp].append((t, *[m.position[i] for i in idx], *[m.effort[i] for i in idx], m.position[n.index("openarm_left_finger_joint1")]))
        elif tp == "/left_arm_gravity_controller/commands":
            S[tp].append((t, *m.data[:7]))
        elif tp == "/build1/ground_truth/finger_joint_states":
            S[tp].append((t, *m.position[:4]))
        elif tp in ("/build1/ground_truth/item/pose", "/build1/ground_truth/tote/pose"):
            p = m.pose.position; S[tp].append((t, p.x, p.y, p.z))
        elif tp == "/build1/rtf_stats":
            d = json.loads(m.data); S[tp].append((t, d["rtf_last"], d["rtf_mean"], d["sim_s"], d["wall_s"]))
        elif tp == "/sil/drive/status":
            d = json.loads(m.data); S[tp].append((t, d["statusword"], d["target_rpm"][0], d["target_rpm"][1], d["wheel_rad_s"][0], d["wheel_rad_s"][1], d["fault_code"], d["x"], d["y"], d["yaw"]))
        elif tp == "/sil/bms/status":
            d = json.loads(m.data); S[tp].append((t, d["soc_percent"], d["pack_voltage_v"], d["current_a"]))
arrs = {k.strip("/").replace("/", "__"): np.array(v, dtype=float) for k, v in S.items()}
for k, v in arrs.items(): print(k, v.shape, v[0, 0] if len(v) else None, v[-1, 0] if len(v) else None)
plan_t = np.array([p[0] for p in plans]); print("plans", len(plans), set(p[1] for p in plans), [len(p[2]) for p in plans][:10])
np.savez_compressed(out, plan_t=plan_t, plan_xy=np.array([p[2] for p in plans], dtype=object), **arrs)
