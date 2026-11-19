# SPDX-License-Identifier: MIT
"""Grocery run 44299879412, second pass: command chain, device states, arm, fingers, RTF and item contact forces."""
import sys, json, collections, math
from pathlib import Path
import numpy as np
from rosbags.highlevel import AnyReader
S = collections.defaultdict(list)
LJ = [f"openarm_left_joint{i}" for i in range(1, 8)]
T = {"/cmd_vel_nav", "/cmd_vel_smoothed", "/cmd_vel", "/sil/lift/status", "/sil/gz/joint_states", "/arm/joint_states",
     "/build1/ground_truth/finger_joint_states", "/sil/drive/status", "/sil/bms/status", "/build1/rtf_stats",
     "/build1/contacts/world_item", "/sil/lift/command", "/sil/lift/permit", "/left_arm_controller/controller_state"}
pairs = collections.Counter()
with AnyReader([Path(sys.argv[1])]) as r:
    conns = [c for c in r.connections if c.topic in T]
    for c, t, raw in r.messages(connections=conns):
        t /= 1e9; tp = c.topic
        if tp == "/left_arm_controller/controller_state":
            if int(t * 25) == int((t - 0.0005) * 25): continue   # keep about 25 Hz
        m = r.deserialize(raw, c.msgtype)
        if tp.startswith("/cmd_vel"): S[tp].append((t, m.linear.x, m.angular.z))
        elif tp == "/sil/lift/status":
            d = json.loads(m.data); S[tp].append((t, d["position_mm"], d["velocity_mm_s"], d["state"], int(bool(d["permit"])), d["motor_current_ma"], d["last_seq"], d["cmd_result"]))
        elif tp == "/sil/gz/joint_states": S[tp].append((t, m.position[list(m.name).index("lift_joint")]))
        elif tp == "/arm/joint_states":
            n = list(m.name); idx = [n.index(j) for j in LJ]
            S[tp].append((t, *[m.position[i] for i in idx], *[m.effort[i] for i in idx], m.position[n.index("openarm_left_finger_joint1")]))
        elif tp == "/build1/ground_truth/finger_joint_states": S[tp].append((t, *m.position[:4]))
        elif tp == "/sil/drive/status":
            d = json.loads(m.data); S[tp].append((t, d["statusword"], d["target_rpm"][0], d["target_rpm"][1], d["wheel_rad_s"][0], d["wheel_rad_s"][1], d["fault_code"], d["x"], d["y"], d["yaw"], d["v"], d["w"]))
        elif tp == "/sil/bms/status":
            d = json.loads(m.data); S[tp].append((t, d["soc_percent"], d["pack_voltage_v"], d["current_a"]))
        elif tp == "/build1/rtf_stats":
            d = json.loads(m.data); S[tp].append((t, d["rtf_last"], d["rtf_mean"], d["sim_s"], d["wall_s"]))
        elif tp == "/sil/lift/command": S[tp].append((t, m.data))
        elif tp == "/sil/lift/permit": S[tp].append((t, int(m.data)))
        elif tp == "/left_arm_controller/controller_state":
            e = list(m.error.positions) if len(m.error.positions) else [np.nan] * 7
            S[tp].append((t, *e[:7]))
        elif tp == "/build1/contacts/world_item":
            fin = tray = 0.0
            for ct in m.contacts:
                a, b = ct.collision1.name, ct.collision2.name; pairs[(a, b)] += 1
                f = sum(math.sqrt(w.body_1_wrench.force.x ** 2 + w.body_1_wrench.force.y ** 2 + w.body_1_wrench.force.z ** 2) for w in ct.wrenches)
                if "openarm" in a + b: fin += f
                elif "tote" in a + b: tray += f
            S["contacts"].append((t, fin, tray))
for k, v in S.items(): print(k, len(v), v[0][0], v[-1][0])
for k, v in pairs.most_common(12): print(v, k)
np.savez_compressed(sys.argv[2], **{k.strip("/").replace("/", "__"): np.array(v, dtype=float) for k, v in S.items()})
