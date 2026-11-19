# SPDX-License-Identifier: MIT
"""Scratch (not for merge): recorded excerpts of run 44448676604 for the converter tests of the Proposed rules.

Copies consecutive recorded messages, unchanged apart from images (shape and stamps only), into one JSON fixture:
the first second of the cameras, /arm/joint_states and the held gripper command (episode origin), controller-state
messages (arm commands), the lift command with the lift state around each command and at the end of the run (lift
units and phase), /cmd_vel around each gap (held commands) and samples just outside the model limits (tolerance).
Read only. Simulation evidence only.
"""
import argparse
import json
from pathlib import Path

from rosbags.highlevel import AnyReader

S = 1_000_000_000
CAMERAS = ("/left_wrist_camera/image", "/right_wrist_camera/image", "/head_camera/rgbd/image", "/base_camera/rgbd/image")
ORIGIN_WINDOW = (6_000_000_000, 7_000_000_000)


def stamp(msg):
    h = getattr(msg, "header", None)
    return None if h is None else int(h.stamp.sec) * S + int(h.stamp.nanosec)


def row(msg, receipt):
    r = {"receipt_ns": receipt}
    h = getattr(msg, "header", None)
    if h is not None:
        r["stamp_ns"], r["frame_id"] = stamp(msg), h.frame_id
    if hasattr(msg, "joint_names"):
        r["joint_names"] = list(msg.joint_names)
        for f in ("reference", "feedback"):
            p = getattr(msg, f)
            r[f] = {"positions": [float(v) for v in p.positions], "velocities": [float(v) for v in p.velocities]}
    elif hasattr(msg, "height") and hasattr(msg, "encoding"):
        r.update(height=int(msg.height), width=int(msg.width), encoding=msg.encoding)
    elif hasattr(msg, "name") and hasattr(msg, "position"):
        r.update(name=list(msg.name), position=[float(v) for v in msg.position])  # velocity, effort not used
    elif hasattr(msg, "linear"):
        r.update(linear=[msg.linear.x, msg.linear.y, msg.linear.z], angular=[msg.angular.x, msg.angular.y, msg.angular.z])
    elif hasattr(msg, "data"):
        r["data"] = msg.data if isinstance(msg.data, (bool, int, float)) else str(msg.data)
    return r


def main():
    p = argparse.ArgumentParser()
    p.add_argument("bag", type=Path)
    a = p.parse_args()
    wanted = CAMERAS + ("/arm/joint_states", "/build1/capture/gripper_command", "/left_arm_controller/controller_state",
                        "/right_arm_controller/controller_state", "/joint_states", "/sil/lift/command", "/cmd_vel")
    rows = {t: [] for t in wanted}
    defs = {}
    with AnyReader([a.bag]) as reader:
        for c in reader.connections:
            if c.topic in wanted:
                defs[c.topic] = {"type": c.msgtype, "msgdef": getattr(c, "msgdef", None) and c.msgdef.data,
                                 "msgdef_encoding": getattr(c, "msgdef", None) and c.msgdef.format.name}
        for conn, receipt, raw in reader.messages(connections=[c for c in reader.connections if c.topic in wanted]):
            rows[conn.topic].append(row(reader.deserialize(raw, conn.msgtype), receipt))
    key = lambda r: r.get("stamp_ns", r["receipt_ns"])
    within = lambda rs, lo, hi: [r for r in rs if lo <= key(r) <= hi]
    out = {"_provenance": {
        "source": "openAMRobot/openamr-upperbody-sw Actions run 44448676604 (Baseline 1, opt-in AI capture, sim-oracle, "
                  "simulation time), artifact baseline1-acceptance-attempt1, bag ws/build1_logs/bag",
        "extracted_by": "openAMRobot/openamr-upperbody-sw branch scratch/ai-capture-e2e, tools/scratch/excerpts_b1.py",
        "content": "consecutive recorded messages; images reduced to stamps and shape, JointState to name and position"},
        "message_definitions": defs}
    for t in CAMERAS + ("/arm/joint_states", "/build1/capture/gripper_command"):
        out[t] = within(rows[t], *ORIGIN_WINDOW)
    for t in ("/left_arm_controller/controller_state", "/right_arm_controller/controller_state"):
        moving = [r for r in rows[t] if any(abs(v) > 1e-3 for v in r["reference"]["velocities"])]
        out[t] = rows[t][:3] + moving[len(moving) // 2: len(moving) // 2 + 5]
    out["/sil/lift/command"] = rows["/sil/lift/command"]
    lift = [r for r in rows["/joint_states"] if "lift_joint" in r["name"]]
    windows = [(c["receipt_ns"] - S // 5, c["receipt_ns"] + S // 2) for c in rows["/sil/lift/command"]]
    windows += [(62_900_000_000, 63_200_000_000), (115_700_000_000, 116_000_000_000)]
    out["/joint_states"] = [{"window_ns": list(w), "rows": within(lift, *w)} for w in windows]
    vel = rows["/cmd_vel"]
    gaps = [i for i in range(1, len(vel)) if vel[i]["receipt_ns"] - vel[i - 1]["receipt_ns"] > S // 10]
    out["/cmd_vel"] = {"first": vel[:3], "last": vel[-3:],
                       "gaps": [{"before": vel[max(0, i - 4):i], "after": vel[i:i + 2]} for i in gaps]}
    arm = rows["/arm/joint_states"]
    j4 = lambda r: min(r["position"][r["name"].index(n)] for n in ("openarm_left_joint4", "openarm_right_joint4"))
    out["limits"] = {"/arm/joint_states": [r for r in arm if j4(r) < 0][:3],
                     "/joint_states": [r for r in lift if r["position"][r["name"].index("lift_joint")] < 0][:3]}
    print("=== EXCERPTS JSON")
    print(json.dumps(out))
    print("=== END")


if __name__ == "__main__":
    main()
