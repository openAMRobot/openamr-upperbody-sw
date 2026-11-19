# SPDX-License-Identifier: MIT
"""Scratch (not for merge): recorded facts of run 44448676604 that the Proposed converter rules need.

Prints one JSON document: topic inventory, first samples of every capture stream (origin rule), controller-state
excerpts (arm command extraction), the lift command, permit and state around each command (lift units, phase and
stowed-pose evidence), odom twist and arm pose while the lift moves, samples outside the model limits, and gaps of the
command streams (held commands). Read only. Simulation evidence only.
"""
import argparse
import json
from pathlib import Path

from rosbags.highlevel import AnyReader

PARK = {f"openarm_{s}_joint{i}": (0.1 if i == 4 else 0.0) for s in ("left", "right") for i in range(1, 8)}
STREAMS = ("/left_wrist_camera/image", "/right_wrist_camera/image", "/head_camera/rgbd/image", "/base_camera/rgbd/image",
           "/arm/joint_states", "/left_arm_controller/controller_state", "/build1/capture/gripper_command",
           "/right_arm_controller/controller_state", "/joint_states", "/sil/lift/command", "/odom", "/cmd_vel")
EXTRA = ("/sil/lift/permit", "/sil/lift/status", "/lift_controller/controller_state", "/sil/lift/emulator_state")


def stamp(msg):
    h = getattr(msg, "header", None)
    return None if h is None else int(h.stamp.sec) * 1_000_000_000 + int(h.stamp.nanosec)


def point(p):
    if p is None:
        return None
    return {k: [float(v) for v in getattr(p, k, [])] for k in ("positions", "velocities")}


def row(msg, receipt):
    r = {"receipt_ns": receipt, "stamp_ns": stamp(msg)}
    h = getattr(msg, "header", None)
    if h is not None:
        r["frame_id"] = h.frame_id
    if hasattr(msg, "joint_names"):
        r["joint_names"] = list(msg.joint_names)
        for f in ("reference", "feedback", "error", "output", "desired", "actual"):
            if hasattr(msg, f):
                r[f] = point(getattr(msg, f))
    if hasattr(msg, "name") and hasattr(msg, "position"):
        r["name"] = list(msg.name)
        r["position"] = [float(v) for v in msg.position]
    if hasattr(msg, "data") and not hasattr(msg, "height"):
        d = msg.data
        r["data"] = d if isinstance(d, (bool, int, float, str)) else str(d)[:400]
    if hasattr(msg, "linear"):
        r["linear"] = [msg.linear.x, msg.linear.y, msg.linear.z]
        r["angular"] = [msg.angular.x, msg.angular.y, msg.angular.z]
    tw = getattr(getattr(msg, "twist", None), "twist", None)
    if tw is not None:
        r["twist"] = [tw.linear.x, tw.linear.y, tw.linear.z, tw.angular.x, tw.angular.y, tw.angular.z]
    if hasattr(msg, "height") and hasattr(msg, "encoding"):
        r["image"] = [int(msg.height), int(msg.width), msg.encoding]
    return r


def main():
    p = argparse.ArgumentParser()
    p.add_argument("bag", type=Path)
    a = p.parse_args()
    rows = {t: [] for t in STREAMS + EXTRA}
    inventory = {}
    with AnyReader([a.bag]) as reader:
        for c in reader.connections:
            inventory.setdefault(c.topic, {"type": c.msgtype, "count": 0})
            inventory[c.topic]["count"] += c.msgcount
        conns = [c for c in reader.connections if c.topic in rows]
        for conn, receipt, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            r = row(msg, receipt)
            if "image" in r:  # keep stamps only
                r = {"receipt_ns": r["receipt_ns"], "stamp_ns": r["stamp_ns"], "image": r["image"]}
            rows[conn.topic].append(r)
    out = {"inventory": inventory}
    t = lambda r: r["stamp_ns"] if r["stamp_ns"] is not None else r["receipt_ns"]
    out["span"] = {k: [t(v[0]), t(v[-1]), len(v)] for k, v in rows.items() if v}
    # first samples of every stream (origin rule)
    out["first"] = {k: [{"receipt_ns": r["receipt_ns"], "stamp_ns": r["stamp_ns"]} for r in v[:8]] for k, v in rows.items()}
    # controller-state excerpts
    for k in ("/left_arm_controller/controller_state", "/right_arm_controller/controller_state",
              "/lift_controller/controller_state"):
        v = rows[k]
        out.setdefault("controller_excerpt", {})[k] = v[:3] + ([v[len(v) // 2]] if v else [])
        diffs = 0
        for r in v:
            ref, fb = r.get("reference") or {}, r.get("feedback") or {}
            if ref.get("positions") and len(ref["positions"]) != len(r["joint_names"]):
                diffs += 1
        out.setdefault("controller_ref_len_mismatch", {})[k] = diffs
        out.setdefault("controller_ref_empty", {})[k] = sum(1 for r in v if not (r.get("reference") or {}).get("positions"))
    # lift command, permit, status
    out["lift_command"] = rows["/sil/lift/command"]
    out["lift_permit"] = rows["/sil/lift/permit"][:60]
    out["lift_permit_transitions"] = [r for i, r in enumerate(rows["/sil/lift/permit"])
                                      if i == 0 or r.get("data") != rows["/sil/lift/permit"][i - 1].get("data")]
    out["lift_status_excerpt"] = rows["/sil/lift/status"][:3]
    js = [(r["stamp_ns"], r["receipt_ns"], r["position"][r["name"].index("lift_joint")])
          for r in rows["/joint_states"] if "lift_joint" in r.get("name", [])]
    out["lift_joint_names_first"] = rows["/joint_states"][0]["name"] if rows["/joint_states"] else None
    moving = [(a1, b1) for (a0, _, b0), (a1, _, b1) in zip(js, js[1:]) if b0 != b1]
    out["lift_moving_samples"] = len(moving)
    # moving intervals (gap > 1 s splits)
    intervals = []
    for s, v in moving:
        if intervals and s - intervals[-1][1] <= 1_000_000_000:
            intervals[-1][1] = s
        else:
            intervals.append([s, s])
    out["lift_moving_intervals"] = intervals
    # tiny-change filter: changes > 1e-4 m
    big = [(a1, b1 - b0) for (a0, _, b0), (a1, _, b1) in zip(js, js[1:]) if abs(b1 - b0) > 1e-5]
    bi = []
    for s, _ in big:
        if bi and s - bi[-1][1] <= 1_000_000_000:
            bi[-1][1] = s
        else:
            bi.append([s, s])
    out["lift_moving_intervals_gt_1e-5"] = bi
    out["lift_below_zero"] = {"count": sum(1 for x in js if x[2] < 0), "min": min((x[2] for x in js), default=None),
                              "max": max((x[2] for x in js), default=None),
                              "first": [x for x in js if x[2] < 0][:5]}
    around = []
    for cmd in rows["/sil/lift/command"]:
        c = cmd["receipt_ns"]
        around.append({"command": cmd, "lift_before": [x for x in js if c - 300_000_000 <= x[0] <= c][-3:],
                       "lift_after_5s": [x for x in js if c < x[0] <= c + 5_000_000_000][::25]})
    out["lift_around_commands"] = around
    # odom twist and arm pose while the lift moves (every moving sample)
    odom = [(r["stamp_ns"], r["twist"]) for r in rows["/odom"]]
    arm = [(r["stamp_ns"], dict(zip(r["name"], r["position"]))) for r in rows["/arm/joint_states"]]
    import bisect
    ot = [o[0] for o in odom]
    at = [x[0] for x in arm]
    maxtw = [0.0] * 6
    nonzero = 0
    maxdev = {}
    for s, _ in moving:
        i = min(max(bisect.bisect_left(ot, s), 0), len(ot) - 1)
        tw = odom[i][1]
        nonzero += any(v != 0 for v in tw)
        maxtw = [max(m, abs(v)) for m, v in zip(maxtw, tw)]
        j = min(max(bisect.bisect_left(at, s), 0), len(at) - 1)
        for n, target in PARK.items():
            maxdev[n] = max(maxdev.get(n, 0.0), abs(arm[j][1].get(n, float("nan")) - target))
    out["lift_moving_odom"] = {"max_abs_twist": maxtw, "nonzero_samples": nonzero, "moving": len(moving)}
    out["lift_moving_arm_max_dev_from_park_rad"] = maxdev
    # outside model limits on /arm/joint_states (joint4 lower 0)
    neg = {}
    for s, d in arm:
        for n in ("openarm_left_joint4", "openarm_right_joint4"):
            if d.get(n, 0) < 0:
                e = neg.setdefault(n, {"count": 0, "min": 0.0, "first": None, "last": None})
                e["count"] += 1
                e["min"] = min(e["min"], d[n])
                e["first"] = e["first"] or s
                e["last"] = s
    out["joint4_negative"] = neg
    out["arm_rows_joint4_negative_excerpt"] = [r for r in rows["/arm/joint_states"]
                                               if r["position"][r["name"].index("openarm_left_joint4")] < 0][:2]
    out["lift_rows_negative_excerpt"] = [r for r in rows["/joint_states"]
                                         if "lift_joint" in r["name"] and r["position"][r["name"].index("lift_joint")] < 0][:2]
    # gaps of command and state streams
    for k in ("/cmd_vel", "/build1/capture/gripper_command", "/sil/lift/command", "/left_arm_controller/controller_state",
              "/odom", "/joint_states", "/arm/joint_states") + STREAMS[:4]:
        ts = [t(r) for r in rows[k]]
        gaps = sorted(((b - a, a) for a, b in zip(ts, ts[1:])), reverse=True)[:6]
        out.setdefault("largest_gaps", {})[k] = gaps
    out["cmd_vel_gaps_gt_100ms"] = [(a, b - a) for a, b in zip([t(r) for r in rows["/cmd_vel"]],
                                                                [t(r) for r in rows["/cmd_vel"]][1:]) if b - a > 100_000_000][:40]
    out["cmd_vel_last"] = rows["/cmd_vel"][-3:]
    out["gripper_first"] = rows["/build1/capture/gripper_command"][:2]
    print("=== PROBE JSON")
    print(json.dumps(out))
    print("=== END")


if __name__ == "__main__":
    main()
