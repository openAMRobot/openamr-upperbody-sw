#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""DIAGNOSTIC ONLY (upperbody #136). Reads the recorded bags of finished mission legs and separates three things:
ground truth motion, wheel-kinematic motion (wheel joint positions of the plant) and /odom. No ROS needed (rosbags)."""
import math
import sys
from pathlib import Path

from rosbags.highlevel import AnyReader

TRUTH, ODOM = "/build1/ground_truth/odom", "/odom"
JOINTS = ("/sil/gz/joint_states", "/base/joint_states", "/joint_states")
SIL = ("/sil/base/odom", "/cmd_vel", "/sil/fw/wheel_speeds")


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def unwrap(values):
    out, offset, last = [], 0.0, None
    for v in values:
        if last is not None:
            d = v - last
            if d > math.pi:
                offset -= 2 * math.pi
            elif d < -math.pi:
                offset += 2 * math.pi
        last = v
        out.append(v + offset)
    return out


def nearest(series, t, start=0):
    i = start
    while i + 1 < len(series) and abs(series[i + 1][0] - t) <= abs(series[i][0] - t):
        i += 1
    return i


def analyse(bag):
    print(f"\n===== BAG {bag}")
    with AnyReader([bag]) as reader:
        counts = {c.topic: c.msgcount for c in reader.connections}
        for topic in sorted(counts):
            if any(k in topic for k in ("odom", "joint_states", "cmd_vel", "wheel", "clock", "sil/base", "amcl", "tf")):
                print(f"topic {topic} n {counts[topic]}")
        wanted = [c for c in reader.connections if c.topic in (TRUTH, ODOM) + JOINTS + SIL]
        truth, odom, joints, sil_odom, cmd = [], [], {}, [], []
        for connection, stamp, raw in reader.messages(connections=wanted):
            try:
                msg = reader.deserialize(raw, connection.msgtype)
            except Exception as error:  # noqa: BLE001
                print(f"cannot read {connection.topic}: {error}")
                continue
            t = stamp * 1e-9
            hs = getattr(getattr(msg, "header", None), "stamp", None)
            sim = hs.sec + hs.nanosec * 1e-9 if hs is not None else float("nan")
            if connection.topic == TRUTH:
                p = msg.pose.pose
                truth.append((sim, p.position.x, p.position.y, yaw_of(p.orientation), msg.twist.twist.linear.x, msg.twist.twist.angular.z, t))
            elif connection.topic == ODOM:
                p = msg.pose.pose
                odom.append((sim, p.position.x, p.position.y, yaw_of(p.orientation), msg.twist.twist.linear.x, msg.twist.twist.angular.z, t))
            elif connection.topic in JOINTS:
                names = list(msg.name)
                if "left_wheel_joint" in names and "right_wheel_joint" in names:
                    joints.setdefault(connection.topic, []).append(
                        (sim, msg.position[names.index("left_wheel_joint")], msg.position[names.index("right_wheel_joint")]))
            elif connection.topic == "/sil/base/odom":
                sil_odom.append((sim, msg.twist.twist.linear.x, msg.twist.twist.angular.z, t))
            elif connection.topic == "/cmd_vel":
                tw = getattr(msg, "twist", msg)
                cmd.append((t, tw.linear.x, tw.angular.z))
    print(f"samples truth {len(truth)} odom {len(odom)} sil_base_odom {len(sil_odom)} cmd_vel {len(cmd)} "
          + " ".join(f"{k} {len(v)}" for k, v in joints.items()))
    if len(truth) < 10 or len(odom) < 10:
        print("not enough data")
        return
    truth.sort()
    odom.sort()
    ty, oy = unwrap([s[3] for s in truth]), unwrap([s[3] for s in odom])
    # rates in simulation time
    for name, series in (("truth", truth), ("odom", odom)):
        span = series[-1][0] - series[0][0]
        print(f"{name}: sim time {series[0][0]:.2f} to {series[-1][0]:.2f} s, mean rate {len(series) / max(span, 1e-9):.1f} Hz of sim time")
    if sil_odom:
        sil_odom.sort()
        gaps = sorted(b[0] - a[0] for a, b in zip(sil_odom, sil_odom[1:]))
        wall = sorted(b[3] - a[3] for a, b in zip(sil_odom, sil_odom[1:]))
        print(f"/sil/base/odom sample gap, sim s: median {gaps[len(gaps)//2]:.4f} p95 {gaps[int(len(gaps)*.95)]:.4f} max {gaps[-1]:.3f}; "
              f"wall s: median {wall[len(wall)//2]:.4f} max {wall[-1]:.3f}; gaps over 0.5 sim s: {sum(g > 0.5 for g in gaps)}")
    # wheel kinematics of the plant
    kin = None
    for topic in JOINTS:
        if topic in joints and len(joints[topic]) > 10:
            series = sorted(joints[topic])
            r, sep = 0.098, 0.400
            x = y = yaw = 0.0
            kin = [(series[0][0], 0.0, 0.0, 0.0)]
            for a, b in zip(series, series[1:]):
                dl, dr = (b[1] - a[1]) * r, (b[2] - a[2]) * r
                d, dyaw = (dl + dr) / 2.0, (dr - dl) / sep
                x += d * math.cos(yaw + dyaw / 2.0)
                y += d * math.sin(yaw + dyaw / 2.0)
                yaw += dyaw
                kin.append((b[0], x, y, yaw))
            print(f"wheel kinematics from {topic} (r {r}, separation {sep}): final x {x:.4f} y {y:.4f} yaw {yaw:.4f}")
            break
    # timeline every 2 s of sim time: truth, odom, kinematics relative to their own start
    t0 = max(truth[0][0], odom[0][0])
    i0, j0 = nearest(truth, t0), nearest(odom, t0)
    tx0, tyaw0 = (truth[i0][1], truth[i0][2]), ty[i0]
    ox0, oyaw0 = (odom[j0][1], odom[j0][2]), oy[j0]
    print(f"start offsets at sim {t0:.2f}: truth ({tx0[0]:.4f}, {tx0[1]:.4f}, yaw {tyaw0:.4f}) odom ({ox0[0]:.4f}, {ox0[1]:.4f}, yaw {oyaw0:.4f})")
    print("sim_t  truth_x truth_y truth_yaw | odom_x odom_y odom_yaw | abs_pos_diff yaw_diff | truth_v truth_w odom_v odom_w | kin_yaw")
    end, t, i, j, k = min(truth[-1][0], odom[-1][0]), t0, i0, j0, 0
    rows = []
    while t <= end:
        i, j = nearest(truth, t, i), nearest(odom, t, j)
        kyaw = float("nan")
        if kin:
            k = nearest(kin, t, k)
            kyaw = kin[k][3] - kin[nearest(kin, t0)][3]
        rows.append((t, truth[i][1], truth[i][2], ty[i], odom[j][1], odom[j][2], oy[j],
                     math.hypot(odom[j][1] - truth[i][1], odom[j][2] - truth[i][2]), oy[j] - ty[i],
                     truth[i][4], truth[i][5], odom[j][4], odom[j][5], kyaw))
        t += 2.0
    for row in rows:
        print(f"{row[0]:7.1f} {row[1]:7.3f} {row[2]:7.3f} {row[3]:7.3f} | {row[4]:7.3f} {row[5]:7.3f} {row[6]:7.3f} | {row[7]:6.4f} {row[8]:7.4f} | "
              f"{row[9]:6.3f} {row[10]:6.3f} {row[11]:6.3f} {row[12]:6.3f} | {row[13]:7.3f}")
    # where does the difference grow: standing, straight, turning (by ground truth speeds)
    buckets = {"standing": [0.0, 0.0, 0.0], "straight": [0.0, 0.0, 0.0], "turning": [0.0, 0.0, 0.0]}
    i = j = 0
    step = 0.1
    t = t0
    prev = None
    while t <= end:
        i, j = nearest(truth, t, i), nearest(odom, t, j)
        cur = (ty[i], oy[j], truth[i][1], truth[i][2], odom[j][1], odom[j][2], abs(truth[i][4]), abs(truth[i][5]))
        if prev is not None:
            kind = "standing" if prev[6] < 0.005 and prev[7] < 0.01 else ("turning" if prev[7] >= 0.05 else "straight")
            b = buckets[kind]
            b[0] += step
            b[1] += (cur[1] - prev[1]) - (cur[0] - prev[0])                     # odom yaw change minus truth yaw change
            b[2] += math.hypot(cur[4] - prev[4], cur[5] - prev[5]) - math.hypot(cur[2] - prev[2], cur[3] - prev[3])  # path length difference
        prev = cur
        t += step
    for kind, (seconds, dyaw, dpath) in buckets.items():
        print(f"bucket {kind:9s}: {seconds:7.1f} sim s, odom-minus-truth yaw change {dyaw:+.4f} rad, path length difference {dpath:+.4f} m")
    total_truth_turn = sum(abs(b - a) for a, b in zip(ty, ty[1:]))
    total_odom_turn = sum(abs(b - a) for a, b in zip(oy, oy[1:]))
    print(f"total absolute turning: truth {total_truth_turn:.3f} rad, odom {total_odom_turn:.3f} rad, ratio {total_odom_turn / max(total_truth_turn, 1e-9):.4f}")


def main():
    roots = [Path(p) for p in sys.argv[1:]]
    bags = sorted({m.parent for root in roots for m in root.rglob("metadata.yaml")})
    print(f"bags found: {len(bags)}")
    for bag in bags:
        try:
            analyse(bag)
        except Exception as error:  # noqa: BLE001
            print(f"analysis of {bag} failed: {error!r}")


if __name__ == "__main__":
    main()
