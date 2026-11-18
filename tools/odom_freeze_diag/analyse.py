# SPDX-License-Identifier: MIT
"""Diagnostics only (do not merge): the odometry chain in a finished grocery demo or Baseline 1 bag.

Reads one artifact directory (bag/*.mcap, mission JSON) and prints ODOMDIAG lines:
  - the topics the recorder discovered, with counts (a topic the recorder never discovered has no entry at all);
  - /rosout lines of the odometry chain nodes (base_state_adapter, sil_stm32_host_client, sil_zlac8015d);
  - /tf parent > child pairs with counts and publishers' frames;
  - ground truth path length, net rotation and final pose;
  - dead reckoning from the Gazebo wheel joint velocities (/sil/gz/joint_states) with r 0.098 m, separation 0.400 m,
    against ground truth: separates wheel slip (physics) from loss in the CAN or DDS chain;
  - dead reckoning from /sil/base/odom twist when recorded, against the same ground truth.
Simulation evidence only.
"""
import glob
import json
import math
import os
import sys
from collections import Counter, defaultdict

from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

R, S = 0.098, 0.400
CHAIN = ("base_state_adapter", "sil_stm32_host_client", "sil_zlac8015d", "wheel_command_adapter", "sil_ctl017")


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def out(tag, **kw):
    print("ODOMDIAG " + json.dumps({"tag": tag, **kw}, default=str), flush=True)


def main(directory):
    bags = sorted(glob.glob(os.path.join(directory, "**", "*.mcap"), recursive=True))
    out("files", directory=directory, bags=bags,
        json=[p for p in glob.glob(os.path.join(directory, "**", "*.json"), recursive=True)][:40])
    if not bags:
        return
    wanted = {"/rosout", "/tf", "/build1/ground_truth/odom", "/sil/gz/joint_states", "/sil/base/odom", "/odom",
              "/cmd_vel", "/amcl_pose", "/sil/drive/status", "/base/joint_states"}
    counts = Counter()
    rosout = Counter()
    tf_pairs = Counter()
    gt = []          # (t, x, y, yaw)
    wheels = []      # (t, wl, wr)
    sil = []         # (t, v, w)
    odom = []        # (t, x, y, yaw)
    cmd_nonzero = 0
    drive_status = []
    with open(bags[0], "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        summary = reader.get_summary()
        channels = {c.id: c.topic for c in summary.channels.values()}
        stats = summary.statistics.channel_message_counts if summary.statistics else {}
        out("recorded_topics", topics={channels[i]: n for i, n in sorted(stats.items(), key=lambda kv: channels[kv[0]])})
        for schema, channel, message, msg in reader.iter_decoded_messages(topics=sorted(wanted)):
            topic = channel.topic
            counts[topic] += 1
            t = message.log_time * 1e-9
            if topic == "/rosout":
                if any(n in msg.name for n in CHAIN) or "odom" in msg.msg.lower():
                    rosout[(msg.name, msg.level, msg.msg[:200])] += 1
            elif topic == "/tf":
                for tr in msg.transforms:
                    tf_pairs[(tr.header.frame_id, tr.child_frame_id)] += 1
            elif topic == "/build1/ground_truth/odom":
                p = msg.pose.pose
                gt.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, p.position.x, p.position.y, yaw_of(p.orientation)))
            elif topic == "/sil/gz/joint_states":
                names = list(msg.name)
                if "left_wheel_joint" in names and "right_wheel_joint" in names and len(msg.velocity) == len(names):
                    st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                    wheels.append((st, msg.velocity[names.index("left_wheel_joint")], msg.velocity[names.index("right_wheel_joint")]))
            elif topic == "/sil/base/odom":
                st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                sil.append((st, msg.twist.twist.linear.x, msg.twist.twist.angular.z))
            elif topic == "/odom":
                p = msg.pose.pose
                odom.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, p.position.x, p.position.y, yaw_of(p.orientation)))
            elif topic == "/cmd_vel":
                if abs(msg.linear.x) > 1e-4 or abs(msg.angular.z) > 1e-4:
                    cmd_nonzero += 1
            elif topic == "/sil/drive/status" and len(drive_status) < 3:
                drive_status.append(msg.data[:300])
    out("decoded_counts", counts=dict(counts), cmd_vel_nonzero=cmd_nonzero, drive_status_first=drive_status)
    for (name, level, text), n in sorted(rosout.items(), key=lambda kv: -kv[1])[:60]:
        out("rosout", node=name, level=level, count=n, text=text)
    out("tf_pairs", pairs={f"{a}>{b}": n for (a, b), n in sorted(tf_pairs.items())})

    gt.sort()
    if len(gt) > 1:
        length = sum(math.dist(gt[i][1:3], gt[i - 1][1:3]) for i in range(1, len(gt)))
        rot = sum(abs(wrap(gt[i][3] - gt[i - 1][3])) for i in range(1, len(gt)))
        out("ground_truth", samples=len(gt), t0=gt[0][0], t1=gt[-1][0], path_m=round(length, 3), abs_rotation_rad=round(rot, 3),
            final=[round(v, 3) for v in gt[-1][1:]])

    def reckon(samples, to_vw):
        """Integrate (t, a, b) samples to a pose track starting at the first ground truth pose."""
        samples = sorted(samples)
        if len(samples) < 2 or not gt:
            return None
        x, y, th = gt[0][1], gt[0][2], gt[0][3]
        track = [(samples[0][0], x, y, th)]
        rot = 0.0
        for i in range(1, len(samples)):
            dt = samples[i][0] - samples[i - 1][0]
            if dt <= 0 or dt > 1.0:
                continue
            v, w = to_vw(samples[i - 1])
            x += v * math.cos(th + 0.5 * w * dt) * dt
            y += v * math.sin(th + 0.5 * w * dt) * dt
            th = wrap(th + w * dt)
            rot += abs(w * dt)
            track.append((samples[i][0], x, y, th))
        return track, rot

    def compare(name, result):
        if not result:
            out(name, available=False)
            return
        track, rot = result
        gts = {round(g[0], 2): g for g in gt}
        rows = []
        step = max(1, len(track) // 40)
        for p in track[::step] + [track[-1]]:
            g = gts.get(round(p[0], 2))
            if g:
                rows.append({"t": round(p[0], 2), "err_m": round(math.dist(p[1:3], g[1:3]), 3),
                             "yaw_err_rad": round(wrap(p[3] - g[3]), 3), "est": [round(v, 3) for v in p[1:]],
                             "truth": [round(v, 3) for v in g[1:]]})
        out(name, abs_rotation_rad=round(rot, 3), rows=rows)

    compare("dead_reckoning_gz_wheels", reckon(wheels, lambda s: (R * (s[1] + s[2]) / 2.0, R * (s[2] - s[1]) / S)))
    compare("dead_reckoning_sil_base_odom", reckon(sil, lambda s: (s[1], s[2])))
    if odom and gt:
        gts = {round(g[0], 2): g for g in gt}
        step = max(1, len(odom) // 40)
        rows = []
        for p in odom[::step] + [odom[-1]]:
            g = gts.get(round(p[0], 2))
            if g:
                rows.append({"t": round(p[0], 2), "err_m": round(math.dist(p[1:3], g[1:3]), 3), "yaw_err_rad": round(wrap(p[3] - g[3]), 3)})
        out("odom_vs_truth", rows=rows)


if __name__ == "__main__":
    for d in sys.argv[1:]:
        try:
            main(d)
        except Exception as error:  # noqa: BLE001 - diagnostics print and go on
            out("error", directory=d, error=repr(error))
