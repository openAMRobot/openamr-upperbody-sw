# SPDX-License-Identifier: MIT
"""Grocery run 44299879412: ground-truth pose, wheel odometry, AMCL, plans, LiDAR scans, item and tote poses, tf_static."""
import sys, math, collections, json
from pathlib import Path
import numpy as np
from rosbags.highlevel import AnyReader
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
S = collections.defaultdict(list); plans = []; scans = []; static = []; phase = []
T = {"/build1/ground_truth/odom", "/sil/base/odom", "/amcl_pose", "/plan", "/scan_filtered", "/tf_static",
     "/build1/ground_truth/item/pose", "/build1/ground_truth/tote/pose", "/grocery_demo/phase", "/grocery_demo/events"}
with AnyReader([Path(sys.argv[1])]) as r:
    conns = [c for c in r.connections if c.topic in T]
    for c, t, raw in r.messages(connections=conns):
        t /= 1e9; tp = c.topic; m = r.deserialize(raw, c.msgtype)
        if tp in ("/build1/ground_truth/odom", "/sil/base/odom", "/amcl_pose"):
            p = m.pose.pose; S[tp].append((t, p.position.x, p.position.y, yaw(p.orientation)))
        elif tp == "/plan":
            plans.append((t, np.array([(q.pose.position.x, q.pose.position.y) for q in m.poses]).reshape(-1, 2)))
        elif tp == "/scan_filtered":
            scans.append((t, m.header.frame_id, m.angle_min, m.angle_increment, np.asarray(m.ranges, dtype=np.float32), m.range_max))
        elif tp == "/tf_static":
            for tr in m.transforms:
                q = tr.transform; static.append((tr.header.frame_id, tr.child_frame_id, q.translation.x, q.translation.y, q.translation.z, yaw(q.rotation)))
        elif tp in ("/build1/ground_truth/item/pose", "/build1/ground_truth/tote/pose"):
            p = m.pose.position; S[tp].append((t, p.x, p.y, p.z))
        else:
            phase.append((t, tp, m.data))
for k, v in S.items(): print(k, len(v), v[0][0], v[-1][0])
print("plans", len(plans), "scans", len(scans), set(s[1] for s in scans))
print([s for s in static if "laser" in s[1] or "lidar" in s[1] or s[1] in set(x[1] for x in scans)])
np.savez_compressed(sys.argv[2], plan_t=np.array([p[0] for p in plans]), plan_xy=np.array([p[1] for p in plans], dtype=object),
                    scan_t=np.array([s[0] for s in scans]), scan_meta=np.array([(s[2], s[3], s[5]) for s in scans]),
                    scan_r=np.array([s[4] for s in scans], dtype=object), scan_frame=scans[0][1] if scans else "",
                    static=np.array(static, dtype=object), msgs=np.array(phase, dtype=object),
                    **{k.strip("/").replace("/", "__"): np.array(v) for k, v in S.items()})
