# SPDX-License-Identifier: MIT
"""Parking variability: decompose the final station error of finished Baseline 1 runs from their recordings.

Reads, per run directory (an extracted baseline1-acceptance artifact): build1_logs/bag/*.mcap and
build1_logs/baseline1_mission.json. Prints one line `PARKING_JSON {...}` per run. Diagnostics only: it reruns nothing.

Decomposition at the stop, in the station frame (along: + towards the shelf; left: + to the robot's left):
  total = T - G  (Gazebo ground truth against the station goal)
  nav   = M - G  (where Nav2 believes the base is, map > base_footprint from /tf, against the goal)
  loc   = T - M  (AMCL map pose against ground truth)
total = nav + loc exactly, per axis.
"""
import glob
import json
import math
import os
import sys
from bisect import bisect_right

from mcap_ros2.reader import read_ros2_messages

GOAL = (0.0, -2.085, -math.pi / 2.0)
TOPICS = ["/tf", "/tf_static", "/amcl_pose", "/build1/ground_truth/odom", "/odom", "/plan", "/cmd_vel_nav", "/scan_filtered"]


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def compose(p, q):
    x, y, t = p
    return (x + q[0] * math.cos(t) - q[1] * math.sin(t), y + q[0] * math.sin(t) + q[1] * math.cos(t), wrap(t + q[2]))


def station_frame(dx, dy):
    g = GOAL[2]
    return dx * math.cos(g) + dy * math.sin(g), -dx * math.sin(g) + dy * math.cos(g)


class Series:
    def __init__(self):
        self.t, self.v = [], []

    def add(self, t, v):
        self.t.append(t)
        self.v.append(v)

    def sort(self):
        order = sorted(range(len(self.t)), key=lambda i: self.t[i])
        self.t = [self.t[i] for i in order]
        self.v = [self.v[i] for i in order]

    def at(self, t):
        i = bisect_right(self.t, t) - 1
        return self.v[i] if i >= 0 else None


def load(bag):
    tf, static = {}, {}
    s = {k: Series() for k in ("amcl", "truth", "odom", "cmd")}
    amcl_cov, plans, scans = [], [], []
    for m in read_ros2_messages(bag, topics=TOPICS):
        topic, msg = m.channel.topic, m.ros_msg
        t = m.log_time_ns * 1e-9
        if topic in ("/tf", "/tf_static"):
            for tr in msg.transforms:
                st = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
                key = (tr.header.frame_id.lstrip("/"), tr.child_frame_id.lstrip("/"))
                pose = (tr.transform.translation.x, tr.transform.translation.y, yaw_of(tr.transform.rotation))
                if topic == "/tf_static":
                    static[key] = pose
                else:
                    # Ordered by when the transform was available (bag receive time), not by its header stamp:
                    # AMCL future-dates map > odom by transform_tolerance (1.0 s).
                    tf.setdefault(key, Series()).add(t, pose)
        elif topic == "/amcl_pose":
            st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            p = msg.pose.pose
            s["amcl"].add(st, (p.position.x, p.position.y, yaw_of(p.orientation)))
            c = msg.pose.covariance
            amcl_cov.append((st, math.sqrt(max(c[0], 0)), math.sqrt(max(c[7], 0)), math.sqrt(max(c[35], 0))))
        elif topic == "/build1/ground_truth/odom":
            p = msg.pose.pose
            s["truth"].add(t, (p.position.x, p.position.y, yaw_of(p.orientation)))
        elif topic == "/odom":
            p = msg.pose.pose
            s["odom"].add(t, (p.position.x, p.position.y, yaw_of(p.orientation)))
        elif topic == "/cmd_vel_nav":
            s["cmd"].add(t, (msg.linear.x, msg.angular.z))
        elif topic == "/plan":
            if msg.poses:
                a, b = msg.poses[0].pose, msg.poses[-1].pose
                length = sum(math.hypot(msg.poses[i + 1].pose.position.x - msg.poses[i].pose.position.x,
                                        msg.poses[i + 1].pose.position.y - msg.poses[i].pose.position.y)
                             for i in range(len(msg.poses) - 1))
                plans.append((t, len(msg.poses), a.position.x, a.position.y, b.position.x, b.position.y, length))
        elif topic == "/scan_filtered":
            scans.append((t, sum(1 for r in msg.ranges if math.isfinite(r) and msg.range_min <= r <= msg.range_max)))
    for v in list(tf.values()) + list(s.values()):
        v.sort()
    return tf, static, s, amcl_cov, plans, scans


def lookup(tf, static, parent, child, t):
    """parent > child at time t through a chain of the latest transforms (static or dynamic), breadth first."""
    edges = {}
    for (p, c) in list(tf.keys()) + list(static.keys()):
        edges.setdefault(p, []).append(c)
    frontier, seen = [(parent, (0.0, 0.0, 0.0))], {parent}
    while frontier:
        nxt = []
        for frame, pose in frontier:
            if frame == child:
                return pose
            for c in edges.get(frame, []):
                if c in seen:
                    continue
                if (frame, c) in static:
                    q = static[(frame, c)]
                else:
                    q = tf[(frame, c)].at(t)
                    if q is None:
                        continue
                seen.add(c)
                nxt.append((c, compose(pose, q)))
        frontier = nxt
    return None


def analyse(run_dir):
    out = {"dir": os.path.basename(run_dir.rstrip("/"))}
    bags = glob.glob(os.path.join(run_dir, "**", "bag", "*.mcap"), recursive=True)
    missions = glob.glob(os.path.join(run_dir, "**", "baseline1_mission.json"), recursive=True)
    if not bags or not missions:
        out["error"] = f"bag {len(bags)}, mission json {len(missions)}"
        return out
    data = json.load(open(missions[0]))
    nav = [smp for smp in data.get("trace", []) if smp.get("phase") == "nav_to_station"]
    if not nav:
        out["error"] = "no nav_to_station trace samples"
        return out
    t0, t_end = nav[0]["sim"], nav[-1]["sim"]
    out["rtf_nav_mean"] = round(sum(x["rtf"] for x in nav if x.get("rtf")) / max(1, sum(1 for x in nav if x.get("rtf"))), 3)
    tf, static, s, amcl_cov, plans, scans = load(bags[0])
    base = "base_footprint"

    def mpose(t):
        return lookup(tf, static, "map", base, t)

    def err(p, q):
        along, left = station_frame(p[0] - q[0], p[1] - q[1])
        return along, left, wrap(p[2] - q[2])

    T, M = s["truth"].at(t_end), mpose(t_end)
    if T is None or M is None:
        out["error"] = f"truth {T is not None}, map pose {M is not None} at {t_end}"
        out["tf_edges"] = {f"{k[0]}>{k[1]}": [len(v.t), round(v.t[0], 2), round(v.t[-1], 2)] for k, v in tf.items()
                           if k[1] in ("odom", "base_footprint", "base_link") or k[0] in ("map", "odom")}
        out["tf_static_edges"] = [f"{k[0]}>{k[1]}" for k in static if k[1] in ("base_footprint", "base_link")]
        out["bag_truth_span"] = [round(s["truth"].t[0], 2), round(s["truth"].t[-1], 2)] if s["truth"].t else None
        return out
    tot, navr, loc = err(T, GOAL), err(M, GOAL), err(T, M)
    out.update({"t0": round(t0, 2), "t_end": round(t_end, 2),
                "total_along": tot[0], "total_left": tot[1], "total_xy": math.hypot(tot[0], tot[1]), "total_yaw": tot[2],
                "nav_along": navr[0], "nav_left": navr[1], "nav_xy": math.hypot(navr[0], navr[1]), "nav_yaw": navr[2],
                "loc_along": loc[0], "loc_left": loc[1], "loc_xy": math.hypot(loc[0], loc[1]), "loc_yaw": loc[2]})
    O = s["odom"].at(t_end)
    if O:
        od = err(T, O)
        out.update({"odom_along": od[0], "odom_left": od[1], "odom_yaw": od[2]})
    # Stage boundary: the first time Nav2's map pose is inside the global goal checker (0.35 m, 0.35 rad).
    times = [t for t in s["truth"].t if t0 <= t <= t_end]
    t_s2 = None
    for t in times[::10]:
        p = mpose(t)
        if p and math.hypot(p[0] - GOAL[0], p[1] - GOAL[1]) <= 0.35 and abs(wrap(p[2] - GOAL[2])) <= 0.35:
            t_s2 = t
            break
    out["t_stage2"] = round(t_s2, 2) if t_s2 else None
    if t_s2:
        T2, M2 = s["truth"].at(t_s2), mpose(t_s2)
        e2, l2 = err(M2, GOAL), err(T2, M2)
        out.update({"stage2_sim_s": round(t_end - t_s2, 2), "s2_start_nav_along": e2[0], "s2_start_nav_left": e2[1],
                    "s2_start_nav_yaw": e2[2], "s2_start_dist": math.hypot(e2[0], e2[1]),
                    "s2_start_loc_along": l2[0], "s2_start_loc_left": l2[1], "s2_start_loc_xy": math.hypot(l2[0], l2[1]),
                    "s2_start_loc_yaw": l2[2]})
        # Stage 2: AMCL updates, map > odom corrections, the motion and time near the goal.
        upd = [t for t in s["amcl"].t if t_s2 <= t <= t_end]
        out["amcl_updates_stage2"] = len(upd)
        out["amcl_updates_stage1"] = sum(1 for t in s["amcl"].t if t0 <= t < t_s2)
        corr = []
        mo = tf.get(("map", "odom"))
        if mo:
            prev = mo.at(t_s2)
            for t, v in zip(mo.t, mo.v):
                if t_s2 < t <= t_end and prev is not None:
                    d = math.hypot(v[0] - prev[0], v[1] - prev[1])
                    if d > 1e-4 or abs(wrap(v[2] - prev[2])) > 1e-4:
                        corr.append((t, d, wrap(v[2] - prev[2])))
                    prev = v
        out["map_odom_jumps_stage2"] = len(corr)
        out["map_odom_jump_max_stage2"] = max((c[1] for c in corr), default=0.0)
        out["map_odom_jump_sum_stage2"] = sum(c[1] for c in corr)
        last_upd = max([t for t in s["amcl"].t if t <= t_end], default=None)
        out["amcl_last_update_age"] = round(t_end - last_upd, 2) if last_upd else None
        # loc error after each stage-2 AMCL update (the /amcl_pose itself against truth at its stamp)
        out["s2_amcl_pose_err"] = [round(math.hypot(*err(s["truth"].at(t), s["amcl"].at(t))[:2]), 4) for t in upd]
        path, rot, near, prevT = 0.0, 0.0, 0.0, None
        for t in times:
            if t < t_s2:
                continue
            p = s["truth"].at(t)
            if prevT:
                path += math.hypot(p[0] - prevT[1][0], p[1] - prevT[1][1])
                rot += abs(wrap(p[2] - prevT[1][2]))
                m = mpose(t)
                if m and math.hypot(m[0] - GOAL[0], m[1] - GOAL[1]) <= 0.05:
                    near += t - prevT[0]
            prevT = (t, p)
        out.update({"s2_truth_path": path, "s2_truth_rotation": rot, "s2_time_within_5cm_belief": round(near, 2)})
        cmds = [(t, v) for t, v in zip(s["cmd"].t, s["cmd"].v) if t_s2 <= t <= t_end]
        out["s2_cmd_msgs"] = len(cmds)
        out["s2_cmd_reverse"] = sum(1 for t, v in cmds if v[0] < -1e-3)
        out["s2_plans"] = [(round(p[0], 2), p[1], round(p[6], 3)) for p in plans if p[0] >= t_s2 - 1.0]
    # Before departure: AMCL updates and covariance.
    pre = [c for c in amcl_cov if c[0] <= t0]
    out["amcl_msgs_before_departure"] = len(pre)
    if pre:
        out["sigma_before_departure"] = [round(v, 4) for v in pre[-1][1:]]
    if t_s2:
        cs = [c for c in amcl_cov if c[0] <= t_s2]
        if cs:
            out["sigma_at_stage2"] = [round(v, 4) for v in cs[-1][1:]]
    ce = [c for c in amcl_cov if c[0] <= t_end]
    if ce:
        out["sigma_at_end"] = [round(v, 4) for v in ce[-1][1:]]
    # Localisation error along the whole drive (map pose against truth, 1 Hz).
    track = []
    t = t0
    while t <= t_end:
        p, q = s["truth"].at(t), mpose(t)
        if p and q:
            e = err(p, q)
            track.append((round(t - t0, 1), round(e[0], 4), round(e[1], 4), round(e[2], 4)))
        t += 1.0
    out["loc_track"] = track
    out["loc_xy_max_drive"] = max((math.hypot(e[1], e[2]) for e in track), default=None)
    # Path of stage 1: the furthest the truth went from the straight segments, and the turn point.
    xs = [s["truth"].at(t) for t in times if (t_s2 is None or t < t_s2)]
    if xs:
        out["s1_min_x"] = min(p[0] for p in xs)
        out["s1_max_x"] = max(p[0] for p in xs)
        out["s1_min_y"] = min(p[1] for p in xs)
    sc = [n for (t, n) in scans if abs(t - t_end) < 1.0]
    out["beams_with_return_end"] = sc[-1] if sc else None
    out["plans_total"] = len(plans)
    return out


if __name__ == "__main__":
    for d in sys.argv[1:]:
        try:
            r = analyse(d)
        except Exception as e:  # noqa: BLE001 - one bad artifact does not stop the others
            r = {"dir": os.path.basename(d.rstrip("/")), "error": f"{type(e).__name__}: {e}"}
        print("PARKING_JSON " + json.dumps(r, default=lambda x: round(x, 5) if isinstance(x, float) else str(x)), flush=True)
