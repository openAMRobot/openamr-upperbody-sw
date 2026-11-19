# SPDX-License-Identifier: MIT
"""Scratch (not for merge): one grasp-only run at a logged base pose. Reads the diag_grasp trace (contacts, joints,
item, plans), the side bag (controller state, gravity commands, emulator status, RTF) and prints the evidence lines
around the end of grasp_reach. Usage: repro.py <log dir> <label>"""
import collections
import glob
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from common import at, bag_messages, quantities  # noqa: E402

root, label = sys.argv[1], sys.argv[2]
logs = sorted(glob.glob(root + "/**/diag_grasp.json", recursive=True))
traces = sorted(glob.glob(root + "/**/diag_grasp_trace.json", recursive=True))
print(label, "logs", logs, "traces", traces)
if logs:
    q, data = quantities(logs[0])
    print(label, "RESULT", data.get("result"), data.get("reason", "")[:600])
    for name in ("contact_sensor_links", "grasp_face_yaw", "grasp_reach_cartesian_fraction", "grasp_reach_moveit_error_code",
                 "grasp_reach_tracking_error_max", "grasp_reach_servo_torque_max", "grasp_approach_servo_torque_max",
                 "item_pushed_before_closing", "finger_position_on_item", "item_height_gain_after_lift",
                 "diag_contact_messages_received"):
        if name in q:
            print(label, "Q", name, q[name]["value"], q[name]["unit"], q[name].get("ok"), q[name].get("note", "")[:160])
    for e in data.get("events", []):
        if "grasp_reach" in e["text"]:
            print(label, "EVENT", e["text"][:500])
if not traces:
    sys.exit(0)
tr = json.load(open(traces[0]))
marks = dict((name, t) for t, name in tr["marks"])
plan = [p for p in tr["plans"] if p["label"] == "grasp_reach"]
print(label, "marks", {k: round(v, 3) for k, v in marks.items() if k.startswith("grasp")})
if not plan:
    print(label, "no executed grasp_reach plan")
    sys.exit(0)
plan = plan[0]
names = plan["joint_names"]
t_end = plan["t_result"]
end = dict(zip(names, plan["points"][-1]))
print(label, "grasp_reach success at sim", round(t_end, 3), "end point", {k: round(v, 5) for k, v in end.items()})
j1 = [p[names.index("openarm_left_joint1")] for p in plan["points"]]
print(label, "plan j1 start %.5f min %.5f max %.5f end %.5f, duration %.3f s, %d points" % (
    j1[0], min(j1), max(j1), j1[-1], plan["times"][-1], len(j1)))
samples = [s for s in tr["samples"] if "error" not in s]
contacts = tr["contacts"]


def nearest(t):
    best = None
    for s in samples:
        if s["t"] <= t:
            best = s
    return best


window = (t_end - plan["times"][-1] - 0.5, t_end + 3.0)
print(label, "dt | j1 phys | j1 drv | j1 drv-end | j6 phys-end | tau j1 phys | item x y z (mm from first) | contacts (link>other fn)")
first_item = None
for k in range(-int(plan["times"][-1] * 10) - 5, 31):
    t = t_end + 0.1 * k
    s = nearest(t)
    if s is None:
        continue
    if first_item is None and "item" in s:
        first_item = s["item"][0]
    cs = [c for c in contacts if t - 0.05 <= c["t"] <= t + 0.05 and c["link"].startswith("openarm_left")]
    agg, n = collections.defaultdict(float), collections.Counter()
    for c in cs:
        key = (c["link"].replace("openarm_left_", ""), c["other"].split("::")[0] + "::" + c["other"].split("::")[-1])
        agg[key] += 0.0 if math.isnan(c["fn"]) else c["fn"]
        n[key] += 1
    item = s.get("item")
    d = [1000 * (item[0][i] - first_item[i]) for i in range(3)] if item and first_item else None
    print("%+5.1f | %.5f | %.5f | %+.5f | %+.5f | %+.3f | %s | %s" % (
        0.1 * k, s["q"].get("openarm_left_joint1", math.nan), s["q_driver"].get("openarm_left_joint1", math.nan),
        s["q_driver"].get("openarm_left_joint1", math.nan) - end["openarm_left_joint1"],
        s["q"].get("openarm_left_joint6", math.nan) - end.get("openarm_left_joint6", math.nan),
        s["tau"].get("openarm_left_joint1", math.nan), "%.2f %.2f %.2f" % tuple(d) if d else "-",
        "; ".join(f"{a}>{b} mean fn {v / n[(a, b)]:.2f} N x{n[(a, b)]}" for (a, b), v in sorted(agg.items())) or "none"))
# Contact pairs over the whole reach and hold
reach0 = marks.get("grasp_reach_start", t_end - 30)
pairs = collections.defaultdict(list)
for c in contacts:
    if reach0 <= c["t"] <= t_end + 3.0:
        pairs[(c["link"], c["other"], c["sensor"])].append(c)
print(label, "CONTACT PAIRS from grasp_reach_start to success + 3 s (sim time)")
for (link, other, sensor), cs in sorted(pairs.items(), key=lambda kv: kv[1][0]["t"]):
    fn = [c["fn"] for c in cs if not math.isnan(c["fn"])]
    ft = [c["ft"] for c in cs if not math.isnan(c["ft"])]
    dep = [c["depth"] for c in cs if not math.isnan(c["depth"])]
    print(f"  {link} <> {other} [sensor {sensor}]: {len(cs)} points, t {cs[0]['t'] - t_end:+.3f} to {cs[-1]['t'] - t_end:+.3f} s "
          f"from success, fn max {max(fn) if fn else 'nan'}, ft max {max(ft) if ft else 'nan'}, depth max {max(dep) if dep else 'nan'}, "
          f"first point {[round(v, 4) for v in cs[0]['point']]} normal {[round(v, 3) for v in cs[0]['normal']]}")
print(label, "contact message counts", tr.get("contact_message_counts"))
# Side bag: controller state, gravity commands, emulator status, RTF
rows = bag_messages(root, ["/left_arm_controller/controller_state", "/left_arm_gravity_controller/commands",
                           "/sim/left_arm_emulator/status", "/build1/rtf_stats", "/execute_trajectory/_action/status"])
cs = rows["/left_arm_controller/controller_state"]
print(label, "side bag: controller_state", len(cs), "gravity", len(rows["/left_arm_gravity_controller/commands"]),
      "emulator status", len(rows["/sim/left_arm_emulator/status"]), "rtf", len(rows["/build1/rtf_stats"]))
J = "openarm_left_joint1"
print(label, "dt | ref j1 | fb j1 | err j1 | ref vel | ff j1 | emulator j1 torque mit_q mit_kp")
for k in range(-15, 31):
    t = t_end + 0.1 * k
    c, g, e = at(cs, t), at(rows["/left_arm_gravity_controller/commands"], t), at(rows["/sim/left_arm_emulator/status"], t)
    out = ["%+5.1f" % (0.1 * k)]
    if c:
        m = c[1]
        i = list(m.joint_names).index(J)
        out += ["%.5f" % m.reference.positions[i], "%.5f" % m.feedback.positions[i], "%+.5f" % m.error.positions[i],
                "%+.4f" % (m.reference.velocities[i] if m.reference.velocities else math.nan)]
    if g:
        out.append("%+.3f" % g[1].data[0])
    if e:
        for st in e[1].status:
            if st.name.endswith("joint1") and "finger" not in st.name:
                v = {kv.key: kv.value for kv in st.values}
                out.append(" ".join(f"{k}={v.get(k)}" for k in ("torque", "mit_q", "mit_kp", "mit_kd", "mit_tau", "position")))
    print(" | ".join(out))
rtf = rows["/build1/rtf_stats"]
if rtf:
    print(label, "RTF near the reach end", str(at(rtf, t_end)[1])[:400] if at(rtf, t_end) else None)
