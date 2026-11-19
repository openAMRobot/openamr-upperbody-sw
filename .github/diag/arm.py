"""Scratch (not for merge): joint 1 of the left arm around the grasp_reach end, from a Baseline 1 bag."""
import glob, json, math, sys
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

root, target_j1 = sys.argv[1], float(sys.argv[2])
for p in sorted(glob.glob(root + "/**/*", recursive=True)):
    if p.endswith((".mcap", ".json", ".yaml", ".log")) and "candump" not in p:
        print("file", p)
bags = sorted(glob.glob(root + "/**/*.mcap", recursive=True))
mission = sorted(glob.glob(root + "/**/baseline1_mission.json", recursive=True))
phase = None
if mission:
    d = json.load(open(mission[0]))
    for ph in d["phases"]:
        if ph["phase"] == "arm_approach":
            phase = (ph["sim_start"], ph["sim_end"])
    for e in d["events"]:
        if "grasp_reach" in e["text"] or "gravity" in e["text"].lower() or "payload" in e["text"].lower():
            print("event", e)
print("arm_approach sim window", phase)
TOPICS = {"/left_arm_controller/controller_state", "/arm/joint_states", "/sim/joint_states",
          "/left_arm_gravity_controller/commands", "/build1/ground_truth/item/pose", "/joint_states",
          "/build1/ground_truth/finger_joint_states", "/build1/planning_scene/exclude"}
J = "openarm_left_joint1"
rows = {t: [] for t in TOPICS}
for bag in bags:
    print("bag", bag)
    with open(bag, "rb") as f:
        r = make_reader(f, decoder_factories=[DecoderFactory()])
        summ = r.get_summary()
        for cid, ch in summ.channels.items():
            print("  topic", ch.topic, summ.statistics.channel_message_counts.get(cid))
        for schema, ch, msg, ros in r.iter_decoded_messages(topics=list(TOPICS)):
            t = msg.log_time * 1e-9
            if phase and not (phase[0] - 1 <= t <= phase[1] + 1):
                continue
            rows[ch.topic].append((t, ros))
cs = rows["/left_arm_controller/controller_state"]
print("controller_state msgs in window", len(cs))
# reference reaching the reach end point
hit = [t for t, m in cs if abs(m.reference.positions[list(m.joint_names).index(J)] - target_j1) < 1e-4]
print("reference at the grasp_reach end point from", hit[:1], "to", hit[-1:] if hit else None, "count", len(hit))
if not hit:
    sys.exit(0)
t0 = hit[0]
def at(series, t):
    best = None
    for ts, m in series:
        if ts <= t:
            best = (ts, m)
        else:
            break
    return best
def jidx(m, name):
    names = list(m.name) if hasattr(m, "name") else list(m.joint_names)
    return names.index(name) if name in names else None
print("t-t0 | ref | fb | err | ref_vel | fb_vel | arm_js pos | arm_js effort | sim pos | sim vel | gravity cmd j1 | item x y z")
for k in range(-30, 61):
    t = t0 + 0.1 * k
    c = at(cs, t); a = at(rows["/arm/joint_states"], t); s = at(rows["/sim/joint_states"], t)
    g = at(rows["/left_arm_gravity_controller/commands"], t); it = at(rows["/build1/ground_truth/item/pose"], t)
    out = ["%+5.1f" % (0.1 * k)]
    if c:
        m = c[1]; i = list(m.joint_names).index(J)
        out += ["%.5f" % m.reference.positions[i], "%.5f" % m.feedback.positions[i], "%+.5f" % m.error.positions[i],
                "%+.4f" % (m.reference.velocities[i] if m.reference.velocities else float('nan')),
                "%+.4f" % (m.feedback.velocities[i] if m.feedback.velocities else float('nan'))]
    if a:
        i = jidx(a[1], J)
        out += ["%.5f" % a[1].position[i], "%+.3f" % (a[1].effort[i] if a[1].effort else float('nan'))] if i is not None else ["-", "-"]
    if s:
        i = jidx(s[1], J)
        out += ["%.5f" % s[1].position[i], "%+.4f" % (s[1].velocity[i] if s[1].velocity else float('nan'))] if i is not None else ["-", "-"]
    if g:
        out += ["%+.3f" % g[1].data[0], "age %.3f" % (t - g[0])]
    if it:
        p = it[1].pose.position; out += ["%.4f %.4f %.4f" % (p.x, p.y, p.z)]
    print(" | ".join(out))
gc = rows["/left_arm_gravity_controller/commands"]
gaps = [(gc[i + 1][0] - gc[i][0], gc[i][0]) for i in range(len(gc) - 1)]
gaps.sort(reverse=True)
print("gravity command gaps (s, at):", [(round(a, 3), round(b, 2)) for a, b in gaps[:8]], "msgs", len(gc))

# all joints at the stuck time and after the release, for offline kinematics
def snapshot(label, t):
    c = at(cs, t); f = at(rows["/build1/ground_truth/finger_joint_states"], t); it = at(rows["/build1/ground_truth/item/pose"], t)
    m = c[1]
    print("SNAP", label, "t", round(t, 3), "names", list(m.joint_names))
    print("SNAP", label, "ref", [round(v, 5) for v in m.reference.positions])
    print("SNAP", label, "fb", [round(v, 5) for v in m.feedback.positions])
    if f: print("SNAP", label, "fingers", list(f[1].name), [round(v, 5) for v in f[1].position])
    if it:
        p, o = it[1].pose.position, it[1].pose.orientation
        print("SNAP", label, "item", [round(p.x, 5), round(p.y, 5), round(p.z, 5)], [round(o.x, 5), round(o.y, 5), round(o.z, 5), round(o.w, 5)])
end = [t for t, m in cs if abs(m.reference.positions[list(m.joint_names).index(J)] - target_j1) < 1e-5 and abs(m.reference.velocities[list(m.joint_names).index(J)]) < 1e-6]
te = end[0] if end else t0 + 1.2
print("trajectory end (reference at rest at the end point)", round(te, 3))
for label, dt in (("end", 0.0), ("hold", 0.4), ("hold2", 0.9), ("after", 2.0)):
    snapshot(label, te + dt)
