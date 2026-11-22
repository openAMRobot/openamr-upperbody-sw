# SPDX-License-Identifier: MIT
import glob, json
def mission(d):
    f = glob.glob(d + "/**/baseline1_mission.json", recursive=True)
    return json.load(open(f[0])) if f else None
M = {k: mission(k) for k in ("fail", "pass")}
for k, m in M.items():
    print("=====", k, None if m is None else list(m.keys()))
names = []
Q = {}
for k, m in M.items():
    Q[k] = {}
    for p in m["phases"]:
        if p["phase"] in ("arm_approach", "grasp", "lift_object"):
            for q in p["quantities"]:
                Q[k][q["name"]] = q.get("value")
                if q["name"] not in names: names.append(q["name"])
for n in names:
    a, b = Q["fail"].get(n), Q["pass"].get(n)
    if a != b: print(f"Q {n}: fail={json.dumps(a)[:150]} | pass={json.dumps(b)[:150]}")
for k, m in M.items():
    for e in m.get("events", []):
        t = e.get("text", "")
        if any(w in t for w in ("commanded end point", "contact", "palm", "grasp_", "reading minus")): print("EV", k, t[:420])
    for key in m:
        if "contact" in key.lower(): print("KEY", k, key, json.dumps(m[key])[:1500])
