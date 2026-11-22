# SPDX-License-Identifier: MIT
import glob, json, os
def load(pat):
    f = sorted(glob.glob(pat, recursive=True))
    return (f[0], json.load(open(f[0]))) if f else (None, None)
p, d = load("a132/**/scn02_lift_permit.json")
print("=== scn02", p, list(d.keys()) if d else None)
if d:
    for e in d.get("events", []): print("  EV", json.dumps(e)[:300])
    for k in d:
        if k not in ("events", "quantities", "observed"): print("  KEY", k, json.dumps(d[k])[:600])
p, d = load("a18/**/scn18_stop_during_nav.json")
print("=== scn18", p, list(d.keys()) if d else None)
if d:
    for e in d.get("events", []): print("  EV", json.dumps(e)[:300])
    print("  TRACE", json.dumps(d.get("stop_state_trace")))
for f in sorted(glob.glob("aclk/**/clock_capture/*", recursive=True)) + sorted(glob.glob("aclk/**/clock_gate.json", recursive=True)):
    print("=== ", f, os.path.getsize(f))
    try: print(open(f, errors="replace").read()[:2500])
    except Exception as e: print(e)
