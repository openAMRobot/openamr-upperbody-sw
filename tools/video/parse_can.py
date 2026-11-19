# SPDX-License-Identifier: MIT
"""candump -L logs -> per bus: host wall time, Gazebo time (interpolated from the mission trace wall/sim pairs), id, data."""
import sys, json, glob, os
import numpy as np
logdir, mission, out = sys.argv[1], sys.argv[2], sys.argv[3]
tr = json.load(open(mission))["trace"]
w = np.array([x["wall"] for x in tr]); s = np.array([x["sim"] for x in tr])
res = {}
for f in sorted(glob.glob(os.path.join(logdir, "record_candump_*.log"))):
    bus = os.path.basename(f)[len("record_candump_"):-4]
    ts, ids, data = [], [], []
    with open(f) as fh:
        for line in fh:
            if "#" not in line: continue
            a, _b, c = line.split()
            ts.append(float(a[1:-1])); i, d = c.split("#", 1); ids.append(int(i, 16)); data.append(d[:20])
    ts = np.array(ts); res[bus + "__wall"] = ts; res[bus + "__sim"] = np.interp(ts, w, s, left=np.nan, right=np.nan)
    res[bus + "__id"] = np.array(ids, dtype=np.int64); res[bus + "__data"] = np.array(data, dtype="S20")
    print(bus, len(ts), ts[0], ts[-1], np.nanmin(res[bus + "__sim"]), np.nanmax(res[bus + "__sim"]), "rate wall", len(ts) / (ts[-1] - ts[0]))
np.savez(out, **res)
