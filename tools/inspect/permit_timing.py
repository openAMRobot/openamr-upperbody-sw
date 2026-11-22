# SPDX-License-Identifier: MIT
"""permit_timing.py CANDUMP_L_FILE: for every new lift command (210h, node 10h) print the permit frames (410h) around
it and the next status (190h) result byte. Wall time from candump -L. Diagnostic only (upperbody #132)."""
import re, sys
rows = []
for line in open(sys.argv[1], errors="replace"):
    m = re.match(r"\((\d+\.\d+)\)\s+\S+\s+([0-9A-Fa-f]+)#([0-9A-Fa-f]*)", line)
    if m: rows.append((float(m.group(1)), int(m.group(2), 16), m.group(3).upper()))
if not rows: sys.exit("no frames")
t0 = rows[0][0]
permit = [(t, d) for t, i, d in rows if i == 0x410]
gaps = [b[0] - a[0] for a, b in zip(permit, permit[1:])]
print(f"frames {len(rows)}, span {rows[-1][0]-t0:.1f} s wall; 410h frames {len(permit)}")
if gaps:
    g = sorted(gaps); print(f"410h wall gap: median {g[len(g)//2]*1000:.1f} ms, p99 {g[int(len(g)*0.99)]*1000:.1f} ms, max {g[-1]*1000:.1f} ms")
    for a, b in zip(permit, permit[1:]):
        if b[0] - a[0] > 3 * g[len(g)//2]: print(f"  long 410h gap {1000*(b[0]-a[0]):.0f} ms at t={a[0]-t0:.3f} s ({a[1]} -> {b[1]})")
last_cmd = None; refused = 0
for k, (t, i, d) in enumerate(rows):
    if i != 0x210 or d == last_cmd: continue
    last_cmd = d
    before = [(pt, pd) for pt, pd in permit if pt <= t][-3:]
    after = [(pt, pd) for pt, pd in permit if pt > t][:1]
    res = next(((rt, rd) for rt, ri, rd in rows[k + 1:] if ri == 0x190 and len(rd) >= 14 and rd[8:10] == d[2:4]), None)
    code = res[1][10:12] if res else "--"
    refused += code == "80"
    print(f"cmd 210h#{d} at t={t-t0:.3f} s -> 190h result {code}" + (f" after {1000*(res[0]-t):.1f} ms" if res else ""))
    for pt, pd in before: print(f"    410h#{pd} {1000*(t-pt):8.1f} ms before the command")
    for pt, pd in after: print(f"    410h#{pd} {1000*(pt-t):8.1f} ms after the command")
# Status frames (190h): state, flags, position, stop, last_seq, result, counter. Print every change of state or result.
prev = None
for t, i, d in rows:
    if i != 0x190 or len(d) < 14: continue
    key = (d[0:4], d[8:12])
    if key != prev:
        near = [pt for pt, _ in permit if pt <= t][-1:]
        print(f"status 190h#{d} at t={t-t0:.3f} s; last 410h {1000*(t-near[0]):.1f} ms earlier" if near else f"status 190h#{d} at t={t-t0:.3f} s")
        prev = key
print(f"REFUSED_COMMANDS {refused}")
