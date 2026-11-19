# SPDX-License-Identifier: MIT
"""Scratch (not for merge): MIT commands and motor replies of the left arm around the grasp_reach end, from candump.

Usage: can_hold.py <candump vcan_arm_l log> <joint 1 end point rad>
The end of the reach is found on the bus itself: the first time the MIT position command of motor 1 stays within
2e-4 rad of the end point for 1 s. Times are the candump wall clock, printed relative to that moment."""
import sys

sys.path.insert(0, __import__("os").path.dirname(__file__))
from common import arm_can, at, TYPES, DM  # noqa: E402

path, end = sys.argv[1], float(sys.argv[2])
motors = arm_can(path)
cmd1 = motors[1]["cmd"]
print("frames: motor 1 commands", len(cmd1), "replies", len(motors[1]["state"]))
t_end = None
for i, (t, c) in enumerate(cmd1):
    if abs(c["q"] - end) < 2e-4:
        j = i
        while j < len(cmd1) and abs(cmd1[j][1]["q"] - end) < 2e-4:
            j += 1
        if j < len(cmd1) and cmd1[j - 1][0] - t >= 1.0 or j == len(cmd1) and cmd1[-1][0] - t >= 1.0:
            t_end = t
            break
if t_end is None:
    print("no hold at the end point found on the bus")
    sys.exit(0)
print("command at the end point from wall", round(t_end, 3))
peak = {}
print("dt | m1 cmd q | kp | kd | tau_ff | reply q | reply dq | reply tau | kp*(qc-q)+tau_ff | m6 cmd q | m6 reply q | m6 reply tau")
for k in range(-20, 41):
    t = t_end + 0.1 * k
    c, s = at(cmd1, t), at(motors[1]["state"], t)
    c6, s6 = at(motors[6]["cmd"], t), at(motors[6]["state"], t)
    if not (c and s):
        continue
    c, s = c[1], s[1]
    law = c["kp"] * (c["q"] - s["q"]) + c["tau"]
    print("%+5.1f | %.5f | %.2f | %.3f | %+.3f | %.5f | %+.4f | %+.3f | %+.3f | %.5f | %.5f | %+.3f" % (
        0.1 * k, c["q"], c["kp"], c["kd"], c["tau"], s["q"], s["dq"], s["tau"], law,
        c6[1]["q"] if c6 else float("nan"), s6[1]["q"] if s6 else float("nan"), s6[1]["tau"] if s6 else float("nan")))
for m in range(1, 8):
    cap = min(DM[TYPES[m - 1]][2], (40, 40, 27, 27, 7, 7, 7)[m - 1])
    window = [s for t, s in motors[m]["state"] if t_end - 2.0 <= t <= t_end + 3.0]
    taus = [abs(s["tau"]) for s in window]
    codes = sorted({s["code"] for s in window})
    cmds = [c for t, c in motors[m]["cmd"] if t_end - 2.0 <= t <= t_end + 3.0]
    print(f"motor {m} {TYPES[m-1]}: |reply tau| max {max(taus) if taus else None:.3f} N m of cap {cap} "
          f"(saturated: {bool(taus) and max(taus) >= cap - 0.05}); codes {codes}; kp {sorted({round(c['kp'], 2) for c in cmds})}; "
          f"kd {sorted({round(c['kd'], 3) for c in cmds})}")
