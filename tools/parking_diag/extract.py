# SPDX-License-Identifier: MIT
"""Per-run navigation and localisation numbers from a Baseline 1 acceptance job log (read-only)."""
import re, sys, json, os
STAMP = re.compile(r'\[(\d{10}\.\d+)\]')
def parse(path):
    L = open(path, errors='replace').read().splitlines()
    out = {"run": os.path.basename(path)[:-4]}
    for l in L:
        m = re.search(r'BASELINE1_STATION_ALIGNMENT=([01])', l)
        if m and "align" not in out: out["align"] = m.group(1)
        m = re.search(r'src/openamr-upperbody-sw ([0-9a-f]{7})', l)
        if m and "sha" not in out: out["sha"] = m.group(1)
        m = re.search(r'src/openamr-platform-sw ([0-9a-f]{7})', l)
        if m and "pf" not in out: out["pf"] = m.group(1)
    # mission block
    try:
        i0 = next(i for i, l in enumerate(L) if 'event: nav goal work_shelf' in l)
    except StopIteration:
        return out
    out["bt"] = "station" if 'navigate_to_station' in L[i0] else "default"
    try:
        i1 = next(i for i in range(i0, len(L)) if 'nav_status = ' in L[i])
    except StopIteration:
        i1 = len(L)
    for l in L[i0:]:
        m = re.search(r'BUILD1 baseline1_mission (\w+) = (-?[0-9.]+)', l)
        if m and m.group(1) not in out:
            out[m.group(1)] = float(m.group(2))
        if 'station_reach_yaw_error' in l: break
    for l in L[:i0]:
        m = re.search(r'BUILD1 baseline1_mission (gazebo_rtf_at_start) = (-?[0-9.]+)', l)
        if m: out[m.group(1)] = float(m.group(2))
    t_goal = float(STAMP.search(L[i0+3]).group(1)) if False else None
    ev = []; rtf = []; missed = 0; passing = 0
    for l in L[i0:i1+1]:
        s = STAMP.search(l); t = float(s.group(1)) if s else None
        if 'Begin navigating' in l: ev.append(("begin", t))
        elif 'Received a goal' in l and 'controller_server' in l: ev.append(("recv", t))
        elif 'Reached the goal!' in l: ev.append(("reached", t))
        elif 'Failed to make progress' in l: ev.append(("noprogress", t))
        elif 'controller_server' in l and '[ERROR]' in l and 'inflation' not in l: ev.append(("cerr:" + l.split(']: ')[-1][:60], t))
        elif 'behavior_server' in l and 'Running' in l: ev.append(("rec:" + l.split('Running ')[-1].strip(), t))
        elif 'bt_navigator' in l and ('aborted' in l.lower() or 'failed' in l.lower()): ev.append(("bt:" + l.split(']: ')[-1][:60], t))
        elif 'planner_server' in l and ('[WARN]' in l or '[ERROR]' in l): ev.append(("plan:" + l.split(']: ')[-1][:60], t))
        m = re.search(r'real-time factor: last ([0-9.]+)', l)
        if m: rtf.append((t, float(m.group(1))))
        if 'Control loop missed' in l: missed += 1
    out["events"] = [(e, round(t, 2) if t else None) for e, t in ev]
    out["missed_rate"] = missed
    # stages
    first_reached = next((t for e, t in ev if e == "reached"), None)
    begin = next((t for e, t in ev if e == "begin"), None)
    end = [t for e, t in ev if e == "reached"][-1] if first_reached else None
    if first_reached:
        recv2 = next((t for e, t in ev if e == "recv" and t > first_reached), None)
        out["stage1_wall"] = round(first_reached - begin, 1)
        out["stage2_wall"] = round(end - recv2, 1) if recv2 and end else None
        out["stage2_attempts"] = sum(1 for e, t in ev if e == "recv" and t > first_reached)
        out["stage2_noprogress"] = sum(1 for e, t in ev if e == "noprogress" and t > first_reached)
        out["stage1_noprogress"] = sum(1 for e, t in ev if e == "noprogress" and t < first_reached)
        out["stage1_recoveries"] = sum(1 for e, t in ev if e.startswith("rec:") and t < first_reached)
        r1 = [v for t, v in rtf if begin <= t <= first_reached]
        r2 = [v for t, v in rtf if t >= first_reached]
        out["rtf_stage1"] = round(sum(r1)/len(r1), 3) if r1 else None
        out["rtf_stage2"] = round(sum(r2)/len(r2), 3) if r2 else None
        if out["rtf_stage2"] and out.get("stage2_wall"): out["stage2_sim_est"] = round(out["stage2_wall"] * out["rtf_stage2"], 1)
    rr = [v for t, v in rtf]
    out["rtf_nav"] = round(sum(rr)/len(rr), 3) if rr else None
    return out
if __name__ == "__main__":
    print(json.dumps([parse(p) for p in sys.argv[1:]], indent=1))
