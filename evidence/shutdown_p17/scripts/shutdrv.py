#!/usr/bin/env python3
import os, random, signal, subprocess, sys, time, glob, json
tag = sys.argv[1]; out = os.environ.get("OUT", "st"); os.makedirs(out, exist_ok=True)
part = f"sd_{os.getpid()}_{tag}"; mark = f"{out}/{tag}.mark"
try: os.unlink(mark)
except FileNotFoundError: pass
env = dict(os.environ, RMW_IMPLEMENTATION="rmw_cyclonedds_cpp", ROS_DOMAIN_ID=str(random.randint(101, 200)),
           ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST", GZ_PARTITION=part, MARK=mark)
log = open(f"{out}/{tag}.log", "w")
p = subprocess.Popen(["launch_test", os.environ.get("TEST", "shut_test.py")], env=env, stdout=log, stderr=subprocess.STDOUT,
                     start_new_session=True, preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_DFL))
def mine():
    r = {}
    for d in glob.glob("/proc/[0-9]*"):
        try:
            if f"GZ_PARTITION={part}".encode() not in open(d + "/environ", "rb").read().split(b"\0"): continue
            state = [l for l in open(d + "/status").read().splitlines() if l.startswith("State:")][0].split()[1]
            if state == "Z": continue
            cmd = open(d + "/cmdline", "rb").read().replace(b"\0", b" ").decode(errors="replace")
            r[int(d.split("/")[2])] = cmd[:60]
        except Exception: pass
    return r
t_end = None; gone = {}; seen = {}
deadline = time.time() + 200
while time.time() < deadline:
    now = mine()
    for pid, c in now.items(): seen.setdefault(pid, c)
    if t_end is None and os.path.exists(mark):
        try: t_end = float(open(mark).read())
        except ValueError: pass
    if t_end is not None:
        for pid in seen:
            if pid not in now and pid not in gone: gone[pid] = round(time.time() - t_end, 2)
        if p.poll() is not None and not now: break
        if time.time() - t_end > 40: break
    time.sleep(0.1)
left = mine()
isgz = lambda c: any(os.path.basename(t) == "gz" for t in c.split()[:2]) and "sim" in c.split()[:4]
gz = {pid: c for pid, c in seen.items() if isgz(c)}
rp = f"/tmp/oa_gz_stop_{part}.json"
try: helper = json.load(open(rp))
except Exception: helper = None
print(json.dumps({"tag": tag, "helper": helper, "all_exit_s": {f"{pid} {c[:24]}": gone.get(pid) for pid, c in seen.items() if gone.get(pid, 0) is None or gone.get(pid, 0) > 0.0 and ("gz" in c or "ruby" in c)}, "launch_test_rc": p.poll(),
       "gz_exit_s_after_test_end": {f"{pid} {'wrapper' if c.startswith('ruby') else 'server'}": gone.get(pid) for pid, c in gz.items()},
       "survivors": {pid: c for pid, c in left.items() if isgz(c)},
       "other_left": {pid: c[:40] for pid, c in left.items() if not isgz(c)}}), flush=True)
for pid in left:
    try: os.kill(pid, signal.SIGKILL)
    except Exception: pass
try: os.killpg(p.pid, signal.SIGKILL)
except Exception: pass
