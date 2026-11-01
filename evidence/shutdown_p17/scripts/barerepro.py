#!/usr/bin/env python3
"""Bare gz sim server (no ROS, no model of ours): start, run 6 s, ask it to stop, time its exit. usage: barerepro.py N service|sigint"""
import os, signal, subprocess, sys, time, glob
n, method = int(sys.argv[1]), sys.argv[2]
world = "/opt/ros/jazzy/opt/gz_sim_vendor/share/gz/gz-sim8/worlds/empty.sdf"
res = []
for i in range(n):
    part = f"bare_{os.getpid()}_{i}"
    env = dict(os.environ, GZ_PARTITION=part)
    log = open(f"{os.environ['OUT']}/bare_{method}_{i}.log", "w")
    p = subprocess.Popen(["gz", "sim", "-r", "-s", "-v", "2", "--physics-engine", "gz-physics-bullet-featherstone-plugin", world],
                         env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    time.sleep(6.0)
    t0 = time.monotonic()
    if method == "service":
        subprocess.run(["gz", "service", "-s", "/server_control", "--reqtype", "gz.msgs.ServerControl", "--reptype", "gz.msgs.Boolean",
                        "--timeout", "3000", "--req", "stop: true"], env=env, capture_output=True)
    else:
        p.send_signal(signal.SIGINT)
    while p.poll() is None and time.monotonic() - t0 < 20:
        time.sleep(0.05)
    hung = p.poll() is None
    if hung:
        os.killpg(p.pid, signal.SIGKILL); p.wait()
    seg = "Segmentation fault" in open(f"{os.environ['OUT']}/bare_{method}_{i}.log").read()
    res.append((round(time.monotonic() - t0, 2), "HUNG" if hung else "ok", "segv" if seg else "", p.returncode))
    print(i, res[-1], flush=True)
print("SUMMARY", method, "hung", sum(r[1] == "HUNG" for r in res), "segv", sum(bool(r[2]) for r in res), "of", n, flush=True)
