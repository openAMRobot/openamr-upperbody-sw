#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""RViz screenshots of the combined model at the lift stops x base plates (P19). Real RViz on the real display (WSLg, :0), captured window by
window with xgrab.py (PIL's root grab fails under WSLg). Not a URDF render.

    source /opt/ros/jazzy/setup.bash && source <ws>/install/setup.bash
    python3 capture_rviz.py <outdir> [L1000:bp000 L1175:bp050 ...]

Each shot starts its own stack on a private ROS_DOMAIN_ID (localhost only), waits for the RViz window, lets the stack settle, reads lift_joint
from /joint_states (so the record says what the model was at, not what was asked), captures, and stops the stack by process group (SIGINT first).
Writes <outdir>/rviz_<stop>_<bp>.png and <outdir>/rviz_<stop>_<bp>.json."""
import json
import os
import re
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
STOPS = {"L1000": 0.0, "L1175": 0.175, "L1350": 0.350}
SETTLE_S = 50      # after the RViz window exists: the model, the planning scene and the first frames (software rendering, about 9 fps)
READ_LIFT = """
import sys, rclpy
from sensor_msgs.msg import JointState
rclpy.init(); n = rclpy.create_node('p19_lift_reader'); out = []
n.create_subscription(JointState, '/joint_states', lambda m: out.append(m.position[m.name.index('lift_joint')]) if 'lift_joint' in m.name else None, 10)
end = __import__('time').time() + 25
while rclpy.ok() and not out and __import__('time').time() < end: rclpy.spin_once(n, timeout_sec=0.5)
print(out[0] if out else 'NONE')
"""


def window_id():
    tree = subprocess.run(["xwininfo", "-root", "-tree"], capture_output=True, text=True).stdout
    for line in tree.splitlines():
        if "capture.rviz" in line and "RViz" in line:
            return re.search(r"0x[0-9a-f]+", line).group(0)
    return None


def pids_in_domain(domain):
    found = []
    for p in os.listdir("/proc"):
        if p.isdigit():
            try:
                env = open(f"/proc/{p}/environ", "rb").read().split(b"\0")
            except OSError:
                continue
            if f"ROS_DOMAIN_ID={domain}".encode() in env:
                found.append(int(p))
    return found


def shoot(stop, bp, domain, outdir):
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1", PYTHONDONTWRITEBYTECODE="1")
    log = open(os.path.join(outdir, f"rviz_{stop}_{bp}.launch.log"), "w")
    proc = subprocess.Popen(["ros2", "launch", os.path.join(HERE, "capture.launch.py"), f"lift_position:={STOPS[stop]}", f"base_plate:={bp}"],
                            env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
                            preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_DFL))
    record = {"stop": stop, "base_plate": bp, "lift_position_asked_m": STOPS[stop], "ros_domain_id": domain}
    try:
        t0, win = time.time(), None
        while time.time() - t0 < 120 and win is None:
            time.sleep(2)
            win = window_id()
        if win is None:
            raise RuntimeError("no RViz window within 120 s")
        time.sleep(SETTLE_S)
        lift = subprocess.run([sys.executable, "-c", READ_LIFT], env=env, capture_output=True, text=True, timeout=60).stdout.strip()
        record["lift_joint_read_m"] = None if lift == "NONE" else float(lift)
        png = os.path.join(outdir, f"rviz_{stop}_{bp}.png")
        subprocess.run([sys.executable, os.path.join(HERE, "xgrab.py"), win, png], check=True)
        record["png"], record["window"] = os.path.basename(png), win
        record["ok"] = record["lift_joint_read_m"] is not None and abs(record["lift_joint_read_m"] - STOPS[stop]) < 1e-6
    finally:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=30)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            record["stack_needed_sigkill"] = True
        time.sleep(3)
        left = pids_in_domain(domain)
        for pid in left:
            try:
                os.kill(pid, signal.SIGKILL)
            except OSError:
                pass
        record["left_running_after_stop"] = len(left)
    json.dump(record, open(os.path.join(outdir, f"rviz_{stop}_{bp}.json"), "w"), indent=1)
    print(json.dumps(record))
    return record


if __name__ == "__main__":
    outdir = sys.argv[1]
    os.makedirs(outdir, exist_ok=True)
    wanted = sys.argv[2:] or [f"{s}:{b}" for b in ("bp000", "bp050") for s in STOPS]
    for i, item in enumerate(wanted):
        stop, bp = item.split(":")
        shoot(stop, bp, 171 + i, outdir)
