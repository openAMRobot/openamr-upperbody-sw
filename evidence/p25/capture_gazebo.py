#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""One Gazebo GUI screenshot of the combined model at the lift's zero stop (L1000) (P25). Real `gz sim` with its window on the real display
(WSLg, :0), captured with the RViz capture's xgrab.py. Same discipline as capture_rviz.py: a private ROS_DOMAIN_ID and GZ_PARTITION, the
launch in its own process group (SIGINT first), anything left in the domain or the partition killed by pid, and the record says what was left.

    source /opt/ros/jazzy/setup.bash && source <ws>/install/setup.bash
    python3 capture_gazebo.py <outdir> [bp050]

Writes <outdir>/gazebo_L1000_<bp>.png and .json (the lift joint as the simulator reports it, the camera pose that was asked for)."""
import json
import math
import os
import re
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
XGRAB = os.path.join(HERE, "..", "renders", "rviz", "xgrab.py")
SETTLE_S = 45
DOMAIN, PARTITION = "181", "p25_gui"
# Where to put the camera: a point in front-left of the robot, looking at the column and the arm profile.
EYE, TARGET = (1.05, -0.9, 1.0), (0.2, 0.0, 0.8)

READ_LIFT = """
import rclpy
from sensor_msgs.msg import JointState
rclpy.init(); n = rclpy.create_node('p25_lift_reader'); out = []
n.create_subscription(JointState, '/joint_states', lambda m: out.append(m.position[m.name.index('lift_joint')]) if 'lift_joint' in m.name else None, 10)
end = __import__('time').time() + 25
while rclpy.ok() and not out and __import__('time').time() < end: rclpy.spin_once(n, timeout_sec=0.5)
print(out[0] if out else 'NONE')
"""


def quaternion_look_at(eye, target):
    dx, dy, dz = (t - e for t, e in zip(target, eye))
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(-dz, math.hypot(dx, dy))        # gz: positive pitch looks down
    cy, sy, cp, sp = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(pitch / 2), math.sin(pitch / 2)
    return (-sy * sp, cy * sp, sy * cp, cy * cp)   # roll 0: (x, y, z, w) of Rz(yaw) Ry(pitch)


def window_id():
    tree = subprocess.run(["xwininfo", "-root", "-tree"], capture_output=True, text=True).stdout
    for line in tree.splitlines():
        if "Gazebo" in line and "gz" not in line.split('"')[0] and re.search(r"\d+x\d+", line) and "1x1" not in line:
            m = re.search(r"0x[0-9a-f]+", line)
            if m:
                return m.group(0), line.strip()
    return None, None


def pids_with_env(*needles):
    found = []
    for p in os.listdir("/proc"):
        if p.isdigit() and int(p) != os.getpid():
            try:
                env = open(f"/proc/{p}/environ", "rb").read().split(b"\0")
            except OSError:
                continue
            if all(n.encode() in env for n in needles):
                found.append(int(p))
    return found


def main(outdir, bp):
    os.makedirs(outdir, exist_ok=True)
    env = dict(os.environ, ROS_DOMAIN_ID=DOMAIN, ROS_LOCALHOST_ONLY="1", GZ_PARTITION=PARTITION, PYTHONDONTWRITEBYTECODE="1")
    log = open(os.path.join(outdir, f"gazebo_L1000_{bp}.launch.log"), "w")
    proc = subprocess.Popen(["ros2", "launch", "openamrobot_upperbody_gazebo", "sim.launch.py", f"base_plate:={bp}", "headless:=false"],
                            env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
                            preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_DFL))
    record = {"stop": "L1000", "base_plate": bp, "ros_domain_id": int(DOMAIN), "gz_partition": PARTITION, "camera_eye": EYE, "camera_target": TARGET}
    try:
        t0, win = time.time(), None
        while time.time() - t0 < 150 and win is None:
            time.sleep(3)
            win, title = window_id()
        if win is None:
            raise RuntimeError("no Gazebo window within 150 s")
        record["window_line"] = title
        time.sleep(SETTLE_S)
        x, y, z, w = quaternion_look_at(EYE, TARGET)
        req = f"pose: {{position: {{x: {EYE[0]}, y: {EYE[1]}, z: {EYE[2]}}} orientation: {{x: {x}, y: {y}, z: {z}, w: {w}}}}}"
        moved = subprocess.run(["gz", "service", "-s", "/gui/move_to/pose", "--reqtype", "gz.msgs.GUICamera", "--reptype", "gz.msgs.Boolean",
                                "--timeout", "5000", "--req", req], env=env, capture_output=True, text=True)
        record["move_to_pose"] = (moved.stdout + moved.stderr).strip()
        time.sleep(8)
        lift = subprocess.run([sys.executable, "-c", READ_LIFT], env=env, capture_output=True, text=True, timeout=60).stdout.strip()
        record["lift_joint_read_m"] = None if lift == "NONE" else float(lift)
        png = os.path.join(outdir, f"gazebo_L1000_{bp}.png")
        subprocess.run([sys.executable, XGRAB, win, png], check=True)
        record["png"], record["window"] = os.path.basename(png), win
        record["ok"] = record["lift_joint_read_m"] is not None and abs(record["lift_joint_read_m"]) < 1e-3
    finally:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=40)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            record["launch_needed_sigkill"] = True
        time.sleep(3)
        left = pids_with_env(f"ROS_DOMAIN_ID={DOMAIN}") + pids_with_env(f"GZ_PARTITION={PARTITION}")
        left = sorted(set(left))
        record["left_running_after_stop"] = len(left)
        for pid in left:
            try:
                os.kill(pid, signal.SIGKILL)
            except OSError:
                pass
    json.dump(record, open(os.path.join(outdir, f"gazebo_L1000_{bp}.json"), "w"), indent=1)
    print(json.dumps(record))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else "bp050")
