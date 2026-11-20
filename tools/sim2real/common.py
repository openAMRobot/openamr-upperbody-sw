# SPDX-License-Identifier: MIT
"""Shared parts of the sim-to-real bench scripts (docs/sim_to_real_protocol.md).

Nothing here bypasses a stop or talks to a motor directly: every motion is a goal or a Twist sent to a controller that already
exists (arm_controllers.yaml, the platform-sw base), and the scripts never touch the hardwired E-stop chain.

    --target sim|robot   required. robot asks the operator before EVERY motion; sim does not ask.
    --dry-run            nothing is sent: every publish and every goal is logged to the session and dropped.
    --session-root DIR   where the session directory goes (default ./sim2real_sessions).

The speed caps are fixed in this file (protocol section "Speed caps"); a request above a cap is clamped down to it and the
clamp is written to the session log, never silently widened.

Standard library, numpy and PyYAML only; rclpy is imported only by the parts that talk to ROS, so the pure functions (caps,
geometry, CSV, session files) import and test without ROS.
"""
import argparse
import csv
import datetime
import json
import math
import os
import re
import signal
import subprocess
import sys

import yaml

# Speed caps (protocol "Preconditions and safety"). Upper bounds: a script may go slower, never faster.
BASE_MAX_LINEAR_MPS = 0.2
BASE_MAX_ANGULAR_RADPS = 0.5
ARM_MAX_VELOCITY_SCALING = 0.1
GRIPPER_MAX_EFFORT = 5.0

# Names this repository defines for every profile (ros2/src/openamrobot_upperbody_control/config/arm_controllers.yaml: one file
# for mock, gazebo and real, ADR-003). Base and lift names are NOT here: the base is platform-sw's and the real lift has no
# interface in this repository (ros2_control.xacro, openamr_lift_ros2_control: "real nothing"). Those are CLI arguments.
ARM_CONTROLLER = {"left": "/left_arm_controller", "right": "/right_arm_controller"}
GRIPPER_ACTION = {"left": "/left_gripper_controller/gripper_cmd", "right": "/right_gripper_controller/gripper_cmd"}
LIFT_ACTION_SIM = "/lift_controller/follow_joint_trajectory"      # mock and gazebo profiles only
SIM_CMD_VEL = "/cmd_vel"   # platform-sw ros2/src/openamrobot_gazebo/config/gz_bridge.yaml (ROS_TO_GZ, DiffDrive)
SIM_ODOM = "/odom"         # same file, GZ_TO_ROS from the DiffDrive plugin. Odometry is recorded, never the reference.
GZ_MODEL = "openamrobot"

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))


class CapLog:
    """Collects every clamp so the session can say what was asked for and what was sent."""
    def __init__(self):
        self.events = []

    def note(self, what, asked, sent):
        self.events.append({"what": what, "asked": asked, "sent": sent})


def clamp_abs(value, cap, what, log=None):
    """Clamp |value| to cap, keeping the sign. A non-finite value is refused (ValueError), never clamped to the cap."""
    if not math.isfinite(value):
        raise ValueError(f"{what}: non-finite request {value!r}")
    sent = max(-cap, min(cap, value))
    if sent != value and log is not None:
        log.note(what, value, sent)
    return sent


def cap_base_twist(linear_x, angular_z, log=None):
    return (clamp_abs(linear_x, BASE_MAX_LINEAR_MPS, "base linear.x m/s", log),
            clamp_abs(angular_z, BASE_MAX_ANGULAR_RADPS, "base angular.z rad/s", log))


def cap_velocity_scaling(scaling, log=None):
    if not math.isfinite(scaling) or scaling <= 0.0:
        raise ValueError(f"velocity scaling must be > 0, got {scaling!r}")
    return clamp_abs(scaling, ARM_MAX_VELOCITY_SCALING, "arm velocity scaling", log)


def cap_gripper_effort(effort, log=None):
    if not math.isfinite(effort) or effort <= 0.0:
        raise ValueError(f"gripper max_effort must be > 0, got {effort!r}")
    return clamp_abs(effort, GRIPPER_MAX_EFFORT, "gripper max_effort", log)


def segment_duration(q_from, q_to, vel_limits, scaling):
    """Time for a joint-space segment at `scaling` of each joint's URDF velocity limit (the slowest joint decides).
    The same meaning as MoveIt's max_velocity_scaling_factor for a single straight segment."""
    worst = 0.0
    for a, b, v in zip(q_from, q_to, vel_limits):
        if v <= 0.0:
            raise ValueError("velocity limit must be > 0")
        worst = max(worst, abs(b - a) / (v * scaling))
    return worst


# ---------------------------------------------------------------------------------------------------------------- arguments
def base_parser(description, robot_required=()):
    """robot_required: (flag, help) pairs that have NO default and must be given with --target robot (names not in the repo)."""
    p = argparse.ArgumentParser(description=description)
    p.add_argument("--target", choices=["sim", "robot"], required=True)
    p.add_argument("--dry-run", action="store_true", help="send nothing; log what would be sent")
    p.add_argument("--session-root", default="sim2real_sessions")
    p.add_argument("--operator", default="", help="name of the operator at the stop (robot: required)")
    p.add_argument("--note", default="")
    for flag, hlp in robot_required:
        p.add_argument(flag, default=None, help=hlp + " REQUIRED with --target robot; no default (not defined in this repository)")
    p._robot_required = [f for f, _ in robot_required]
    return p


def parse(p, argv=None):
    a = p.parse_args(argv)
    if a.target == "robot":
        missing = [f for f in p._robot_required if getattr(a, f.lstrip("-").replace("-", "_")) in (None, "")]
        if not a.operator:
            missing.append("--operator")
        if missing:
            p.error("--target robot needs " + ", ".join(missing))
    return a


# ---------------------------------------------------------------------------------------------------------------- session
def git_sha(path):
    try:
        out = subprocess.run(["git", "-C", path, "rev-parse", "HEAD"], capture_output=True, text=True, timeout=10)
        dirty = subprocess.run(["git", "-C", path, "status", "--porcelain"], capture_output=True, text=True, timeout=10)
        return {"sha": out.stdout.strip() or None, "dirty": bool(dirty.stdout.strip()) if out.returncode == 0 else None}
    except (OSError, subprocess.SubprocessError):
        return {"sha": None, "dirty": None}


def config_id(xacro_args=("hardware_profile:=gazebo",)):
    """sha256 config id from ros2/src/openamrobot_upperbody_description/scripts/print_config_id.py (needs the ROS env).
    Returns (id or None, the command, stderr tail)."""
    script = os.path.join(REPO_ROOT, "ros2", "src", "openamrobot_upperbody_description", "scripts", "print_config_id.py")
    cmd = [sys.executable, script, *xacro_args]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
    except (OSError, subprocess.SubprocessError) as e:
        return None, " ".join(cmd), str(e)
    m = re.search(r"sha256:[0-9a-f]{64}", r.stdout)
    return (m.group(0) if m else None), " ".join(cmd), r.stderr[-400:]


class Session:
    def __init__(self, test, args, extra_repos=(), config_args=None):
        stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self.dir = os.path.join(args.session_root, f"{stamp}_{test}_{args.target}")
        os.makedirs(self.dir, exist_ok=False)
        self.test, self.args, self.caps, self.sent = test, args, CapLog(), []
        cid, cmd, err = config_id(config_args or (("hardware_profile:=gazebo",) if args.target == "sim" else ("hardware_profile:=real",)))
        repos = {"openamr-upperbody-sw": git_sha(REPO_ROOT)}
        for name, path in extra_repos:
            repos[name] = git_sha(path)
        self.meta = {"test": test, "target": args.target, "dry_run": args.dry_run, "started_utc": stamp,
                     "operator": args.operator, "note": args.note,
                     "argv": sys.argv, "config_id": cid, "config_id_command": cmd, "config_id_stderr": err if not cid else "",
                     "repos": repos, "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"), "gz_partition": os.environ.get("GZ_PARTITION"),
                     "caps": {"base_linear_mps": BASE_MAX_LINEAR_MPS, "base_angular_radps": BASE_MAX_ANGULAR_RADPS,
                              "arm_velocity_scaling": ARM_MAX_VELOCITY_SCALING, "gripper_max_effort": GRIPPER_MAX_EFFORT}}
        self.write_meta()

    def path(self, name):
        return os.path.join(self.dir, name)

    def write_meta(self, **more):
        self.meta.update(more)
        self.meta["cap_events"] = self.caps.events
        self.meta["sent_count"] = len(self.sent)
        with open(self.path("session.yaml"), "w") as f:
            yaml.safe_dump(self.meta, f, sort_keys=False)

    def log_sent(self, what, payload):
        self.sent.append({"what": what, "payload": payload, "dry_run": self.args.dry_run})
        with open(self.path("sent.jsonl"), "a") as f:
            f.write(json.dumps(self.sent[-1]) + "\n")


def write_csv(path, header, rows):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        for r in rows:
            w.writerow(r)


def write_metrics(session, rows):
    """rows: (test, metric, value, unit, source). The file compare.py reads."""
    write_csv(session.path("metrics.csv"), ["test", "metric", "value", "unit", "source"], rows)


# ---------------------------------------------------------------------------------------------------------------- operator
def confirm(args, what, ask=input):
    """robot: the operator types 'yes' before every motion; anything else aborts the script. sim: no question."""
    if args.target != "robot":
        return True
    ans = ask(f"\n[operator {args.operator}] about to: {what}\n  E-stop in hand, area clear? type 'yes' to send: ").strip().lower()
    if ans != "yes":
        raise SystemExit(f"operator did not confirm: {what}")
    return True


class StopGuard:
    """Ctrl+C: run every registered stop action (cancel goals, zero Twist) once, then exit 130. Registered actions must be
    idempotent. Installed as the SIGINT handler and also callable directly."""
    def __init__(self):
        self.actions, self.fired = [], False
        signal.signal(signal.SIGINT, self._handler)

    def add(self, fn):
        self.actions.append(fn)

    def stop(self):
        if self.fired:
            return
        self.fired = True
        for fn in self.actions:
            try:
                fn()
            except Exception as e:      # a stop action must not stop the next one
                print(f"stop action failed: {e}", file=sys.stderr)

    def _handler(self, signum, frame):
        print("\nCtrl+C: cancelling goals and sending zero velocity", file=sys.stderr)
        self.stop()
        raise SystemExit(130)


# ---------------------------------------------------------------------------------------------------------------- geometry
def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def unwrap_sum(yaws):
    """Total signed rotation along a sampled yaw sequence (handles turns past +-pi)."""
    return sum(wrap(b - a) for a, b in zip(yaws, yaws[1:]))


def floor_turn_angle(a0, b0, a1, b1):
    """T3 by trigonometry. a0, b0: floor marks (x, y) under corners A and B before the turn; a1, b1: after.
    theta = atan2(u0 x u1, u0 . u1) with u = B - A. Signed, CCW positive, in (-pi, pi]."""
    u0 = (b0[0] - a0[0], b0[1] - a0[1])
    u1 = (b1[0] - a1[0], b1[1] - a1[1])
    return math.atan2(u0[0] * u1[1] - u0[1] * u1[0], u0[0] * u1[0] + u0[1] * u1[1])


# ---------------------------------------------------------------------------------------------------------------- Gazebo
_POSE_RE = re.compile(r"^\s*- Pose \[ XYZ \(m\) \] \[ RPY \(rad\) \]:\s*\n\s*\[([^\]]+)\]\s*\n\s*\[([^\]]+)\]", re.M)


def parse_gz_pose(text):
    """The '- Pose' block of `gz model -m M -p` / `-l LINK` (not 'Inertial Pose'). Returns (xyz, rpy)."""
    m = _POSE_RE.search(text)
    if not m:
        raise ValueError("no '- Pose' block")
    return [float(v) for v in m.group(1).split()], [float(v) for v in m.group(2).split()]


def gz_pose(link=None, model=GZ_MODEL):
    """Gazebo ground truth (sim only). Needs GZ_PARTITION of the running simulation."""
    cmd = ["gz", "model", "-m", model] + (["-l", link] if link else ["-p"])
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
    xyz, rpy = parse_gz_pose(r.stdout)
    return {"xyz": xyz, "rpy": rpy, "cmd": " ".join(cmd)}


# ---------------------------------------------------------------------------------------------------------------- URDF
def load_urdf_joints(urdf_xml):
    """{joint: {type, parent, child, xyz, rpy, axis, lower, upper, velocity, effort}} and {link: (mass, com_xyz, com_rpy)}."""
    import xml.etree.ElementTree as ET
    root = ET.fromstring(urdf_xml)
    joints, links = {}, {}
    for j in root.findall("joint"):
        o, ax, lim = j.find("origin"), j.find("axis"), j.find("limit")
        joints[j.get("name")] = {
            "type": j.get("type"), "parent": j.find("parent").get("link"), "child": j.find("child").get("link"),
            "xyz": [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()],
            "rpy": [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()],
            "axis": [float(v) for v in (ax.get("xyz") if ax is not None else "1 0 0").split()],
            "lower": float(lim.get("lower", "nan")) if lim is not None else float("nan"),
            "upper": float(lim.get("upper", "nan")) if lim is not None else float("nan"),
            "velocity": float(lim.get("velocity", "nan")) if lim is not None else float("nan"),
            "effort": float(lim.get("effort", "nan")) if lim is not None else float("nan"),
            "mimic": (j.find("mimic").get("joint"), float(j.find("mimic").get("multiplier", "1")), float(j.find("mimic").get("offset", "0")))
            if j.find("mimic") is not None else None}
    for l in root.findall("link"):
        i = l.find("inertial")
        if i is None:
            continue
        o = i.find("origin")
        links[l.get("name")] = (float(i.find("mass").get("value")),
                                [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()],
                                [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()])
    return joints, links


def _rot(rpy):
    import numpy as np
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _axis_rot(axis, q):
    import numpy as np
    a = np.array(axis, float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * K @ K


def forward_kinematics(joints, q, root):
    """World (root-frame) 4x4 transforms of every link below `root`; q: {joint: position}; mimic joints follow their leader.
    Also returns, per movable joint, its axis in the root frame and its origin point."""
    import numpy as np
    children = {}
    for n, j in joints.items():
        children.setdefault(j["parent"], []).append(n)
    T = {root: np.eye(4)}
    jaxis = {}
    stack = [root]
    while stack:
        link = stack.pop()
        for n in children.get(link, []):
            j = joints[n]
            A = np.eye(4)
            A[:3, :3] = _rot(j["rpy"])
            A[:3, 3] = j["xyz"]
            Tj = T[link] @ A
            v = q.get(n, 0.0)
            if j["mimic"]:
                lead, mul, off = j["mimic"]
                v = q.get(lead, 0.0) * mul + off
            M = np.eye(4)
            if j["type"] in ("revolute", "continuous"):
                M[:3, :3] = _axis_rot(j["axis"], v)
            elif j["type"] == "prismatic":
                M[:3, 3] = np.array(j["axis"], float) / np.linalg.norm(j["axis"]) * v
            if j["type"] in ("revolute", "continuous", "prismatic"):
                jaxis[n] = (Tj[:3, :3] @ (np.array(j["axis"], float) / np.linalg.norm(j["axis"])), Tj[:3, 3].copy(), j["type"])
            T[j["child"]] = Tj @ M
            stack.append(j["child"])
    return T, jaxis


def gravity_torques(joints, links, q, root, joint_names, g=(0.0, 0.0, -9.80665)):
    """Static model torque each joint must supply to hold `q` against gravity (N*m; N for prismatic), from the URDF masses
    and centres of mass only (no friction, no armature, no contact). tau_i = -axis_i . sum_{links below i} (c - o_i) x (m g)."""
    import numpy as np
    T, jaxis = forward_kinematics(joints, q, root)
    below = {}
    for n in joint_names:
        # links in the subtree of joint n
        sub, stack = [], [joints[n]["child"]]
        while stack:
            l = stack.pop()
            sub.append(l)
            stack += [joints[m]["child"] for m in joints if joints[m]["parent"] == l]
        below[n] = sub
    gv = np.array(g)
    out = {}
    for n in joint_names:
        axis, origin, typ = jaxis[n]
        tau = 0.0
        for l in below[n]:
            if l not in links or l not in T:
                continue
            m, cxyz, _ = links[l]
            c = T[l][:3, :3] @ np.array(cxyz) + T[l][:3, 3]
            F = m * gv
            tau += -float(axis @ np.cross(c - origin, F)) if typ != "prismatic" else -float(axis @ F)
        out[n] = tau
    return out


def fingertip_gap(urdf_xml, side, angle):
    """Model jaw opening (m): smallest distance between the distal thirds of the two finger collision meshes of one hand, finger_joint1
    at `angle`, finger_joint2 by its <mimic>. The method of ros2/src/openamrobot_upperbody_moveit/test/test_gripper_sign.py
    fingertip_gap (M05), re-implemented here so this tool does not import a test module. Needs ament_index_python for package://."""
    import struct
    import xml.etree.ElementTree as ET
    import numpy as np
    from ament_index_python.packages import get_package_share_directory
    root = ET.fromstring(urdf_xml)

    def floats(text, default):
        return tuple(float(v) for v in text.split()) if text else default

    tips = []
    for index in (1, 2):
        j = root.find(f"joint[@name='openarm_{side}_finger_joint{index}']")
        mim = j.find("mimic")
        q = angle if mim is None else angle * float(mim.get("multiplier", "1")) + float(mim.get("offset", "0"))
        o = j.find("origin")
        R, t = _rot(floats(o.get("rpy"), (0, 0, 0))), np.array(floats(o.get("xyz"), (0, 0, 0)))
        turn = _axis_rot(floats(j.find("axis").get("xyz"), (1, 0, 0)), q)
        pts = []
        for c in root.find(f"link[@name='{j.find('child').get('link')}']").findall("collision"):
            mesh = c.find("geometry/mesh")
            co = c.find("origin")
            cR = _rot(floats(co.get("rpy") if co is not None else None, (0, 0, 0)))
            ct = np.array(floats(co.get("xyz") if co is not None else None, (0, 0, 0)))
            scale = np.array(floats(mesh.get("scale"), (1, 1, 1)))
            pkg, rel = re.fullmatch(r"package://([^/]+)/(.*)", mesh.get("filename")).groups()
            data = open(os.path.join(get_package_share_directory(pkg), rel), "rb").read()
            n = struct.unpack("<I", data[80:84])[0]
            v = np.array([struct.unpack_from("<3f", data, 84 + 50 * i + 12 + 12 * k) for i in range(n) for k in range(3)])[::5]
            pts.append(((R @ (turn @ (cR @ (v * scale).T + ct[:, None]))) + t[:, None]).T)
        P = np.vstack(pts)
        cut = np.sort(P[:, 2])[len(P) // 3]
        tips.append(P[P[:, 2] <= cut])
    a, b = tips[0], tips[1][::3]
    return float(min(np.min(np.linalg.norm(b - p, axis=1)) for p in a))


def datum(name):
    """Value of a datum in ros2/src/openamrobot_upperbody_description/config/datums.yaml (never a literal in a script)."""
    path = os.path.join(REPO_ROOT, "ros2", "src", "openamrobot_upperbody_description", "config", "datums.yaml")
    with open(path) as f:
        d = yaml.safe_load(f)
    for section in d.values():
        if isinstance(section, dict) and name in section and isinstance(section[name], dict):
            return section[name]["value"]
    raise KeyError(name)
