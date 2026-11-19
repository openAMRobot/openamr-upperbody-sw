#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Bind a recorded Baseline 1 (or grocery) ROS bag to the C9 capture channels, read only (Proposed).

The bag-to-LeRobot converter is openamrobot-embodied-ai's `oa-convert-bag` (EMBODIED-AI-REPO; Satya's capture
conversion work). This tool does not convert and does not write a dataset. It reads one recorded bag and states, for
every channel of the Proposed C9 component profile (openamrobot-interfaces #39) and for the four C9 camera features,
which recorded topic and field carries it, at what rate, in which unit and within which range, or why nothing does.
It also writes a candidate capture profile in the converter's `capture_version: 1` shape, with the run's provenance
(run id, commit SHAs, acceptance profile, pose_source and object_pose_source as declared by /build1/run_info), so the
converter's own gates can be run against it unchanged (.github/workflows/bag-to-lerobot.yml).

Exit status: 0 when the report was written (whatever it says), 2 when the bag or an input cannot be read.
Simulation evidence only: never hardware, physical acceptance or safety evidence.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import sys
from pathlib import Path

# The Build 1 lift command (/sil/lift/command, std_msgs/Int8) is a stop index; rig.py LIFT_STOP_M gives its target.
LIFT_STOP_M = (0.0, 0.175, 0.350)
LIFT_STOP_ID = ("L1000", "L1175", "L1350")
# Recorded JointState topics, in the order a joint is looked up (the component profile names /arm/joint_states).
JOINT_STATE_TOPICS = ("/arm/joint_states", "/joint_states", "/sil/gz/joint_states")
# C9 camera feature -> (image topic, camera_info topic) of build1_robot.urdf.xacro part 3, as the opt-in AI capture of
# baseline1_record.py records them. The base camera is the RGB-D sensor 3a, not the simulation-only station camera
# (/base_camera/color/image_raw, part 4), which shares its frame.
C9_CAMERAS = {
    "observation.images.camera_wrist_left": ("/left_wrist_camera/image", "/left_wrist_camera/camera_info"),
    "observation.images.camera_wrist_right": ("/right_wrist_camera/image", "/right_wrist_camera/camera_info"),
    "observation.images.head": ("/head_camera/rgbd/image", "/head_camera/rgbd/camera_info"),
    "observation.images.base": ("/base_camera/rgbd/image", "/base_camera/rgbd/camera_info"),
}
IMAGE_TYPES = ("sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage")
# Held gripper command of the opt-in AI capture (baseline1_record.py GripperCommandHold, Proposed).
GRIPPER_COMMAND_TOPIC = "/build1/capture/gripper_command"
# Arm emulator status: mit_q is the gripper motor's last commanded position as received over CAN (motor side).
EMULATOR_STATUS_TOPICS = ("/sim/left_arm_emulator/status", "/sim/right_arm_emulator/status")
ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
ACCEPTANCE_LINE = re.compile(r"acceptance profile: ([a-z-]+), pose_source: ([a-z_]+)")
OBJECT_POSE_LINE = re.compile(r"\]: object_pose_source: ([a-z_]+)")
# Upper end of the pinned driver's parallel_link (prismatic) finger range, metres (openarm_ros2 4e837e1).
PARALLEL_LINK_MAX_M = 0.044
RANGE_TOL = 1e-3


class BagError(Exception):
    pass


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def stamp_ns(msg):
    header = getattr(msg, "header", None)
    if header is None:
        return None
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def object_pose_source_from_log(text):
    """object_pose_source as the launch logs it ('[launch.user]: object_pose_source: ground_truth (...)'), or None."""
    for line in text.splitlines():
        m = OBJECT_POSE_LINE.search(ANSI.sub("", line))
        if m:
            return m.group(1)
    return None


def acceptance_from_log(text):
    """(acceptance profile, pose_source) from the echo line of baseline1-acceptance.yml, or (None, None).

    The job log has a timestamp prefix and may carry terminal colour codes; the echoed command itself (with
    ${BASELINE1_PROFILE}) never matches."""
    for line in text.splitlines():
        m = ACCEPTANCE_LINE.search(ANSI.sub("", line))
        if m:
            return m.group(1), m.group(2)
    return None, None


class Stamps:
    """Timestamps of one topic as the converter's read_bag orders them: header stamp, else bag receipt time."""

    def __init__(self):
        self.header = {"last": None, "nonincreasing": 0, "equal": 0, "decreasing": 0}
        self.receipt = {"last": None, "nonincreasing": 0, "equal": 0, "decreasing": 0}

    @staticmethod
    def _add(slot, t):
        if slot["last"] is not None and t <= slot["last"]:
            slot["nonincreasing"] += 1
            slot["equal" if t == slot["last"] else "decreasing"] += 1
        slot["last"] = t if slot["last"] is None else max(slot["last"], t)

    def add(self, receipt, header_t):
        self._add(self.receipt, receipt)
        if header_t is not None:
            self._add(self.header, header_t)

    def summary(self, has_header):
        return {"has_header": has_header, "receipt_nonincreasing": self.receipt["nonincreasing"],
                "receipt_equal": self.receipt["equal"], "receipt_decreasing": self.receipt["decreasing"],
                "header_nonincreasing": self.header["nonincreasing"] if has_header else None,
                "header_equal": self.header["equal"] if has_header else None,
                "header_decreasing": self.header["decreasing"] if has_header else None,
                "strictly_increasing_as_read": (self.header if has_header else self.receipt)["nonincreasing"] == 0}


class Series:
    """Count, time span and value range of one recorded quantity."""

    def __init__(self):
        self.n, self.t0, self.t1, self.lo, self.hi = 0, None, None, math.inf, -math.inf
        self.values = set()
        self.max_header_lag_ns = None

    def add(self, t, value, header_t=None):
        self.n += 1
        self.t0 = t if self.t0 is None else min(self.t0, t)
        self.t1 = t if self.t1 is None else max(self.t1, t)
        if value is not None and math.isfinite(value):
            self.lo, self.hi = min(self.lo, value), max(self.hi, value)
            if len(self.values) < 16:
                self.values.add(round(value, 6))
        if header_t is not None:
            lag = abs(t - header_t)
            self.max_header_lag_ns = lag if self.max_header_lag_ns is None else max(self.max_header_lag_ns, lag)

    def summary(self):
        rate = (self.n - 1) / ((self.t1 - self.t0) * 1e-9) if self.n > 1 and self.t1 > self.t0 else None
        return {"samples": self.n, "rate_hz": None if rate is None else round(rate, 2),
                "min": None if self.n == 0 or self.lo == math.inf else round(self.lo, 6),
                "max": None if self.n == 0 or self.hi == -math.inf else round(self.hi, 6),
                "max_receipt_minus_header_ns": self.max_header_lag_ns}


def read(bag_dir, component_profile):
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore

    bag_dir = Path(bag_dir)
    if not (bag_dir / "metadata.yaml").is_file():
        raise BagError(f"no rosbag2 metadata.yaml in {bag_dir}")
    joints = [c["joint"] for c in component_profile["channels"] if c.get("joint")]
    state = {}            # (topic, joint) -> Series of position
    command = {}          # (topic, joint) -> Series of reference position (JointTrajectoryControllerState)
    other = {}            # name -> Series
    frames, run_info, topics, stamps, camera_info, motor_command = {}, None, {}, {}, {}, {}
    with AnyReader([bag_dir], default_typestore=get_typestore(Stores.ROS2_JAZZY)) as reader:
        for c in reader.connections:
            topics[c.topic] = {"type": c.msgtype, "count": c.msgcount, "bytes": 0}
        start, end = reader.start_time, reader.end_time
        for conn, _t, raw in reader.messages():      # size of every topic; nothing is deserialized here
            topics[conn.topic]["bytes"] += len(raw)
        wanted = [c for c in reader.connections if c.msgtype in (
            "sensor_msgs/msg/JointState", "control_msgs/msg/JointTrajectoryControllerState", "geometry_msgs/msg/Twist",
            "std_msgs/msg/Int8", "std_msgs/msg/String", "nav_msgs/msg/Odometry", "sensor_msgs/msg/CameraInfo",
            "diagnostic_msgs/msg/DiagnosticArray", *IMAGE_TYPES)
            and c.topic != "/build1/rtf_stats"]
        for conn, t, raw in reader.messages(connections=wanted):
            msg = reader.deserialize(raw, conn.msgtype)
            ht = stamp_ns(msg)
            if ht is not None:
                frames.setdefault(conn.topic, msg.header.frame_id)
            stamps.setdefault(conn.topic, Stamps()).add(t, ht)
            if conn.msgtype == "sensor_msgs/msg/CameraInfo":
                camera_info.setdefault(conn.topic, {"frame_id": msg.header.frame_id, "width": int(msg.width),
                                                    "height": int(msg.height), "k": [round(float(v), 4) for v in msg.k],
                                                    "distortion_model": msg.distortion_model})
                continue
            if conn.msgtype == "diagnostic_msgs/msg/DiagnosticArray":
                if conn.topic in EMULATOR_STATUS_TOPICS:
                    for status in msg.status:
                        joint = status.name.rsplit("/", 1)[-1]
                        if joint in joints:
                            value = {kv.key: kv.value for kv in status.values}.get("mit_q")
                            try:
                                motor_command.setdefault((conn.topic, joint), Series()).add(t, float(value), ht)
                            except (TypeError, ValueError):
                                pass
                continue
            if conn.msgtype == "sensor_msgs/msg/JointState":
                if conn.topic == GRIPPER_COMMAND_TOPIC:
                    for i, name in enumerate(msg.name):
                        if name in joints and i < len(msg.position):
                            command.setdefault((conn.topic, name), Series()).add(t, float(msg.position[i]), ht)
                    continue
                if conn.topic not in JOINT_STATE_TOPICS:
                    continue
                for i, name in enumerate(msg.name):
                    if name in joints and i < len(msg.position):
                        state.setdefault((conn.topic, name), Series()).add(t, float(msg.position[i]), ht)
            elif conn.msgtype == "control_msgs/msg/JointTrajectoryControllerState":
                ref = getattr(msg, "reference", None)
                positions = list(getattr(ref, "positions", None) if ref is not None and getattr(ref, "positions", None) is not None else [])
                for i, name in enumerate(msg.joint_names):
                    if name in joints and i < len(positions):
                        command.setdefault((conn.topic, name), Series()).add(t, float(positions[i]), ht)
            elif conn.msgtype == "geometry_msgs/msg/Twist":
                other.setdefault(conn.topic + ":linear.x", Series()).add(t, float(msg.linear.x))
                other.setdefault(conn.topic + ":angular.z", Series()).add(t, float(msg.angular.z))
            elif conn.msgtype == "nav_msgs/msg/Odometry":
                other.setdefault(conn.topic + ":twist.twist.linear.x", Series()).add(t, float(msg.twist.twist.linear.x), ht)
                other.setdefault(conn.topic + ":twist.twist.angular.z", Series()).add(t, float(msg.twist.twist.angular.z), ht)
            elif conn.msgtype == "std_msgs/msg/Int8":
                other.setdefault(conn.topic + ":data", Series()).add(t, float(msg.data))
            elif conn.msgtype == "std_msgs/msg/String" and conn.topic == "/build1/run_info" and run_info is None:
                try:
                    run_info = json.loads(msg.data)
                except ValueError:
                    run_info = {"unparsed": msg.data}
            elif conn.msgtype in IMAGE_TYPES:
                other.setdefault(conn.topic + ":image", Series()).add(t, None, ht)
    return {"topics": topics, "start_ns": start, "end_ns": end, "state": state, "command": command, "other": other,
            "frames": frames, "run_info": run_info, "stamps": {k: v.summary(k in frames) for k, v in stamps.items()},
            "camera_info": camera_info, "motor_command": motor_command}


def bind(data, component_profile):
    """One entry per component-profile channel: observation and action binding, or the reason there is none."""
    rows = []
    for c in component_profile["channels"]:
        name, joint, rng = c["name"], c.get("joint"), c.get("range")
        row = {"index": c["index"], "name": name, "component": c["component"], "unit": c["unit"], "range": rng}
        obs = act = None
        if c["component"] != "base" and joint:
            for topic in JOINT_STATE_TOPICS:
                if (topic, joint) in data["state"]:
                    obs = {"topic": topic, "field": "position", "joint": joint, **data["state"][(topic, joint)].summary()}
                    break
            if c["component"] in ("left_arm", "right_arm"):
                side = c["component"].split("_")[0]
                key = (f"/{side}_arm_controller/controller_state", joint)
                if key in data["command"]:
                    act = {"topic": key[0], "field": "reference.positions", "joint": joint,
                           "semantics": "JointTrajectoryController reference sent to the position command interface",
                           **data["command"][key].summary()}
                else:
                    act = {"unbound": f"no {key[0]} sample names {joint}"}
            elif c["component"].endswith("_gripper"):
                key = (GRIPPER_COMMAND_TOPIC, joint)
                if key in data["command"]:
                    act = {"topic": GRIPPER_COMMAND_TOPIC, "field": "position", "joint": joint, "semantics":
                           "held GripperCommand goal command.position (opt-in AI capture, Proposed); before the side's "
                           "first goal, the finger position measured at capture start", **data["command"][key].summary()}
                else:
                    act = {"unbound": "the gripper is commanded by control_msgs/action/GripperCommand goals "
                                      "(rig.py gripper()); a ROS 2 Jazzy bag records topics, not action goals, and "
                                      f"{GRIPPER_COMMAND_TOPIC} (opt-in AI capture) is not in this bag"}
                trace = [k for k in data["motor_command"] if k[1] == joint]
                if trace:
                    act["motor_side_trace"] = {"topic": trace[0][0], "key": "mit_q", "semantics":
                                               "gripper motor's last MIT position command as received over CAN, motor "
                                               "units; DiagnosticArray strings, not readable by a field path",
                                               **data["motor_command"][trace[0]].summary()}
            elif c["component"] == "lift":
                s = data["other"].get("/sil/lift/command:data")
                if s is not None:
                    stops = sorted(int(v) for v in s.values)
                    act = {"topic": "/sil/lift/command", "field": "data", "semantics":
                           "stop index; target lift_joint position LIFT_STOP_M[index] (rig.py), not a continuous command",
                           "stop_indices": stops, "targets_m": [LIFT_STOP_M[i] for i in stops if 0 <= i < 3],
                           **s.summary()}
                else:
                    act = {"unbound": "no /sil/lift/command in this bag"}
            source = c.get("state_source") or ""
            if obs is not None and source and not source.startswith(obs["topic"] + " "):
                obs["component_profile_state_source"] = source.split(" ")[0]
                obs["state_source_differs_from_profile"] = True
            if obs is not None and rng is not None and obs["min"] is not None:
                obs["within_profile_range"] = (rng[0] - RANGE_TOL <= obs["min"]) and (obs["max"] <= rng[1] + RANGE_TOL)
        else:
            field = {"base_linear_x": "linear.x", "base_angular_z": "angular.z"}.get(name)
            odom = data["other"].get(f"/odom:twist.twist.{field}")
            obs = ({"topic": "/odom", "field": f"twist.twist.{field}", **odom.summary()} if odom else
                   {"unbound": "the profile's /odom is not in this bag",
                    "available_not_bound": "/build1/ground_truth/odom (simulator ground truth, not a robot estimate)"
                    if "/build1/ground_truth/odom" in data["topics"] else None})
            cmd = data["other"].get(f"/cmd_vel:{field}")
            act = ({"topic": "/cmd_vel", "field": field, "semantics": "requested_velocity at the base command input "
                    "(after Nav2's velocity smoother and collision monitor)", **cmd.summary()} if cmd else
                   {"unbound": "no /cmd_vel in this bag"})
        row["observation"] = obs if obs is not None else {"unbound": f"no recorded JointState names {joint}"}
        row["action"] = act if act is not None else {"unbound": f"no command source is known for component {c['component']}"}
        rows.append(row)
    return rows


def gripper_unit(rows):
    """Finger positions above the parallel_link maximum (0.044) cannot be metres of that branch."""
    out = {}
    for r in rows:
        if r["component"].endswith("_gripper") and "min" in r["observation"]:
            peak = max(abs(r["observation"]["min"]), abs(r["observation"]["max"]))
            out[r["name"]] = {"peak_abs": peak, "profile_unit": r["unit"], "range": r["range"],
                              "verdict": "radians (exceeds the 0.044 m parallel_link maximum)" if peak > PARALLEL_LINK_MAX_M
                              else "not distinguishable from metres by value (peak <= 0.044)"}
    return out


def cameras(data):
    images = {t: v for t, v in data["topics"].items() if v["type"] in IMAGE_TYPES}
    out = {}
    for key, (topic, info) in C9_CAMERAS.items():
        if topic in images and topic + ":image" in data["other"]:
            out[key] = {"topic": topic, "type": images[topic]["type"], "frame_id": data["frames"].get(topic),
                        "camera_info_topic": info if info in data["camera_info"] else None,
                        "camera_info": data["camera_info"].get(info), **data["other"][topic + ":image"].summary()}
        else:
            out[key] = {"unbound": f"{topic} is not in the bag (recorded only with the opt-in AI capture)"}
    return {"bound": out, "image_topics_in_bag": sorted(images)}


def candidate_grid(cams, fps, max_skew_ns):
    """Candidate export grid: the converter refuses a reused camera frame, so fps is at most the slowest camera rate;
    max_skew_ns covers half the slowest camera period. Not approved values."""
    rates = [v["rate_hz"] for v in cams["bound"].values() if v.get("rate_hz")]
    if fps is None:
        fps = max(1, int(min(rates))) if len(rates) == len(C9_CAMERAS) else 10
    if max_skew_ns is None:
        max_skew_ns = int(round(0.5e9 / min(rates))) if len(rates) == len(C9_CAMERAS) else 50_000_000
    return fps, max_skew_ns


def pins(path):
    """pins.txt of the Baseline 1 artifact: '<checkout dir> <40-hex SHA>' per line."""
    if not path or not Path(path).is_file():
        return []
    out = []
    for line in Path(path).read_text().splitlines():
        parts = line.split()
        if len(parts) == 2 and len(parts[1]) == 40 and all(ch in "0123456789abcdef" for ch in parts[1]):
            out.append((Path(parts[0]).name, parts[1]))
    return out


def candidate_profile(args, data, rows, component_profile_path, component_profile, cams=None):
    """The converter's capture_version 1 shape. status stays proposed: only an owner registers a profile."""
    info = data["run_info"] or {}
    streams, seen = [], {}

    def stream(topic, component, **extra):
        if topic in seen:
            return seen[topic]
        sid = topic.strip("/").replace("/", "_")
        streams.append({"id": sid, "component": component, "topic": topic, "type": data["topics"][topic]["type"],
                        "frame_id": data["frames"].get(topic, ""), "clock_domain": "ros.sim_time",
                        "timestamp_source": "header" if topic in data["frames"] else "bag",
                        **({} if topic in data["frames"] else {"frame_id_source": "configuration"}), **extra})
        seen[topic] = sid
        return sid

    for key, cam in (cams or {"bound": {}})["bound"].items():
        if "topic" in cam:
            info_ = cam.get("camera_info") or {}
            # A candidate identity of the simulated intrinsics, not a calibration: Gazebo's camera_info, no distortion.
            calibration = (f"gazebo-camera-info:{cam['camera_info_topic']}:{info_.get('width')}x{info_.get('height')}:"
                           f"fx{info_['k'][0]}" if info_.get("k") else None)
            stream(cam["topic"], "camera", camera_key=key, **({"calibration_id": calibration} if calibration else {}))

    state_channels, action_channels = [], []
    for r in rows:
        o, a = r["observation"], r["action"]
        if "topic" in o:
            ch = {"name": r["name"], "component": r["component"], "unit": r["unit"], "source_unit": r["unit"],
                  "field": o["field"], "range": r["range"], "stream": stream(o["topic"], r["component"])}
            if o.get("joint"):
                ch["joint"] = o["joint"]
            state_channels.append(ch)
        if "topic" in a:
            ch = {"name": r["name"], "component": r["component"], "unit": r["unit"], "source_unit": r["unit"],
                  "field": a["field"], "range": r["range"], "stream": stream(a["topic"], r["component"])}
            if a.get("joint"):
                ch["joint"] = a["joint"]
            if r["component"] == "base":
                ch["semantics"] = "requested_velocity"
            if r["component"] == "lift":
                ch["source_unit"] = "stop_index"
            if r["component"].endswith("_gripper"):
                ch["semantics"] = "held_gripper_goal"
            action_channels.append(ch)
    commits = dict(pins(args.pins))
    commits.update({k: v for k, v in {"openamr-upperbody-sw": args.run_sha, "openamr-platform-sw": args.platform_sha,
                                      "openamrobot-interfaces": args.interfaces_sha}.items() if v})
    return {
        "capture_version": 1,
        "profile_id": "openamrobot-baseline1-gazebo-capture-candidate",
        "status": "proposed",
        "origin": "simulation",
        "episode_id": f"baseline1-run-{args.run_id}",
        "dataset_id": "openamrobot/baseline1-gazebo-candidate",
        "dataset_revision": f"run-{args.run_id}",
        "task": "Baseline 1: drive to the work shelf, lift, grasp the item from the tote, lift it, retreat",
        "robot_configuration_id": info.get("robot_configuration_id") or "unknown: not declared in /build1/run_info",
        "fps": args.fps,
        "max_skew_ns": args.max_skew_ns,
        "units": "radians for every angular channel (decision #79); metres for the lift; m/s and rad/s for the base",
        "streams": streams,
        "components": list(dict.fromkeys(r["component"] for r in rows)),
        "state_channels": state_channels,
        "action_channels": action_channels,
        "state_names": [c["name"] for c in state_channels],
        "action_names": [c["name"] for c in action_channels],
        "source_commits": commits,
        "component_contract": {"path": str(component_profile_path), "sha256": sha256_file(component_profile_path)},
        "provenance": {"run_id": args.run_id, "run_head_sha": args.run_sha, "artifact": args.artifact,
                       "launch_profile": info.get("profile"), "acceptance_profile": args.acceptance_profile, "pose_source": info.get("planning_pose_source", info.get("pose_source")),
                       "object_pose_source": info.get("object_pose_source"), "run_info": info,
                       "time": "simulation time (/clock); bag recorded with --use-sim-time"},
    }


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--bag", required=True)
    p.add_argument("--component-profile", required=True, help="interfaces #39 profiles/c9/mobile-lift-qpos-v1-proposed.json")
    p.add_argument("--out", required=True)
    p.add_argument("--run-id", default="local")
    p.add_argument("--run-sha", default=None)
    p.add_argument("--platform-sha", default=None)
    p.add_argument("--interfaces-sha", default=None)
    p.add_argument("--artifact", default=None)
    p.add_argument("--pins", default=None, help="pins.txt of the run artifact")
    p.add_argument("--acceptance-profile", default=None, help="as the run's log declares it (sim-oracle | perception)")
    p.add_argument("--run-log", default=None, help="job log of the recording run; read for the acceptance profile line")
    p.add_argument("--fps", type=int, default=None, help="candidate export grid (default: slowest camera rate, else 10; "
                                                          "not an approved value)")
    p.add_argument("--max-skew-ns", type=int, default=None, help="candidate tolerance (default: half the slowest camera "
                                                                 "period, else 50 ms; not an approved value)")
    args = p.parse_args(argv)
    pose_source_from_log = object_pose_from_log = None
    if args.run_log and Path(args.run_log).is_file():
        log_text = Path(args.run_log).read_text(errors="replace")
        profile_from_log, pose_source_from_log = acceptance_from_log(log_text)
        object_pose_from_log = object_pose_source_from_log(log_text)
        if not args.acceptance_profile:
            args.acceptance_profile = profile_from_log
    if not args.acceptance_profile:
        args.acceptance_profile = "not declared in the run log"
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    try:
        profile_path = Path(args.component_profile).resolve()
        component_profile = json.loads(profile_path.read_text())
        data = read(args.bag, component_profile)
    except (BagError, OSError, ValueError, KeyError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    rows = bind(data, component_profile)
    cams = cameras(data)
    args.fps, args.max_skew_ns = candidate_grid(cams, args.fps, args.max_skew_ns)
    bound_obs = sum("topic" in r["observation"] for r in rows)
    bound_act = sum("topic" in r["action"] for r in rows)
    bound_cam = sum("topic" in v for v in cams["bound"].values())
    sim_time = data["start_ns"] is not None and data["start_ns"] < 10**15 and "/clock" in data["topics"]
    duration_s = None if data["start_ns"] is None else (data["end_ns"] - data["start_ns"]) * 1e-9
    total_bytes = sum(f.stat().st_size for f in Path(args.bag).iterdir() if f.is_file())
    camera_bytes = sum(v["bytes"] for t, v in data["topics"].items() if v["type"] in IMAGE_TYPES)
    size = {"files_mb": round(total_bytes / 1e6, 1), "message_mb": round(sum(v["bytes"] for v in data["topics"].values()) / 1e6, 1),
            "camera_image_mb": round(camera_bytes / 1e6, 1),
            "mb_per_sim_minute": round(total_bytes / 1e6 / (duration_s / 60.0), 1) if duration_s else None,
            "camera_mb_per_sim_minute": round(camera_bytes / 1e6 / (duration_s / 60.0), 1) if duration_s else None}
    report = {
        "status": "Proposed analysis; simulation evidence only",
        "bag": {"path": str(args.bag), "start_ns": data["start_ns"], "end_ns": data["end_ns"],
                "duration_s": None if data["start_ns"] is None else round((data["end_ns"] - data["start_ns"]) * 1e-9, 3),
                "simulation_time": sim_time,
                "files_sha256": {f.name: sha256_file(f) for f in sorted(Path(args.bag).iterdir()) if f.is_file()},
                "size": size, "topics": data["topics"]},
        "acceptance": {"profile": args.acceptance_profile, "pose_source_from_log": pose_source_from_log,
                       "object_pose_source_from_log": object_pose_from_log},
        "timestamps": data["stamps"],
        "run_info": data["run_info"],
        "channels": rows,
        "cameras": cams,
        "gripper_unit": gripper_unit(rows),
        "totals": {"channels": len(rows), "observation_bound": bound_obs, "action_bound": bound_act,
                   "c9_cameras": len(C9_CAMERAS), "c9_cameras_bound": bound_cam},
        "c9_v1_complete": bound_cam == len(C9_CAMERAS) and bound_obs == len(rows) and bound_act == len(rows),
    }
    (out / "binding.json").write_text(json.dumps(report, indent=1) + "\n")
    cand = candidate_profile(args, data, rows, profile_path, component_profile, cams)
    prov = cand["provenance"]
    if data["run_info"] is None:   # /build1/run_info not recorded: take the run log's declarations and say so
        prov["pose_source"], prov["object_pose_source"] = pose_source_from_log, object_pose_from_log
        prov["declarations_from"] = "run log (no /build1/run_info in the bag)"
    else:
        prov["declarations_from"] = "/build1/run_info"
    (out / "capture_profile_candidate.json").write_text(json.dumps(cand, indent=1) + "\n")
    lines = [f"# Bag binding: run {args.run_id}", "",
             f"Simulation time: {sim_time}; duration {report['bag']['duration_s']} s; "
             f"acceptance profile {args.acceptance_profile}; launch profile {(data['run_info'] or {}).get('profile')}; planning_pose_source {(data['run_info'] or {}).get('planning_pose_source')}; "
             f"object_pose_source {(data['run_info'] or {}).get('object_pose_source')}", "",
             f"Channels {len(rows)}: observation bound {bound_obs}, action bound {bound_act}; "
             f"C9 cameras bound {bound_cam}/{len(C9_CAMERAS)}; C9 v1 complete: {report['c9_v1_complete']}", "",
             f"Bag: {size['files_mb']} MB on disk, {size['mb_per_sim_minute']} MB per minute of Gazebo time "
             f"(camera images {size['camera_image_mb']} MB, {size['camera_mb_per_sim_minute']} MB per minute); "
             f"candidate grid {args.fps} fps, max skew {args.max_skew_ns} ns", "",
             "| # | channel | unit | observation | rate Hz | min | max | in range | action |", "|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        o, a = r["observation"], r["action"]
        lines.append(f"| {r['index']} | {r['name']} | {r['unit']} | {o.get('topic', 'UNBOUND')} | {o.get('rate_hz', '')} | "
                     f"{o.get('min', '')} | {o.get('max', '')} | {o.get('within_profile_range', '')} | "
                     f"{a.get('topic', 'UNBOUND: ' + a.get('unbound', ''))} |")
    lines += ["", "| C9 camera | binding | rate Hz | frame | camera_info |", "|---|---|---|---|---|"]
    lines += [f"| {k} | {v.get('topic', 'UNBOUND: ' + v.get('unbound', ''))} | {v.get('rate_hz', '')} | "
              f"{v.get('frame_id', '')} | {v.get('camera_info_topic', '')} |" for k, v in cams["bound"].items()]
    used = [s["topic"] for s in cand["streams"]]
    lines += ["", "| candidate stream | timestamp source | strictly increasing as read_bag reads it | equal | decreasing |",
              "|---|---|---|---|---|"]
    for s_ in cand["streams"]:
        st = data["stamps"].get(s_["topic"], {})
        key = "header" if s_["timestamp_source"] == "header" else "receipt"
        lines.append(f"| {s_['topic']} | {s_['timestamp_source']} | {st.get('strictly_increasing_as_read')} | "
                     f"{st.get(key + '_equal')} | {st.get(key + '_decreasing')} |")
    lines += ["", f"Streams in the candidate: {len(used)}"]
    lines += ["", "Gripper unit: " + json.dumps(report["gripper_unit"])]
    (out / "summary.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))
    return 0


if __name__ == "__main__":
    sys.exit(main())
