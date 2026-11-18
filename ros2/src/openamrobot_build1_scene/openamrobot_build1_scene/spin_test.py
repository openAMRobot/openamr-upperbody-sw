# SPDX-License-Identifier: MIT
"""Turn-slip experiment (diag/spin-test, scratch branch): turns in place and straight drives in firmware_sil, per model
variant (spin_variants.py), with the yaw split into its parts.

EXPERIMENT ONLY. No limit here is an acceptance value: the scenario records numbers and fails only when the data
cannot be taken (no wheel states, no contacts, a segment that does not move).

Per segment it records
  commanded      the body yaw (or distance) asked for, and the wheel angles the drive emulator demanded
                 (/build1/drive/*_wheel_cmd_vel integrated over Gazebo time)
  wheels         the Gazebo wheel joint angles (/sil/gz/joint_states)
  kinematic      r * (dR - dL) / track from the Gazebo wheel angles, with the model's radius and track
  odometry       /odom of base_state_adapter.py (the host test client's 606Ch speeds, integrated at 50 Hz)
  ground truth   /build1/ground_truth/odom (the body)
  split          residual = odometry - truth = (odometry - kinematic) + (kinematic - truth): drive chain + contact
  contacts       every contact with the floor (one gz contact sensor on the floor): normal force per drive wheel, of
                 the four castors together and of anything else; the force-weighted lateral position of each drive
                 wheel's contact in the base frame, hence the effective track
Yaw is unwrapped by sampling at 20 Hz of Gazebo time, so a turn of pi is not folded.

Simulation only. Not hardware, physical acceptance or safety evidence.
"""
import math
import os
import re
import threading
import xml.etree.ElementTree as ET


class ScenarioFailed(AssertionError):
    """rig.ScenarioFailed without importing rclpy at module level (the static tests import this module)."""


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


# rad/s on /cmd_vel: Nav2's in-place rate (rotation_scaling_factor 0.5 x v_angular_max 0.5, v_angular_min_in_place 0.25).
# The host client rounds each wheel to 5 r/min, which is 0.2566 rad/s of body yaw rate.
TURN_RATE = 0.25
STRAIGHT_SPEED = 0.2        # m/s, max_vel_x of the platform Nav2 parameters
RPM_TO_RAD_S = 2.0 * math.pi / 60.0
TURNS = (("turn_pos_half_pi", math.pi / 2.0), ("turn_neg_half_pi", -math.pi / 2.0),
         ("turn_pos_pi", math.pi), ("turn_neg_pi", -math.pi))
REPEATS = 3
STRAIGHT_M = 2.0
SAMPLE_S = 0.05
FLOOR_CONTACT_TOPIC = "/build1/contacts/world_floor"
GRAVITY = 9.81
PARK = [0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0]
CASTER = re.compile(r"(fl|fr|bl|br)_wheel_link")


def quantized_rate(body_rate, radius, track):
    """Body yaw rate after the host client rounds each wheel to whole r/min (stm32_host.DriveHost.target_rpm)."""
    wheel = body_rate * (track / 2.0) / radius
    rpm = round(abs(wheel) / RPM_TO_RAD_S)
    return math.copysign(rpm * RPM_TO_RAD_S * radius / (track / 2.0), body_rate)


def quantized_speed(speed, radius):
    rpm = round(abs(speed / radius) / RPM_TO_RAD_S)
    return math.copysign(rpm * RPM_TO_RAD_S * radius, speed)


def classify(name):
    """left | right | caster | other, from the scoped collision name of a floor contact."""
    if "left_wheel_link" in name:
        return "left"
    if "right_wheel_link" in name:
        return "right"
    if CASTER.search(name):
        return "caster"
    return "other"


def lateral(point, pose):
    """y of a world point in the base frame of pose (x, y, yaw)."""
    dx, dy = point[0] - pose[0], point[1] - pose[1]
    return -math.sin(pose[2]) * dx + math.cos(pose[2]) * dy


def contact_sample(contacts, pose):
    """One floor-contact sample -> {part: normal N}, {side: (sum F*y, sum F, n points, sum y)}, names.
    `contacts`: [(collision name, [(point, normal, force)])]."""
    load = {"left": 0.0, "right": 0.0, "caster": 0.0, "other": 0.0}
    lat = {"left": [0.0, 0.0, 0, 0.0], "right": [0.0, 0.0, 0, 0.0]}
    names = set()
    for name, points in contacts:
        part = classify(name)
        names.add(name)
        for point, normal, force in points:
            length = math.sqrt(sum(c * c for c in normal)) or 1.0
            fn = abs(sum(f * n for f, n in zip(force, normal)) / length)
            load[part] += fn
            if part in lat:
                y = lateral(point, pose)
                acc = lat[part]
                acc[0] += fn * y
                acc[1] += fn
                acc[2] += 1
                acc[3] += y
    return load, lat, names


def summarize_contacts(samples):
    """Mean normal loads and the effective track over samples [(load, lat)]. None where nothing was seen."""
    if not samples:
        return {}
    n = len(samples)
    out = {f"normal_{part}_N": sum(s[0][part] for s in samples) / n for part in ("left", "right", "caster", "other")}
    total = sum(out[f"normal_{p}_N"] for p in ("left", "right", "caster", "other"))
    out["normal_total_N"] = total
    out["caster_share"] = out["normal_caster_N"] / total if total > 0 else None
    ys = {}
    for side in ("left", "right"):
        f = sum(s[1][side][1] for s in samples)
        if f > 0:
            ys[side] = sum(s[1][side][0] for s in samples) / f
        else:
            count = sum(s[1][side][2] for s in samples)
            ys[side] = sum(s[1][side][3] for s in samples) / count if count else None
        out[f"contact_y_{side}_m"] = ys[side]
        out[f"contact_points_{side}_mean"] = sum(s[1][side][2] for s in samples) / n
    out["effective_track_m"] = (ys["left"] - ys["right"]) if ys["left"] is not None and ys["right"] is not None else None
    out["forces_reported"] = total > 0
    return out


def split(kind, commanded, kinematic, odometry, truth):
    """Ratios and the residual split of one segment."""
    out = {"residual": odometry - truth, "drive_chain": odometry - kinematic, "contact": kinematic - truth}
    out["ratio_odom_truth"] = odometry / truth if abs(truth) > 1e-9 else None
    out["ratio_kinematic_truth"] = kinematic / truth if abs(truth) > 1e-9 else None
    out["ratio_odom_kinematic"] = odometry / kinematic if abs(kinematic) > 1e-9 else None
    out["ratio_truth_commanded"] = truth / commanded if abs(commanded) > 1e-9 else None
    return out


def model_mass(urdf):
    root = ET.fromstring(urdf)
    return sum(float(m.get("value")) for m in root.iter("mass"))


class Demand:
    """The drive emulator's per-wheel demand (rad/s), integrated over Gazebo time."""

    def __init__(self, rig):
        from std_msgs.msg import Float64
        self.rig, self.lock = rig, threading.Lock()
        self.angle = {"left": 0.0, "right": 0.0}
        self.last = {"left": None, "right": None}
        for side in ("left", "right"):
            rig.node.create_subscription(Float64, f"/build1/drive/{side}_wheel_cmd_vel",
                                         lambda m, s=side: self.on_demand(s, m.data), 50)

    def on_demand(self, side, value):
        now = self.rig.sim_time()
        with self.lock:
            last = self.last[side]
            if last is not None and now > last[0]:
                self.angle[side] += last[1] * (now - last[0])
            self.last[side] = (now, value)

    def read(self):
        """Integrated demand up to now (the held value carried to the present)."""
        now = self.rig.sim_time()
        with self.lock:
            out = {}
            for side in ("left", "right"):
                last = self.last[side]
                extra = last[1] * (now - last[0]) if last is not None and now > last[0] else 0.0
                out[side] = self.angle[side] + extra
            return out


class FloorContacts:
    def __init__(self, rig):
        from ros_gz_interfaces.msg import Contacts
        self.rig, self.lock, self.latest, self.count = rig, threading.Lock(), None, 0
        rig.node.create_subscription(Contacts, FLOOR_CONTACT_TOPIC, self.on_contacts, 20)

    def on_contacts(self, msg):
        contacts = []
        for contact in msg.contacts:
            name = contact.collision2.name if "floor" in contact.collision1.name else contact.collision1.name
            points = []
            for i, p in enumerate(contact.positions):
                n = contact.normals[i] if i < len(contact.normals) else None
                w = contact.wrenches[i].body_1_wrench.force if i < len(contact.wrenches) else None
                points.append(((p.x, p.y, p.z), (n.x, n.y, n.z) if n else (0.0, 0.0, 1.0), (w.x, w.y, w.z) if w else (0.0, 0.0, 0.0)))
            contacts.append((name, points))
        with self.lock:
            self.latest, self.count = contacts, self.count + 1

    def get(self):
        with self.lock:
            return self.latest


def _geometry(rig):
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String
    from openamrobot_build1_scene import spin_variants
    rig.node.create_subscription(String, "/sim/robot_description", lambda m: rig._store("/sim/robot_description", m.data),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    if not rig.wait(lambda: rig.get("/sim/robot_description") is not None, 30.0, "plant description"):
        raise ScenarioFailed("no /sim/robot_description")
    urdf = rig.get("/sim/robot_description")
    geometry = spin_variants.contact_geometry(urdf)
    geometry["mass_kg"] = model_mass(urdf)
    return geometry


def _segment(rig, log, drive, demand, floor, geometry, name, kind, repeat, command, seconds, nominal):
    radius, track = geometry["radius"], geometry["track"]
    rig.wait_stopped(10.0)
    rig.sleep_sim(1.0)
    pose0, odom0 = rig.truth(), rig.odom()
    gz0 = (rig.joint("left_wheel_joint", "/sil/gz/joint_states"), rig.joint("right_wheel_joint", "/sil/gz/joint_states"))
    demand0 = demand.read()
    truth_yaw = odom_yaw = 0.0
    last_truth, last_odom = pose0[2], odom0[2]
    samples, names, pitches, steady = [], set(), [], []
    start = rig.sim_time()
    drive.command = command
    phase = "commanded"
    stopped_since = None
    while True:
        rig.sleep_sim(SAMPLE_S)
        now = rig.sim_time()
        pose, odom = rig.truth(), rig.odom()
        truth_yaw += angle_diff(pose[2], last_truth)
        odom_yaw += angle_diff(odom[2], last_odom)
        last_truth, last_odom = pose[2], odom[2]
        q = rig.get("/build1/ground_truth/odom").pose.pose.orientation
        pitches.append(math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))))
        contacts = floor.get()
        if contacts is not None:
            load, lat, seen = contact_sample(contacts, pose)
            names |= seen
            samples.append((load, lat))
            if phase == "commanded" and now - start > 2.0:
                steady.append((load, lat))
        if phase == "commanded" and now - start >= seconds:
            drive.command = (0.0, 0.0)
            phase, stop_time = "settle", now
        elif phase == "settle":
            if rig.base_stopped():
                stopped_since = stopped_since if stopped_since is not None else now
            else:
                stopped_since = None
            # 1.5 s at least: base_state_adapter integrates the last 606Ch sample until a new one arrives
            if (stopped_since is not None and now - stopped_since >= 0.5 and now - stop_time >= 1.5) or now - stop_time > 10.0:
                break
    rig.sleep_sim(0.5)
    pose1, odom1 = rig.truth(), rig.odom()
    truth_yaw += angle_diff(pose1[2], last_truth)
    odom_yaw += angle_diff(odom1[2], last_odom)
    gz1 = (rig.joint("left_wheel_joint", "/sil/gz/joint_states"), rig.joint("right_wheel_joint", "/sil/gz/joint_states"))
    demand1 = demand.read()
    d_left, d_right = gz1[0] - gz0[0], gz1[1] - gz0[1]
    record = {"name": name, "kind": kind, "repeat": repeat, "command": list(command), "seconds": seconds, "nominal": nominal,
              "demand_left_rad": demand1["left"] - demand0["left"], "demand_right_rad": demand1["right"] - demand0["right"],
              "wheel_left_rad": d_left, "wheel_right_rad": d_right,
              "truth_distance_m": math.hypot(pose1[0] - pose0[0], pose1[1] - pose0[1]),
              "odom_distance_m": math.hypot(odom1[0] - odom0[0], odom1[1] - odom0[1]),
              "kinematic_distance_m": radius * (d_left + d_right) / 2.0,
              "truth_yaw_rad": truth_yaw, "odom_yaw_rad": odom_yaw, "kinematic_yaw_rad": radius * (d_right - d_left) / track,
              "demand_yaw_rad": radius * ((demand1["right"] - demand0["right"]) - (demand1["left"] - demand0["left"])) / track,
              "mean_pitch_rad": sum(pitches) / len(pitches) if pitches else None,
              "start_pose": list(pose0), "end_pose": list(pose1), "rtf": rig.rtf(), "sim_start_s": start}
    if kind == "turn":
        record.update(split(kind, nominal, record["kinematic_yaw_rad"], odom_yaw, truth_yaw))
        record["commanded_yaw_rad"] = command[1] * seconds
    else:
        record.update({"residual_distance_m": record["odom_distance_m"] - record["truth_distance_m"],
                       "heading_drift_truth_rad": truth_yaw, "heading_drift_odom_rad": odom_yaw})
    record["contacts_whole"] = summarize_contacts(samples)
    record["contacts_steady"] = summarize_contacts(steady)
    record["contact_names"] = sorted(names)
    if kind == "turn":
        log.event(f"{name} r{repeat}: wheels L {d_left:+.4f} R {d_right:+.4f} rad, kinematic {record['kinematic_yaw_rad']:+.4f}, "
                  f"odom {odom_yaw:+.4f}, truth {truth_yaw:+.4f}, residual {record['residual']:+.4f} rad "
                  f"(drive chain {record['drive_chain']:+.4f}, contact {record['contact']:+.4f}), "
                  f"track_eff {record['contacts_steady'].get('effective_track_m')}, castor share {record['contacts_steady'].get('caster_share')}")
    else:
        log.event(f"{name} r{repeat}: truth {record['truth_distance_m']:.4f} m, odom {record['odom_distance_m']:.4f} m, "
                  f"kinematic {record['kinematic_distance_m']:.4f} m, heading truth {truth_yaw:+.4f} odom {odom_yaw:+.4f} rad")
    return record


def run(rig, log):
    try:
        _run(rig, log)
    except ScenarioFailed as failed:
        from openamrobot_build1_scene import rig as rig_module
        raise rig_module.ScenarioFailed(str(failed)) from failed


def _run(rig, log):
    from openamrobot_build1_scene.scenarios import CmdVel
    variant = os.environ.get("BUILD1_SPIN_VARIANT", "")
    info = rig.get("/build1/run_info") or {}
    log.data["spin_variant"] = variant
    log.data["spin_variant_launch"] = info.get("spin_variant")
    log.data["spin_variant_edits"] = info.get("spin_variant_edits")
    if info.get("spin_variant") != variant:
        raise ScenarioFailed(f"the launch applied variant {info.get('spin_variant')!r}, the test asked for {variant!r}")
    geometry = _geometry(rig)
    log.data["geometry"] = geometry
    for key, value in sorted(geometry.items()):
        log.quantity(f"model_{key}", value, "m" if key != "mass_kg" else "kg", kind="model")
    if not rig.wait(lambda: rig.get("/sil/gz/joint_states") is not None, 30.0, "Gazebo wheel joint states"):
        raise ScenarioFailed("no /sil/gz/joint_states")
    demand, floor = Demand(rig), FloorContacts(rig)
    if not rig.wait(lambda: floor.get() is not None, 30.0, "floor contacts"):
        raise ScenarioFailed(f"no {FLOOR_CONTACT_TOPIC}: the floor contact sensor or its bridge is missing")
    handles = {side: rig.follow_joint_trajectory(side, PARK, 3.0) for side in ("left", "right")}
    for side, handle in handles.items():
        log.require(handle is not None and rig.succeeded(rig.wait_result(handle, 30.0, f"{side} park")[0]), f"{side} arm to park")
    radius, track = geometry["radius"], geometry["track"]
    rate = quantized_rate(TURN_RATE, radius, track)
    speed = quantized_speed(STRAIGHT_SPEED, radius)
    log.quantity("turn_rate_after_rpm_rounding", rate, "rad/s", kind="model", note=f"/cmd_vel {TURN_RATE} rad/s")
    log.quantity("straight_speed_after_rpm_rounding", speed, "m/s", kind="model", note=f"/cmd_vel {STRAIGHT_SPEED} m/s")
    segments = []
    drive = CmdVel(rig)
    try:
        rig.sleep_sim(2.0)
        # rest load split, arms parked, base still
        rest = []
        for _ in range(40):
            rig.sleep_sim(SAMPLE_S)
            contacts = floor.get()
            if contacts is not None:
                load, lat, _ = contact_sample(contacts, rig.truth())
                rest.append((load, lat))
        log.data["rest_contacts"] = summarize_contacts(rest)
        log.event(f"rest contacts: {log.data['rest_contacts']}")
        for repeat in range(1, REPEATS + 1):
            for name, angle in TURNS:
                seconds = abs(angle) / abs(rate)
                segments.append(_segment(rig, log, drive, demand, floor, geometry, name, "turn", repeat,
                                         (0.0, math.copysign(TURN_RATE, angle)), seconds, angle))
            direction = 1.0 if repeat % 2 else -1.0   # forward, back, forward: the floor along +x is free for 3 m
            segments.append(_segment(rig, log, drive, demand, floor, geometry, "straight_fwd" if direction > 0 else "straight_back",
                                     "straight", repeat, (direction * STRAIGHT_SPEED, 0.0), STRAIGHT_M / speed, direction * STRAIGHT_M))
    finally:
        drive.stop()
        log.data["spin_segments"] = segments
    turns = [s for s in segments if s["kind"] == "turn"]
    log.require(len(turns) == REPEATS * len(TURNS), f"{len(turns)} turns recorded")
    log.require(all(abs(s["truth_yaw_rad"]) > 0.5 * abs(s["nominal"]) for s in turns), "every turn moved the body")
    log.require(floor.count > 0, "floor contacts were received")
    if turns:
        log.quantity("mean_residual_per_turn", sum(s["residual"] for s in turns) / len(turns), "rad")
        log.quantity("mean_drive_chain_per_turn", sum(s["drive_chain"] for s in turns) / len(turns), "rad")
        log.quantity("mean_contact_per_turn", sum(s["contact"] for s in turns) / len(turns), "rad")
        log.quantity("summed_ratio_odom_truth", sum(abs(s["odom_yaw_rad"]) for s in turns) / sum(abs(s["truth_yaw_rad"]) for s in turns), "1")
