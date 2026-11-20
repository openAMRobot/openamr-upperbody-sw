# SPDX-License-Identifier: MIT
"""The recorder of the arm diagnostics: what the physics did during a grasp and a return to park, sampled from the
running stack, with a bag excerpt of the raw messages. It observes and commands nothing.

Sources
- /build1/contacts/<link>: gz contact sensors on the arm links (BUILD1_CONTACT_SENSORS=1, diagnostics.py).
- /sim/joint_states: the plant (Gazebo's joint_state_broadcaster with devices): position and applied effort.
- /joint_states: what the driver reads over CAN. /build1/ground_truth/finger_joint_states: all four fingers.
- /build1/ground_truth/item/pose and /build1/ground_truth/odom: the item and the base in the world.
- /sil/gz/joint_states: the lift in the physics. /robot_description: the model for the TCP forward kinematics.

Simulation only.
"""
import json
import math
import os
import subprocess
import threading
import time

import numpy as np

from openamrobot_build1_scene import diagnostics as dx
from openamrobot_build1_scene.gravity import GravityModel

FINGERS = "/build1/ground_truth/finger_joint_states"
BAG_TOPICS = ("/sim/joint_states", "/joint_states", FINGERS, "/build1/ground_truth/item/pose", "/build1/ground_truth/odom",
              "/sil/gz/joint_states")


class Recorder:
    def __init__(self, rig, links):
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from rclpy.serialization import serialize_message
        from std_msgs.msg import String
        self.rig, self.links = rig, list(links)
        self.serialize = serialize_message
        self.lock = threading.Lock()
        self.samples, self.contacts, self.marks, self.raw = [], [], [], []
        self.types = {}
        self.running = False
        self.model, self.urdf = None, None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        rig.node.create_subscription(String, "/robot_description", self._on_description, latched)
        self.contact_counts = {}
        try:
            from ros_gz_interfaces.msg import Contacts
            self.world_links = [dx.world_contact_link(model) for model in dx.WORLD_SENSOR_MODELS]
            for link in self.links + self.world_links:
                rig.node.create_subscription(Contacts, dx.ros_contact_topic(link), lambda m, l=link: self._on_contacts(l, m), 50)
            self.types.update({dx.ros_contact_topic(link): "ros_gz_interfaces/msg/Contacts" for link in self.links + self.world_links})
            self.contacts_available = True
        except ImportError:
            self.contacts_available = False
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import JointState
        for topic, kind, name in (("/sim/joint_states", JointState, "sensor_msgs/msg/JointState"),
                                  ("/joint_states", JointState, "sensor_msgs/msg/JointState"),
                                  (FINGERS, JointState, "sensor_msgs/msg/JointState"),
                                  ("/sil/gz/joint_states", JointState, "sensor_msgs/msg/JointState"),
                                  ("/build1/ground_truth/item/pose", PoseStamped, "geometry_msgs/msg/PoseStamped"),
                                  ("/build1/ground_truth/odom", Odometry, "nav_msgs/msg/Odometry")):
            self.types[topic] = name
            rig.node.create_subscription(kind, topic, lambda m, t=topic: self._on_raw(t, m), 50)
        self.emulator, self.feedforward = {}, {}
        from std_msgs.msg import Float64MultiArray
        for side in ("left", "right"):
            rig.node.create_subscription(Float64MultiArray, f"/{side}_arm_gravity_controller/commands",
                                         lambda m, s=side: self._on_feedforward(s, m), 10)
        try:
            from diagnostic_msgs.msg import DiagnosticArray
            for side in ("left", "right"):
                rig.node.create_subscription(DiagnosticArray, f"/sim/{side}_arm_emulator/status", self._on_emulator, 10)
        except ImportError:
            pass
        rig.plans = []
        rig.recorder = self

    # ------------------------------------------------------------ inputs
    def _on_description(self, msg):
        with self.lock:
            if self.model is None:
                self.urdf, self.model = msg.data, GravityModel(msg.data)

    def _on_feedforward(self, side, msg):
        with self.lock:
            self.feedforward[side] = list(msg.data)

    def _on_emulator(self, msg):
        """The arm emulator's own view of each motor: applied torque and the last MIT command it received."""
        for status in msg.status:
            joint = status.name.rsplit("/", 1)[-1]
            if "finger" not in joint:
                continue
            values = {kv.key: kv.value for kv in status.values}
            try:
                entry = {k: float(values[k]) for k in ("torque", "position", "mit_q", "mit_kp", "mit_kd")}
            except (KeyError, ValueError):
                continue
            with self.lock:
                self.emulator[joint] = entry

    def _on_raw(self, topic, msg):
        if self.running:
            with self.lock:
                self.raw.append((topic, self.serialize(msg), self.rig.node.get_clock().now().nanoseconds))

    def _on_contacts(self, link, msg):
        self.contact_counts[link] = self.contact_counts.get(link, 0) + 1
        if not self.running:
            return
        t = self.rig.sim_time()
        rows = []
        for contact in msg.contacts:
            if link.startswith("world_"):
                # A world sensor: collision1 is the world model's, collision2 the touching one. The row is keyed by the
                # robot link that touches (model::link::collision), so that one table holds both kinds of sensor.
                model = link[len("world_"):]
                mine, theirs = contact.collision1.name, contact.collision2.name
                if not mine.startswith(model + "::") and theirs.startswith(model + "::"):
                    mine, theirs = theirs, mine
                parts = theirs.split("::")
                row_link = parts[1] if len(parts) >= 3 and parts[0] == "openamrobot" else theirs
                other, source = mine, link
            else:
                other = contact.collision2.name if link in contact.collision1.name else contact.collision1.name
                row_link, source = link, link
            for i, point in enumerate(contact.positions):
                force = contact.wrenches[i].body_1_wrench.force if i < len(contact.wrenches) else None
                normal = contact.normals[i] if i < len(contact.normals) else None
                f = (force.x, force.y, force.z) if force is not None else (math.nan,) * 3
                n = (normal.x, normal.y, normal.z) if normal is not None else (math.nan,) * 3
                normal_force, tangential = dx.normal_tangential(f, n) if normal is not None else (math.nan, math.nan)
                rows.append({"t": t, "link": row_link, "sensor": source, "other": other, "point": (point.x, point.y, point.z), "force": f,
                             "normal": n, "fn": normal_force, "ft": tangential,
                             "depth": contact.depths[i] if i < len(contact.depths) else math.nan})
        with self.lock:
            self.contacts += rows
            self.raw.append((dx.ros_contact_topic(link), self.serialize(msg), self.rig.node.get_clock().now().nanoseconds))

    # ------------------------------------------------------------ sampling
    def start(self, period_wall_s=0.02):
        self.running = True
        self.thread = threading.Thread(target=self._loop, args=(period_wall_s,), daemon=True)
        self.thread.start()
        return self

    def stop(self):
        self.running = False
        if getattr(self, "thread", None) is not None:
            self.thread.join(timeout=5.0)

    def mark(self, label):
        with self.lock:
            self.marks.append((self.rig.sim_time(), label))

    def _joint_map(self, topic, field):
        message = self.rig.get(topic)
        if message is None:
            return {}
        values = getattr(message, field)
        return {name: float(values[i]) for i, name in enumerate(message.name) if i < len(values)}

    def _loop(self, period):
        last = None
        while self.running:
            time.sleep(period)
            t = self.rig.sim_time()
            if last is not None and t <= last:
                continue
            last = t
            try:
                self.samples.append(self._sample(t))
            except Exception as error:  # noqa: BLE001 - one bad sample is skipped, the trace goes on
                self.samples.append({"t": t, "error": str(error)})

    def _sample(self, t):
        physics = self._joint_map("/sim/joint_states", "position")
        effort = self._joint_map("/sim/joint_states", "effort")
        driver = self._joint_map("/joint_states", "position")
        driver_effort = self._joint_map("/joint_states", "effort")
        fingers = self._joint_map(FINGERS, "position")
        with self.lock:
            emulator = {joint: dict(values) for joint, values in self.emulator.items()}
            feedforward = {side: list(values) for side, values in self.feedforward.items()}
        sample = {"t": t, "q": physics, "tau": effort, "q_driver": driver, "tau_driver": driver_effort, "fingers": fingers,
                  "emulator": emulator, "feedforward": feedforward}
        item = self.rig.get("/build1/ground_truth/item/pose")
        if item is not None:
            p, o = item.pose.position, item.pose.orientation
            sample["item"] = ((p.x, p.y, p.z), (o.x, o.y, o.z, o.w))
        odom = self.rig.get("/build1/ground_truth/odom")
        if odom is not None:      # scratch (diag/arm-geom-*): the base pose with its tilt
            _p, _o = odom.pose.pose.position, odom.pose.pose.orientation
            sample["odom"] = ((_p.x, _p.y, _p.z), (_o.x, _o.y, _o.z, _o.w))
        lift = self._joint_map("/sil/gz/joint_states", "position").get("lift_joint")
        with self.lock:
            model = self.model
        if odom is not None and model is not None:
            positions = dict(driver)
            positions.update(physics)
            if lift is not None:
                positions["lift_joint"] = lift
            links, _ = model._frames(positions)
            p, o = odom.pose.pose.position, odom.pose.pose.orientation
            R_world_base, p_world_base = dx.quat_matrix(o.x, o.y, o.z, o.w), np.array((p.x, p.y, p.z))
            R_root_base, p_root_base = links["base_footprint"] if "base_footprint" in links else (np.eye(3), np.zeros(3))
            for side in ("left", "right"):
                for frame in (f"openarm_{side}_grasp_frame", f"openarm_{side}_ee_base_link"):
                    if frame in links:
                        R, pp = links[frame]
                        R_base = R_root_base.T @ R
                        p_base = R_root_base.T @ (pp - p_root_base)
                        sample[frame] = ((R_world_base @ R_base).tolist(), (p_world_base + R_world_base @ p_base).tolist())
        return sample

    # ------------------------------------------------------------ windows and numbers
    def mark_time(self, label, last=False):
        times = [t for t, name in self.marks if name == label]
        if not times:
            return None
        return times[-1] if last else times[0]

    def window(self, start, end):
        t0, t1 = self.mark_time(start), self.mark_time(end, last=True)
        if t0 is None or t1 is None:
            return []
        return [s for s in self.samples if t0 <= s["t"] <= t1 and "error" not in s]

    def contacts_between(self, t0, t1, links=None):
        return [c for c in self.contacts if t0 <= c["t"] <= t1 and (links is None or c["link"] in links)]

    def item_in_tcp(self, sample, side):
        frame = f"openarm_{side}_grasp_frame"
        if frame not in sample or "item" not in sample:
            return None
        R, p = np.array(sample[frame][0]), np.array(sample[frame][1])
        return dx.in_frame(R, p, sample["item"][0]).tolist()

    def write(self, directory, name):
        os.makedirs(directory, exist_ok=True)
        with self.lock:
            trace = {"marks": self.marks, "samples": self.samples, "contacts": self.contacts, "plans": self.rig.plans,
                     "contact_message_counts": self.contact_counts}
        path = os.path.join(directory, f"{name}_trace.json")
        with open(path, "w", encoding="utf-8") as f:
            json.dump(trace, f, default=float)
        return path

    def write_bag(self, directory, name):
        """The raw messages since start() as a bag (mcap, else sqlite3). -> path or an error text."""
        import rosbag2_py
        with self.lock:
            raw = list(self.raw)
        topics = sorted({topic for topic, _, _ in raw})
        last_error = None
        for storage in ("mcap", "sqlite3"):
            path = os.path.join(directory, f"{name}_bag_{storage}")
            try:
                writer = rosbag2_py.SequentialWriter()
                writer.open(rosbag2_py.StorageOptions(uri=path, storage_id=storage),
                            rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
                for number, topic in enumerate(topics):
                    try:
                        metadata = rosbag2_py.TopicMetadata(id=number, name=topic, type=self.types[topic], serialization_format="cdr")
                    except TypeError:
                        metadata = rosbag2_py.TopicMetadata(name=topic, type=self.types[topic], serialization_format="cdr")
                    writer.create_topic(metadata)
                for topic, data, stamp in raw:
                    writer.write(topic, data, stamp)
                del writer
                return path
            except Exception as error:  # noqa: BLE001 - a bag is an artifact, never a check
                last_error = f"{storage}: {type(error).__name__}: {error}"
        return f"bag not written ({last_error})"


def gz_probe(topics, echo_s=4.0):
    """What gz transport shows for the contact sensors: the advertised contact topics, and whether one message arrives on
    each of `topics` within `echo_s` wall seconds. Text for the log; a probe failure is reported, not raised."""
    lines = []
    try:
        listed = subprocess.run(["gz", "topic", "-l"], capture_output=True, text=True, timeout=20).stdout.split()
        contact = [t for t in listed if "contact" in t]
        lines.append(f"gz topic -l: {len(listed)} topics, {len(contact)} with 'contact': {contact[:24]}")
    except Exception as error:  # noqa: BLE001
        lines.append(f"gz topic -l failed: {error}")
    for topic in topics:
        try:
            done = subprocess.run(["timeout", str(echo_s), "gz", "topic", "-e", "-n", "1", "-t", topic],
                                  capture_output=True, text=True, timeout=echo_s + 10)
            text = " ".join(done.stdout.split())
            lines.append(f"gz topic -e {topic}: exit {done.returncode}, {len(done.stdout)} bytes: {text[:600]}")
        except Exception as error:  # noqa: BLE001
            lines.append(f"gz topic -e {topic} failed: {error}")
    return lines


# ---------------------------------------------------------------- the numbers of one grasp

def grasp_numbers(recorder, side, mass, mu):
    """Per-run numbers of a grasp-and-lift from the recorded marks: closed, lift_start, lift_end, hold_end, released."""
    out = {}
    fingers = dx.FINGER_LINKS[side]
    t_closed, t_lift0 = recorder.mark_time("closed"), recorder.mark_time("lift_start")
    t_lift1, t_hold1 = recorder.mark_time("lift_end"), recorder.mark_time("hold_end")
    held = recorder.window("closed", "hold_end")
    out["samples_held"] = len(held)
    if held:
        rel = [(s["t"], recorder.item_in_tcp(s, side)) for s in held if recorder.item_in_tcp(s, side) is not None]
        result = dx.slip(rel)
        out["slip_total_m"], out["slip_max_m"] = result["distance"], result["max"]
        out["slip_onset_s_after_lift_start"] = (result["onset"] - t_lift0) if (result["onset"] is not None and t_lift0 is not None) else None
        if rel:
            out["item_in_tcp_at_close"] = [round(v, 4) for v in rel[0][1]]
            out["item_in_tcp_at_hold_end"] = [round(v, 4) for v in rel[-1][1]]
        first, final = held[0], held[-1]
        tcp = f"openarm_{side}_grasp_frame"
        hand = f"openarm_{side}_ee_base_link"
        if tcp in first and tcp in final and "item" in first and "item" in final:
            out["tcp_rise_m"] = final[tcp][1][2] - first[tcp][1][2]
            out["hand_rise_m"] = final[hand][1][2] - first[hand][1][2]
            out["item_rise_m"] = final["item"][0][2] - first["item"][0][2]
            out["item_rise_minus_tcp_rise_m"] = out["item_rise_m"] - out["tcp_rise_m"]
            out["hand_rotation_deg"] = math.degrees(dx.rotation_angle(np.array(first[hand][0]).T @ np.array(final[hand][0])))
            out["item_tilt_deg"] = math.degrees(dx.rotation_angle(
                dx.quat_matrix(*first["item"][1]).T @ dx.quat_matrix(*final["item"][1])))
        lifting = [s for s in held if t_lift0 is not None and t_lift1 is not None and t_lift0 <= s["t"] <= t_lift1 and tcp in s]
        out["tcp_peak_vertical_accel_mps2"] = dx.peak_vertical_accel(dx.resample([(s["t"], s[tcp][1][2]) for s in lifting], 0.05))
        for joint in (f"openarm_{side}_finger_joint1", f"openarm_{side}_finger_joint2"):
            values = [s["fingers"].get(joint) for s in held if s["fingers"].get(joint) is not None]
            efforts = [s["tau"].get(joint) for s in held if s["tau"].get(joint) is not None]
            if values:
                out[f"{joint}_mean_rad"] = float(np.mean(values))
                out[f"{joint}_on_limit_fraction"] = float(np.mean([dx.on_limit(v, 0.0, 0.7854) != "" for v in values]))
            if efforts:
                out[f"{joint}_physics_effort_mean_Nm"] = float(np.mean(efforts))
        q1 = out.get(f"openarm_{side}_finger_joint1_mean_rad")
        q2 = out.get(f"openarm_{side}_finger_joint2_mean_rad")
        if q1 is not None and q2 is not None:
            out["finger_joint2_minus_joint1_rad"] = q2 - q1
        finger = f"openarm_{side}_finger_joint1"
        motor = [s["emulator"][finger] for s in held if finger in s.get("emulator", {})]
        if motor:
            out["emulator_finger_torque_mean_Nm"] = float(np.mean([m["torque"] for m in motor]))
            out["emulator_finger_mit_kp"] = float(np.mean([m["mit_kp"] for m in motor]))
            out["emulator_finger_mit_q"] = float(np.mean([m["mit_q"] for m in motor]))
            out["emulator_finger_kp_times_error_Nm"] = float(np.mean([m["mit_kp"] * (m["mit_q"] - m["position"]) for m in motor]))
        out["gripper_command_max_effort_Nm"] = 3.0
        driver_efforts = [s["tau_driver"].get(f"openarm_{side}_finger_joint1") for s in held
                          if s["tau_driver"].get(f"openarm_{side}_finger_joint1") is not None]
        if driver_efforts:
            out["finger_joint1_driver_effort_mean_Nm"] = float(np.mean(driver_efforts))
    if t_closed is not None and t_hold1 is not None:
        hold = recorder.contacts_between(t_lift1 if t_lift1 is not None else t_closed, t_hold1)
        out["required_normal_per_finger_N"] = dx.required_normal_force(mass, mu)
        accel = out.get("tcp_peak_vertical_accel_mps2") or 0.0
        out["required_normal_per_finger_at_peak_accel_N"] = dx.required_normal_force(mass, mu, accel=accel)
        for number, link in enumerate(fingers, 1):
            on_item = [c for c in hold if c["link"] == link and "item" in c["other"]]
            if any(c.get("sensor") == "world_item" for c in on_item):     # one sensor per contact: the item's when it reports
                on_item = [c for c in on_item if c.get("sensor") == "world_item"]
            out[f"finger{number}_contact_sensor"] = sorted({c.get("sensor") for c in on_item})
            elsewhere = sorted({c["other"] for c in hold if c["link"] == link and "item" not in c["other"]})
            stamps = sorted({c["t"] for c in on_item})
            per_message = [[c for c in on_item if c["t"] == t] for t in stamps]
            out[f"finger{number}_item_contact_messages"] = len(stamps)
            if per_message:
                out[f"finger{number}_points_per_message_mean"] = float(np.mean([len(m) for m in per_message]))
                out[f"finger{number}_normal_N_mean"] = float(np.nanmean([sum(c["fn"] for c in m) for m in per_message]))
                out[f"finger{number}_tangential_N_mean"] = float(np.nanmean([math.dist((0, 0, 0), tuple(
                    sum(c["force"][k] for c in m) for k in range(3))) for m in per_message]))
                out[f"finger{number}_tangential_sum_N_mean"] = float(np.nanmean([sum(c["ft"] for c in m) for m in per_message]))
                points = np.array([c["point"] for m in per_message for c in m])
                out[f"finger{number}_contact_spread_m"] = float(np.max(np.ptp(points, axis=0))) if len(points) else None
            out[f"finger{number}_other_contacts"] = elsewhere
    return out


def return_numbers(recorder, side, label, limits):
    """Numbers of one return-to-park move: from mark `label`_start to `label`_end."""
    out = {}
    window = recorder.window(f"{label}_start", f"{label}_end")
    joints = [f"openarm_{side}_joint{i}" for i in range(1, 8)]
    out["samples"] = len(window)
    if window:
        for joint in joints:
            taus = [abs(s["tau"][joint]) for s in window if joint in s["tau"]]
            hits = sorted({dx.on_limit(s["q"][joint], *limits[joint]) for s in window if joint in s["q"]} - {""})
            out[f"{joint[-6:]}_max_abs_effort_Nm"] = max(taus) if taus else None
            out[f"{joint[-6:]}_limit_hit"] = ",".join(hits)
        final = window[-1]
        out["final_q"] = {joint[-6:]: round(final["q"].get(joint, math.nan), 4) for joint in joints}
        ff = [s["feedforward"].get(side) for s in window if s.get("feedforward", {}).get(side)]
        if ff:
            out["feedforward_final_Nm"] = [round(v, 2) for v in ff[-1]]
            out["feedforward_zero_fraction"] = float(np.mean([all(v == 0.0 for v in f) for f in ff]))
        out["final_physics_effort_Nm"] = {joint[-6:]: round(final["tau"].get(joint, math.nan), 2) for joint in joints}
    t0, t1 = recorder.mark_time(f"{label}_start"), recorder.mark_time(f"{label}_end", last=True)
    if t0 is not None and t1 is not None:
        world = [c for c in recorder.contacts_between(t0, t1) if c["link"].startswith(f"openarm_{side}_")]
        pairs = {}
        for c in world:
            parts = c["other"].split("::")
            key = f"{c['link'].replace(f'openarm_{side}_', '')}~{parts[0]}:{parts[-1]}"
            pairs[key] = max(pairs.get(key, 0.0), math.dist((0, 0, 0), c["force"]) if not math.isnan(c["force"][0]) else 0.0)
        out["contacts_max_force_N"] = {k: round(v, 2) for k, v in sorted(pairs.items())}
        plans = [p for p in recorder.rig.plans if p["label"] == label]
        if plans:
            plan = plans[-1]
            executed = [[s["q"][j] for j in plan["joint_names"]] for s in window if all(j in s["q"] for j in plan["joint_names"])]
            out["planned_points"] = len(plan["points"])
            out["path_deviation_rad"] = dx.path_deviation(plan["points"], executed)
            hand = tcp_path_deviation(recorder, side, plan, window)
            if hand is not None:
                out["hand_path_deviation_m"] = hand
            out["planned_limit_margin_rad"] = dx.planned_limit_margin(plan["points"], [limits[j] for j in plan["joint_names"]])
    return out


def tcp_path_deviation(recorder, side, plan, window):
    """Largest distance (m) of the executed hand base from the hand base path of the plan, both by forward kinematics of
    the model in base_footprint (the other joints as recorded at the start of the move). None without the model."""
    model = recorder.model
    if model is None or not window:
        return None
    frame = f"openarm_{side}_ee_base_link"
    base = dict(window[0]["q_driver"])
    base.update(window[0]["q"])

    def hand(positions):
        links, _ = model._frames(positions)
        R0, p0 = links["base_footprint"] if "base_footprint" in links else (np.eye(3), np.zeros(3))
        return R0.T @ (links[frame][1] - p0)

    planned = []
    for point in plan["points"]:
        q = dict(base)
        q.update(zip(plan["joint_names"], point))
        planned.append(hand(q))
    if len(planned) == 1:
        planned = planned * 2
    worst = 0.0
    for sample in window[::5]:
        q = dict(base)
        q.update(sample["q"])
        p = hand(q)
        best = math.inf
        for a, b in zip(planned[:-1], planned[1:]):
            ab = b - a
            denominator = float(ab @ ab)
            t = 0.0 if denominator == 0.0 else min(1.0, max(0.0, float((p - a) @ ab) / denominator))
            best = min(best, float(np.linalg.norm(p - (a + t * ab))))
        worst = max(worst, best)
    return worst
