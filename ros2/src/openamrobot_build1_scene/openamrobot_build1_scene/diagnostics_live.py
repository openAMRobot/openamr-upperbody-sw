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
            for link in self.links:
                rig.node.create_subscription(Contacts, dx.ros_contact_topic(link), lambda m, l=link: self._on_contacts(l, m), 50)
            self.types.update({dx.ros_contact_topic(link): "ros_gz_interfaces/msg/Contacts" for link in self.links})
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
        rig.plans = []
        rig.recorder = self

    # ------------------------------------------------------------ inputs
    def _on_description(self, msg):
        with self.lock:
            if self.model is None:
                self.urdf, self.model = msg.data, GravityModel(msg.data)

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
            other = contact.collision2.name if link in contact.collision1.name else contact.collision1.name
            for i, point in enumerate(contact.positions):
                force = contact.wrenches[i].body_1_wrench.force if i < len(contact.wrenches) else None
                normal = contact.normals[i] if i < len(contact.normals) else None
                f = (force.x, force.y, force.z) if force is not None else (math.nan,) * 3
                n = (normal.x, normal.y, normal.z) if normal is not None else (math.nan,) * 3
                normal_force, tangential = dx.normal_tangential(f, n) if normal is not None else (math.nan, math.nan)
                rows.append({"t": t, "link": link, "other": other, "point": (point.x, point.y, point.z), "force": f,
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
        sample = {"t": t, "q": physics, "tau": effort, "q_driver": driver, "tau_driver": driver_effort, "fingers": fingers}
        item = self.rig.get("/build1/ground_truth/item/pose")
        if item is not None:
            p, o = item.pose.position, item.pose.orientation
            sample["item"] = ((p.x, p.y, p.z), (o.x, o.y, o.z, o.w))
        odom = self.rig.get("/build1/ground_truth/odom")
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
        out["tcp_peak_vertical_accel_mps2"] = dx.peak_vertical_accel([(s["t"], s[tcp][1][2]) for s in lifting])
        for joint in (f"openarm_{side}_finger_joint1", f"openarm_{side}_finger_joint2"):
            values = [s["fingers"].get(joint) for s in held if s["fingers"].get(joint) is not None]
            efforts = [s["tau"].get(joint) for s in held if s["tau"].get(joint) is not None]
            if values:
                out[f"{joint}_mean_rad"] = float(np.mean(values))
                out[f"{joint}_on_limit_fraction"] = float(np.mean([dx.on_limit(v, 0.0, 0.7854) != "" for v in values]))
            if efforts:
                out[f"{joint}_physics_effort_mean_Nm"] = float(np.mean(efforts))
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
            out["planned_limit_margin_rad"] = dx.planned_limit_margin(plan["points"], [limits[j] for j in plan["joint_names"]])
    return out
