# SPDX-License-Identifier: MIT
"""Arm and grasp diagnostics of the Build 1 scene: contact sensors for the arm links, and the arithmetic that turns the
recorded physics traces into numbers. No ROS import at module level: the static tests check every function here.

Contact sensors are added only when the environment variable BUILD1_CONTACT_SENSORS is "1" (the diagnostics workflow,
.github/workflows/build1-arm-diagnostics.yml). They observe and never act: a gz contact sensor changes no force.
The robot model has no self_collide, so Gazebo computes no contact between two links of the robot; an arm link can
touch the world (shelf, tote, item, floor, fixtures) only.

Simulation only. A contact force of the simulator is development evidence, not a measurement of a gripper.
"""
import math
import re
import xml.etree.ElementTree as ET

import numpy as np

ENV = "BUILD1_CONTACT_SENSORS"
CONTACT_RATE_HZ = 100
CONTACT_SYSTEM = '<plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>'
PHYSICS_SYSTEM = '<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>'
ARM_LINKS = ("link1", "link2", "link3", "link4", "link5", "link6", "ee_base_link", "ee_link1", "ee_link2")
FINGER_LINKS = {side: (f"openarm_{side}_ee_link1", f"openarm_{side}_ee_link2") for side in ("left", "right")}
SLIP_ONSET_M = 0.002          # item origin moved this far in the TCP frame: the onset of slip
LIMIT_TOLERANCE_RAD = 0.01    # a joint this close to a position limit counts as on the limit
GRAVITY = 9.81


def enabled(environ):
    return environ.get(ENV, "") == "1"


def contact_links(sides=("left", "right")):
    return [f"openarm_{side}_{name}" for side in sides for name in ARM_LINKS]


def gz_contact_topic(link):
    return f"/build1/gz/contacts/{link}"


def ros_contact_topic(link):
    return f"/build1/contacts/{link}"


def sdf_collision_names(urdf_text, link):
    """The SDF names that sdformat gives the collisions of `link` when it converts the URDF (sdformat 14
    parser_urdf.cc, CreateCollisions): the URDF name, or the link name when there is none, plus "_collision", plus
    "_N" from the second collision on. A link joined to its parent by a fixed joint is lumped into the parent and
    renamed, so it is refused here."""
    root = ET.fromstring(urdf_text)
    element = root.find(f"link[@name='{link}']")
    if element is None:
        raise ValueError(f"no link {link!r} in the model")
    for joint in root.findall("joint"):
        if joint.find("child").get("link") == link and joint.get("type") == "fixed":
            raise ValueError(f"{link} hangs on a fixed joint: sdformat lumps its collisions into the parent")
    names = []
    for count, collision in enumerate(element.findall("collision")):
        name = (collision.get("name") or link) + "_collision"
        names.append(name if count == 0 else f"{name}_{count}")
    if not names:
        raise ValueError(f"{link} has no collision")
    return names


def with_contact_sensors(urdf_text, links, rate_hz=CONTACT_RATE_HZ):
    """The URDF with one gz contact sensor per collision of each link in `links`, publishing gz.msgs.Contacts on
    gz_contact_topic(link). Links, joints and every physical value are unchanged."""
    blocks = []
    for link in links:
        for number, collision in enumerate(sdf_collision_names(urdf_text, link)):
            topic = gz_contact_topic(link) if number == 0 else f"{gz_contact_topic(link)}_{number}"
            blocks.append(f'<gazebo reference="{link}"><sensor name="build1_contact_{link}_{number}" type="contact">'
                          f"<always_on>true</always_on><update_rate>{rate_hz}</update_rate><topic>{topic}</topic>"
                          f"<contact><collision>{collision}</collision><topic>{topic}</topic></contact></sensor></gazebo>")
    if urdf_text.count("</robot>") != 1:
        raise ValueError("the model has no single </robot> end tag")
    return urdf_text.replace("</robot>", "".join(blocks) + "</robot>")


WORLD_SENSOR_MODELS = ("item", "tote", "fixtures")


def world_contact_link(model):
    """The name the recorder gives the contact sensor of a world model (its topics use it like a link name)."""
    return f"world_{model}"


def world_with_contact_system(world_text, models=WORLD_SENSOR_MODELS, rate_hz=CONTACT_RATE_HZ):
    """The world with gz-sim's Contact system, which the contact sensors need (inserted once, after the Physics system),
    and one contact sensor on the link of each model in `models` over all of its collisions. These sensors are part of
    the world file, not of the spawned robot: they report the robot's links as the other collision."""
    if CONTACT_SYSTEM in world_text:
        return world_text
    if world_text.count(PHYSICS_SYSTEM) != 1:
        raise ValueError("the world has no single Physics system element")
    out = world_text.replace(PHYSICS_SYSTEM, PHYSICS_SYSTEM + CONTACT_SYSTEM)
    for model in models:
        start = out.find(f'<model name="{model}">')
        if start < 0:
            raise ValueError(f"the world has no model {model!r}")
        end = out.index("</link>", start)
        names = re.findall(r'<collision name="([^"]+)"', out[start:end])
        if not names:
            raise ValueError(f"model {model!r} has no collision in its first link")
        topic = gz_contact_topic(world_contact_link(model))
        collisions = "".join(f"<collision>{name}</collision>" for name in names)
        sensor = (f'<sensor name="build1_contact_{model}" type="contact"><always_on>true</always_on>'
                  f"<update_rate>{rate_hz}</update_rate><topic>{topic}</topic><contact>{collisions}<topic>{topic}</topic>"
                  "</contact></sensor>")
        out = out[:end] + sensor + out[end:]
    return out


def contact_bridge(links):
    """ros_gz_bridge entries for the contact sensors (gz.msgs.Contacts to ros_gz_interfaces/msg/Contacts)."""
    return [{"ros_topic_name": ros_contact_topic(link), "gz_topic_name": gz_contact_topic(link),
             "ros_type_name": "ros_gz_interfaces/msg/Contacts", "gz_type_name": "gz.msgs.Contacts", "direction": "GZ_TO_ROS"}
            for link in links]


# ---------------------------------------------------------------- arithmetic on recorded traces

def normal_tangential(force, normal):
    """|F . n| and |F - (F . n) n| of a contact force F with contact normal n (n need not be unit length)."""
    f, n = np.asarray(force, dtype=float), np.asarray(normal, dtype=float)
    length = np.linalg.norm(n)
    if length == 0.0:
        return float("nan"), float("nan")
    n = n / length
    along = float(f @ n)
    return abs(along), float(np.linalg.norm(f - along * n))


def required_normal_force(mass, mu, fingers=2, accel=0.0, safety=1.0, gravity=GRAVITY):
    """The normal force each finger must press with so that friction carries the item: m (g + a) / (fingers * mu),
    times a safety factor. Two-finger pinch, Coulomb friction, the item's weight shared equally."""
    return safety * mass * (gravity + accel) / (fingers * mu)


def slip(samples, threshold=SLIP_ONSET_M):
    """Slip of the item against the TCP. `samples`: [(t, (x, y, z) of the item in the TCP frame)], the first taken just
    after the fingers closed. -> {distance: final displacement (m), max: largest displacement, onset: t of the first
    sample past `threshold` or None, vertical: final displacement along z of the world is not known here, so the
    caller adds it}."""
    if not samples:
        return {"distance": None, "max": None, "onset": None}
    t0, p0 = samples[0]
    p0 = np.asarray(p0, dtype=float)
    onset, largest = None, 0.0
    for t, p in samples:
        d = float(np.linalg.norm(np.asarray(p, dtype=float) - p0))
        largest = max(largest, d)
        if onset is None and d > threshold:
            onset = t
    return {"distance": float(np.linalg.norm(np.asarray(samples[-1][1], dtype=float) - p0)), "max": largest, "onset": onset}


def on_limit(position, lower, upper, tolerance=LIMIT_TOLERANCE_RAD):
    """'lower', 'upper' or '' for a joint position against its limits."""
    if position <= lower + tolerance:
        return "lower"
    if position >= upper - tolerance:
        return "upper"
    return ""


def joint_limits(urdf_text, names):
    """{joint: (lower, upper)} from the URDF."""
    root = ET.fromstring(urdf_text)
    out = {}
    for name in names:
        limit = root.find(f"joint[@name='{name}']/limit")
        if limit is not None:
            out[name] = (float(limit.get("lower")), float(limit.get("upper")))
    return out


def _segment_distance(p, a, b):
    """Largest-component distance from p to the segment a-b, measured at the closest point in the Euclidean sense."""
    ab = b - a
    denominator = float(ab @ ab)
    s = 0.0 if denominator == 0.0 else min(1.0, max(0.0, float((p - a) @ ab) / denominator))
    return float(np.max(np.abs(p - (a + s * ab))))


def path_deviation(planned, executed):
    """Largest distance (rad, the worst joint) of an executed joint vector from the planned joint-space polyline.
    `planned`, `executed`: lists of equal-length joint vectors."""
    points = [np.asarray(p, dtype=float) for p in planned]
    if not points or not executed:
        return None
    if len(points) == 1:
        points = points * 2
    worst = 0.0
    for sample in executed:
        q = np.asarray(sample, dtype=float)
        worst = max(worst, min(_segment_distance(q, points[i], points[i + 1]) for i in range(len(points) - 1)))
    return worst


def planned_limit_margin(planned, limits_by_index):
    """Smallest distance (rad) of any planned point of any joint to that joint's limit. limits_by_index: [(lower, upper)]."""
    margin = math.inf
    for point in planned:
        for value, (lower, upper) in zip(point, limits_by_index):
            margin = min(margin, value - lower, upper - value)
    return margin


def resample(samples, spacing):
    """Every sample at least `spacing` seconds after the last one kept: sampled states repeat between messages, and a
    second difference over repeated values is noise, not acceleration."""
    out = []
    for t, value in samples:
        if not out or t - out[-1][0] >= spacing:
            out.append((t, value))
    return out


def peak_vertical_accel(samples):
    """Largest |d2z/dt2| (m/s^2) of [(t, z)] by central differences over the recorded samples; None with fewer than 3."""
    if len(samples) < 3:
        return None
    t = np.array([s[0] for s in samples], dtype=float)
    z = np.array([s[1] for s in samples], dtype=float)
    worst = 0.0
    for i in range(1, len(t) - 1):
        h1, h2 = t[i] - t[i - 1], t[i + 1] - t[i]
        if h1 <= 0.0 or h2 <= 0.0:
            continue
        accel = 2.0 * (h1 * z[i + 1] - (h1 + h2) * z[i] + h2 * z[i - 1]) / (h1 * h2 * (h1 + h2))
        worst = max(worst, abs(accel))
    return worst


# ---------------------------------------------------------------- poses

def quat_matrix(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rotation_angle(R):
    return math.acos(max(-1.0, min(1.0, (float(np.trace(R)) - 1.0) / 2.0)))


def in_frame(frame_R, frame_p, point):
    """`point` (world) in the frame (R, p) given in the world."""
    return frame_R.T @ (np.asarray(point, dtype=float) - np.asarray(frame_p, dtype=float))


def state_contact_pairs(contacts):
    """MoveIt's contacts of one state check (moveit_msgs/ContactInformation, or (body_1, body_2, depth) tuples) as
    {"a~b": largest depth in m}, the two names sorted so that a pair is one key whichever body MoveIt names first.
    The start state of a return is in collision exactly when this is not empty: what error -10 means, by pair."""
    pairs = {}
    for contact in contacts:
        if isinstance(contact, tuple):
            first, second, depth = contact
        else:
            first, second, depth = contact.contact_body_1, contact.contact_body_2, contact.depth
        key = "~".join(sorted((str(first), str(second))))
        pairs[key] = max(pairs.get(key, 0.0), abs(float(depth)))
    return dict(sorted(pairs.items()))
