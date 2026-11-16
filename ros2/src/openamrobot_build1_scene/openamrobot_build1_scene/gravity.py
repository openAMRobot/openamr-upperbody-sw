# SPDX-License-Identifier: MIT
"""Static gravity torque of the arm joints, from the URDF's own masses, centres of mass and joint axes. No ROS import.

Why it exists. The pinned OpenArm driver (openarm_ros2 4e837e1, OpenArmHW::write) sends every arm motor the MIT frame
{kp, kd, q, dq, tau} with tau taken from the joint's effort command interface. The trajectory controllers claim only the
position interfaces, so tau is 0 and nothing holds the arm up but kp times the position error. With the driver's gains
(kp 70, 70, 70, 60, 10, 10, 10) the arm settles below every commanded pose: 0.145 rad on joint 1 and 74 mm at the hand
for the grasp pose of scenario 4 (docs/build1_arm_diagnosis.md). This module computes the missing term, the torque
that holds the arm still against gravity at its measured position, so that it can be sent through the effort interface
the driver already reads.

What it is NOT. Not a safety function, not a dynamics model (no velocity or acceleration terms), not a payload model:
an object in the hand is not included. The masses are the model's; none of them is a measurement of a delivered arm.
"""
import math
import xml.etree.ElementTree as ET

import numpy as np

GRAVITY = (0.0, 0.0, -9.80665)   # m/s^2 in the root frame of the model (base_footprint, level floor). Gazebo's default -9.8 differs by 0.07 %
ARM_JOINTS = {side: [f"openarm_{side}_joint{i}" for i in range(1, 8)] for side in ("left", "right")}


def _xyz(element, key, default=(0.0, 0.0, 0.0)):
    if element is None or element.get(key) is None:
        return np.array(default, dtype=float)
    return np.array([float(v) for v in element.get(key).split()], dtype=float)


def _rpy_matrix(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _axis_rotation(axis, angle):
    x, y, z = axis
    c, s, v = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return np.array([[c + x * x * v, x * y * v - z * s, x * z * v + y * s],
                     [y * x * v + z * s, c + y * y * v, y * z * v - x * s],
                     [z * x * v - y * s, z * y * v + x * s, c + z * z * v]])


class GravityModel:
    """The kinematic tree of a URDF with its masses. `torques(positions)` gives the holding torque of every joint asked for."""

    def __init__(self, urdf_text):
        root = ET.fromstring(urdf_text)
        self.links = {}
        for link in root.findall("link"):
            inertial = link.find("inertial")
            mass, com = 0.0, np.zeros(3)
            if inertial is not None and inertial.find("mass") is not None:
                mass = float(inertial.find("mass").get("value"))
                com = _xyz(inertial.find("origin"), "xyz")
            self.links[link.get("name")] = (mass, com)
        self.joints, self.children = {}, {}
        for joint in root.findall("joint"):
            origin = joint.find("origin")
            axis = _xyz(joint.find("axis"), "xyz", (1.0, 0.0, 0.0))
            norm = np.linalg.norm(axis)
            mimic = joint.find("mimic")
            entry = {"type": joint.get("type"), "parent": joint.find("parent").get("link"), "child": joint.find("child").get("link"),
                     "xyz": _xyz(origin, "xyz"), "rot": _rpy_matrix(_xyz(origin, "rpy")), "axis": axis / norm if norm > 0 else axis,
                     "mimic": None if mimic is None else (mimic.get("joint"), float(mimic.get("multiplier", 1.0)), float(mimic.get("offset", 0.0)))}
            self.joints[joint.get("name")] = entry
            self.children.setdefault(entry["parent"], []).append(joint.get("name"))
        child_links = {j["child"] for j in self.joints.values()}
        roots = [name for name in self.links if name not in child_links]
        if len(roots) != 1:
            raise ValueError(f"the URDF has {len(roots)} root links: {roots}")
        self.root = roots[0]

    def _value(self, name, positions):
        joint = self.joints[name]
        if joint["mimic"] is not None:
            source, multiplier, offset = joint["mimic"]
            return multiplier * float(positions.get(source, 0.0)) + offset
        return float(positions.get(name, 0.0))

    def _frames(self, positions):
        """{link: (R, p)} in the root frame, and {joint: (axis in the root frame, joint origin in the root frame)}."""
        links, joints = {self.root: (np.eye(3), np.zeros(3))}, {}
        stack = [self.root]
        while stack:
            parent = stack.pop()
            R_parent, p_parent = links[parent]
            for name in self.children.get(parent, []):
                joint = self.joints[name]
                R = R_parent @ joint["rot"]
                p = p_parent + R_parent @ joint["xyz"]
                joints[name] = (R @ joint["axis"], p)
                q = self._value(name, positions) if joint["type"] in ("revolute", "continuous", "prismatic") else 0.0
                if joint["type"] in ("revolute", "continuous"):
                    R = R @ _axis_rotation(joint["axis"], q)
                elif joint["type"] == "prismatic":
                    p = p + R @ (joint["axis"] * q)
                links[joint["child"]] = (R, p)
                stack.append(joint["child"])
        return links, joints

    def _subtree_links(self, joint_name):
        cache = self.__dict__.setdefault("_subtrees", {})
        if joint_name not in cache:
            out, stack = [], [self.joints[joint_name]["child"]]
            while stack:
                link = stack.pop()
                if self.links.get(link, (0.0, None))[0] > 0.0:
                    out.append(link)
                stack += [self.joints[j]["child"] for j in self.children.get(link, [])]
            cache[joint_name] = out
        return cache[joint_name]

    def torques(self, positions, joint_names, gravity=GRAVITY):
        """Holding torque (N m, or N for a prismatic joint) of each joint in `joint_names` at `positions` ({joint: value};
        missing joints are at 0). The sign is the one the joint's effort command needs: tau = -sum(m g . dc/dq).
        Per joint: the subtree's mass M and first moment S = sum(m c) give tau = -g . (axis x (S - M p)) for a revolute
        joint and -M g . axis for a prismatic one."""
        g = np.array(gravity, dtype=float)
        links, joints = self._frames(positions)
        out = []
        for name in joint_names:
            axis, origin = joints[name]
            mass, moment = 0.0, np.zeros(3)
            for link in self._subtree_links(name):
                m, com = self.links[link]
                R, p = links[link]
                mass += m
                moment += m * (p + R @ com)
            if self.joints[name]["type"] in ("revolute", "continuous"):
                out.append(-float(g @ np.cross(axis, moment - mass * origin)))
            else:
                out.append(-mass * float(g @ axis))
        return out
