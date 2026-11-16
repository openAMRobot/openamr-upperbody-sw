# SPDX-License-Identifier: MIT
"""A small, dependency-light reading of the expanded URDF: links, joints, inertials, geometry, and forward kinematics.

This is the reference side of every parity test. It reads the expansion text and nothing else. Forward kinematics follows the
URDF specification: a joint frame is the parent frame times <origin> (rpy is fixed-axis X, then Y, then Z), and the joint motion
is applied about <axis> in the joint frame. A mimic joint takes multiplier * q(leader) + offset.
"""
import dataclasses
import math
import xml.etree.ElementTree as ET

import numpy as np


def rpy_matrix(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_angle_matrix(axis, angle):
    axis = np.asarray(axis, float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s, C = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                     [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                     [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


def transform(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    T = np.eye(4)
    T[:3, :3] = rpy_matrix(rpy)
    T[:3, 3] = xyz
    return T


def _floats(text, default):
    return tuple(float(v) for v in text.split()) if text is not None else default


def _origin(element):
    o = element.find("origin") if element is not None else None
    if o is None:
        return transform()
    return transform(_floats(o.get("xyz"), (0.0, 0.0, 0.0)), _floats(o.get("rpy"), (0.0, 0.0, 0.0)))


@dataclasses.dataclass
class Inertial:
    mass: float
    origin: np.ndarray  # 4x4 in link frame
    inertia: np.ndarray  # 3x3 about the CoM, in the inertial frame


@dataclasses.dataclass
class Geometry:
    kind: str  # box, cylinder, sphere, mesh
    params: dict
    origin: np.ndarray
    use: str  # visual or collision
    index: int
    name: str | None = None
    material: str | None = None


@dataclasses.dataclass
class Link:
    name: str
    inertial: Inertial | None
    visuals: list
    collisions: list


@dataclasses.dataclass
class Joint:
    name: str
    type: str
    parent: str
    child: str
    origin: np.ndarray
    axis: np.ndarray
    lower: float | None = None
    upper: float | None = None
    effort: float | None = None
    velocity: float | None = None
    damping: float | None = None
    friction: float | None = None
    mimic: dict | None = None


class URDF:
    def __init__(self, text):
        self.root = ET.fromstring(text)
        self.name = self.root.get("name")
        self.links = {}
        self.joints = {}
        for l in self.root.findall("link"):
            self.links[l.get("name")] = self._link(l)
        for j in self.root.findall("joint"):
            self.joints[j.get("name")] = self._joint(j)
        self.child_joint = {j.child: j for j in self.joints.values()}
        roots = [n for n in self.links if n not in self.child_joint]
        if len(roots) != 1:
            raise ValueError(f"expected one root link, found {roots}")
        self.root_link = roots[0]
        self.children = {n: [] for n in self.links}
        for j in self.joints.values():
            self.children[j.parent].append(j)
        self.gazebo = self.root.findall("gazebo")
        self.ros2_control = self.root.findall("ros2_control")

    @staticmethod
    def _geometry(e, use, index):
        g = list(e.find("geometry"))[0]
        if g.tag == "box":
            params = {"size": _floats(g.get("size"), None)}
        elif g.tag == "cylinder":
            params = {"radius": float(g.get("radius")), "length": float(g.get("length"))}
        elif g.tag == "sphere":
            params = {"radius": float(g.get("radius"))}
        elif g.tag == "mesh":
            params = {"filename": g.get("filename"), "scale": _floats(g.get("scale"), (1.0, 1.0, 1.0))}
        else:
            raise ValueError(f"unknown geometry {g.tag}")
        m = e.find("material")
        return Geometry(kind=g.tag, params=params, origin=_origin(e), use=use, index=index, name=e.get("name"),
                        material=m.get("name") if m is not None else None)

    def _link(self, l):
        inertial = None
        i = l.find("inertial")
        if i is not None:
            t = i.find("inertia")
            I = np.array([[float(t.get("ixx")), float(t.get("ixy")), float(t.get("ixz"))],
                          [float(t.get("ixy")), float(t.get("iyy")), float(t.get("iyz"))],
                          [float(t.get("ixz")), float(t.get("iyz")), float(t.get("izz"))]])
            inertial = Inertial(mass=float(i.find("mass").get("value")), origin=_origin(i), inertia=I)
        visuals = [self._geometry(v, "visual", k) for k, v in enumerate(l.findall("visual"))]
        collisions = [self._geometry(c, "collision", k) for k, c in enumerate(l.findall("collision"))]
        return Link(name=l.get("name"), inertial=inertial, visuals=visuals, collisions=collisions)

    @staticmethod
    def _joint(j):
        axis = j.find("axis")
        lim = j.find("limit")
        dyn = j.find("dynamics")
        mim = j.find("mimic")
        out = Joint(name=j.get("name"), type=j.get("type"), parent=j.find("parent").get("link"), child=j.find("child").get("link"),
                    origin=_origin(j), axis=np.array(_floats(axis.get("xyz"), (1.0, 0.0, 0.0)) if axis is not None else (1.0, 0.0, 0.0)))
        if lim is not None:
            out.lower = float(lim.get("lower")) if lim.get("lower") is not None else None
            out.upper = float(lim.get("upper")) if lim.get("upper") is not None else None
            out.effort = float(lim.get("effort")) if lim.get("effort") is not None else None
            out.velocity = float(lim.get("velocity")) if lim.get("velocity") is not None else None
        if dyn is not None:
            out.damping = float(dyn.get("damping", 0.0))
            out.friction = float(dyn.get("friction", 0.0))
        if mim is not None:
            out.mimic = {"joint": mim.get("joint"), "multiplier": float(mim.get("multiplier", 1.0)), "offset": float(mim.get("offset", 0.0))}
        return out

    def movable(self):
        return [j for j in self.joints.values() if j.type != "fixed"]

    def tree_order(self):
        """Links in depth-first order from the root, children in document order."""
        order, stack = [], [self.root_link]
        while stack:
            name = stack.pop()
            order.append(name)
            stack.extend(j.child for j in reversed(self.children[name]))
        return order

    def resolve(self, q):
        """Full joint positions with mimic followers filled in. Unknown names fail; missing joints are 0."""
        q = dict(q)
        for name in q:
            if name not in self.joints:
                raise KeyError(name)
        for j in self.joints.values():
            if j.mimic is not None:
                q[j.name] = j.mimic["multiplier"] * q.get(j.mimic["joint"], 0.0) + j.mimic["offset"]
        return q

    def joint_motion(self, j, value):
        T = np.eye(4)
        if j.type in ("revolute", "continuous"):
            T[:3, :3] = axis_angle_matrix(j.axis, value)
        elif j.type == "prismatic":
            T[:3, 3] = np.asarray(j.axis) / np.linalg.norm(j.axis) * value
        elif j.type != "fixed":
            raise ValueError(f"joint type {j.type} is not supported by the reference kinematics")
        return T

    def fk(self, q=None, base=None):
        """World pose (4x4) of every link. base is the pose of the root link (default identity)."""
        q = self.resolve(q or {})
        poses = {self.root_link: np.eye(4) if base is None else np.asarray(base)}
        for name in self.tree_order():
            for j in self.children[name]:
                poses[j.child] = poses[name] @ j.origin @ self.joint_motion(j, q.get(j.name, 0.0))
        return poses

    def joint_frames(self, q=None):
        """World pose of each joint frame (parent frame times origin, before the motion)."""
        q = self.resolve(q or {})
        links = self.fk(q)
        return {j.name: links[j.parent] @ j.origin for j in self.joints.values()}

    def total_mass(self):
        return sum(l.inertial.mass for l in self.links.values() if l.inertial is not None)
