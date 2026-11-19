# SPDX-License-Identifier: MIT
"""Clearance between the robot and the work shelf during an in-place turn at the station. No ROS import.

openamr-platform-sw #167 (from 99ec17f) corrects a target behind the robot by turning to it first, up to a half turn in
place, next to the shelf. This module measures how close any collision geometry of the robot comes to the shelf during
that turn, from the expanded URDF (the model the simulation runs), the arm joint values given (park, the carry pose of
the mission) and the shelf boxes of config/scene.yaml.

Method: every collision shape is turned into surface points (box faces on a grid, cylinder rings and lines, sphere
points, the vertices of a mesh), placed in base_footprint by forward kinematics. A differential drive with its drive
wheels on the base_link y axis turns about the base_footprint origin, so the points are rotated about that axis in
steps over the turn, both directions. Each shelf box is a vertical prism over its height range; a point inside that
range is measured to the box's floor rectangle. The result is the smallest distance (negative: the point is inside a
box) with the angle, the robot link and the shelf box. Accuracy is limited by the point spacing (`spacing`, 0.01 m)
and the angle step; mesh shapes are measured at their vertices only. Geometry only: no controller tracking error, no
localization error beyond the `overshoot` given, no dynamics. Simulation-model evidence, not a robot measurement.
"""
import math
import struct
import xml.etree.ElementTree as ET

import numpy as np

from openamrobot_build1_scene import scene as scene_model


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _axis_angle(axis, angle):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k


def _origin(element):
    t = np.eye(4)
    if element is not None:
        t[:3, :3] = _rpy(*[float(v) for v in element.get("rpy", "0 0 0").split()])
        t[:3, 3] = [float(v) for v in element.get("xyz", "0 0 0").split()]
    return t


def link_poses(urdf_text, positions, root="base_footprint"):
    """{link: 4x4 pose in `root`} for every link below `root`, joints at `positions` (default 0)."""
    tree = ET.fromstring(urdf_text)
    children = {}
    for joint in tree.findall("joint"):
        children.setdefault(joint.find("parent").get("link"), []).append(joint)
    poses, stack = {root: np.eye(4)}, [root]
    while stack:
        parent = stack.pop()
        for joint in children.get(parent, []):
            motion = np.eye(4)
            value = positions.get(joint.get("name"), 0.0)
            axis_element = joint.find("axis")
            axis = [float(v) for v in (axis_element.get("xyz") if axis_element is not None else "1 0 0").split()]
            if joint.get("type") in ("revolute", "continuous"):
                motion[:3, :3] = _axis_angle(axis, value)
            elif joint.get("type") == "prismatic":
                motion[:3, 3] = np.asarray(axis) * value
            child = joint.find("child").get("link")
            poses[child] = poses[parent] @ _origin(joint.find("origin")) @ motion
            stack.append(child)
    return poses


def read_stl(path):
    """Vertices (n x 3) of a binary or ASCII STL file."""
    with open(path, "rb") as f:
        data = f.read()
    if len(data) >= 84:
        count = struct.unpack("<I", data[80:84])[0]
        if 84 + 50 * count == len(data):
            out = np.empty((count * 3, 3))
            for i in range(count):
                start = 84 + 50 * i + 12
                out[3 * i:3 * i + 3] = np.array(struct.unpack("<9f", data[start:start + 36])).reshape(3, 3)
            return out
    text = data.decode("ascii", errors="ignore")
    return np.array([[float(v) for v in line.split()[1:4]] for line in text.splitlines()
                     if line.strip().startswith("vertex")])


def _grid(lo, hi, spacing):
    return np.linspace(lo, hi, max(2, int(math.ceil((hi - lo) / spacing)) + 1))


def shape_points(geometry, resolve, spacing=0.01):
    """Surface points of one URDF <geometry> child, in the shape's own frame."""
    tag = geometry.tag
    if tag == "box":
        sx, sy, sz = [float(v) / 2.0 for v in geometry.get("size").split()]
        pts = []
        for axis, (a, b, c) in enumerate(((sx, sy, sz), (sy, sz, sx), (sz, sx, sy))):
            u, v = np.meshgrid(_grid(-b, b, spacing), _grid(-c, c, spacing))
            for sign in (-1.0, 1.0):
                face = np.stack([np.full(u.size, sign * a), u.ravel(), v.ravel()], axis=1)
                pts.append(np.roll(face, axis, axis=1))
        return np.concatenate(pts)
    if tag == "cylinder":
        r, length = float(geometry.get("radius")), float(geometry.get("length"))
        n = max(16, int(2 * math.pi * r / spacing))
        angles = np.linspace(0, 2 * math.pi, n, endpoint=False)
        ring = np.stack([r * np.cos(angles), r * np.sin(angles)], axis=1)
        return np.concatenate([np.column_stack([ring, np.full(n, z)]) for z in _grid(-length / 2, length / 2, spacing)])
    if tag == "sphere":
        r = float(geometry.get("radius"))
        n = max(64, int(4 * math.pi * r * r / spacing ** 2))
        k = np.arange(n) + 0.5
        phi, theta = np.arccos(1 - 2 * k / n), math.pi * (1 + 5 ** 0.5) * k
        return r * np.stack([np.cos(theta) * np.sin(phi), np.sin(theta) * np.sin(phi), np.cos(phi)], axis=1)
    if tag == "mesh":
        scale = np.array([float(v) for v in geometry.get("scale", "1 1 1").split()])
        return read_stl(resolve(geometry.get("filename"))) * scale
    raise ValueError(f"unsupported collision geometry {tag}")


def robot_points(urdf_text, positions, resolve, spacing=0.01):
    """(points n x 3 in base_footprint, link name per point) of every collision shape."""
    poses = link_poses(urdf_text, positions)
    pts, names = [], []
    for link in ET.fromstring(urdf_text).findall("link"):
        if link.get("name") not in poses:
            continue
        for collision in link.findall("collision"):
            local = shape_points(collision.find("geometry")[0], resolve, spacing)
            t = poses[link.get("name")] @ _origin(collision.find("origin"))
            pts.append(local @ t[:3, :3].T + t[:3, 3])
            names += [link.get("name")] * len(local)
    return np.concatenate(pts), np.array(names)


def shelf_prisms(scene, name="work_shelf", objects=True, solid=False):
    """[(box name, centre x y, half sizes x y, yaw, z low, z high)] of the shelf unit's boxes, and with `objects` the
    tote, item and payload of scene.movable_objects (the tote from its level up by its height, as placed on the board).
    `solid` replaces the unit by one box over its whole outline and height: a shelf closed at the front (bins, panels,
    stock up to the front edge), the conservative case the open model of scene.yaml does not show."""
    pose = scene_model.shelf_poses(scene)[name]
    boxes = scene_model.shelf_boxes(scene, name, pose)
    if solid:
        shelf = scene["shelf"]
        length, depth = scene_model.value(shelf["unit_length"]), scene_model.value(shelf["depth"])
        height = max(scene_model.value(level) for level in shelf["levels"])
        out = [(f"{name}_solid", (pose[0], pose[1]), (length / 2, depth / 2), pose[2], 0.0, height)]
    else:
        out = [(box, (cx, cy), (sx / 2, sy / 2), yaw, cz - sz / 2, cz + sz / 2)
               for box, (sx, sy, sz), (cx, cy, cz), yaw in boxes]
    if objects and name == "work_shelf":
        for obj, body in scene_model.movable_objects(scene).items():
            sx, sy, sz = body["size"]
            x, y, z, yaw = body["pose"]
            zlo, zhi = (z, z + sz) if obj == "tote" else (z - sz / 2, z + sz / 2)
            out.append((obj, (x, y), (sx / 2, sy / 2), yaw, zlo, zhi))
    return out


def _rect_distance(xy, centre, half, yaw):
    """Signed planar distance of points xy (n x 2) to a rotated rectangle: negative inside."""
    c, s = math.cos(yaw), math.sin(yaw)
    d = xy - np.asarray(centre)
    local = np.abs(np.stack([c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]], axis=1)) - np.asarray(half)
    outside = np.linalg.norm(np.maximum(local, 0.0), axis=1)
    inside = np.minimum(np.max(local, axis=1), 0.0)
    return outside + inside


def sweep(points, names, prisms, station, turn=math.pi, step_deg=0.5, overshoot=0.0):
    """Minimum clearance during an in-place turn of up to `turn` rad in either direction from `station` (x, y, yaw),
    with the base `overshoot` m further along its heading (towards the shelf it faces).
    -> {"min_clearance_m", "angle_deg", "link", "box", "at_station_m"}."""
    x0, y0, yaw0 = station
    x0, y0 = x0 + overshoot * math.cos(yaw0), y0 + overshoot * math.sin(yaw0)
    best = {"min_clearance_m": math.inf}
    at_station = math.inf
    angles = np.radians(np.arange(-math.degrees(turn), math.degrees(turn) + 1e-9, step_deg))
    for box, centre, half, yaw, zlo, zhi in prisms:
        mask = (points[:, 2] >= zlo) & (points[:, 2] <= zhi)
        if not mask.any():
            continue
        p, n = points[mask], names[mask]
        for angle in angles:
            c, s = math.cos(yaw0 + angle), math.sin(yaw0 + angle)
            xy = np.stack([x0 + c * p[:, 0] - s * p[:, 1], y0 + s * p[:, 0] + c * p[:, 1]], axis=1)
            d = _rect_distance(xy, centre, half, yaw)
            i = int(np.argmin(d))
            if abs(angle) < 1e-12:
                at_station = min(at_station, float(d[i]))
            if d[i] < best["min_clearance_m"]:
                best = {"min_clearance_m": float(d[i]), "angle_deg": round(math.degrees(angle), 2),
                        "link": str(n[i]), "box": box, "point_base_footprint": [round(float(v), 4) for v in p[i]]}
    best["at_station_m"] = at_station
    return best
