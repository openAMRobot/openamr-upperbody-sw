#!/usr/bin/env python3
"""V01 items 1, 2 and 4 from the expanded URDF: the reading `tf2_echo <from> <to>` would give, from the joint origins and the lift joint.
Usage: v01_fk.py [--json out.json]. Needs `xacro` on PATH and the workspace sourced."""
import json
import math
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np


def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def T_from(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rpy_to_R(*rpy)
    T[:3, 3] = xyz
    return T


def floats(s, default):
    return [float(v) for v in s.split()] if s else list(default)


class Model:
    def __init__(self, urdf_text):
        self.root = ET.fromstring(urdf_text)
        self.parent = {}      # child link -> (joint element, parent link)
        for j in self.root.findall("joint"):
            self.parent[j.find("child").get("link")] = (j, j.find("parent").get("link"))
        self.links = {l.get("name"): l for l in self.root.findall("link")}

    def T(self, link, q=None):
        """base-chain transform root -> link, with joint positions q {joint name: value} for prismatic/revolute joints."""
        q = q or {}
        chain = []
        while link in self.parent:
            j, link = self.parent[link]
            chain.append(j)
        T = np.eye(4)
        for j in reversed(chain):
            o = j.find("origin")
            Tj = T_from(floats(o.get("xyz") if o is not None else None, (0, 0, 0)), floats(o.get("rpy") if o is not None else None, (0, 0, 0)))
            if j.get("type") in ("prismatic", "revolute", "continuous") and j.get("name") in q:
                axis = np.array(floats(j.find("axis").get("xyz") if j.find("axis") is not None else None, (1, 0, 0)))
                val = q[j.get("name")]
                M = np.eye(4)
                if j.get("type") == "prismatic":
                    M[:3, 3] = axis * val
                else:
                    a = axis / np.linalg.norm(axis)
                    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
                    M[:3, :3] = np.eye(3) + math.sin(val) * K + (1 - math.cos(val)) * K @ K
                Tj = Tj @ M
            T = T @ Tj
        return T

    def rel(self, a, b, q=None):
        """pose of b in a's frame, as tf2_echo a b prints it: (xyz, rpy)."""
        T = np.linalg.inv(self.T(a, q)) @ self.T(b, q)
        R = T[:3, :3]
        pitch = -math.asin(max(-1.0, min(1.0, R[2, 0])))
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
        return T[:3, 3].tolist(), [roll, pitch, yaw]


def expand(**kw):
    args = ["xacro", "-o", "/dev/stdout", f"{kw.pop('file')}"] + [f"{k}:={v}" for k, v in kw.items()]
    out = subprocess.run(["xacro", args[3]] + args[4:], capture_output=True, text=True, check=True).stdout
    return out


if __name__ == "__main__":
    from ament_index_python.packages import get_package_share_directory as share
    combined = share("openamrobot_upperbody_description") + "/urdf/combined_robot.xacro"
    out = {}
    STOPS = {"L1000": 0.0, "L1175": 0.175, "L1350": 0.350}
    for plate in ("bp000", "bp050"):
        m = Model(expand(file=combined, base_plate=plate, hardware_profile="mock"))
        r = {}
        r["column_base"] = m.rel("base_footprint", "lift_column_base_link")[0]
        for stop, v in STOPS.items():
            q = {"lift_joint": v}
            r[f"left_base@{stop}"] = m.rel("base_footprint", "openarm_left_base_link", q)[0]
            r[f"right_base@{stop}"] = m.rel("base_footprint", "openarm_right_base_link", q)[0]
            r[f"head_mount@{stop}"] = m.rel("base_footprint", "head_camera_mount_link", q)[0]
        r["carriage_to_left_base"] = m.rel("lift_carriage_link", "openarm_left_base_link")[0]
        r["carriage_to_right_base"] = m.rel("lift_carriage_link", "openarm_right_base_link")[0]
        r["head_optical_rpy_in_mount_deg"] = [math.degrees(a) for a in m.rel("head_camera_mount_link", "head_camera_optical_frame")[1]]
        r["head_link_rpy_in_mount_deg"] = [math.degrees(a) for a in m.rel("head_camera_mount_link", "head_camera_link")[1]]
        r["wrist_left"] = m.rel("openarm_left_ee_base_link", "left_wrist_camera_link")[0] if "left_wrist_camera_link" in m.links else None
        r["wrist_right"] = m.rel("openarm_right_ee_base_link", "right_wrist_camera_link")[0] if "right_wrist_camera_link" in m.links else None
        prof = m.links["arm_profile_link"]
        r["arm_profile_visual"] = [(g.find("geometry/box").get("size") if g.find("geometry/box") is not None else "non-box",
                                    g.find("origin").get("xyz") if g.find("origin") is not None else "0 0 0") for g in prof.findall("visual")]
        r["arm_profile_collision"] = [(g.find("geometry/box").get("size") if g.find("geometry/box") is not None else "non-box",
                                       g.find("origin").get("xyz") if g.find("origin") is not None else "0 0 0") for g in prof.findall("collision")]
        r["arm_profile_in_carriage"] = m.rel("lift_carriage_link", "arm_profile_link")[0]
        r["column_links_with_visual"] = [n for n in m.links if n.startswith("lift_") and m.links[n].findall("visual")]
        r["column_links_with_collision"] = [n for n in m.links if n.startswith("lift_") and m.links[n].findall("collision")]
        r["lift_links"] = [n for n in m.links if n.startswith("lift_")]
        r["lift_joint"] = {k: (v.get("type")) for k, v in ((j.get("name"), j) for j in m.root.findall("joint")) if k == "lift_joint"}
        out[plate] = r
    print(json.dumps(out, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)))
