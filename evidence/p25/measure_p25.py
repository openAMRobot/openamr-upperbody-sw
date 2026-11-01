#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""P25 measurements on the expanded combined model (baseline configuration, both base plates, the three lift stops). Nothing is changed.

    measure_p25.py OUT.json        needs the workspace sourced (xacro on the PATH, the installed description package)

What it reads: the collision boxes of lift_column_base_link (fixed stage), lift_carriage_link (moving stage) and arm_profile_link, and the
joint origins between them, from the expanded URDF. All three boxes are axis-aligned in the world (every rotation on the path is zero; the
script asserts it), so the intersection of two of them is exact:

    overlap extent per axis   = min(upper) - max(lower), when positive on all three axes the boxes overlap
    overlap volume            = the product of the three extents
    penetration depth         = the smallest of the three extents (the shortest translation along a coordinate axis that separates them,
                                which is what a collision checker reports as the depth of an axis-aligned box pair)
    gap (separated boxes)     = per axis max(0, max(lower) - min(upper)); distance = their Euclidean norm

The volume is cross-checked by counting uniform random points of the first box that fall in the second (seeded, 2e6 points), an
independent method that shares no code with the extent arithmetic.

Item 5 is an experiment on a COPY of the expanded text: lift_column_base_joint re-parented to base_link with the z origin lowered by the
base_footprint_joint's 0.098 m, and without that lowering; the model on disk is untouched.
"""
import json
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "ros2", "src", "openamrobot_upperbody_description", "test"))
from urdf_utils import zero_pose_poses  # noqa: E402

STOPS = [("L1000", 0.0), ("L1175", 0.175), ("L1350", 0.350)]
PLATES = ["bp000", "bp050"]
FIXED = ("lift_column_base_link", "placeholder_lift_column_fixed_stage")
MOVING = ("lift_carriage_link", "placeholder_lift_column_moving_stage")
PROFILE = ("arm_profile_link", None)


def expand(base_plate):
    from ament_index_python.packages import get_package_share_directory
    combined = os.path.join(get_package_share_directory("openamrobot_upperbody_description"), "urdf", "combined_robot.xacro")
    return subprocess.run(["xacro", combined, f"base_plate:={base_plate}"], capture_output=True, text=True, check=True).stdout


def floats(text, default="0 0 0"):
    return [float(v) for v in (text or default).split()]


def collision_box(root, link_name, collision_name):
    link = next(l for l in root.findall("link") if l.get("name") == link_name)
    found = [c for c in link.findall("collision") if collision_name is None or c.get("name") == collision_name]
    assert len(found) == 1, (link_name, collision_name, len(found))
    c = found[0]
    origin = c.find("origin")
    xyz = floats(origin.get("xyz") if origin is not None else None)
    rpy = floats(origin.get("rpy") if origin is not None else None)
    assert rpy == [0.0, 0.0, 0.0], (link_name, rpy)
    return {"link": link_name, "origin_present": origin is not None, "origin_xyz": xyz, "size": floats(c.find("geometry/box").get("size"))}


def world_aabb(box, poses, positions):
    pose = poses[box["link"]]
    assert np.allclose(pose[:3, :3], np.eye(3)), box["link"]
    centre = pose[:3, 3] + np.array(box["origin_xyz"])
    half = np.array(box["size"]) / 2
    return centre - half, centre + half


def pair(a, b):
    """Exact axis-aligned box pair: per-axis overlap extents (negative = gap), volume, depth, distance."""
    lo = np.maximum(a[0], b[0])
    hi = np.minimum(a[1], b[1])
    extent = hi - lo                                   # positive: overlap on that axis; negative: separation on that axis
    overlapping = bool(np.all(extent > 1e-12))
    volume = float(np.prod(extent)) if overlapping else 0.0
    gaps = np.maximum(0.0, -extent)
    return {"overlap_extent_xyz_m": [float(v) for v in np.maximum(extent, 0.0)],
            "separation_xyz_m": [float(v) for v in gaps],
            "overlapping": overlapping,
            "overlap_volume_m3": volume,
            "penetration_depth_m": float(extent.min()) if overlapping else 0.0,
            "distance_m": float(np.linalg.norm(gaps))}


def monte_carlo_volume(a, b, n=2_000_000, seed=25):
    rng = np.random.default_rng(seed)
    pts = a[0] + rng.random((n, 3)) * (a[1] - a[0])
    inside = np.all((pts >= b[0]) & (pts <= b[1]), axis=1)
    return float(inside.mean() * np.prod(a[1] - a[0]))


def point_to_box_distance(p, box):
    d = np.maximum(np.maximum(box[0] - p, p - box[1]), 0.0)
    return float(np.linalg.norm(d)), [float(v) for v in d]


def main(out):
    result = {"model": "combined_robot.xacro, baseline configuration (dold_v4_350 / hp25 / bt10 / openarm / mock), base plate and lift stop as listed",
              "configurations": {}}
    for plate in PLATES:
        text = expand(plate)
        root = ET.fromstring(text)
        boxes = {"fixed_stage": collision_box(root, *FIXED), "moving_stage": collision_box(root, *MOVING), "arm_profile": collision_box(root, *PROFILE)}
        for stop, q in STOPS:
            poses = zero_pose_poses(root, {"lift_joint": q})
            ab = {k: world_aabb(v, poses, q) for k, v in boxes.items()}
            fixed, moving, prof = ab["fixed_stage"], ab["moving_stage"], ab["arm_profile"]
            axis_x = poses["lift_column_base_link"][0, 3]
            fm = pair(fixed, moving)
            fm["overlap_volume_monte_carlo_m3"] = monte_carlo_volume(moving, fixed)
            fm["overlap_z_range_m"] = [float(max(fixed[0][2], moving[0][2])), float(min(fixed[1][2], moving[1][2]))]
            fm["engaged_length_of_moving_stage_inside_fixed_stage_m"] = fm["overlap_extent_xyz_m"][2]
            left = poses["openarm_left_base_link"][:3, 3]
            right = poses["openarm_right_base_link"][:3, 3]
            d_left, dv_left = point_to_box_distance(left, prof)
            d_right, dv_right = point_to_box_distance(right, prof)
            cfg = {
                "lift_joint_m": q,
                "column_axis_x_from_base_footprint_m": float(axis_x),
                "fixed_stage_z_range_m": [float(fixed[0][2]), float(fixed[1][2])],
                "moving_stage_z_range_m": [float(moving[0][2]), float(moving[1][2])],
                "column_section_xy_m": boxes["fixed_stage"]["size"][:2],
                "column_vs_carriage": fm,
                "arm_profile": {
                    "x_range_from_column_axis_m": [float(prof[0][0] - axis_x), float(prof[1][0] - axis_x)],
                    "y_range_m": [float(prof[0][1]), float(prof[1][1])],
                    "z_range_m": [float(prof[0][2]), float(prof[1][2])],
                    "vs_fixed_stage": pair(prof, fixed),
                    "vs_moving_stage": pair(prof, moving),
                    "rear_face_to_column_front_face_m": float(prof[0][0] - fixed[1][0]),
                    "bottom_above_fixed_stage_top_m": float(prof[0][2] - fixed[1][2]),
                    "bottom_above_moving_stage_top_m": float(prof[0][2] - moving[1][2]),
                    "centre_x_from_column_axis_m": float((prof[0][0] + prof[1][0]) / 2 - axis_x),
                    "far_face_x_from_column_axis_m": float(prof[1][0] - axis_x),
                    "far_face_beyond_column_front_face_m": float(prof[1][0] - fixed[1][0]),
                    "centre_z_above_fixed_stage_top_m": float((prof[0][2] + prof[1][2]) / 2 - fixed[1][2]),
                    "footprint_overlap_with_column_top_face_m2": float(max(0.0, min(prof[1][0], fixed[1][0]) - max(prof[0][0], fixed[0][0]))
                                                                       * max(0.0, min(prof[1][1], fixed[1][1]) - max(prof[0][1], fixed[0][1]))),
                    "box_origin_present_in_urdf": boxes["arm_profile"]["origin_present"],
                    "box_centre_in_link_frame_m": boxes["arm_profile"]["origin_xyz"],
                },
                "arm_mounts": {
                    "left_base_link_xyz_m": [float(v) for v in left], "right_base_link_xyz_m": [float(v) for v in right],
                    "left_distance_to_profile_box_m": d_left, "left_per_axis_m": dv_left,
                    "right_distance_to_profile_box_m": d_right, "right_per_axis_m": dv_right,
                    "left_z_above_profile_bottom_m": float(left[2] - prof[0][2]),
                },
            }
            result["configurations"][f"{stop}/{plate}"] = cfg

        # item 5: re-parenting experiment on copies
        baseline_poses = zero_pose_poses(root, {"lift_joint": 0.175})
        for label, z in (("reparented_with_compensation", 0.304 - 0.098), ("reparented_without_compensation", 0.304)):
            trial = ET.fromstring(text)
            joint = next(j for j in trial.findall("joint") if j.get("name") == "lift_column_base_joint")
            assert joint.find("parent").get("link") == "base_footprint"
            joint.find("parent").set("link", "base_link")
            origin = joint.find("origin")
            xyz = floats(origin.get("xyz"))
            origin.set("xyz", f"{xyz[0]} {xyz[1]} {z}")
            poses = zero_pose_poses(trial, {"lift_joint": 0.175})
            moved = {n: float(np.abs(poses[n] - baseline_poses[n]).max()) for n in baseline_poses}
            changed = sorted(n for n, d in moved.items() if d > 1e-9)
            result.setdefault("reparent_experiment_" + plate, {})[label] = {
                "joint_origin_z_m": z, "links_whose_pose_in_base_footprint_changed": len(changed), "of_links": len(moved),
                "largest_change_m": max(moved.values()),
                "changed_links_sample": changed[:6],
            }
        baseline_parent = next(j for j in root.findall("joint") if j.get("name") == "lift_column_base_joint")
        result.setdefault("lift_column_base_joint_" + plate, {"parent": baseline_parent.find("parent").get("link"),
                                                              "origin_xyz": floats(baseline_parent.find("origin").get("xyz"))})
        base_joint = next(j for j in root.findall("joint") if j.get("name") == "base_footprint_joint")
        result.setdefault("base_footprint_joint", {"origin_xyz": floats(base_joint.find("origin").get("xyz")),
                                                   "parent": base_joint.find("parent").get("link"), "child": base_joint.find("child").get("link")})
    with open(out, "w") as f:
        json.dump(result, f, indent=2)
        f.write("\n")

    print(f"{'config':14} {'overlap vol m3':>15} {'MC check m3':>12} {'depth(min axis) m':>18} {'overlap z m':>12}   profile: rear-face-to-front-face / bottom above top (fixed, moving)")
    for key, c in result["configurations"].items():
        fm, ap = c["column_vs_carriage"], c["arm_profile"]
        print(f"{key:14} {fm['overlap_volume_m3']:15.6f} {fm['overlap_volume_monte_carlo_m3']:12.6f} {fm['penetration_depth_m']:18.4f} "
              f"{fm['overlap_extent_xyz_m'][2]:12.4f}   {ap['rear_face_to_column_front_face_m']:.4f} / {ap['bottom_above_fixed_stage_top_m']:.4f}, {ap['bottom_above_moving_stage_top_m']:.4f}")


if __name__ == "__main__":
    main(sys.argv[1])
