#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""P28 measurements on the expanded combined model: the lift column on the DOLD datasheet's sections, the carriage as the inner stage, the adapter
plate, at the baseline configuration, both base plates, the three lift stops. Nothing is changed.

    measure_p28.py OUT.json [P25_MEASUREMENTS.json]       needs the workspace sourced (xacro on the PATH, the installed packages)

Part 1 (geometry, exact: every box is axis-aligned in the world, asserted): the same quantities evidence/p25/measure_p25.py measured, on the new boxes:
  the overlap of the two stages (volume, depth) and how far the inner section sits inside the outer one on each side; the plate against the column
  (rear and front edge from the column axis, footprint over the column top), against the arm profile (gap, containment) and against the fixed stage
  top; the arm profile against the column; the arm mounts. The P25 figures are read from the second argument when given, to print before and after.
Part 2 (V01 item 3, MoveIt's collision check, at each of the six configurations): with the lift held at the stop and every other joint at its default,
  the ENABLED pairs of geometry-bearing links that collide (the quantity item 3 names: 0 expected), and the disabled pairs that touch (information).
"""
import json
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
for cand in (os.path.join(HERE, "..", "..", "ros2", "src", "openamrobot_upperbody_description", "test"),
             "/home/slash/openamr_ws/src/openamr-upperbody-sw/ros2/src/openamrobot_upperbody_description/test"):
    if os.path.isdir(cand):
        sys.path.insert(0, cand)
        break
from urdf_utils import zero_pose_poses  # noqa: E402

STOPS = [("L1000", 0.0), ("L1175", 0.175), ("L1350", 0.350)]
PLATES = ["bp000", "bp050"]
BOXES = {"fixed_stage": ("lift_column_base_link", "placeholder_lift_column_fixed_stage"),
         "moving_stage": ("lift_carriage_link", "placeholder_lift_column_moving_stage"),
         "adapter_plate": ("adapter_plate_link", "adapter_plate"),
         "arm_profile": ("arm_profile_link", None)}


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
    assert floats(origin.get("rpy") if origin is not None else None) == [0.0, 0.0, 0.0]
    return {"link": link_name, "origin_xyz": floats(origin.get("xyz") if origin is not None else None),
            "size": floats(c.find("geometry/box").get("size"))}


def aabb(box, poses):
    pose = poses[box["link"]]
    assert np.allclose(pose[:3, :3], np.eye(3)), box["link"]
    centre = pose[:3, 3] + np.array(box["origin_xyz"])
    half = np.array(box["size"]) / 2
    return centre - half, centre + half


def pair(a, b):
    extent = np.minimum(a[1], b[1]) - np.maximum(a[0], b[0])
    overlapping = bool(np.all(extent > 1e-12))
    gaps = np.maximum(0.0, -extent)
    return {"overlapping": overlapping, "overlap_extent_xyz_m": [float(v) for v in np.maximum(extent, 0.0)],
            "overlap_volume_m3": float(np.prod(extent)) if overlapping else 0.0,
            "penetration_depth_m": float(extent.min()) if overlapping else 0.0,
            "separation_xyz_m": [float(v) for v in gaps], "distance_m": float(np.linalg.norm(gaps))}


def monte_carlo_volume(a, b, n=2_000_000, seed=28):
    rng = np.random.default_rng(seed)
    pts = a[0] + rng.random((n, 3)) * (a[1] - a[0])
    return float(np.all((pts >= b[0]) & (pts <= b[1]), axis=1).mean() * np.prod(a[1] - a[0]))


def geometry():
    out = {}
    for plate in PLATES:
        root = ET.fromstring(expand(plate))
        boxes = {k: collision_box(root, *v) for k, v in BOXES.items()}
        for stop, q in STOPS:
            poses = zero_pose_poses(root, {"lift_joint": q})
            b = {k: aabb(v, poses) for k, v in boxes.items()}
            axis_x = float(poses["lift_column_base_link"][0, 3])
            fixed, moving, plt, prof = b["fixed_stage"], b["moving_stage"], b["adapter_plate"], b["arm_profile"]
            fm = pair(fixed, moving)
            fm["overlap_volume_monte_carlo_m3"] = monte_carlo_volume(moving, fixed)
            fm["inner_section_margin_inside_outer_m"] = {"front": float(fixed[1][0] - moving[1][0]), "rear": float(moving[0][0] - fixed[0][0]),
                                                         "left": float(fixed[1][1] - moving[1][1]), "right": float(moving[0][1] - fixed[0][1])}
            fm["inner_stage_fully_inside_outer_stage_section"] = all(v > 0 for v in fm["inner_section_margin_inside_outer_m"].values())
            fm["engaged_length_m"] = fm["overlap_extent_xyz_m"][2]
            out[f"{stop}/{plate}"] = {
                "lift_joint_m": q, "column_axis_x_from_base_footprint_m": axis_x,
                "outer_section_xy_m": boxes["fixed_stage"]["size"][:2], "inner_section_xy_m": boxes["moving_stage"]["size"][:2],
                "inner_stage_length_m": boxes["moving_stage"]["size"][2],
                "stage_vs_stage": fm,
                "adapter_plate": {
                    "size_m": boxes["adapter_plate"]["size"],
                    "rear_edge_from_column_axis_m": float(plt[0][0] - axis_x), "front_edge_from_column_axis_m": float(plt[1][0] - axis_x),
                    "rear_edge_minus_column_rear_face_m": float(plt[0][0] - fixed[0][0]),
                    "front_edge_beyond_column_front_face_m": float(plt[1][0] - fixed[1][0]),
                    "lateral_range_m": [float(plt[0][1]), float(plt[1][1])],
                    "bottom_z_minus_fixed_stage_top_m": float(plt[0][2] - fixed[1][2]),
                    "bottom_z_minus_moving_stage_top_m": float(plt[0][2] - moving[1][2]),
                    "footprint_over_column_top_face_m2": float(max(0.0, min(plt[1][0], fixed[1][0]) - max(plt[0][0], fixed[0][0]))
                                                               * max(0.0, min(plt[1][1], fixed[1][1]) - max(plt[0][1], fixed[0][1]))),
                    "vs_arm_profile": pair(plt, prof),
                    "arm_profile_gap_to_plate_top_m": float(prof[0][2] - plt[1][2]),
                    "arm_profile_footprint_inside_plate": bool(plt[0][0] <= prof[0][0] and prof[1][0] <= plt[1][0] + 1e-12
                                                               and plt[0][1] <= prof[0][1] and prof[1][1] <= plt[1][1]),
                    "arm_profile_front_face_minus_plate_front_edge_m": float(prof[1][0] - plt[1][0]),
                },
                "arm_profile": {
                    "vs_fixed_stage": pair(prof, fixed),
                    "x_range_from_column_axis_m": [float(prof[0][0] - axis_x), float(prof[1][0] - axis_x)],
                    "rear_face_to_column_front_face_m": float(prof[0][0] - fixed[1][0]),
                    "footprint_over_column_top_face_m2": float(max(0.0, min(prof[1][0], fixed[1][0]) - max(prof[0][0], fixed[0][0]))
                                                               * max(0.0, min(prof[1][1], fixed[1][1]) - max(prof[0][1], fixed[0][1]))),
                    "bottom_above_fixed_stage_top_m": float(prof[0][2] - fixed[1][2]),
                },
                "arm_mounts_xyz_m": {s: [float(v) for v in poses[f"openarm_{s}_base_link"][:3, 3]] for s in ("left", "right")},
                "head_camera_mount_xyz_m": [float(v) for v in poses["head_camera_mount_link"][:3, 3]],
            }
    return out


def collisions():
    from openamrobot_upperbody_moveit.acm_derive import geometry_links
    from openamrobot_upperbody_moveit.acm_scene import open_scene
    from openamrobot_upperbody_moveit.moveit_config import srdf_path
    import itertools
    from validate_acm import pair_key, srdf_pairs  # noqa: E402
    pairs = srdf_pairs(srdf_path())
    out = {}
    for plate in PLATES:
        with open_scene(base_plate=plate, node_name=f"p28_{plate}") as scene:
            geo = sorted(geometry_links(scene.urdf_root))
            enabled = [pair_key(a, b) for a, b in itertools.combinations(geo, 2) if pair_key(a, b) not in pairs]
            disabled = sorted(pairs)
            for stop, q in STOPS:
                scene.set_default_state()
                scene.set_joint_positions({"lift_joint": q})
                hit = sorted(scene.colliding(enabled))
                touching = sorted(scene.colliding(disabled))
                out[f"{stop}/{plate}"] = {"geometry_links": len(geo), "pairs_total": len(geo) * (len(geo) - 1) // 2, "enabled_pairs": len(enabled),
                                          "enabled_pairs_colliding": [list(p) for p in hit],
                                          "disabled_pairs_touching_or_overlapping": len(touching),
                                          "disabled_touching_pairs_involving_the_lift_column_or_plate": [list(p) for p in touching if
                                                                                                      set(p) & {"lift_column_base_link", "lift_carriage_link", "adapter_plate_link", "arm_profile_link"}]}
    return out


def main(out, p25=None):
    result = {"model": "combined_robot.xacro, baseline configuration (dold_v4_350 / hp25 / bt10 / openarm / mock), base plate and lift stop as listed",
              "geometry": geometry()}
    if p25 and os.path.exists(p25):
        old = json.load(open(p25))["configurations"]
        result["p25_before"] = {k: {"stage_overlap_volume_m3": v["column_vs_carriage"]["overlap_volume_m3"],
                                    "stage_penetration_depth_m": v["column_vs_carriage"]["penetration_depth_m"],
                                    "arm_profile_bottom_above_moving_stage_top_m": v["arm_profile"]["bottom_above_moving_stage_top_m"],
                                    "arm_profile_footprint_over_column_top_m2": v["arm_profile"]["footprint_overlap_with_column_top_face_m2"],
                                    "arm_mounts": v["arm_mounts"]["left_base_link_xyz_m"]} for k, v in old.items()}
    result["collision_check_item_3"] = collisions()
    with open(out, "w") as f:
        json.dump(result, f, indent=2)
        f.write("\n")
    print(f"{'config':14} {'stage overlap m3':>17} {'(P25)':>10} {'depth m':>8} {'plate-profile gap':>18} {'profile on plate':>17} {'enabled pairs colliding':>24}")
    for key, g in result["geometry"].items():
        c = result["collision_check_item_3"][key]
        before = result.get("p25_before", {}).get(key, {}).get("stage_overlap_volume_m3", float("nan"))
        print(f"{key:14} {g['stage_vs_stage']['overlap_volume_m3']:17.6f} {before:10.6f} {g['stage_vs_stage']['penetration_depth_m']:8.4f} "
              f"{g['adapter_plate']['arm_profile_gap_to_plate_top_m']:18.6f} {str(g['adapter_plate']['arm_profile_footprint_inside_plate']):>17} "
              f"{len(c['enabled_pairs_colliding'])} of {c['enabled_pairs']:>3}")


if __name__ == "__main__":
    sys.path.insert(0, "/home/slash/openamr_ws/src/openamr-upperbody-sw/ros2/src/openamrobot_upperbody_moveit/scripts")
    main(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else None)
