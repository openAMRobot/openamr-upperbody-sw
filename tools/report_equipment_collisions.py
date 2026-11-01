#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""REPORT ONLY: the P-03 item 24 moving equipment against both OpenArm arms in the six V01 configurations.

    source /opt/ros/jazzy/setup.bash && source <workspace>/install/setup.bash
    python3 tools/report_equipment_collisions.py [--json out.json]

For bp000 and bp050, for the tablet in landscape (the model) and in portrait (P-03 rev20.2 item 26, never drawn), and at the lift stops
L1000 / L1175 / L1350, it checks every pair (moving equipment link, arm link) for collision in the default state and in every named arm
pose of the SRDF (both arms in the same named pose, grippers at their defaults). The SRDF's disabled list is not consulted: the scene's
own allowed-collision matrix forbids exactly the pairs under test (openamrobot_upperbody_moveit.acm_scene).

It changes nothing and decides nothing: a collision is reported with its pair and configuration, and no part is moved to clear it.

Portrait is a report-only datum (getac_tablet_portrait_size). Its pivot is NOT stated (datum getac_tablet_portrait_pivot is null); this
report assumes the portrait box has the landscape box's centre, and says so in its output. That is an assumption of the report, not a datum.
Exit status: 0 whatever was found (it is a report), 2 when it could not run.
"""
import argparse
import copy
import json
import os
import subprocess
import sys
import tempfile

MOVING_EQUIPMENT = ["getac_tablet_link", "head_light_link", "lift_flag_rod_link", "lift_flag_link", "head_camera_link", "head_camera_bracket_link"]
BASE_PLATES = ["bp000", "bp050"]
ORIENTATIONS = ["landscape", "portrait"]
PIVOT_ASSUMPTION = ("portrait box centred where the landscape box is centred (pivot not stated: datum getac_tablet_portrait_pivot is null); "
                    "the head light stays at its landscape position")


def worker(base_plate, orientation, out_path):
    """One MoveItPy per process (a MoveItPy that is torn down segfaults): leaves with os._exit."""
    import xml.etree.ElementTree as ET

    import moveit_configs_utils
    import yaml
    from ament_index_python.packages import get_package_share_directory
    from openamrobot_upperbody_moveit.acm_scene import open_scene
    from openamrobot_upperbody_moveit.moveit_config import lift_stops

    share = get_package_share_directory("openamrobot_upperbody_description")
    datums_file = os.path.join(share, "config", "datums.yaml")
    if orientation == "portrait":
        with open(datums_file) as f:
            doc = yaml.safe_load(f)
        doc = copy.deepcopy(doc)
        land = doc["datums"]["getac_tablet_size"]["value"]
        port = doc["datums"]["getac_tablet_portrait_size"]["value"]
        doc["datums"]["getac_tablet_size"]["value"] = list(port)
        # the head light does not move with the tablet: its height derives from the tablet's bottom edge, so the gap is shifted by the
        # change of half-height to keep the head light where the landscape model has it (a negative gap is an overlap)
        doc["datums"]["head_light_gap_below_tablet"]["value"] -= (port[2] - land[2]) / 2
        datums_file = os.path.join(tempfile.mkdtemp(), "datums_portrait.yaml")
        with open(datums_file, "w") as f:
            yaml.safe_dump(doc, f)
        original = moveit_configs_utils.MoveItConfigsBuilder.robot_description

        def with_datums(self, file_path=None, mappings=None):
            return original(self, file_path=file_path, mappings={**(mappings or {}), "datums_file": datums_file})
        moveit_configs_utils.MoveItConfigsBuilder.robot_description = with_datums

    with open_scene(base_plate, node_name=f"equipment_report_{orientation}") as scene:
        arm_links = [n for n in scene.link_names if n.startswith(("openarm_left_", "openarm_right_"))]
        geometry = {l.get("name") for l in scene.urdf_root.findall("link") if l.find("collision") is not None}
        arm_links = [n for n in arm_links if n in geometry]
        pairs = [(e, a) for e in MOVING_EQUIPMENT for a in arm_links]
        # information: the moving equipment against every other geometry link (mounting faces that touch by design show up here; the
        # SRDF disables those as Adjacent); and a control that the check sees a collision at all
        others = sorted(geometry - set(arm_links) - set(MOVING_EQUIPMENT))
        other_pairs = [(e, o) for e in MOVING_EQUIPMENT for o in others] + [
            (a, b) for i, a in enumerate(MOVING_EQUIPMENT) for b in MOVING_EQUIPMENT[i + 1:]]
        srdf = ET.fromstring(scene.config["robot_description_semantic"])
        poses = {}
        for state in srdf.findall("group_state"):
            if state.get("group") in ("left_arm", "right_arm"):
                poses.setdefault(state.get("name"), {}).update({j.get("name"): float(j.get("value")) for j in state.findall("joint")})
        tablet = scene.urdf_root.find("link[@name='getac_tablet_link']/collision/geometry/box").get("size")
        rows = []
        for stop, q in lift_stops().items():
            for pose_name, joints in [("default", {})] + sorted(poses.items()):
                scene.set_default_state()
                scene.set_joint_positions({"lift_joint": q, **joints})
                rows.append({"base_plate": base_plate, "lift_stop": stop, "arm_pose": pose_name, "orientation": orientation,
                             "colliding": [list(p) for p in scene.colliding(pairs)],
                             "other_links_colliding": [list(p) for p in scene.colliding(other_pairs)]})
        # control: the flag is bolted to the rod (faces touching by design, Adjacent in the SRDF): the check must see that contact
        scene.set_default_state()
        control = {"pair": ["lift_flag_rod_link", "lift_flag_link"],
                   "colliding": bool(scene.colliding([("lift_flag_rod_link", "lift_flag_link")]))}
    with open(out_path, "w") as f:
        json.dump({"base_plate": base_plate, "orientation": orientation, "tablet_box": tablet, "pairs_checked": len(pairs),
                   "arm_links": arm_links, "rows": rows,
                   "control": control}, f, indent=1)
    sys.stdout.flush()
    os._exit(0)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--json", help="write the full result here")
    parser.add_argument("--worker", nargs=3, metavar=("BASE_PLATE", "ORIENTATION", "OUT"), help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.worker:
        worker(*args.worker)
        return 0
    results = []
    with tempfile.TemporaryDirectory() as tmp:
        for base_plate in BASE_PLATES:
            for orientation in ORIENTATIONS:
                out = os.path.join(tmp, f"{base_plate}_{orientation}.json")
                done = subprocess.run([sys.executable, os.path.abspath(__file__), "--worker", base_plate, orientation, out],
                                      capture_output=True, text=True, timeout=1800)
                if not os.path.isfile(out):
                    print(f"report_equipment_collisions: {base_plate}/{orientation} produced no result (exit {done.returncode}):\n"
                          f"{done.stdout[-1500:]}\n{done.stderr[-1500:]}", file=sys.stderr)
                    return 2
                with open(out) as f:
                    results.append(json.load(f))
    print("P-03 item 24 moving equipment vs both arms, six V01 configurations (REPORT ONLY; nothing moved to clear a collision)")
    print(f"portrait: {PIVOT_ASSUMPTION}")
    for result in results:
        print(f"\n{result['orientation']} {result['base_plate']}: tablet box {result['tablet_box']}, {result['pairs_checked']} pairs "
              f"({len(MOVING_EQUIPMENT)} moving equipment links x {len(result['arm_links'])} arm links)")
        for row in result["rows"]:
            hits = ", ".join(f"{a} <-> {b}" for a, b in row["colliding"]) or "none"
            other = ", ".join(f"{a} <-> {b}" for a, b in row["other_links_colliding"]) or "none"
            print(f"  {row['lift_stop']} {row['arm_pose']:<9} arms: {hits}   | other links (information): {other}")
        print(f"  control {result['control']['pair']}: colliding={result['control']['colliding']}")
    if args.json:
        with open(args.json, "w") as f:
            json.dump({"pivot_assumption": PIVOT_ASSUMPTION, "results": results}, f, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
