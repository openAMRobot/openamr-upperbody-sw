#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Compose the MuJoCo grocery dark store around the generated robot MJCF, which is included unchanged.

  python3 simulation/tools/generate.py --config bp000 --platform-sw <pin checkout> --openarm-description <pin checkout> --out build/sim
  python3 simulation/scenes/grocery/compose_mujoco.py --robot build/sim/bp000/openamrobot_bp000.xml --out build/grocery/mujoco

Writes into --out: a byte-identical copy of the robot MJCF and its meshes/ directory, grocery_scene.xml (the store, which includes
the robot file with <include>) and compose_manifest.json (SHA-256 of the generator output and of the copy). The robot file is never
parsed into the scene text or rewritten: MuJoCo merges it at load time. The scene is a build product: never commit it.
MuJoCo only; this is not a ROS, Gazebo or firmware integration.
"""
import argparse
import hashlib
import json
import pathlib
import shutil
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import store as st  # noqa: E402

SCENE_NAME = "grocery_scene.xml"
# Robot geoms are contype 1 / conaffinity 2 and the generated floor contype 2 / conaffinity 1. Scene geoms use 3 / 3, so they touch
# the robot and each other; the static ones never touch the floor plane (both belong to the world body).
CONTACT = 'contype="3" conaffinity="3"'


def sha256(path):
    return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()


def _f(v):
    return " ".join(f"{x:.6g}" for x in v)


def _quat_yaw(yaw):
    import math
    return (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))


def _geom(e, contact=True, extra=""):
    colour = f'rgba="{_f(e["rgba"])}"'
    con = CONTACT if contact else 'contype="0" conaffinity="0"'
    yaw = f' quat="{_f(_quat_yaw(e["yaw"]))}"' if e.get("yaw") else ""
    if e["kind"] == "box":
        half = [s / 2.0 for s in e["size"]]
        return f'<geom name="{e["name"]}" type="box" size="{_f(half)}" pos="{_f(e["pos"])}"{yaw} {colour} {con} group="1"{extra}/>'
    return (f'<geom name="{e["name"]}" type="cylinder" size="{e["radius"]:.6g} {e["height"] / 2.0:.6g}" pos="{_f(e["pos"])}" '
            f'{colour} {con} group="1"{extra}/>')


def _box_inertia(mass, size):
    x, y, z = size
    return (mass * (y * y + z * z) / 12.0, mass * (x * x + z * z) / 12.0, mass * (x * x + y * y) / 12.0)


def scene_xml(layout, robot_file):
    """The scene text. `robot_file` is the include path, relative to the scene file."""
    tote, product = layout["tote"], layout["product"]
    lines = [
        "<!-- Grocery dark store demo scene, composed by simulation/scenes/grocery/compose_mujoco.py from store.yaml. -->",
        "<!-- The robot is the generated MJCF, included unchanged. MuJoCo experiment scene, not ROS or firmware integrated. -->",
        '<mujoco model="grocery_dark_store">',
        f'  <include file="{robot_file}"/>',
        "  <visual>",
        '    <headlight ambient="0.25 0.25 0.25" diffuse="0.35 0.35 0.35" specular="0.05 0.05 0.05"/>',
        '    <quality shadowsize="2048" offsamples="4"/>',
        '    <map znear="0.005" zfar="8" shadowclip="1.2"/>',
        "  </visual>",
        # znear, zfar and shadowclip are relative to the model extent; fixed here so the large visual floor does not change them.
        '  <statistic extent="8" center="1.2 0 0.7"/>',
        "  <asset>",
        '    <texture name="store_sky" type="skybox" builtin="gradient" rgb1="0.82 0.85 0.90" rgb2="0.35 0.38 0.44" width="256" height="256"/>',
        '    <texture name="store_floor_tex" type="2d" builtin="checker" rgb1="0.46 0.47 0.48" rgb2="0.43 0.44 0.45" width="512" height="512"/>',
        '    <material name="store_floor" texture="store_floor_tex" texrepeat="100 100" reflectance="0.12" specular="0.3"/>',
        "  </asset>",
        "  <worldbody>",
    ]
    for k, light in enumerate(layout["lights"]):
        lines.append(f'    <light name="store_light{k}" pos="{_f(light["pos"])}" dir="{_f(light["dir"])}" diffuse="0.45 0.45 0.43" '
                     f'specular="0.1 0.1 0.1" castshadow="{"true" if k == 0 else "false"}" cutoff="70" exponent="2"/>')
    # Visual floor (visual only; the generated floor plane carries the contacts).
    # 2 mm above the generated floor plane so the two never depth-fight; it covers the view out to the far clipping plane.
    lines.append('    <geom name="store_floor_visual" type="box" size="50 50 0.001" pos="1.4 0 0.001" material="store_floor" '
                 'contype="0" conaffinity="0" group="1"/>')
    for group in ("shelves", "fillers", "packing"):
        lines.append(f"    <!-- {group} -->")
        lines += ["    " + _geom(e) for e in layout[group]]
    lines.append("    <!-- floor markings -->")
    lines += ["    " + _geom(e, contact=False) for e in layout["markings"]]
    # The tote on the pick shelf: one free body of five boxes.
    lines.append(f'    <body name="tote" pos="{_f(tote["pos"])}">')
    lines.append('      <freejoint name="tote_free"/>')
    lines.append(f'      <inertial pos="0 0 {tote["size"][2] / 3.0:.6g}" mass="{tote["mass"]:.6g}" '
                 f'diaginertia="{_f(_box_inertia(tote["mass"], tote["size"]))}"/>')
    for name, size, centre in st.tote_parts(tote["size"], tote["wall"]):
        lines.append("      " + _geom({"name": f"tote_{name}", "kind": "box", "size": size, "pos": centre, "yaw": 0.0,
                                       "rgba": (0.12, 0.38, 0.72, 1)}, extra=f' friction="{tote["friction"]:.6g} 0.005 0.0001"'))
    lines.append("    </body>")
    # The target product. priority="1" makes its friction the pair coefficient against the fingers (the robot model keeps its own).
    lines.append(f'    <body name="{product["name"]}" pos="{_f(product["pos"])}">')
    lines.append(f'      <freejoint name="{product["name"]}_free"/>')
    lines.append(f'      <inertial pos="0 0 0" mass="{product["mass"]:.6g}" diaginertia="{_f(_box_inertia(product["mass"], product["size"]))}"/>')
    lines.append("      " + _geom({"name": f'{product["name"]}_geom', "kind": "box", "size": product["size"], "pos": (0, 0, 0), "yaw": 0.0,
                                   "rgba": product["rgba"]}, extra=f' priority="1" friction="{product["friction"]:.6g} 0.005 0.0001"'))
    lines.append(f'      <site name="{product["name"]}_site" size="0.004" rgba="1 1 1 0"/>')
    lines.append("    </body>")
    lines.append("  </worldbody>")
    lines.append("</mujoco>")
    return "\n".join(lines) + "\n"


def compose(robot_mjcf, out_dir, store_path=st.STORE):
    robot_mjcf = pathlib.Path(robot_mjcf).resolve()
    out_dir = pathlib.Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    copy = out_dir / robot_mjcf.name
    shutil.copyfile(robot_mjcf, copy)
    meshes = robot_mjcf.parent / "meshes"
    if meshes.is_dir():
        if (out_dir / "meshes").exists():
            shutil.rmtree(out_dir / "meshes")
        shutil.copytree(meshes, out_dir / "meshes")
    layout = st.layout(st.load(store_path))
    (out_dir / SCENE_NAME).write_text(scene_xml(layout, robot_mjcf.name))
    header = robot_mjcf.read_text().splitlines()[0]
    manifest = {"robot_source": str(robot_mjcf), "robot_source_sha256": sha256(robot_mjcf), "robot_copy": copy.name,
                "robot_copy_sha256": sha256(copy), "robot_header": header, "scene": SCENE_NAME, "store_sha256": sha256(store_path),
                "counts": {k: len(layout[k]) for k in ("shelves", "fillers", "markings", "packing")}}
    (out_dir / "compose_manifest.json").write_text(json.dumps(manifest, indent=1) + "\n")
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--robot", required=True, help="the generated MJCF, simulation/tools/generate.py output")
    parser.add_argument("--out", default="build/grocery/mujoco")
    opts = parser.parse_args()
    print(json.dumps(compose(opts.robot, opts.out)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
