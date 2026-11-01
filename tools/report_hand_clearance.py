#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""REPORT ONLY: minimum clearance between each hand (the gripper links) and the base top cover at L1000 in the rest pose, bp000 and bp050.

    source /opt/ros/jazzy/setup.bash && source <workspace>/install/setup.bash
    python3 tools/report_hand_clearance.py [--json out.json]

Rest pose: the state the stack starts in, every arm and finger joint at 0 (ros2_control initial_value 0.0; the SRDF's `home` arm pose and the
`open` gripper pose), the lift at 0 (calibration stop L1000). Hand: the gripper links of each arm that carry collision geometry
(openarm_{side}_ee_base_link, _ee_link1, _ee_link2, upstream meshes).

Two figures per hand, because the model has two "covers":
  collision  MoveIt's distance (FCL) between the hand's collision meshes and base_link's collision box, whose top face is the cover top
             (platform-sw: 0.780 x 0.580 x 0.240 m box, top 0.295 m above the floor). The SRDF is not consulted: the scene's own allowed-collision
             matrix forbids only the pairs measured (openamrobot_upperbody_moveit.acm_scene). This is what planning sees.
  visual     the distance between the same hand meshes and base_link's visual cover mesh (platform-sw meshes/visual/base_covers.stl), computed here
             from the meshes (closest point of each vertex of one mesh to the triangles of the other, both ways; exact for face-vertex contacts,
             an upper bound by at most the mesh's edge-edge gap otherwise).
Both are the same robot posed the same way; neither is a measurement of hardware. Exit 0 whatever it finds, 2 when it could not run.
"""
import argparse
import json
import os
import struct
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

import numpy as np

BASE_PLATES = ["bp000", "bp050"]
SIDES = ["left", "right"]
COVER_MESH = "base_covers.stl"


def load_stl(path):
    with open(path, "rb") as f:
        data = f.read()
    if data[:5] == b"solid" and b"facet" in data[:400]:
        import re
        v = np.array([[float(x) for x in m.groups()] for m in re.finditer(rb"vertex\s+(\S+)\s+(\S+)\s+(\S+)", data)])
        return v.reshape(-1, 3, 3)
    n = struct.unpack("<I", data[80:84])[0]
    rec = np.frombuffer(data[84:84 + 50 * n], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return rec["v"].reshape(-1, 3, 3).astype(float)


def resolve(uri):
    from ament_index_python.packages import get_package_share_directory
    if uri.startswith("package://"):
        package, rest = uri[len("package://"):].split("/", 1)
        return os.path.join(get_package_share_directory(package), rest)
    return uri[len("file://"):] if uri.startswith("file://") else uri


def origin_matrix(element):
    t = np.eye(4)
    o = element.find("origin")
    if o is not None:
        r, p, y = [float(v) for v in o.get("rpy", "0 0 0").split()]
        cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
        t[:3, :3] = (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
                     @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))
        t[:3, 3] = [float(v) for v in o.get("xyz", "0 0 0").split()]
    return t


def link_meshes(root, link, kind):
    """[(4x4 in the link, triangles (n,3,3))] of the link's <kind> meshes."""
    out = []
    for element in root.findall(f"link[@name='{link}']/{kind}"):
        mesh = element.find("geometry/mesh")
        if mesh is None:
            continue
        tris = load_stl(resolve(mesh.get("filename")))
        scale = np.array([float(v) for v in mesh.get("scale", "1 1 1").split()])
        out.append((origin_matrix(element), tris * scale))
    return out


def transform(t, tris):
    flat = tris.reshape(-1, 3) @ t[:3, :3].T + t[:3, 3]
    return flat.reshape(-1, 3, 3)


def point_triangle_distance(points, tris):
    """Distance of each point to the nearest triangle (Ericson, Real-Time Collision Detection 5.1.5), vectorised over triangles."""
    best = np.full(len(points), np.inf)
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    ab, ac = b - a, c - a
    for i, p in enumerate(points):
        ap, bp, cp = p - a, p - b, p - c
        d1, d2 = np.einsum("ij,ij->i", ab, ap), np.einsum("ij,ij->i", ac, ap)
        d3, d4 = np.einsum("ij,ij->i", ab, bp), np.einsum("ij,ij->i", ac, bp)
        d5, d6 = np.einsum("ij,ij->i", ab, cp), np.einsum("ij,ij->i", ac, cp)
        va, vb, vc = d3 * d6 - d5 * d4, d5 * d2 - d1 * d6, d1 * d4 - d3 * d2
        q = np.empty_like(a)
        region_a = (d1 <= 0) & (d2 <= 0)
        region_b = (d3 >= 0) & (d4 <= d3)
        region_c = (d6 >= 0) & (d5 <= d6)
        edge_ab = (vc <= 0) & (d1 >= 0) & (d3 <= 0)
        edge_ac = (vb <= 0) & (d2 >= 0) & (d6 <= 0)
        edge_bc = (va <= 0) & ((d4 - d3) >= 0) & ((d5 - d6) >= 0)
        with np.errstate(divide="ignore", invalid="ignore"):
            denom = 1.0 / (va + vb + vc)
            v, w = vb * denom, vc * denom
            q[:] = a + ab * v[:, None] + ac * w[:, None]
            t_ab = d1 / (d1 - d3)
            t_ac = d2 / (d2 - d6)
            t_bc = (d4 - d3) / ((d4 - d3) + (d5 - d6))
        q[edge_bc] = (b + (c - b) * t_bc[:, None])[edge_bc]
        q[edge_ac] = (a + ac * t_ac[:, None])[edge_ac]
        q[edge_ab] = (a + ab * t_ab[:, None])[edge_ab]
        q[region_c] = c[region_c]
        q[region_b] = b[region_b]
        q[region_a] = a[region_a]
        best[i] = np.sqrt(np.min(np.einsum("ij,ij->i", p - q, p - q)))
    return best


def mesh_distance(tris_a, tris_b, margin=0.25):
    """Approximate minimum distance between two triangle soups (vertex-face both ways), restricted to the parts within `margin` of each other."""
    lo_a, hi_a = tris_a.reshape(-1, 3).min(0), tris_a.reshape(-1, 3).max(0)
    lo_b, hi_b = tris_b.reshape(-1, 3).min(0), tris_b.reshape(-1, 3).max(0)
    near_b = tris_b[np.all((tris_b.max(1) >= lo_a - margin) & (tris_b.min(1) <= hi_a + margin), axis=1)]
    near_a = tris_a[np.all((tris_a.max(1) >= lo_b - margin) & (tris_a.min(1) <= hi_b + margin), axis=1)]
    if not len(near_a) or not len(near_b):
        return float("inf")
    pa = np.unique(near_a.reshape(-1, 3), axis=0)
    pb = np.unique(near_b.reshape(-1, 3), axis=0)
    return float(min(point_triangle_distance(pa, near_b).min(), point_triangle_distance(pb, near_a).min()))


def worker(base_plate, out_path):
    """One MoveItPy per process (a MoveItPy that is torn down segfaults): leaves with os._exit."""
    from moveit.core import collision_detection as cd
    from openamrobot_upperbody_moveit.acm_scene import open_scene

    with open_scene(base_plate, node_name="hand_clearance") as scene:
        scene.set_default_state()
        rest = {name: 0.0 for name in scene.state.joint_positions if name.startswith("openarm_") or name == "lift_joint"}
        scene.set_joint_positions(rest)
        root = scene.urdf_root
        geometry = {l.get("name") for l in root.findall("link") if l.find("collision") is not None}
        cover_tris = [transform(scene.state.get_global_link_transform("base_link") @ t, tris)
                      for t, tris in link_meshes(root, "base_link", "visual")]
        cover_files = [m.get("filename") for m in root.findall("link[@name='base_link']/visual/geometry/mesh")]
        cover = cover_tris[[i for i, f in enumerate(cover_files) if f.endswith(COVER_MESH)][0]]
        cover_top = float(cover.reshape(-1, 3)[:, 2].max())
        box = root.find("link[@name='base_link']/collision")
        box_top = float(scene.state.get_global_link_transform("base_link")[2, 3] + origin_matrix(box)[2, 3]
                        + float(box.find("geometry/box").get("size").split()[2]) / 2)
        rows = []
        for side in SIDES:
            hand = sorted(n for n in geometry if n.startswith(f"openarm_{side}_ee_"))
            per_link = {}
            for link in hand:
                request = cd.CollisionRequest()
                request.distance = True
                result = cd.CollisionResult()
                scene.scene.check_collision(request, result, scene.state, scene.forbidding([(link, "base_link")]))
                tris = np.concatenate([transform(scene.state.get_global_link_transform(link) @ t, m)
                                       for t, m in link_meshes(root, link, "collision")])
                per_link[link] = {"collision_m": float(result.distance), "colliding": bool(result.collision),
                                  "visual_cover_m": mesh_distance(tris, cover), "lowest_z_m": float(tris.reshape(-1, 3)[:, 2].min())}
            rows.append({"side": side, "links": per_link,
                         "min_collision_m": min(v["collision_m"] for v in per_link.values()),
                         "min_visual_cover_m": min(v["visual_cover_m"] for v in per_link.values()),
                         "lowest_z_m": min(v["lowest_z_m"] for v in per_link.values())})
        joints = {k: v for k, v in scene.state.joint_positions.items() if k in rest}
    with open(out_path, "w") as f:
        json.dump({"base_plate": base_plate, "rest_joint_positions": joints, "cover_top_visual_z_m": cover_top,
                   "cover_top_collision_box_z_m": box_top, "rows": rows}, f, indent=1)
    sys.stdout.flush()
    os._exit(0)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--json", help="write the full result here")
    parser.add_argument("--worker", nargs=2, metavar=("BASE_PLATE", "OUT"), help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.worker:
        worker(*args.worker)
        return 0
    results = []
    with tempfile.TemporaryDirectory() as tmp:
        for base_plate in BASE_PLATES:
            out = os.path.join(tmp, f"{base_plate}.json")
            done = subprocess.run([sys.executable, os.path.abspath(__file__), "--worker", base_plate, out], capture_output=True, text=True,
                                  timeout=1800)
            if not os.path.isfile(out):
                print(f"report_hand_clearance: {base_plate} produced no result (exit {done.returncode}):\n{done.stdout[-1500:]}\n"
                      f"{done.stderr[-1500:]}", file=sys.stderr)
                return 2
            with open(out) as f:
                results.append(json.load(f))
    print("Hand (gripper links) to base top cover, L1000, rest pose (every arm and finger joint 0), REPORT ONLY")
    for result in results:
        print(f"\n{result['base_plate']}: cover top {result['cover_top_visual_z_m']:.4f} m (visual mesh), "
              f"{result['cover_top_collision_box_z_m']:.4f} m (collision box top)")
        for row in result["rows"]:
            print(f"  {row['side']:<5} hand: {row['min_collision_m'] * 1000:7.1f} mm to the collision box (MoveIt), "
                  f"{row['min_visual_cover_m'] * 1000:7.1f} mm to the visual cover mesh; lowest point {row['lowest_z_m']:.4f} m")
            for link, v in sorted(row["links"].items()):
                print(f"      {link:<28} {v['collision_m'] * 1000:7.1f} mm / {v['visual_cover_m'] * 1000:7.1f} mm"
                      + ("  COLLIDING" if v["colliding"] else ""))
    if args.json:
        with open(args.json, "w") as f:
            json.dump(results, f, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
