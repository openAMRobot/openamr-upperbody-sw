#!/usr/bin/env python3
"""Offscreen renders of the combined model (the URDF's own visual geometry, forward kinematics from the expanded URDF) at the three lift stops
plus an isometric. No simulator. Usage: render_v01.py OUTDIR. Needs the workspace sourced, vtk, numpy, lxml.

What is drawn: every <visual> of every link (meshes as the URDF references them, boxes, cylinders, spheres), at the joint values of the stop
(all arm joints 0, lift_joint = the stop). The lift column has NO geometry in the model; its envelope (section 0.240 x 0.240 from the datum,
length from option B's retracted length) is drawn as a thin translucent wire box in magenta ONLY so the eye can place it: it is an annotation,
not model geometry, and the lateral side is the one invented number. Frames of interest are drawn as small axes. Heights are labelled."""
import json
import math
import os
import subprocess
import sys

import numpy as np
import vtk
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory as share

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import v01_fk as fk  # noqa: E402

OUT = sys.argv[1]
os.makedirs(OUT, exist_ok=True)
STOPS = [("L1000", 0.0), ("L1175", 0.175), ("L1350", 0.350)]


def resolve(uri):
    assert uri.startswith("package://"), uri
    pkg, rel = uri[len("package://"):].split("/", 1)
    return os.path.join(share(pkg), rel)


def read_dae(path):
    """[(vertices Nx3, triangles Mx3)] of a COLLADA file: positions, polylist/triangles, node matrices, unit."""
    ns = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
    root = ET.parse(path).getroot()
    q = lambda el, p: el.findall(p, ns)
    unit = 1.0
    u = root.find("c:asset/c:unit", ns)
    if u is not None and u.get("meter"):
        unit = float(u.get("meter"))
    geoms = {}
    for g in q(root, "c:library_geometries/c:geometry"):
        mesh = g.find("c:mesh", ns)
        if mesh is None:
            continue
        sources = {}
        for s in q(mesh, "c:source"):
            fa = s.find("c:float_array", ns)
            if fa is not None and fa.text:
                arr = np.array(fa.text.split(), dtype=float)
                stride = int(s.find("c:technique_common/c:accessor", ns).get("stride", "3"))
                sources[s.get("id")] = arr.reshape(-1, stride)
        vert_map = {}
        for v in q(mesh, "c:vertices"):
            for i in q(v, "c:input"):
                if i.get("semantic") == "POSITION":
                    vert_map[v.get("id")] = i.get("source")[1:]
        parts = []
        for prim in q(mesh, "c:triangles") + q(mesh, "c:polylist") + q(mesh, "c:polygons"):
            inputs = q(prim, "c:input")
            vi = [i for i in inputs if i.get("semantic") == "VERTEX"]
            if not vi:
                continue
            off = int(vi[0].get("offset", "0"))
            stride = max(int(i.get("offset", "0")) for i in inputs) + 1
            pos = sources[vert_map[vi[0].get("source")[1:]]]
            tris = []
            if prim.tag.endswith("triangles"):
                p = np.array(prim.find("c:p", ns).text.split(), dtype=int).reshape(-1, stride)[:, off]
                tris = p.reshape(-1, 3)
            elif prim.tag.endswith("polylist"):
                vc = np.array(prim.find("c:vcount", ns).text.split(), dtype=int)
                p = np.array(prim.find("c:p", ns).text.split(), dtype=int).reshape(-1, stride)[:, off]
                k, out = 0, []
                for n in vc:
                    for j in range(1, n - 1):
                        out.append((p[k], p[k + j], p[k + j + 1]))
                    k += n
                tris = np.array(out)
            else:
                out = []
                for pp in q(prim, "c:p"):
                    p = np.array(pp.text.split(), dtype=int).reshape(-1, stride)[:, off]
                    for j in range(1, len(p) - 1):
                        out.append((p[0], p[j], p[j + 1]))
                tris = np.array(out)
            if len(tris):
                parts.append((pos, tris))
        geoms[g.get("id")] = parts
    # node transforms
    result = []

    def node_matrix(node):
        M = np.eye(4)
        for ch in node:
            tag = ch.tag.split("}")[-1]
            if tag == "matrix":
                M = M @ np.array(ch.text.split(), dtype=float).reshape(4, 4)
            elif tag == "translate":
                T = np.eye(4); T[:3, 3] = [float(x) for x in ch.text.split()]; M = M @ T
            elif tag == "scale":
                S = np.diag([*[float(x) for x in ch.text.split()], 1.0]); M = M @ S
            elif tag == "rotate":
                x, y, z, a = [float(v) for v in ch.text.split()]
                a = math.radians(a); n = np.array([x, y, z]); n = n / np.linalg.norm(n)
                K = np.array([[0, -n[2], n[1]], [n[2], 0, -n[0]], [-n[1], n[0], 0]])
                R = np.eye(4); R[:3, :3] = np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K; M = M @ R
        return M

    def walk(node, M):
        M = M @ node_matrix(node)
        for ig in q(node, "c:instance_geometry"):
            gid = ig.get("url")[1:]
            for pos, tris in geoms.get(gid, []):
                result.append((pos @ M[:3, :3].T + M[:3, 3], tris))
        for ch in q(node, "c:node"):
            walk(ch, M)

    scenes = root.findall("c:library_visual_scenes/c:visual_scene", ns)
    if scenes:
        for n in q(scenes[0], "c:node"):
            walk(n, np.eye(4))
    else:
        for parts in geoms.values():
            result += parts
    return [(v * unit, t) for v, t in result]


def polydata(verts, tris):
    pts = vtk.vtkPoints()
    for v in verts:
        pts.InsertNextPoint(*v)
    cells = vtk.vtkCellArray()
    for t in tris:
        cells.InsertNextCell(3, [int(t[0]), int(t[1]), int(t[2])])
    pd = vtk.vtkPolyData(); pd.SetPoints(pts); pd.SetPolys(cells)
    return pd


def actor_from(pd, T, color, opacity=1.0):
    m = vtk.vtkMatrix4x4()
    for i in range(4):
        for j in range(4):
            m.SetElement(i, j, T[i, j])
    tf = vtk.vtkTransform(); tf.SetMatrix(m)
    nf = vtk.vtkPolyDataNormals(); nf.SetInputData(pd); nf.SplittingOff()
    mp = vtk.vtkPolyDataMapper(); mp.SetInputConnection(nf.GetOutputPort())
    a = vtk.vtkActor(); a.SetMapper(mp); a.SetUserTransform(tf)
    a.GetProperty().SetColor(*color); a.GetProperty().SetOpacity(opacity)
    return a


def box_actor(size, T, color, opacity=1.0, wire=False):
    s = vtk.vtkCubeSource(); s.SetXLength(size[0]); s.SetYLength(size[1]); s.SetZLength(size[2])
    pd = s.GetOutput(); s.Update()
    a = actor_from(s.GetOutput(), T, color, opacity)
    if wire:
        a.GetProperty().SetRepresentationToWireframe(); a.GetProperty().SetLineWidth(2.0)
    return a


def axes_at(T, length=0.06):
    a = vtk.vtkAxesActor(); a.SetTotalLength(length, length, length); a.AxisLabelsOff()
    m = vtk.vtkMatrix4x4()
    for i in range(4):
        for j in range(4):
            m.SetElement(i, j, T[i, j])
    tf = vtk.vtkTransform(); tf.SetMatrix(m); a.SetUserTransform(tf)
    return a


def text(msg, x, y, size=18, color=(0, 0, 0)):
    t = vtk.vtkTextActor(); t.SetInput(msg); t.SetPosition(x, y)
    p = t.GetTextProperty(); p.SetFontSize(size); p.SetColor(*color)
    return t


def color_of(link):
    if link.startswith("openarm_left_") or link.startswith("left_wrist"):
        return (0.25, 0.45, 0.85)
    if link.startswith("openarm_right_") or link.startswith("right_wrist"):
        return (0.2, 0.7, 0.35)
    if link in ("arm_profile_link",):
        return (0.95, 0.55, 0.1)
    if "camera" in link:
        return (0.9, 0.15, 0.15)
    return (0.62, 0.62, 0.66)


def floats(s, d):
    return [float(x) for x in s.split()] if s else list(d)


def build_scene(model, lift, mesh_cache, with_frames=True):
    actors = []
    q = {"lift_joint": lift}
    for lname, link in model.links.items():
        Tl = model.T(lname, q)
        for vis in link.findall("visual"):
            o = vis.find("origin")
            To = fk.T_from(floats(o.get("xyz") if o is not None else None, (0, 0, 0)), floats(o.get("rpy") if o is not None else None, (0, 0, 0)))
            T = Tl @ To
            g = vis.find("geometry")
            col = color_of(lname)
            if g.find("box") is not None:
                actors.append(box_actor(floats(g.find("box").get("size"), (1, 1, 1)), T, col, 0.9))
            elif g.find("cylinder") is not None:
                c = g.find("cylinder"); s = vtk.vtkCylinderSource(); s.SetRadius(float(c.get("radius"))); s.SetHeight(float(c.get("length"))); s.SetResolution(32); s.Update()
                R = np.eye(4); R[:3, :3] = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])      # vtk cylinder is along y; URDF's along z
                actors.append(actor_from(s.GetOutput(), T @ R, col))
            elif g.find("sphere") is not None:
                s = vtk.vtkSphereSource(); s.SetRadius(float(g.find("sphere").get("radius"))); s.Update()
                actors.append(actor_from(s.GetOutput(), T, col))
            elif g.find("mesh") is not None:
                m = g.find("mesh"); path = resolve(m.get("filename")); sc = np.array(floats(m.get("scale"), (1, 1, 1)))
                if path not in mesh_cache:
                    if path.lower().endswith(".stl"):
                        r = vtk.vtkSTLReader(); r.SetFileName(path); r.Update()
                        pdat = r.GetOutput()
                        pts = np.array([pdat.GetPoint(i) for i in range(pdat.GetNumberOfPoints())])
                        cells = pdat.GetPolys(); ids = vtk.vtkIdList(); tris = []
                        cells.InitTraversal()
                        while cells.GetNextCell(ids):
                            tris.append((ids.GetId(0), ids.GetId(1), ids.GetId(2)))
                        mesh_cache[path] = [(pts, np.array(tris))]
                    else:
                        mesh_cache[path] = read_dae(path)
                for verts, tris in mesh_cache[path]:
                    actors.append(actor_from(polydata(verts * sc, tris), T, col))
    return actors


def render(model, lift, mesh_cache, fname, label, cam, ortho=True, size=(1100, 1100), notes=()):
    ren = vtk.vtkRenderer(); ren.SetBackground(1, 1, 1)
    for a in build_scene(model, lift, mesh_cache):
        ren.AddActor(a)
    q = {"lift_joint": lift}
    # annotation: column envelope (no geometry in the model), floor line, frame axes
    sect = 0.24
    col_len = 0.0
    try:
        up = fk.yaml_load if False else None
    except Exception:
        pass
    cb = model.T("lift_column_base_link", q)
    box_len = model_state["column_length"]
    Tbox = cb @ fk.T_from([0, 0, box_len / 2], [0, 0, 0])
    ren.AddActor(box_actor([sect, sect, box_len], Tbox, (0.85, 0.1, 0.85), 1.0, wire=True))
    floor = vtk.vtkPlaneSource(); floor.SetOrigin(-0.8, -0.8, 0); floor.SetPoint1(0.8, -0.8, 0); floor.SetPoint2(-0.8, 0.8, 0); floor.Update()
    fa = vtk.vtkActor(); mp = vtk.vtkPolyDataMapper(); mp.SetInputConnection(floor.GetOutputPort()); fa.SetMapper(mp)
    fa.GetProperty().SetColor(0.9, 0.9, 0.9); fa.GetProperty().SetRepresentationToWireframe(); ren.AddActor(fa)
    for link in ("lift_column_base_link", "lift_carriage_link", "head_camera_mount_link", "openarm_left_base_link", "openarm_right_base_link"):
        ren.AddActor(axes_at(model.T(link, q), 0.07))
    win = vtk.vtkRenderWindow(); win.SetOffScreenRendering(1); win.AddRenderer(ren); win.SetSize(*size)
    camera = ren.GetActiveCamera()
    focal = cam["focal"]; camera.SetFocalPoint(*focal); camera.SetPosition(*cam["position"]); camera.SetViewUp(0, 0, 1)
    if ortho:
        camera.ParallelProjectionOn(); camera.SetParallelScale(cam["scale"])
    ren.AddActor2D(text(label, 12, size[1] - 34, 22))
    y = size[1] - 64
    for n in notes:
        ren.AddActor2D(text(n, 12, y, 16)); y -= 22
    win.Render()
    w2i = vtk.vtkWindowToImageFilter(); w2i.SetInput(win); w2i.Update()
    wr = vtk.vtkPNGWriter(); wr.SetFileName(os.path.join(OUT, fname)); wr.SetInputConnection(w2i.GetOutputPort()); wr.Write()


model_state = {}
if __name__ == "__main__":
    combined = share("openamrobot_upperbody_description") + "/urdf/combined_robot.xacro"
    model = fk.Model(fk.expand(file=combined, hardware_profile="mock"))
    import yaml
    cfg = yaml.safe_load(open(share("openamrobot_upperbody_description") + "/config/upperbody_configuration.yaml"))
    model_state["column_length"] = cfg["column"]["dold_v4_350"]["retracted_length"]
    cache = {}
    notes_common = ["magenta wire box: column ENVELOPE at its retracted length 0.530 m. Annotation, NOT model geometry:",
                    "  the model has no column geometry (0.240 x 0.240 section, lateral side invented).",
                    "orange: arm_profile_link 0.120 x 0.060 x 0.340. blue: left arm. green: right arm. red: platform-sw base sensors.",
                    "small axes: lift_column_base_link, lift_carriage_link, head_camera_mount_link (no geometry), openarm_*_base_link."]
    for stop, v in STOPS:
        head_z = model.T("head_camera_mount_link", {"lift_joint": v})[2, 3]
        arm_z = model.T("openarm_left_base_link", {"lift_joint": v})[2, 3]
        render(model, v, cache, f"lift_{stop}_side.png", f"Lift stop {stop}: lift_joint {v:.3f} m, head mount z {head_z:.3f} m, arm base z {arm_z:.3f} m",
               {"focal": (0.15, 0, 0.9), "position": (0.15, 4.0, 0.9), "scale": 1.05}, True, (1100, 1100),
               notes_common + ["side view: x forward to the right, floor line at z = 0."])
    v = 0.175
    render(model, v, cache, "isometric_L1175.png", "OpenAMRobot 2.0, isometric, lift stop L1175 (0.175 m)",
           {"focal": (0.1, 0, 0.75), "position": (2.4, -2.2, 1.9)}, False, (1100, 1100), notes_common)
    print("rendered to", OUT)
