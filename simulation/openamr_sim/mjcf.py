# SPDX-License-Identifier: MIT
"""Write MJCF from the pinned expansion, the joint dynamics profile and the engine-override file.

Mapping (one URDF link is one MuJoCo body with the same name; one URDF joint is one MuJoCo joint with the same name):
  fixed joint            a child body without a joint (welded), never fused, so every link keeps its own body and frame
  revolute / prismatic   hinge / slide with the URDF range, damping, frictionloss; armature from openarm_v2.yaml
  continuous             unlimited hinge
  mimic follower         its own joint plus an equality constraint (SW-09)
  passive castors        the two fixed castor joints become passive hinges (engine_overrides.yaml passive_castors)
  inertial               mass, CoM and the full inertia tensor, rotated into the body frame; never computed from geometry
  visual / collision     geoms in groups 2 and 3; collision geoms collide with the floor and with each other, minus the SRDF's
                         disabled pairs (MoveIt's allowed collision matrix); Gazebo and Isaac keep self collision off
  command interfaces     one position actuator per position interface (arms, grippers, lift), one velocity actuator per wheel

The root link gets a free joint (base_free_joint). Lift positions are joint state: keyframes, never a model change.
Everything the URDF holds that MuJoCo does not represent is listed in the `unsupported` list of the report.
"""
import hashlib
import json
import math
import pathlib
import shutil
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from .urdf_model import URDF

REPO = pathlib.Path(__file__).resolve().parents[2]
PROFILE = REPO / "ros2/src/openamrobot_upperbody_description/config/sim_dynamics/openarm_v2.yaml"
OVERRIDES = REPO / "ros2/src/openamrobot_upperbody_description/config/sim_dynamics/engine_overrides.yaml"
SRDF = REPO / "ros2/src/openamrobot_upperbody_moveit/config/openamrobot.srdf"
CONFIGURATION = REPO / "ros2/src/openamrobot_upperbody_description/config/upperbody_configuration.yaml"

FREE_JOINT = "base_free_joint"
CONTYPE_ROBOT, CONAFFINITY_ROBOT = 1, 2
CONTYPE_FLOOR, CONAFFINITY_FLOOR = 2, 1
ARM_POSES = ("home", "hands_up", "park")
LIFT_STOPS = ("L1000", "L1175", "L1350")


def fmt(v):
    """Shortest text that reads back as the same float64."""
    if isinstance(v, (list, tuple, np.ndarray)):
        return " ".join(fmt(x) for x in v)
    v = float(v)
    if v == 0.0:
        return "0"
    return repr(v)


def mat_to_quat(R):
    """Rotation matrix to unit quaternion (w, x, y, z), w >= 0."""
    R = np.asarray(R, float)
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        q = [(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s]
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        q = [(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s]
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        q = [(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s]
    q = np.array(q)
    q /= np.linalg.norm(q)
    return q if q[0] >= 0 else -q


def load_yaml(path):
    return yaml.safe_load(pathlib.Path(path).read_text())


def srdf_states(path=SRDF):
    """{(state, group): {joint: value}} from the SRDF."""
    root = ET.parse(path).getroot()
    out = {}
    for s in root.findall("group_state"):
        out[(s.get("name"), s.get("group"))] = {j.get("name"): float(j.get("value")) for j in s.findall("joint")}
    return out


def lift_stop_positions(configuration=CONFIGURATION, srdf=SRDF):
    """lift_joint value per calibration stop, from the SRDF, cross-checked with the configuration file's shoulder heights."""
    cfg = load_yaml(configuration)
    geo = cfg["geometry"]
    col = cfg["column"][cfg["baseline"]["column"]]
    shoulder_at_zero = geo["base_plate_top_z"] + col["retracted_length"] + geo["arm_mount"]["z_above_column_top"]
    states = srdf_states(srdf)
    out = {}
    for stop, height in cfg["lift_calibration_stops"].items():
        q = states[(stop, "lift")]["lift_joint"]
        if abs((height - shoulder_at_zero) - q) > 1e-9:
            raise ValueError(f"{stop}: SRDF lift_joint {q} and configuration shoulder height {height} disagree")
        out[stop] = q
    return out


def arm_pose(states, pose):
    q = {}
    for group in ("left_arm", "right_arm"):
        q.update(states[(pose, group)])
    return q


def subtree_links(urdf, link):
    out, stack = [], [link]
    while stack:
        n = stack.pop()
        out.append(n)
        stack.extend(j.child for j in urdf.children[n])
    return out


def subtree_mass(urdf, link):
    return sum(urdf.links[n].inertial.mass for n in subtree_links(urdf, link) if urdf.links[n].inertial is not None)


class Generator:
    def __init__(self, expansion, out_dir, profile=PROFILE, overrides=OVERRIDES):
        self.expansion = expansion
        self.urdf = URDF(expansion.text)
        self.out_dir = pathlib.Path(out_dir)
        self.profile = load_yaml(profile)
        self.ov = load_yaml(overrides)
        self.unsupported = []
        self.mesh_files = {}
        self.castor_joints = {}
        for p in self.ov["passive_castors"]["prefixes"]:
            if "mujoco" in self.ov["passive_castors"]["enabled_in"]:
                self.castor_joints[self.ov["passive_castors"]["swivel"]["joint"].format(prefix=p)] = "swivel"
                self.castor_joints[self.ov["passive_castors"]["roll"]["joint"].format(prefix=p)] = "roll"

    # ------------------------------------------------------------------ meshes
    def _resolve(self, filename):
        if filename.startswith("package://"):
            pkg, rel = filename[len("package://"):].split("/", 1)
            return pathlib.Path(self.expansion.package_dirs[pkg]) / rel
        if filename.startswith("file://"):
            return pathlib.Path(filename[len("file://"):])
        if filename.startswith("$PREFIX/share/"):
            pkg, rel = filename[len("$PREFIX/share/"):].split("/", 1)
            return pathlib.Path(self.expansion.package_dirs[pkg]) / rel
        return pathlib.Path(filename)

    def mesh_asset(self, geom):
        """Name of the MuJoCo mesh asset for a URDF mesh geometry, writing the file into out_dir/meshes."""
        import trimesh

        filename, scale = geom.params["filename"], tuple(geom.params["scale"])
        key = (filename, scale)
        if key in self.mesh_files:
            return self.mesh_files[key]
        src = self._resolve(filename)
        if not src.is_file():
            raise FileNotFoundError(f"mesh {filename} resolves to {src}, which does not exist")
        digest = hashlib.sha1(f"{filename}|{scale}".encode()).hexdigest()[:10]
        stem = pathlib.PurePosixPath(filename.split("://", 1)[-1]).stem
        name = f"{stem}_{digest}"
        meshes = self.out_dir / "meshes"
        meshes.mkdir(parents=True, exist_ok=True)
        ext = src.suffix.lower()
        mirrored = float(np.prod(scale)) < 0
        if ext == ".stl" and not mirrored:
            dst = meshes / f"{name}.stl"
            shutil.copyfile(src, dst)
            entry = {"name": name, "file": f"meshes/{dst.name}", "scale": scale, "converted": None}
        else:
            # DAE (not read by MuJoCo) and mirrored meshes (MuJoCo issue 3604: a negative scale turns the hull inside out) are
            # written again with the scale applied and, for a mirror, the face winding reversed. Same vertices, same frame.
            mesh = trimesh.load(src, force="mesh", process=False)
            mesh.apply_scale(scale)
            if mirrored:
                mesh.invert()
            dst = meshes / f"{name}.obj"
            mesh.export(dst)
            reason = "dae_to_obj" if ext == ".dae" else "mirror_applied"
            if ext == ".dae" and mirrored:
                reason = "dae_to_obj+mirror_applied"
            entry = {"name": name, "file": f"meshes/{dst.name}", "scale": (1.0, 1.0, 1.0), "converted": reason}
        entry["source"] = filename
        self.mesh_files[key] = entry
        return entry

    # ------------------------------------------------------------------ elements
    def _contact_friction(self, link):
        c = self.ov["contact"]
        if link in c["drive_wheel_sliding_friction"]["applies_to"]:
            return c["drive_wheel_sliding_friction"]["value"]
        if link in c["castor_wheel_sliding_friction"]["applies_to"]:
            return c["castor_wheel_sliding_friction"]["value"]
        return c["other_robot_geoms_sliding_friction"]["value"]

    def _geom(self, body, g, link):
        e = ET.SubElement(body, "geom", name=f"{link}__{g.use}_{g.index}")
        pos, quat = g.origin[:3, 3], mat_to_quat(g.origin[:3, :3])
        if g.kind == "box":
            e.set("type", "box")
            e.set("size", fmt([s / 2 for s in g.params["size"]]))
        elif g.kind == "cylinder":
            e.set("type", "cylinder")
            e.set("size", fmt([g.params["radius"], g.params["length"] / 2]))
        elif g.kind == "sphere":
            e.set("type", "sphere")
            e.set("size", fmt([g.params["radius"]]))
        elif g.kind == "mesh":
            asset = self.mesh_asset(g)
            e.set("type", "mesh")
            e.set("mesh", asset["name"])
        e.set("pos", fmt(pos))
        e.set("quat", fmt(quat))
        if g.use == "visual":
            e.set("group", "2")
            e.set("contype", "0")
            e.set("conaffinity", "0")
            e.set("mass", "0")
            if g.material in self.materials:
                e.set("material", g.material)
        else:
            e.set("group", "3")
            e.set("contype", str(CONTYPE_ROBOT))
            self_collision = self.ov["contact"]["mujoco_self_collision"]["value"]
            e.set("conaffinity", str(CONAFFINITY_ROBOT | (CONTYPE_ROBOT if self_collision else 0)))
            e.set("mass", "0")
            tr = self.ov["contact"]["torsional_and_rolling_friction"]["value"]
            e.set("friction", fmt([self._contact_friction(link), tr[0], tr[1]]))
            e.set("rgba", "0.8 0.2 0.2 0.4")

    def _joint(self, body, j):
        if j.name in self.castor_joints:
            kind = self.castor_joints[j.name]
            spec = self.ov["passive_castors"][kind]
            e = ET.SubElement(body, "joint", name=j.name, type="hinge", axis=fmt(spec["axis"]), limited="false",
                              damping=fmt(spec["damping"]["value"]), frictionloss=fmt(spec["frictionloss"]["value"]), armature="0")
            return e
        if j.type == "fixed":
            return None
        mtype = {"revolute": "hinge", "continuous": "hinge", "prismatic": "slide"}.get(j.type)
        if mtype is None:
            raise ValueError(f"{j.name}: URDF joint type {j.type} is not mapped")
        e = ET.SubElement(body, "joint", name=j.name, type=mtype, axis=fmt(j.axis / np.linalg.norm(j.axis)))
        if j.type == "continuous":
            e.set("limited", "false")
        else:
            e.set("limited", "true")
            e.set("range", fmt([j.lower, j.upper]))
        e.set("damping", fmt(j.damping or 0.0))
        e.set("frictionloss", fmt(j.friction or 0.0))
        prof = self.profile["joints"].get(j.name)
        e.set("armature", fmt(prof["armature"]["value"] if prof else 0.0))
        if j.velocity is not None:
            self.unsupported.append({"item": "joint velocity limit", "where": j.name, "value": j.velocity,
                                     "handling": "MuJoCo has no joint velocity limit; not applied"})
        return e

    def _body(self, parent, link, pose):
        body = ET.SubElement(parent, "body", name=link)
        body.set("pos", fmt(pose[:3, 3]))
        body.set("quat", fmt(mat_to_quat(pose[:3, :3])))
        l = self.urdf.links[link]
        return body, l

    def _fill(self, body, link):
        l = self.urdf.links[link]
        if l.inertial is not None:
            R = l.inertial.origin[:3, :3]
            I = R @ l.inertial.inertia @ R.T
            ET.SubElement(body, "inertial", pos=fmt(l.inertial.origin[:3, 3]), mass=fmt(l.inertial.mass),
                          fullinertia=fmt([I[0, 0], I[1, 1], I[2, 2], I[0, 1], I[0, 2], I[1, 2]]))
        for g in l.visuals + l.collisions:
            self._geom(body, g, link)
        for cam in self.cameras.get(link, []):
            # ROS optical frame: +z forward, +y down. MuJoCo camera: -z forward, +y up. A half turn about x maps one to the other.
            ET.SubElement(body, "camera", name=cam["name"], pos="0 0 0", quat="0 1 0 0", fovy=fmt(cam["fovy_deg"]))
        for j in self.urdf.children[link]:
            child = ET.SubElement(body, "body", name=j.child)
            child.set("pos", fmt(j.origin[:3, 3]))
            child.set("quat", fmt(mat_to_quat(j.origin[:3, :3])))
            self._joint(child, j)
            self._fill(child, j.child)

    def _cameras(self):
        out = {}
        for name, spec in self.ov.get("cameras", {}).items():
            fovy = spec["fovy"]
            if fovy.get("status") == "unknown":
                value = 45.0
                self.unsupported.append({"item": "camera field of view", "where": name, "value": "unknown",
                                         "handling": f"MuJoCo default fovy 45 deg; needed: {fovy['needed']}"})
            elif "from_gazebo_sensor" in fovy:
                value = self._gazebo_fovy(fovy["from_gazebo_sensor"])
            else:
                value = fovy["value"]
            out.setdefault(spec["frame"], []).append({"name": name, "fovy_deg": value})
        return out

    def _gazebo_fovy(self, sensor_name):
        for g in self.urdf.gazebo:
            for s in g.iter("sensor"):
                if s.get("name") == sensor_name:
                    hfov = float(s.find(".//horizontal_fov").text)
                    w, h = float(s.find(".//image/width").text), float(s.find(".//image/height").text)
                    return math.degrees(2 * math.atan(math.tan(hfov / 2) * h / w))
        raise KeyError(f"gazebo sensor {sensor_name} is not in the expansion")

    def _actuators(self, root):
        act = ET.SubElement(root, "actuator")
        ov = self.ov["actuators"]
        names = []
        for side in ("left", "right"):
            for k in range(1, 8):
                jn = f"openarm_{side}_joint{k}"
                g = ov["arm_position_gains"]["per_joint"][f"joint{k}"]
                names.append(self._position(act, jn, g["kp"], g["kd"]))
            jn = f"openarm_{side}_finger_joint1"
            names.append(self._position(act, jn, ov["gripper_position_gains"]["kp"], ov["gripper_position_gains"]["kd"]))
        # Lift: stiff servo derived from the carried mass (engine_choice with its method).
        lift = self.urdf.joints["lift_joint"]
        m = subtree_mass(self.urdf, lift.child)
        par = ov["lift_position_servo"]["parameters"]
        kp = m * 9.81 / par["sag"]["value"]
        kv = 2 * par["zeta"]["value"] * math.sqrt(kp * m)
        self.derived["lift_servo"] = {"carried_mass_kg": m, "kp": kp, "kv": kv}
        names.append(self._position(act, "lift_joint", kp, kv))
        self.unsupported.append({"item": "lift velocity limit", "where": "lift_joint", "value": lift.velocity,
                                 "handling": "not enforced by the actuator; the commanded trajectory respects it"})
        # Wheels: velocity servos.
        M = self.urdf.total_mass()
        for jn in ov["wheel_velocity_servo"]["applies_to"]:
            r = self.urdf.links[self.urdf.joints[jn].child].collisions[0].params["radius"]
            kv = (M / 2) * r * r / ov["wheel_velocity_servo"]["parameters"]["tau"]["value"]
            self.derived.setdefault("wheel_servo", {})[jn] = {"radius_m": r, "total_mass_kg": M, "kv": kv}
            ET.SubElement(act, "velocity", name=f"{jn}_velocity", joint=jn, kv=fmt(kv))
            names.append(f"{jn}_velocity")
            self.unsupported.append({"item": "wheel torque limit", "where": jn, "value": "unknown",
                                     "handling": "velocity servo without a force limit; needed: hub motor peak torque"})
        return names

    def _position(self, act, jn, kp, kv):
        j = self.urdf.joints[jn]
        name = f"{jn}_position"
        ET.SubElement(act, "position", name=name, joint=jn, kp=fmt(kp), kv=fmt(kv), ctrllimited="true",
                      ctrlrange=fmt([j.lower, j.upper]), forcelimited="true", forcerange=fmt([-j.effort, j.effort]))
        return name

    def _equality(self, root):
        eq = ET.SubElement(root, "equality")
        mov = self.ov["mimic"]["mujoco"]
        for jn in self.ov["mimic"]["joints"]:
            j = self.urdf.joints[jn]
            if j.mimic is None:
                raise ValueError(f"{jn} is listed as a mimic follower but has no <mimic> in the expansion")
            ET.SubElement(eq, "joint", name=f"{jn}_mimic", joint1=jn, joint2=j.mimic["joint"],
                          polycoef=fmt([j.mimic["offset"], j.mimic["multiplier"], 0, 0, 0]),
                          solref=fmt(mov["solref"]["value"]), solimp=fmt(mov["solimp"]["value"]))
        followers = {j.name for j in self.urdf.joints.values() if j.mimic is not None}
        if followers != set(self.ov["mimic"]["joints"]):
            raise ValueError(f"mimic followers in the expansion {sorted(followers)} differ from engine_overrides {self.ov['mimic']['joints']}")

    def _srdf_excludes(self, root):
        """One <exclude> per SRDF disable_collisions pair whose two links are bodies here (MoveIt's allowed collision matrix)."""
        contact = ET.SubElement(root, "contact")
        pairs = []
        for p in ET.parse(SRDF).getroot().findall("disable_collisions"):
            a, b = p.get("link1"), p.get("link2")
            if a in self.urdf.links and b in self.urdf.links:
                ET.SubElement(contact, "exclude", name=f"srdf__{a}__{b}", body1=a, body2=b)
                pairs.append((a, b))
        self.derived["srdf_excluded_pairs"] = len(pairs)

    def _record_unsupported(self):
        u = self.unsupported
        for g in self.urdf.gazebo:
            for p in g.iter("plugin"):
                u.append({"item": "gazebo plugin", "where": p.get("filename"), "value": p.get("name"),
                          "handling": "not represented; the controllers' interfaces are MuJoCo actuators"})
            for s in g.iter("sensor"):
                handled = any(spec.get("fovy", {}).get("from_gazebo_sensor") == s.get("name") for spec in self.ov.get("cameras", {}).values())
                u.append({"item": f"gazebo sensor ({s.get('type')})", "where": f"{g.get('reference')}/{s.get('name')}", "value": None,
                          "handling": "camera with this field of view" if handled else "not represented"})
            ref = g.get("reference")
            if ref is not None:
                tags = sorted({c.tag for c in g if c.tag != "sensor"})
                if tags:
                    u.append({"item": "gazebo link tags", "where": ref, "value": ",".join(tags),
                              "handling": "not read; contact friction comes from engine_overrides.yaml"})
        for rc in self.urdf.ros2_control:
            u.append({"item": "ros2_control system", "where": rc.get("name"), "value": rc.find("hardware/plugin").text,
                      "handling": "each command interface is one MuJoCo actuator; state interfaces are mjData qpos and qvel"})
        mesh_collisions = sum(1 for l in self.urdf.links.values() for c in l.collisions if c.kind == "mesh")
        u.append({"item": "collision mesh", "where": f"{mesh_collisions} collision meshes", "value": None,
                  "handling": "MuJoCo collides with the convex hull of each mesh"})
        u.append({"item": "drive wheel anisotropic friction", "where": "left_wheel_link, right_wheel_link",
                  "value": "mu2 0.5 along fdir1 (platform-sw #135)", "handling": "one sliding coefficient per geom (mu1 1.5)"})
        u.append({"item": "mimic joint", "where": ", ".join(self.ov["mimic"]["joints"]), "value": "URDF <mimic>",
                  "handling": "soft equality constraint (solref, solimp from openarm_mujoco); a small tracking error remains"})
        dae = sum(1 for m in self.mesh_files.values() if m["converted"] and "dae" in m["converted"])
        u.append({"item": "DAE visual materials", "where": f"{dae} DAE visual meshes", "value": None,
                  "handling": "geometry converted to OBJ; per-face colours of the DAE are not carried"})
        u.append({"item": "castor joints", "where": ", ".join(sorted(self.castor_joints)), "value": "fixed in URDF",
                  "handling": "passive hinges in MuJoCo (engine_overrides.yaml passive_castors); declared topology difference"})

    # ------------------------------------------------------------------ top level
    def build(self, name):
        self.derived = {}
        self.materials = {}
        for m in self.urdf.root.findall("material"):
            c = m.find("color")
            if c is not None:
                self.materials[m.get("name")] = c.get("rgba")
        self.cameras = self._cameras()
        com = self.ov["common"]
        root = ET.Element("mujoco", model=f"openamrobot_{name}")
        ET.SubElement(root, "compiler", angle="radian", autolimits="false", inertiafromgeom="false", balanceinertia="false",
                      boundmass="0", boundinertia="0", meshdir=".", fusestatic="false", discardvisual="false")
        c = self.ov["contact"]
        ET.SubElement(root, "option", timestep=fmt(com["physics_step"]["value"]), gravity=fmt(com["gravity"]["value"]),
                      integrator=c["integrator"]["value"], cone=c["cone"]["value"], impratio=fmt(c["impratio"]["value"]))
        visual = ET.SubElement(root, "visual")
        ET.SubElement(visual, "global", offwidth="1280", offheight="960")
        asset = ET.SubElement(root, "asset")
        ET.SubElement(asset, "texture", name="grid", type="2d", builtin="checker", rgb1="0.6 0.6 0.6", rgb2="0.4 0.4 0.4",
                      width="512", height="512")
        ET.SubElement(asset, "material", name="floor_material", texture="grid", texrepeat="8 8")
        for mname, rgba in self.materials.items():
            ET.SubElement(asset, "material", name=mname, rgba=rgba)
        world = ET.SubElement(root, "worldbody")
        ET.SubElement(world, "light", name="sun", pos="0 0 4", dir="0 0 -1", directional="true")
        tr = c["torsional_and_rolling_friction"]["value"]
        ET.SubElement(world, "geom", name="floor", type="plane", size="0 0 0.05", material="floor_material",
                      contype=str(CONTYPE_FLOOR), conaffinity=str(CONAFFINITY_FLOOR),
                      friction=fmt([c["floor_sliding_friction"]["value"], tr[0], tr[1]]))
        base = ET.SubElement(world, "body", name=self.urdf.root_link, pos="0 0 0", quat="1 0 0 0")
        ET.SubElement(base, "freejoint", name=FREE_JOINT)
        self._fill(base, self.urdf.root_link)
        if self.ov["contact"]["mujoco_self_collision"]["value"]:
            self._srdf_excludes(root)
        actuators = self._actuators(root)
        self._equality(root)
        # Mesh assets are known only after the bodies are written; insert them into <asset>.
        for m in sorted(self.mesh_files.values(), key=lambda m: m["name"]):
            ET.SubElement(asset, "mesh", name=m["name"], file=m["file"], scale=fmt(m["scale"]))
        self._record_unsupported()
        return root, actuators

    def keyframes(self, root, model_path):
        """Add one keyframe per lift stop and SRDF arm pose. Needs a compiled model for the qpos layout."""
        import mujoco

        model = mujoco.MjModel.from_xml_path(str(model_path))
        stops = lift_stop_positions()
        states = srdf_states()
        wheel = self.urdf.joints["left_wheel_joint"]
        r = self.urdf.links[wheel.child].collisions[0].params["radius"]
        axle_z = self.urdf.fk({})[wheel.child][2, 3]
        z0 = max(0.0, r - axle_z)
        self.derived["spawn_height_m"] = z0
        keys = ET.SubElement(root, "keyframe")
        out = []
        for stop in LIFT_STOPS:
            for pose in ARM_POSES:
                q = {"lift_joint": stops[stop], **arm_pose(states, pose)}
                q = self.urdf.resolve(q)
                qpos = model.qpos0.copy()
                adr = model.jnt_qposadr[model.joint(FREE_JOINT).id]
                qpos[adr:adr + 7] = [0, 0, z0, 1, 0, 0, 0]
                ctrl = np.zeros(model.nu)
                for jn, v in q.items():
                    if jn in self.urdf.joints and self.urdf.joints[jn].type != "fixed" and jn not in self.castor_joints:
                        qpos[model.jnt_qposadr[model.joint(jn).id]] = v
                for a in range(model.nu):
                    jn = model.joint(model.actuator_trnid[a, 0]).name
                    if model.actuator(a).name.endswith("_position"):
                        ctrl[a] = q.get(jn, 0.0)
                kname = f"{stop}_{pose}"
                ET.SubElement(keys, "key", name=kname, qpos=fmt(qpos), ctrl=fmt(ctrl))
                out.append(kname)
        return out


def generate(expansion, out_dir):
    """Write out_dir/openamrobot_<name>.xml, its meshes and report.json. Returns the report."""
    import mujoco

    out_dir = pathlib.Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    gen = Generator(expansion, out_dir)
    root, actuators = gen.build(expansion.name)
    path = out_dir / f"openamrobot_{expansion.name}.xml"
    header = (f" Generated by simulation/tools/generate.py from the combined model, configuration {expansion.name}, "
              f"expansion SHA-256 {expansion.sha256} (lock: {expansion.lock_status}). Do not edit; do not commit. ")
    def write():
        ET.indent(root, space="  ")
        path.write_text("<!--" + header + "-->\n" + ET.tostring(root, encoding="unicode") + "\n")
    write()
    keys = gen.keyframes(root, path)
    write()
    model = mujoco.MjModel.from_xml_path(str(path))
    report = {
        "configuration": expansion.name, "args": expansion.args, "expansion_sha256": expansion.sha256,
        "lock_status": expansion.lock_status, "inputs": expansion.inputs, "mjcf": path.name, "mujoco_version": mujoco.__version__,
        "counts": {"bodies": model.nbody - 1, "joints": model.njnt, "dofs": model.nv, "actuators": model.nu, "equalities": model.neq,
                   "geoms": model.ngeom - 1, "meshes": model.nmesh, "cameras": model.ncam, "keyframes": model.nkey,
                   "urdf_links": len(gen.urdf.links), "urdf_joints": len(gen.urdf.joints)},
        "actuators": actuators, "keyframes": keys, "derived": gen.derived,
        "meshes_converted": sorted({m["converted"] for m in gen.mesh_files.values() if m["converted"]}),
        "unsupported": gen.unsupported,
    }
    (out_dir / f"report_{expansion.name}.json").write_text(json.dumps(report, indent=1, default=float) + "\n")
    return report
