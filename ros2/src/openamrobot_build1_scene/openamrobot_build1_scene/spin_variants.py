# SPDX-License-Identifier: MIT
"""Turn-slip experiment (diag/spin-test, scratch branch): model variants of the drive wheel and castor contact.

EXPERIMENT ONLY. Each variant edits the expanded plant URDF text inside build1.launch.py when the environment variable
BUILD1_SPIN_VARIANT names it. No description package, parameter file or decision is changed; without the variable the
launch is unchanged. The edits are confined to the drive wheel and castor contact elements. No ROS import.

Variants
  baseline     the pinned model as it is
  A_iso        drive wheels: mu2 = mu1 (1.5), fdir1 removed. Isotropic friction: no friction direction at all
  A_axial      drive wheels: fdir1 along the cylinder axis (collision z, invariant under wheel spin), mu1 and mu2 swapped
               so the rolling direction keeps mu 1.5 and the axial direction mu 0.5
  B_sphere     drive wheel collision: a sphere of the same radius at the axle, one contact point at the nominal track
  B_thin       drive wheel collision: the cylinder at 4 mm width instead of 46 mm
  C_coplanar   castor spheres lowered by their clearance above the drive wheel contact plane (0.4 mm): all six
               contacts in one plane at rest
  C_friction   castor spheres mu 0.05 (a rolling-resistance stand-in) instead of 0
  ALL          A_iso + B_sphere + C_coplanar

Simulation only. A variant is a diagnostic probe, not a proposed value.
"""
import re
import xml.etree.ElementTree as ET

ENV = "BUILD1_SPIN_VARIANT"
DRIVE_LINKS = ("left_wheel_link", "right_wheel_link")
CASTER_LINKS = ("fl_wheel_link", "fr_wheel_link", "bl_wheel_link", "br_wheel_link")
CASTER_JOINTS = ("fl_wheel_joint", "fr_wheel_joint", "bl_wheel_joint", "br_wheel_joint")
THIN_WIDTH_M = 0.004
CASTER_MU = 0.05

VARIANTS = {
    "baseline": "the pinned model, unchanged",
    "A_iso": "drive wheels isotropic: mu2 = mu1, fdir1 removed",
    "A_axial": "drive wheels: fdir1 0 0 1 (cylinder axis, spin-invariant), mu1 and mu2 swapped",
    "B_sphere": "drive wheel collision: sphere of the wheel radius at the axle (contact at the nominal track)",
    "B_thin": f"drive wheel collision: cylinder width {THIN_WIDTH_M} m instead of the modelled width",
    "C_coplanar": "castor spheres lowered by their clearance above the drive wheel contact plane",
    "C_friction": f"castor spheres mu1 = mu2 = {CASTER_MU}",
    "ALL": "A_iso + B_sphere + C_coplanar",
}
COMPONENTS = {"ALL": ("A_iso", "B_sphere", "C_coplanar")}


def _one(pattern, text, what):
    matches = list(re.finditer(pattern, text, re.S))
    if len(matches) != 1:
        raise ValueError(f"{what}: expected exactly one match, found {len(matches)}")
    return matches[0]


def _friction_block(urdf, link):
    """The <gazebo reference="link"> element that carries <mu1> (span and text)."""
    blocks = [m for m in re.finditer(rf'<gazebo reference="{link}">.*?</gazebo>', urdf, re.S) if "<mu1>" in m.group(0)]
    if len(blocks) != 1:
        raise ValueError(f"{link}: expected one <gazebo reference> element with <mu1>, found {len(blocks)}")
    return blocks[0]


def _tag(block, tag):
    m = re.search(rf"<{tag}>\s*([^<]*?)\s*</{tag}>", block)
    return None if m is None else m.group(1)


def _set_tag(block, tag, value):
    new, n = re.subn(rf"<{tag}>\s*[^<]*?\s*</{tag}>", f"<{tag}>{value}</{tag}>", block)
    if n != 1:
        raise ValueError(f"<{tag}> found {n} times in {block[:60]!r}")
    return new


def _edit_friction(urdf, link, edit):
    block = _friction_block(urdf, link)
    new = edit(block.group(0))
    return urdf[:block.start()] + new + urdf[block.end():]


def _a_iso(urdf, notes):
    for link in DRIVE_LINKS:
        def edit(block, link=link):
            mu1 = _tag(block, "mu1")
            out = _set_tag(block, "mu2", mu1)
            out, n = re.subn(r"<fdir1>[^<]*</fdir1>", "", out)
            notes.append(f"{link}: mu2 {_tag(block, 'mu2')} -> {mu1}, fdir1 {_tag(block, 'fdir1')!r} removed ({n})")
            return out
        urdf = _edit_friction(urdf, link, edit)
    return urdf


def _a_axial(urdf, notes):
    for link in DRIVE_LINKS:
        def edit(block, link=link):
            mu1, mu2 = _tag(block, "mu1"), _tag(block, "mu2")
            out = _set_tag(_set_tag(block, "mu1", mu2), "mu2", mu1)
            if _tag(out, "fdir1") is None:
                raise ValueError(f"{link}: no fdir1")
            out = _set_tag(out, "fdir1", "0 0 1")
            notes.append(f"{link}: fdir1 {_tag(block, 'fdir1')!r} -> '0 0 1' (collision z, the cylinder axis), mu1 {mu1} -> {mu2}, "
                         f"mu2 {mu2} -> {mu1}")
            return out
        urdf = _edit_friction(urdf, link, edit)
    return urdf


def _wheel_cylinder(urdf, link):
    block = _one(rf'<link name="{link}">.*?</link>', urdf, f"link {link}")
    collision = _one(r"<collision>.*?</collision>", block.group(0), f"{link} collision")
    cylinder = _one(r"<cylinder\s[^>]*/>", collision.group(0), f"{link} collision cylinder")
    attributes = dict(re.findall(r'(\w+)="([^"]+)"', cylinder.group(0)))
    start = block.start() + collision.start() + cylinder.start()
    return start, start + len(cylinder.group(0)), float(attributes["radius"]), float(attributes["length"])


def _b_geometry(urdf, notes, replace):
    for link in DRIVE_LINKS:
        start, end, radius, length = _wheel_cylinder(urdf, link)
        new = replace(radius, length)
        notes.append(f"{link}: collision cylinder r {radius} length {length} -> {new}")
        urdf = urdf[:start] + new + urdf[end:]
    return urdf


def _b_sphere(urdf, notes):
    return _b_geometry(urdf, notes, lambda r, length: f'<sphere radius="{r}"/>')


def _b_thin(urdf, notes):
    return _b_geometry(urdf, notes, lambda r, length: f'<cylinder radius="{r}" length="{THIN_WIDTH_M}"/>')


def _xyz(element):
    origin = element.find("origin")
    if origin is None:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    xyz = tuple(float(v) for v in origin.get("xyz", "0 0 0").split())
    rpy = tuple(float(v) for v in origin.get("rpy", "0 0 0").split())
    return xyz, rpy


def link_height(urdf, link):
    """z of the origin of `link` above the root link, through joints without roll or pitch (refused otherwise)."""
    root = ET.fromstring(urdf)
    parents = {j.find("child").get("link"): j for j in root.findall("joint")}
    z, name = 0.0, link
    while name in parents:
        joint = parents[name]
        xyz, rpy = _xyz(joint)
        if abs(rpy[0]) > 1e-9 or abs(rpy[1]) > 1e-9:
            raise ValueError(f"joint {joint.get('name')} has roll or pitch; the height walk handles yaw only")
        z += xyz[2]
        name = joint.find("parent").get("link")
    return z


def contact_geometry(urdf):
    """Heights above the root of the drive wheel lowest point and of each castor sphere's lowest point, the drive radius
    and the track from the wheel joint y offsets."""
    root = ET.fromstring(urdf)
    out = {}
    for link in DRIVE_LINKS:
        collision = root.find(f"link[@name='{link}']/collision/geometry")
        shape = collision.find("cylinder")
        if shape is None:
            shape = collision.find("sphere")
        radius = float(shape.get("radius"))
        out[f"{link}_radius"] = radius
        out[f"{link}_bottom_z"] = link_height(urdf, link) - radius
    for link in CASTER_LINKS:
        sphere = root.find(f"link[@name='{link}']/collision/geometry/sphere")
        out[f"{link}_bottom_z"] = link_height(urdf, link) - float(sphere.get("radius"))
    ys = {}
    for side in ("left", "right"):
        joint = root.find(f"joint[@name='{side}_wheel_joint']")
        ys[side] = _xyz(joint)[0][1]
    out["track"] = ys["left"] - ys["right"]
    out["radius"] = out["left_wheel_link_radius"]
    return out


def _c_coplanar(urdf, notes):
    geometry = contact_geometry(urdf)
    plane = min(geometry[f"{link}_bottom_z"] for link in DRIVE_LINKS)
    for link, joint_name in zip(CASTER_LINKS, CASTER_JOINTS):
        clearance = geometry[f"{link}_bottom_z"] - plane
        block = _one(rf'<joint name="{joint_name}" type="fixed">.*?</joint>', urdf, f"joint {joint_name}")
        origin = _one(r'<origin[^>]*?xyz="([^"]+)"', block.group(0), f"{joint_name} origin")
        x, y, z = (float(v) for v in origin.group(1).split())
        new_xyz = f"{x} {y} {round(z - clearance, 9)}"
        start = block.start() + origin.start(1)
        urdf = urdf[:start] + new_xyz + urdf[start + len(origin.group(1)):]
        notes.append(f"{joint_name}: z {z} -> {round(z - clearance, 9)} (castor clearance {clearance * 1000:.3f} mm removed)")
    return urdf


def _c_friction(urdf, notes):
    for link in CASTER_LINKS:
        def edit(block, link=link):
            out = _set_tag(_set_tag(block, "mu1", CASTER_MU), "mu2", CASTER_MU)
            notes.append(f"{link}: mu1 {_tag(block, 'mu1')} mu2 {_tag(block, 'mu2')} -> {CASTER_MU}")
            return out
        urdf = _edit_friction(urdf, link, edit)
    return urdf


EDITS = {"A_iso": _a_iso, "A_axial": _a_axial, "B_sphere": _b_sphere, "B_thin": _b_thin,
         "C_coplanar": _c_coplanar, "C_friction": _c_friction}


def apply(urdf, variant):
    """-> (edited URDF, list of edit notes). Refuses an unknown variant."""
    if variant not in VARIANTS:
        raise ValueError(f"unknown spin variant {variant!r}: one of {', '.join(VARIANTS)}")
    notes = []
    for part in COMPONENTS.get(variant, () if variant == "baseline" else (variant,)):
        urdf = EDITS[part](urdf, notes)
    if variant == "baseline":
        notes.append("no edit")
    return urdf, notes
