"""Reading the <ros2_control> blocks of an expanded combined URDF, shared by T14, T15 and T16.

Everything is derived from the expansion itself (the joints that need an interface are the arm's non-mimic movable
joints), so a test never carries its own copy of a joint list that could drift from the model.
"""
import re
import xml.etree.ElementTree as ET

import xacro
from ament_index_python.packages import get_package_share_directory

UPSTREAM_TOP_LEVEL = ("assets/robot/openarm_v2.0/urdf/openarm_v20.urdf.xacro")
BASE_JOINT_HINTS = ("wheel", "caster")        # platform-sw drive and caster joints: never under ros2_control


def blocks(root):
    """The <ros2_control> elements of an expansion."""
    return root.findall("ros2_control")


def plugins(root):
    """[(system name, plugin text)] for every <ros2_control> block."""
    return [(b.get("name"), (b.findtext("hardware/plugin") or "").strip()) for b in blocks(root)]


def element_text(root):
    """The expansion as text with comments dropped (ElementTree discards them), so a comment cannot match."""
    return ET.tostring(root, encoding="unicode")


def hardware_params(block):
    return {p.get("name"): (p.text or "").strip() for p in block.findall("hardware/param")}


def controlled_joints(root):
    """{joint name: {"command": [...], "state": [...]}} over every <ros2_control> block."""
    out = {}
    for block in blocks(root):
        for joint in block.findall("joint"):
            name = joint.get("name")
            assert name not in out, f"joint {name} is declared under two ros2_control blocks"
            out[name] = {"command": [c.get("name") for c in joint.findall("command_interface")],
                         "state": [s.get("name") for s in joint.findall("state_interface")]}
    return out


def arm_joints_needing_an_interface(root):
    """Names of the arm joints a controller drives: movable, `openarm_`-prefixed, and not a URDF mimic follower."""
    return sorted(j.get("name") for j in root.findall("joint")
                  if j.get("name").startswith("openarm_") and j.get("type") not in ("fixed", "floating")
                  and j.find("mimic") is None)


def mimic_followers(root):
    return sorted(j.get("name") for j in root.findall("joint") if j.find("mimic") is not None)


def upstream_blocks(use_fake_hardware):
    """The <ros2_control> blocks upstream's own top-level v2.0 file produces, whitespace-normalised."""
    path = get_package_share_directory("openarm_description") + "/" + UPSTREAM_TOP_LEVEL
    root = ET.fromstring(xacro.process_file(path, mappings={"use_fake_hardware": str(use_fake_hardware).lower()}).toxml())
    return normalised(root)


def normalised(root):
    return [re.sub(r"\s+", "", ET.tostring(b, encoding="unicode")) for b in blocks(root)]
