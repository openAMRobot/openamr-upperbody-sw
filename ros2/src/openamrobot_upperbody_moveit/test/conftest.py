"""Shared helpers for this package's tests: the installed package's files, and expansions of the combined model (xacro Python API)."""
import functools
import os
import sys
import xml.etree.ElementTree as ET

import xacro
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory

PACKAGE = "openamrobot_upperbody_moveit"
SHARE = get_package_share_directory(PACKAGE)
DESCRIPTION_SHARE = get_package_share_directory("openamrobot_upperbody_description")
COMBINED = os.path.join(DESCRIPTION_SHARE, "urdf", "combined_robot.xacro")
SRDF_FILE = os.path.join(SHARE, "config", "openamrobot.srdf")
# Option B's configuration (Alex's, on main): base_plate is the one ID that moves geometry here (the column axis, x +0.060 / +0.110).
with open(os.path.join(DESCRIPTION_SHARE, "config", "upperbody_configuration.yaml")) as _f:
    UPPERBODY = yaml.safe_load(_f)
BASE_PLATES = tuple(UPPERBODY["configuration_axes"]["base_plate"])
STOPS = dict(UPPERBODY["lift_calibration_stops"])          # calibration IDs: {L1000: shoulder height m, ...}
LIFT_STROKE = UPPERBODY["column"][UPPERBODY["baseline"]["column"]]["stroke"]


def description_scripts():
    """The description package's installed scripts directory: evidence_common.py is there, and is the one idea of the evidence dir."""
    path = os.path.join(get_package_prefix("openamrobot_upperbody_description"), "lib", "openamrobot_upperbody_description")
    if path not in sys.path:
        sys.path.insert(0, path)
    return path


def read_text(path):
    with open(path) as f:
        return f.read()


def load_yaml(name):
    with open(os.path.join(SHARE, "config", name)) as f:
        return yaml.safe_load(f)


def srdf_text():
    return read_text(SRDF_FILE)


def srdf_root():
    return ET.fromstring(srdf_text())


@functools.lru_cache(maxsize=None)
def urdf_text(base_plate="bp000", hardware_profile="mock"):
    return xacro.process_file(COMBINED, mappings={"base_plate": base_plate, "hardware_profile": hardware_profile}).toxml()


def urdf_root(base_plate="bp000", hardware_profile="mock"):
    return ET.fromstring(urdf_text(base_plate, hardware_profile))


def write_sidecar(check_id, slug, name, result, details):
    """Record a check in the evidence directory the same way the other packages' tests do."""
    description_scripts()
    import evidence_common as ev
    return ev.write_sidecar(check_id, slug, name, result, details, directory=ev.evidence_dir(DESCRIPTION_SHARE))
