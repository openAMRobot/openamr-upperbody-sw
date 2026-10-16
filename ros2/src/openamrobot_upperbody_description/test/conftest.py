"""Shared helpers for the description tests (xacro Python API; no ROS runtime, only the ament index for $(find)).

The expansion runs on the *installed* package so that what is tested is what a user gets from `ros2`.
"""
import copy
import functools
import itertools
import json
import os
import sys
import xml.etree.ElementTree as ET
from collections import Counter

import pytest
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

# scripts/ holds the evidence helpers (evidence_common.py ...), which the tests share with the commands.
SCRIPTS = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, "scripts")
sys.path.insert(0, SCRIPTS)
import evidence_common  # noqa: E402

PKG = "openamrobot_upperbody_description"
SHARE = get_package_share_directory(PKG)
COMBINED = os.path.join(SHARE, "urdf", "combined_robot.xacro")

# Option B's configuration (Alex's, merged on main): the single source of the configuration IDs, the baseline, the column data,
# the geometry and the lift calibration stops. There is no mast and no mast_ids.yaml.
UPPERBODY = evidence_common.load_upperbody(SHARE)
AXES = UPPERBODY["configuration_axes"]
BASELINE = UPPERBODY["baseline"]
STOPS = UPPERBODY["lift_calibration_stops"]            # calibration IDs: {L1000: shoulder height m, ...}
LIFT_STROKE = UPPERBODY["column"][BASELINE["column"]]["stroke"]

DATUMS_FILE = os.path.join(SHARE, "config", "datums.yaml")
SCHEMA_FILE = os.path.join(SHARE, "config", "datums.schema.json")
FIXTURES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fixtures")


def write_sidecar(check_id, slug, name, result, details):
    """Record a check's result and details in the evidence directory (`$OPENAMR_EVIDENCE_DIR`, else
    `<workspace>/evidence`, never a temp dir) for build_evidence.py. Returns the path."""
    return evidence_common.write_sidecar(check_id, slug, name, result, details,
                                         directory=evidence_common.evidence_dir(SHARE))


def read_text(path):
    with open(path) as f:
        return f.read()


def load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def load_datums():
    """Fresh parsed datums.yaml (a deep copy: tests may mutate it)."""
    return copy.deepcopy(load_yaml(DATUMS_FILE))


def load_schema():
    with open(SCHEMA_FILE) as f:
        return json.load(f)


BASE_PLATES = list(AXES["base_plate"])
ARM_PACKAGES = list(evidence_common.ARM_PACKAGES)
HARDWARE_PROFILES = ["mock", "gazebo", "real"]
# Structural tests loop over every base plate x arm package (the lift joint, the mounts and the arms are the same for every head
# pitch and base tilt: those two only move the head camera and the base camera). The one-factor-at-a-time matrix of T01/C01 is
# evidence_common.expansion_matrix.
MATRIX = list(itertools.product(BASE_PLATES, ARM_PACKAGES))
# xacro arg defaults of combined_robot.xacro (what an expansion with no override uses)
ARG_DEFAULTS = {**BASELINE, "arm_package": "openarm", "hardware_profile": "mock", "enable_planar_base": "false"}


def expand_xml(**mappings):
    """Expand combined_robot.xacro with the given args; return the URDF text. Raises xacro.XacroException."""
    return xacro.process_file(COMBINED, mappings={k: str(v) for k, v in mappings.items()}).toxml()


@functools.lru_cache(maxsize=None)
def _cached(items):
    return expand_xml(**dict(items))


def expand(**mappings):
    """Cached expansion -> ElementTree root."""
    return ET.fromstring(_cached(tuple(sorted(mappings.items()))))


def tree_facts(root):
    links = [e.get("name") for e in root.findall("link")]
    joints = root.findall("joint")
    child_of = {j.find("child").get("link"): j.find("parent").get("link") for j in joints}
    return links, [j.get("name") for j in joints], child_of


def duplicates(names):
    return sorted(n for n, c in Counter(names).items() if c > 1)


@pytest.fixture(params=MATRIX, ids=[f"{p}-{a}" for p, a in MATRIX])
def combo(request):
    return request.param


def xacro_files():
    """Every .xacro installed with the package (the sources of truth for what the model references)."""
    out = []
    for dirpath, _, names in os.walk(os.path.join(SHARE, "urdf")):
        # isfile follows symlinks, so a stale link left in install/ by a deleted source (colcon does not remove it) is skipped
        # here and reported by I01 instead of crashing every test that reads the xacro text
        out += [os.path.join(dirpath, n) for n in names if n.endswith(".xacro") and os.path.isfile(os.path.join(dirpath, n))]
    return sorted(out)
