# SPDX-License-Identifier: MIT
"""The arms against Enactic's openarm_mujoco at its pin. Kinematics and inertials must agree; the other differences are the ones
docs/simulators.md lists, no more and no fewer."""
import pathlib
import re

import mujoco
import pytest

from openamr_sim import openarm_compare

DOC = pathlib.Path(__file__).resolve().parents[2] / "docs" / "simulators.md"


@pytest.fixture(scope="module")
def rows(models, openarm_mujoco):
    upstream = mujoco.MjModel.from_xml_path(str(openarm_mujoco / "v2" / "openarm_bimanual.xml"))
    return openarm_compare.compare(mujoco, models["bp000"], upstream)


def test_kinematics_and_inertials_agree(rows):
    for r in rows:
        if r["item"].startswith(("body frame", "mass", "inertia tensor", "joint range", "damping", "frictionloss", "armature",
                                 "equality", "finger_joint2 constraint", "actuator force range")):
            assert r["same"], r


def test_documented_differences(rows, evidence):
    diffs = [r for r in rows if not r["same"]]
    evidence("openarm_mujoco_comparison", {"items": len(rows), "differences": diffs})
    text = DOC.read_text()
    block = text[text.index("<!-- openarm_mujoco differences: begin -->"):text.index("<!-- openarm_mujoco differences: end -->")]
    assert openarm_compare.markdown(rows).strip() in block, "regenerate the table with compare_openarm_mujoco.py"
    count = int(re.search(r"(\d+) items compared, (\d+) differ", block).group(2))
    assert count == len(diffs)
