# SPDX-License-Identifier: MIT
"""Fixtures of the grocery scene tests.

Inputs, from the generator of simulation/tools/generate.py (never committed):
  GROCERY_ROBOT_MJCF   the generated MJCF, e.g. build/sim/bp000/openamrobot_bp000.xml
  GROCERY_ISAAC_JOB    the Isaac import job, e.g. build/sim/bp000/isaac/isaac_job.json (generate.py --isaac)
A missing input is a failure marked BLOCKED, never a skip and never a pass. The store tests need no input.
"""
import os
import pathlib
import sys

import pytest

GROCERY = pathlib.Path(__file__).resolve().parents[1]
SIM = GROCERY.parents[1]
sys.path.insert(0, str(GROCERY))
sys.path.insert(0, str(SIM))


def _input(var):
    path = os.environ.get(var)
    if not path or not pathlib.Path(path).is_file():
        pytest.fail(f"BLOCKED: {var} is not set to a generated file (see simulation/scenes/grocery/tests/conftest.py)", pytrace=False)
    return pathlib.Path(path)


@pytest.fixture(scope="session")
def robot_mjcf():
    return _input("GROCERY_ROBOT_MJCF")


@pytest.fixture(scope="session")
def isaac_job():
    return _input("GROCERY_ISAAC_JOB")


@pytest.fixture(scope="session")
def composed(robot_mjcf, tmp_path_factory):
    import compose_mujoco
    out = tmp_path_factory.mktemp("grocery_mujoco")
    manifest = compose_mujoco.compose(robot_mjcf, out)
    return out, manifest


@pytest.fixture(scope="session")
def scene_xml(composed):
    return composed[0] / composed[1]["scene"]
