# SPDX-License-Identifier: MIT
"""Fixtures of the simulation tests.

Inputs come from environment variables, each a checkout at its pin:
  OPENAMR_PLATFORM_SW   openamr-platform-sw at the pin of upstream_pins.yaml (the combined model reads it)
  OPENAMR_PLATFORM_SW_MAIN  openamr-platform-sw at the commit engine_overrides.yaml cites for friction (pull request 135)
  OPENARM_DESCRIPTION   enactic/openarm_description at the pin
  OPENARM_ROS2          enactic/openarm_ros2 at the pin
  OPENARM_MUJOCO        enactic/openarm_mujoco at the pin of simulation/sim_pins.yaml
A missing input is a failure marked BLOCKED, never a skip and never a pass. xacro must be on PATH (source /opt/ros/jazzy/setup.bash).
SIM_EVIDENCE_DIR, if set, receives one JSON file per physics check and the parity summaries.
"""
import json
import os
import pathlib
import subprocess
import sys

import pytest

SIM = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM))

from openamr_sim import expand as ex  # noqa: E402


def checkout(var, expected=None):
    path = os.environ.get(var)
    if not path or not pathlib.Path(path).is_dir():
        pytest.fail(f"BLOCKED: {var} is not set to a checkout (see simulation/tests/conftest.py)", pytrace=False)
    if expected is not None:
        head = subprocess.run(["git", "-C", path, "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip()
        if head != expected:
            pytest.fail(f"BLOCKED: {var}={path} is at {head or 'no commit'}, expected {expected}", pytrace=False)
    return pathlib.Path(path)


@pytest.fixture(scope="session")
def platform_sw():
    return checkout("OPENAMR_PLATFORM_SW", ex.pins()["openamr_platform_sw"])


@pytest.fixture(scope="session")
def openarm_description():
    return checkout("OPENARM_DESCRIPTION", ex.pins()["openarm_description"])


@pytest.fixture(scope="session")
def openarm_ros2():
    return checkout("OPENARM_ROS2", ex.pins()["openarm_ros2"])


@pytest.fixture(scope="session")
def overrides():
    from openamr_sim.mjcf import OVERRIDES, load_yaml
    return load_yaml(OVERRIDES)


@pytest.fixture(scope="session")
def platform_sw_main(overrides):
    return checkout("OPENAMR_PLATFORM_SW_MAIN", overrides["sources"]["platform_sw_main"]["commit"])


@pytest.fixture(scope="session")
def openarm_mujoco():
    import yaml
    pin = yaml.safe_load((SIM / "sim_pins.yaml").read_text())["openarm_mujoco"]["commit"]
    return checkout("OPENARM_MUJOCO", pin)


@pytest.fixture(scope="session")
def expansions(platform_sw, openarm_description):
    return {name: ex.expand(name, platform_sw, openarm_description) for name in sorted(ex.CONFIGURATIONS)}


@pytest.fixture(scope="session")
def generated(expansions, tmp_path_factory):
    """MJCF for every configuration, generated once per session into a temporary directory (never into the repository)."""
    from openamr_sim import mjcf
    out = tmp_path_factory.mktemp("sim")
    result = {}
    for name, e in expansions.items():
        report = mjcf.generate(e, out / name)
        result[name] = {"dir": out / name, "xml": out / name / report["mjcf"], "report": report, "expansion": e}
    return result


@pytest.fixture(scope="session")
def models(generated):
    import mujoco
    return {name: mujoco.MjModel.from_xml_path(str(g["xml"])) for name, g in generated.items()}


@pytest.fixture(scope="session")
def evidence():
    directory = os.environ.get("SIM_EVIDENCE_DIR")

    def write(name, data):
        if directory:
            pathlib.Path(directory).mkdir(parents=True, exist_ok=True)
            (pathlib.Path(directory) / f"{name}.json").write_text(json.dumps(data, indent=1, default=float) + "\n")
        return data
    return write


@pytest.fixture(scope="session")
def mesh_loader(expansions, tmp_path_factory):
    import numpy as np
    import trimesh
    from openamr_sim.mjcf import Generator
    gen = Generator(next(iter(expansions.values())), tmp_path_factory.mktemp("meshes"))

    def load(g):
        mesh = trimesh.load(gen._resolve(g.params["filename"]), force="mesh", process=False)
        return np.asarray(mesh.vertices) * np.asarray(g.params["scale"])
    return load
