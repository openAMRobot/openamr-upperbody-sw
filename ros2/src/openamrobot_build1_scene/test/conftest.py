# SPDX-License-Identifier: MIT
"""Shared paths and the expanded models for the static tests."""
import os
import subprocess
import sys

import pytest
import yaml

PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPOSITORY = os.path.abspath(os.path.join(PACKAGE_DIR, "..", "..", ".."))
sys.path.insert(0, PACKAGE_DIR)


def package_path(*parts):
    return os.path.join(PACKAGE_DIR, *parts)


def load_yaml(*parts):
    with open(package_path(*parts), encoding="utf-8") as f:
        return yaml.safe_load(f)


def expand(**mappings):
    """The wrapper model expanded with xacro, as build1.launch.py does it."""
    command = ["xacro", package_path("urdf", "build1_robot.urdf.xacro"), "hardware_profile:=gazebo",
               *[f"{key}:={value}" for key, value in mappings.items()]]
    return subprocess.run(command, check=True, capture_output=True, text=True).stdout


@pytest.fixture(scope="session")
def scene():
    from openamrobot_build1_scene import scene as scene_model
    return scene_model.load(package_path("config", "scene.yaml"))


@pytest.fixture(scope="session")
def command_map():
    return load_yaml("config", "command_map.yaml")


@pytest.fixture(scope="session")
def devices_urdf():
    return expand(devices="true", finger_follower="true")


@pytest.fixture(scope="session")
def direct_urdf():
    return expand(devices="false")
