# SPDX-License-Identifier: MIT
"""The full model starts just clear of its contact plane, without a large drop."""
import importlib.util
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import numpy as np
import xacro

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / 'openamrobot_upperbody_description/test'))
from urdf_utils import joint_origin, zero_pose_poses  # noqa: E402


def test_spawn_is_clear_and_within_five_mm_of_the_ground():
    spec = importlib.util.spec_from_file_location('sim_spawn', HERE.parent / 'launch/sim.launch.py')
    sim = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(sim)
    description = Path(get_package_share_directory('openamrobot_upperbody_description'))
    robot = ET.fromstring(xacro.process_file(str(description / 'urdf/combined_robot.xacro')).toxml())
    poses = zero_pose_poses(robot)
    for name in ['left_wheel_link', 'right_wheel_link']:
        collision = robot.find(f"link[@name='{name}']/collision")
        pose = poses[name] @ joint_origin(collision)
        cylinder = collision.find('geometry/cylinder')
        radius, half_length = float(cylinder.get('radius')), float(cylinder.get('length')) / 2
        axis_z = pose[2, 2]
        bottom = pose[2, 3] - radius * np.sqrt(1 - axis_z**2) - half_length * abs(axis_z)
        clearance = bottom + float(sim.SPAWN_HEIGHT)
        # Proposed spawn-placement bound; not a physical performance tolerance.
        assert 0 < clearance <= 0.005, (name, clearance)
